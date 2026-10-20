#include "esp8266.h"
#include <string.h>
#include <stdio.h>

/* ============ 用户配置区 ============ */
#ifndef ESP_WIFI_SSID
#define ESP_WIFI_SSID     "ChinaNet-cUup"
#endif
#ifndef ESP_WIFI_PASSWORD
#define ESP_WIFI_PASSWORD "pmv4awfx"
#endif
#ifndef ESP_TCP_HOST
#define ESP_TCP_HOST      "192.168.1.15"
#endif
#ifndef ESP_TCP_PORT
#define ESP_TCP_PORT      10000
#endif
#ifndef ESP_DEBUG_ENABLE
#define ESP_DEBUG_ENABLE  1
#endif

/* ============ 调试口:由 main.c 注入 ============ */
static UART_HandleTypeDef* s_debugUart = nullptr;

static Esp8266* espInstance = nullptr;
static const Esp8266::Config g_defaultCfg = {
    ESP_WIFI_SSID, ESP_WIFI_PASSWORD, ESP_TCP_HOST, ESP_TCP_PORT
};

/* ============ 构造 ============ */
Esp8266::Esp8266(UART_HandleTypeDef* huart, const Config& cfg)
    : _huart(huart), _cfg(cfg),
      _rxHead(0), _rxTail(0), _rxOverflow(false),
      _lineLen(0),
      _rxParseState(RX_LINE),
      _ipdExpected(0), _ipdRecv(0),
      _state(ST_INIT), _stateTimer(0), _retryCount(0),
      _gotIp(false), _cipRetry(0),
      _txLen(0), _txPending(false), _txTimer(0)
{
    memset(_rxRing,  0, sizeof(_rxRing));
    memset(_lineBuf, 0, sizeof(_lineBuf));
    memset(_ipdBuf,  0, sizeof(_ipdBuf));
    memset(_txBuf,   0, sizeof(_txBuf));
}

void Esp8266::init() {
    sendCmd("AT\r\n");
    changeState(ST_INIT);
}

void Esp8266::changeState(State s) {
    _state = s;
    _stateTimer = HAL_GetTick();
    _retryCount = 0;
    if (s != ST_CIPSTART) _cipRetry = 0;
    if (s != ST_CWJAP)    _gotIp    = false;
}

bool Esp8266::isReady() const {
    return _state == ST_READY;
}

/* ============ 发送 ============ */
void Esp8266::sendCmd(const char* cmd) {
    if (!_huart || !cmd) return;
    HAL_UART_Transmit(_huart, (uint8_t*)cmd, strlen(cmd), 100);
}

bool Esp8266::sendData(const uint8_t* data, uint16_t len) {
    if (_state != ST_READY || _txPending) return false;
    if (!data || len == 0 || len > TX_MAX) return false;

    memcpy(_txBuf, data, len);
    _txLen     = len;
    _txPending = true;
    _txTimer   = HAL_GetTick();

    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u\r\n", (unsigned)len);
    sendCmd(cmd);
    return true;
}

/* ============ RX:中断里只拷贝到环形缓冲 ============ */
void Esp8266::onRxBlock(const uint8_t* data, uint16_t len) {
    if (!data || len == 0) return;
    uint16_t head = _rxHead;
    uint16_t tail = _rxTail;
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (uint16_t)((head + 1u) % RING_SIZE);
        if (next == tail) { _rxOverflow = true; break; }
        _rxRing[head] = data[i];
        head = next;
    }
    _rxHead = head;
}

/* ============ 主循环 ============ */
void Esp8266::process() {
    drainRx();

    if (_rxOverflow) {
        _rxOverflow = false;
        debug("WARN", "RX overflow");
    }

    uint32_t now = HAL_GetTick();

    if (_txPending && (now - _txTimer) > TX_TIMEOUT_MS) {
        _txPending = false;
        debug("WARN", "CIPSEND timeout");
    }

    if (_state != ST_READY && _stateTimer != 0 &&
        (now - _stateTimer) > CMD_TIMEOUT_MS) {
        if (++_retryCount > MAX_RETRY) {
            debug("STATE", "max retry, restart");
            _retryCount = 0;
            sendCmd("AT\r\n");
            changeState(ST_INIT);
        } else {
            debug("STATE", "timeout, retry");
            doRetry();
            _stateTimer = now;
        }
    }
}

void Esp8266::doRetry() {
    char cmd[160];
    switch (_state) {
    case ST_INIT:    sendCmd("AT\r\n");          break;
    case ST_ATE0:    sendCmd("ATE0\r\n");        break;
    case ST_CWMODE:  sendCmd("AT+CWMODE=1\r\n"); break;
    case ST_CWJAP:
        snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n",
                 _cfg.ssid, _cfg.password);
        sendCmd(cmd);
        break;
    case ST_CIPSTART:
        snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n",
                 _cfg.host, (unsigned)_cfg.port);
        sendCmd(cmd);
        break;
    default: break;
    }
}

void Esp8266::drainRx() {
    uint16_t tail = _rxTail;
    while (tail != _rxHead) {
        uint8_t b = _rxRing[tail];
        tail = (uint16_t)((tail + 1u) % RING_SIZE);
        parseByte(b);
    }
    _rxTail = tail;
}

/* ============ 逐字节状态机 ============ */
void Esp8266::parseByte(uint8_t b) {
    /* CIPSEND 的 '>' 提示符 */
    if (_txPending && _rxParseState == RX_LINE && _lineLen == 0 && b == '>') {
        HAL_UART_Transmit(_huart, _txBuf, _txLen, TX_SEND_TIMEOUT);
        _txPending = false;
        return;
    }

    switch (_rxParseState) {

    case RX_LINE:
        if (b == '\n') {
            if (_lineLen > 0 && _lineBuf[_lineLen - 1] == '\r') _lineLen--;
            _lineBuf[_lineLen] = '\0';
            if (_lineLen > 0) completeLine();
            _lineLen = 0;
        } else if (b == ':') {
            /* 判定 "+IPD,<digits>:" */
            if (_lineLen >= 6 && memcmp(_lineBuf, "+IPD,", 5) == 0) {
                uint32_t total = 0;
                bool ok = true;
                for (uint16_t i = 5; i < _lineLen; i++) {
                    char c = _lineBuf[i];
                    if (c < '0' || c > '9') { ok = false; break; }
                    total = total * 10u + (uint32_t)(c - '0');
                }
                if (ok && total > 0) {
                    _ipdExpected = (total > 0xFFFFu) ? 0xFFFFu : (uint16_t)total;
                    _ipdRecv     = 0;
                    _rxParseState = RX_IPD_DATA;
                    _lineLen = 0;
                    break;
                }
            }
            if (_lineLen < LINE_MAX - 1) _lineBuf[_lineLen++] = (char)b;
        } else if (b != '\r') {
            if (_lineLen < LINE_MAX - 1) _lineBuf[_lineLen++] = (char)b;
            else _lineLen = 0;
        }
        break;

    case RX_IPD_DATA:
        if (_ipdRecv < IPD_MAX) _ipdBuf[_ipdRecv] = b;
        _ipdRecv++;
        if (_ipdRecv >= _ipdExpected) {
            uint16_t realLen = (_ipdExpected > IPD_MAX) ? (uint16_t)IPD_MAX
                                                        : _ipdExpected;
            _rxParseState = RX_LINE;
            if (_state != ST_READY) changeState(ST_READY);
            onTcpData(_ipdBuf, realLen);
        }
        break;
    }
}

void Esp8266::completeLine() {
    debug("RX", _lineBuf);
    handleLine(_lineBuf);
}

/* ============ AT 响应行处理(核心修复区) ============ */
void Esp8266::handleLine(const char* line) {

    /* ---- ① WIFI DISCONNECT:只在 READY/CIPSTART 时处理 ---- */
    if (strstr(line, "WIFI DISCONNECT")) {
        if (_state == ST_READY || _state == ST_CIPSTART) {
            debug("NET", "WiFi down, rejoin");
            char cmd[160];
            snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n",
                     _cfg.ssid, _cfg.password);
            sendCmd(cmd);
            _gotIp = false;
            changeState(ST_CWJAP);
        }
        /* INIT/ATE0/CWMODE/CWJAP 阶段忽略,避免打断握手 */
        return;
    }

    /* ---- ② CLOSED:只在 READY 时主动重连 TCP ---- */
    if (strstr(line, "CLOSED")) {
        if (_state == ST_READY) {
            debug("NET", "TCP closed, reconnect");
            char cmd[160];
            snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n",
                     _cfg.host, (unsigned)_cfg.port);
            sendCmd(cmd);
            changeState(ST_CIPSTART);
        }
        /* CIPSTART 期间的 CLOSED 交给状态机重试,不主动重连 */
        return;
    }

    /* ---- ③ 状态机 ---- */
    switch (_state) {
    case ST_INIT:
        if (strstr(line, "OK")) { sendCmd("ATE0\r\n"); changeState(ST_ATE0); }
        break;

    case ST_ATE0:
        if (strstr(line, "OK")) { sendCmd("AT+CWMODE=1\r\n"); changeState(ST_CWMODE); }
        break;

    case ST_CWMODE:
        if (strstr(line, "OK")) {
            char cmd[160];
            snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n",
                     _cfg.ssid, _cfg.password);
            sendCmd(cmd);
            _gotIp = false;
            changeState(ST_CWJAP);
        }
        break;

    case ST_CWJAP:
        if (strstr(line, "WIFI GOT IP")) {
            _gotIp = true;
        } else if (strstr(line, "OK")) {
            if (_gotIp) {
                debug("NET", "WiFi ready, connect TCP");
                char cmd[160];
                snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n",
                         _cfg.host, (unsigned)_cfg.port);
                sendCmd(cmd);
                changeState(ST_CIPSTART);
            }
        } else if (strstr(line, "FAIL") || strstr(line, "ERROR")) {
            if (++_retryCount <= MAX_RETRY) {
                char cmd[160];
                snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n",
                         _cfg.ssid, _cfg.password);
                sendCmd(cmd);
                _gotIp = false;
            }
        }
        break;

    case ST_CIPSTART:
        if (strstr(line, "CONNECT") || strstr(line, "ALREADY CONNECTED")) {
            debug("NET", "TCP connected");
            _cipRetry = 0;
            changeState(ST_READY);
        } else if (strstr(line, "ERROR") || strstr(line, "CONNECT FAIL")) {
            if (++_cipRetry >= 3) {
                debug("NET", "CIPSTART fail x3, rejoin WiFi");
                _cipRetry = 0;
                char cmd[160];
                snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n",
                         _cfg.ssid, _cfg.password);
                sendCmd(cmd);
                _gotIp = false;
                changeState(ST_CWJAP);
            } else {
                debug("NET", "CIPSTART fail, close & retry");
                sendCmd("AT+CIPCLOSE\r\n");
                char cmd[160];
                snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n",
                         _cfg.host, (unsigned)_cfg.port);
                sendCmd(cmd);
                _stateTimer = HAL_GetTick();
            }
        }
        break;

    case ST_READY:
        if (strstr(line, "SEND FAIL")) {
            _txPending = false;
            debug("TX", "send failed");
        }
        break;
    }
}

/* ============ 应用层分发(可继承覆盖) ============ */
void Esp8266::onTcpData(const uint8_t* data, uint16_t len) {
    if (len >= 6 && memcmp(data, "LED_ON",  6) == 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
        debug("CMD", "LED ON");
    } else if (len >= 7 && memcmp(data, "LED_OFF", 7) == 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
        debug("CMD", "LED OFF");
    } else {
        debug("TCP", "unknown payload");
    }
}

/* ============ 调试输出:走 s_debugUart,完全解耦 ============ */
void Esp8266::debug(const char* tag, const char* msg) {
#if ESP_DEBUG_ENABLE
    if (!s_debugUart) return;
    HAL_UART_Transmit(s_debugUart, (uint8_t*)tag,  strlen(tag),   20);
    HAL_UART_Transmit(s_debugUart, (uint8_t*)": ", 2,             20);
    HAL_UART_Transmit(s_debugUart, (uint8_t*)msg,  strlen(msg),  100);
    HAL_UART_Transmit(s_debugUart, (uint8_t*)"\r\n", 2,           20);
#else
    (void)tag; (void)msg;
#endif
}

/* ============ C 接口 ============ */
extern "C" {

void Esp8266_SetDebugUart(UART_HandleTypeDef* huart) {
    s_debugUart = huart;
}

void Esp8266_Init(UART_HandleTypeDef* huart) {
    if (!espInstance) espInstance = new Esp8266(huart, g_defaultCfg);
    espInstance->init();
}

void Esp8266_Process(void) {
    if (espInstance) espInstance->process();
}

bool Esp8266_IsReady(void) {
    return espInstance && espInstance->isReady();
}

bool Esp8266_SendData(const uint8_t* data, uint16_t len) {
    return espInstance && espInstance->sendData(data, len);
}

void Esp8266_OnRxBlock(const uint8_t* data, uint16_t len) {
    if (espInstance) espInstance->onRxBlock(data, len);
}

}