#ifndef __ESP8266_H
#define __ESP8266_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 对外 C 接口 ---- */
void Esp8266_SetDebugUart(UART_HandleTypeDef* huart);   /* 注入调试口 UART1 */
void Esp8266_Init(UART_HandleTypeDef* huart);           /* 传 &huart3        */
bool Esp8266_HwStart(UART_HandleTypeDef* huart);        /* 传 &huart3,启动 IDLE+DMA */
void Esp8266_Process(void);
bool Esp8266_IsReady(void);
bool Esp8266_SendData(const uint8_t* data, uint16_t len);

/* 由 esp8266_port.c 的 IDLE/DMA 回调调用 */
void Esp8266_OnRxBlock(const uint8_t* data, uint16_t len);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class Esp8266 {
public:
    struct Config {
        const char* ssid;
        const char* password;
        const char* host;
        uint16_t    port;
    };

    Esp8266(UART_HandleTypeDef* huart, const Config& cfg);

    void init();
    void process();
    bool sendData(const uint8_t* data, uint16_t len);
    bool isReady() const;

    void onRxBlock(const uint8_t* data, uint16_t len);

protected:
    virtual void onTcpData(const uint8_t* data, uint16_t len);

private:
    enum : uint16_t {
        RING_SIZE = 4096,
        LINE_MAX  = 256,
        IPD_MAX   = 2048,
        TX_MAX    = 256,
        MAX_RETRY = 5
    };
    enum : uint32_t {
        CMD_TIMEOUT_MS  = 8000,
        TX_SEND_TIMEOUT = 1000,
        TX_TIMEOUT_MS   = 3000
    };

    enum RxParseState : uint8_t { RX_LINE = 0, RX_IPD_DATA };
    enum State : uint8_t {
        ST_INIT = 0, ST_ATE0, ST_CWMODE, ST_CWJAP, ST_CIPSTART, ST_READY
    };

    UART_HandleTypeDef* _huart;
    Config _cfg;

    /* 中断 → 主循环交接用的环形缓冲 */
    uint8_t           _rxRing[RING_SIZE];
    volatile uint16_t _rxHead;
    volatile uint16_t _rxTail;
    volatile bool     _rxOverflow;

    /* AT 行缓冲 */
    char     _lineBuf[LINE_MAX];
    uint16_t _lineLen;

    /* +IPD 数据 */
    RxParseState _rxParseState;
    uint16_t     _ipdExpected;
    uint16_t     _ipdRecv;
    uint8_t      _ipdBuf[IPD_MAX];

    /* 状态机 */
    State    _state;
    uint32_t _stateTimer;
    uint8_t  _retryCount;
    bool     _gotIp;        /* CWJAP 已收到 WIFI GOT IP    */
    uint8_t  _cipRetry;     /* CIPSTART 连续失败计数       */

    /* 发送 */
    uint8_t  _txBuf[TX_MAX];
    uint16_t _txLen;
    bool     _txPending;
    bool     _txSending;
    uint32_t _txTimer;

    void drainRx();
    void parseByte(uint8_t b);
    void completeLine();
    void handleLine(const char* line);
    void sendCmd(const char* cmd);
    void changeState(State s);
    void doRetry();
    void debug(const char* tag, const char* msg);
};
#endif /* __cplusplus */

#endif /* __ESP8266_H */