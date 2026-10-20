#include "esp8266.h"
#include <string.h>
#include "usart.h"

static Esp8266* espInstance = nullptr;

Esp8266::Esp8266(UART_HandleTypeDef* huart) : _huart(huart), _rxIndex(0), _frameReady(false), _state(INIT) {}

void Esp8266::init() {
    sendCmd("ATE0\r\n");
    _timer = HAL_GetTick();
}

void Esp8266::sendCmd(const char* cmd) {
    HAL_UART_Transmit(_huart, (uint8_t*)cmd, strlen(cmd), 1000); 
}

void Esp8266::rxCallback(uint8_t byte) {
    if (_rxIndex < sizeof(_rxBuf) - 1) {
        _rxBuf[_rxIndex++] = byte;
    }
    if (byte == '\n') {
        _rxBuf[_rxIndex] = '\0';
        _frameReady = true;
        _rxIndex = 0;
    }
}

void Esp8266::process() {
    uint32_t now = HAL_GetTick(); 
    
    if (_frameReady) {
        // 打印收到的数据
        HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n[收到]: ", 10, 100);
        HAL_UART_Transmit(&huart1, (uint8_t*)_rxBuf, strlen(_rxBuf), 100);

        // 1. 容错处理：只要有 busy 就跳过
        if (strstr(_rxBuf, "busy")) {
            _frameReady = false;
            return;
        }

        // 2. 终极容错：只要收到了 +IPD 数据，说明 TCP 绝对连上了！
        // 直接强制把状态改为 READY，无视之前的握手过程
        if (strstr(_rxBuf, "IPD")) {
            _state = READY;
        }

        // 3. 状态机：配网与连接
        switch (_state) {
            case INIT:
                if (strstr(_rxBuf, "OK")) {
                    sendCmd("AT+CWMODE=1\r\n"); 
                    _state = CWMODE; 
                }
                break;
                
            case CWMODE:
                if (strstr(_rxBuf, "OK")) {
                    sendCmd("AT+CWJAP=\"ChinaNet-cUup\",\"pmv4awfx\"\r\n"); 
                    _state = CWJAP; 
                }
                break;
                
            case CWJAP:
                if (strstr(_rxBuf, "OK") && !strstr(_rxBuf, "GOT IP")) { 
                    // 注意：收到 OK 且不是 GOT IP 后的 OK 才发 CIPSTART
                    sendCmd("AT+CIPSTART=\"TCP\",\"192.168.1.15\",10000\r\n"); 
                    _state = CIPSTART; 
                } else if (strstr(_rxBuf, "FAIL")) {
                    sendCmd("AT+CWJAP=\"ChinaNet-cUup\",\"pmv4awfx\"\r\n"); 
                }
                break;
                
            case CIPSTART:
                // 只要收到 OK 或者 CONNECT，就认为连上了
                if (strstr(_rxBuf, "OK") || strstr(_rxBuf, "CONNECT")) {
                    _state = READY; 
                    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n>>> TCP Connected <<<\r\n", 18, 100);
                } else if (strstr(_rxBuf, "ERROR")) {
                    sendCmd("AT+CIPSTART=\"TCP\",\"192.168.1.15\",10000\r\n"); 
                }
                break;
                
            case READY:
                // 在这个状态下，处理控制指令
                if (strstr(_rxBuf, "IPD")) {
                    // 💡 注意：这里我只匹配了 LED_ON / LED_OFF
                    if (strstr(_rxBuf, "LED_ON")) {
                        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // 低电平点亮
                        HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n>>> LED ON! <<<\r\n", 18, 100); 
                    }
                    else if (strstr(_rxBuf, "LED_OFF")) {
                        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);   // 高电平熄灭
                        HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n>>> LED OFF! <<<\r\n", 19, 100); 
                    }
                }
                break;
        }
        
        _frameReady = false;
        _timer = now; 
    }
}

// ============ 提供给 main.c 的接口实现 ============
extern "C" {

void Esp8266_Init(UART_HandleTypeDef* huart) {
    if (espInstance == nullptr) {
        espInstance = new Esp8266(huart);
    }
    espInstance->init();
}

void Esp8266_Process(void) {
    if (espInstance != nullptr) {
        espInstance->process();
    }
}

void Esp8266_RxCallback(uint8_t byte) {
    if (espInstance != nullptr) {
        espInstance->rxCallback(byte);
    }
}

}