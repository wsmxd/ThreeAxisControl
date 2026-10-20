#ifndef ESP8266_H
#define ESP8266_H

#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// 暴露给 main.c 的 C 接口
void Esp8266_Init(UART_HandleTypeDef* huart);
void Esp8266_Process(void);
void Esp8266_RxCallback(uint8_t byte);

#ifdef __cplusplus
}
#endif

// 真正的 C++ 类定义
#ifdef __cplusplus
class Esp8266 {
public:
    Esp8266(UART_HandleTypeDef* huart);
    void init();
    void process();
    void rxCallback(uint8_t byte);

private:
    UART_HandleTypeDef* _huart;
    uint8_t _rxByte;
    char _rxBuf[512];
    uint16_t _rxIndex;
    bool _frameReady;
    
    enum State { INIT, CWMODE, CWJAP, CIPSTART, READY };
    State _state;
    uint32_t _timer;

    void sendCmd(const char* cmd);
};
#endif

#endif