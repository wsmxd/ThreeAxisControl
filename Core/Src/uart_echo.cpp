#include "uart_echo.h"
#include <string.h>

// 在这里实例化 C++ 对象，main.c 看不到它，但可以通过接口调用它
static UartEcho* echoInstance = nullptr;

// 构造函数
UartEcho::UartEcho(UART_HandleTypeDef* huart) 
    : _huart(huart), _rxIndex(0), _lineReady(false) 
{
}

void UartEcho::init() {
    _rxIndex = 0;
    _lineReady = false;
}

void UartEcho::rxCallback(uint8_t byte) {
    if (byte == '\r' || byte == '\n' || _rxIndex >= sizeof(_rxBuf) - 1) {
        if (_rxIndex > 0) {
            _rxBuf[_rxIndex] = '\0';
            _lineReady = true;
            _rxIndex = 0;
        }
    } else {
        _rxBuf[_rxIndex++] = byte;
    }
}

void UartEcho::process() {
    if (_lineReady) {
        HAL_UART_Transmit(_huart, (uint8_t*)_rxBuf, strlen(_rxBuf), 100);
        uint8_t newline[] = "\r\n";
        HAL_UART_Transmit(_huart, newline, 2, 100);
        _lineReady = false;
    }
}

// ============ 实现提供给 C 语言的外壳接口 ============
extern "C" {

void UartEcho_Init(UART_HandleTypeDef* huart) {
    if (echoInstance == nullptr) {
        echoInstance = new UartEcho(huart);
    }
    echoInstance->init();
}

void UartEcho_Process(void) {
    if (echoInstance != nullptr) {
        echoInstance->process();
    }
}

void UartEcho_RxCallback(uint8_t byte) {
    if (echoInstance != nullptr) {
        echoInstance->rxCallback(byte);
    }
}

} // extern "C"