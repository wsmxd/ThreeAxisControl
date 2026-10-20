#ifndef UART_ECHO_H
#define UART_ECHO_H

#include <stdint.h>
#include "main.h"

// 告诉编译器，这部分代码按 C 语言规则导出
#ifdef __cplusplus
extern "C" {
#endif

// 提供给 main.c 调用的纯 C 函数接口
void UartEcho_Init(UART_HandleTypeDef* huart);
void UartEcho_Process(void);
void UartEcho_RxCallback(uint8_t byte);

#ifdef __cplusplus
}
#endif

// ============ 下面是真正的 C++ 类定义 ============
#ifdef __cplusplus
class UartEcho {
public:
    UartEcho(UART_HandleTypeDef* huart);
    void init();
    void process();
    void rxCallback(uint8_t byte);

private:
    UART_HandleTypeDef* _huart;
    char _rxBuf[128];
    uint16_t _rxIndex;
    bool _lineReady;
};
#endif // __cplusplus

#endif // UART_ECHO_H