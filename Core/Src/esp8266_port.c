#include "esp8266.h"
#include "usart.h"          /* 只用到 huart3 */

#ifndef ESP_DMA_BUF_SIZE
#define ESP_DMA_BUF_SIZE 1024
#endif

static uint8_t            s_dmaBuf[ESP_DMA_BUF_SIZE];
static volatile uint16_t  s_dmaLastPos = 0;
static UART_HandleTypeDef* s_espUart   = 0;

static inline void esp_disable_ht(UART_HandleTypeDef* huart) {
    if (huart && huart->hdmarx) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

bool Esp8266_HwStart(UART_HandleTypeDef* huart) {
    if (!huart) return false;
    s_espUart    = huart;
    s_dmaLastPos = 0;
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, s_dmaBuf, ESP_DMA_BUF_SIZE) != HAL_OK) {
        return false;
    }
    esp_disable_ht(huart);
    return true;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if (huart != s_espUart) return;

    uint16_t curr = Size;
    if (curr == s_dmaLastPos) return;

    if (curr > s_dmaLastPos) {
        Esp8266_OnRxBlock(&s_dmaBuf[s_dmaLastPos],
                          (uint16_t)(curr - s_dmaLastPos));
    } else {
        if (s_dmaLastPos < ESP_DMA_BUF_SIZE) {
            Esp8266_OnRxBlock(&s_dmaBuf[s_dmaLastPos],
                              (uint16_t)(ESP_DMA_BUF_SIZE - s_dmaLastPos));
        }
        if (curr > 0) {
            Esp8266_OnRxBlock(&s_dmaBuf[0], curr);
        }
    }
    s_dmaLastPos = curr;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if (huart != s_espUart) return;
    HAL_UART_DMAStop(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    s_dmaLastPos = 0;
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, s_dmaBuf, ESP_DMA_BUF_SIZE) == HAL_OK) {
        esp_disable_ht(huart);
    }
}