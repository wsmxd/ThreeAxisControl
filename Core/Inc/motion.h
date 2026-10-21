#ifndef __MOTION_H
#define __MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============ 时基频率,必须与 TIM6 配置一致 ============ */
#define TICK_RATE_HZ    20000u

/* ============ 引脚定义 ============ */
/* X 轴 — 已接线 */
#define X_PUL_PORT      GPIOA
#define X_PUL_PIN       GPIO_PIN_8
#define X_DIR_PORT      GPIOC
#define X_DIR_PIN       GPIO_PIN_8
#define X_ENA_PORT      GPIOC
#define X_ENA_PIN       GPIO_PIN_6

/* Y / Z — 硬件到货后改这里 */
#define Y_PUL_PORT      GPIOB
#define Y_PUL_PIN       GPIO_PIN_0
#define Y_DIR_PORT      GPIOB
#define Y_DIR_PIN       GPIO_PIN_1
#define Z_PUL_PORT      GPIOB
#define Z_PUL_PIN       GPIO_PIN_2
#define Z_DIR_PORT      GPIOB
#define Z_DIR_PIN       GPIO_PIN_3

/* 影子轴开关:1=Y/Z 不真实输出(调试阶段);硬件到货后改 0 */
#define Y_Z_SHADOW      1

typedef enum { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2 } AxisId;

typedef struct {
    int32_t position;
    int32_t target;
    int32_t abs_delta;
    int32_t err;
    uint8_t dir;
    bool    active;
} AxisState;

void  Motion_Init(void);
bool  Motion_MoveTo(int32_t x, int32_t y, int32_t z, uint32_t feed_hz);
bool  Motion_IsBusy(void);
void  Motion_Stop(void);
const AxisState* Motion_GetAxis(AxisId id);
void  Motion_TickHandler(void);     /* TIM6 中断里调用 */

#ifdef __cplusplus
}
#endif
#endif