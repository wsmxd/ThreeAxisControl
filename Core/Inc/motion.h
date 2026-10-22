#ifndef __MOTION_H
#define __MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TIM6 tick rate, must match tim.c */
#define TICK_RATE_HZ    20000u
#define MAX_STEP_HZ     (TICK_RATE_HZ / 2u)

#define MOTION_QUEUE_SIZE       32u
#define MOTION_DONE_QUEUE_SIZE  64u

/* X axis, connected */
#define X_PUL_PORT      GPIOA
#define X_PUL_PIN       GPIO_PIN_8
#define X_DIR_PORT      GPIOC
#define X_DIR_PIN       GPIO_PIN_8
#define X_ENA_PORT      GPIOC
#define X_ENA_PIN       GPIO_PIN_6

/* Y / Z, update when hardware is connected */
#define Y_PUL_PORT      GPIOB
#define Y_PUL_PIN       GPIO_PIN_0
#define Y_DIR_PORT      GPIOB
#define Y_DIR_PIN       GPIO_PIN_1
#define Z_PUL_PORT      GPIOB
#define Z_PUL_PIN       GPIO_PIN_2
#define Z_DIR_PORT      GPIOB
#define Z_DIR_PIN       GPIO_PIN_3

/* 1 = Y/Z are logical shadow axes, 0 = real output */
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

typedef struct {
    uint16_t seq;
    uint8_t  more;
    uint32_t steps;
    uint32_t elapsed_ms;
    uint32_t feed_hz;
    int32_t  position[3];
} MotionDone;

void     Motion_Init(void);
bool     Motion_Enqueue(int32_t x, int32_t y, int32_t z, uint32_t feed_hz, uint16_t seq);
uint8_t  Motion_QueueFree(void);
bool     Motion_PopDone(MotionDone *out);
bool     Motion_IsBusy(void);
void     Motion_Stop(void);
const AxisState* Motion_GetAxis(AxisId id);
void     Motion_TickHandler(void);

#ifdef __cplusplus
}
#endif
#endif
