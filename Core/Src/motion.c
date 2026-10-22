#include "motion.h"
#include <string.h>

static AxisState        s_axes[3];
static uint32_t         s_feed_hz;
static uint32_t         s_step_accum;
static uint32_t         s_total_steps;
static uint32_t         s_steps_remaining;
static volatile bool    s_busy;

/* ============ 低层引脚操作 ============ */
static inline void axis_pulse_high(AxisId id) {
    switch (id) {
    case AXIS_X: HAL_GPIO_WritePin(X_PUL_PORT, X_PUL_PIN, GPIO_PIN_SET); break;
#if !Y_Z_SHADOW
    case AXIS_Y: HAL_GPIO_WritePin(Y_PUL_PORT, Y_PUL_PIN, GPIO_PIN_SET); break;
    case AXIS_Z: HAL_GPIO_WritePin(Z_PUL_PORT, Z_PUL_PIN, GPIO_PIN_SET); break;
#else
    case AXIS_Y: case AXIS_Z: break;    /* 影子轴:空实现 */
#endif
    }
}

static inline void axis_pulse_low(AxisId id) {
    switch (id) {
    case AXIS_X: HAL_GPIO_WritePin(X_PUL_PORT, X_PUL_PIN, GPIO_PIN_RESET); break;
#if !Y_Z_SHADOW
    case AXIS_Y: HAL_GPIO_WritePin(Y_PUL_PORT, Y_PUL_PIN, GPIO_PIN_RESET); break;
    case AXIS_Z: HAL_GPIO_WritePin(Z_PUL_PORT, Z_PUL_PIN, GPIO_PIN_RESET); break;
#else
    case AXIS_Y: case AXIS_Z: break;
#endif
    }
}

static inline void axis_set_dir(AxisId id, uint8_t dir) {
    switch (id) {
    case AXIS_X: HAL_GPIO_WritePin(X_DIR_PORT, X_DIR_PIN, dir ? GPIO_PIN_SET : GPIO_PIN_RESET); break;
#if !Y_Z_SHADOW
    case AXIS_Y: HAL_GPIO_WritePin(Y_DIR_PORT, Y_DIR_PIN, dir ? GPIO_PIN_SET : GPIO_PIN_RESET); break;
    case AXIS_Z: HAL_GPIO_WritePin(Z_DIR_PORT, Z_DIR_PIN, dir ? GPIO_PIN_SET : GPIO_PIN_RESET); break;
#else
    case AXIS_Y: case AXIS_Z: break;
#endif
    }
}

/* ============ 初始化 ============ */
void Motion_Init(void) {
    memset(s_axes, 0, sizeof(s_axes));
    s_feed_hz = 1000;
    s_step_accum = 0;
    s_busy = false;

    /* 初始电平 */
    HAL_GPIO_WritePin(X_PUL_PORT, X_PUL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(X_DIR_PORT, X_DIR_PIN, GPIO_PIN_RESET);
    /* ENA 低电平=使能(TB6600 是反逻辑) */
    HAL_GPIO_WritePin(X_ENA_PORT, X_ENA_PIN, GPIO_PIN_RESET);
}

/* ============ 绝对运动 ============ */
bool Motion_MoveTo(int32_t x, int32_t y, int32_t z, uint32_t feed_hz) {
    if (s_busy) return false;
    if (feed_hz == 0 || feed_hz > MAX_STEP_HZ) return false;

    int32_t dx = x - s_axes[AXIS_X].position;
    int32_t dy = y - s_axes[AXIS_Y].position;
    int32_t dz = z - s_axes[AXIS_Z].position;

    int32_t ax = (dx < 0) ? -dx : dx;
    int32_t ay = (dy < 0) ? -dy : dy;
    int32_t az = (dz < 0) ? -dz : dz;

    int32_t maxd = ax;
    if (ay > maxd) maxd = ay;
    if (az > maxd) maxd = az;
    if (maxd == 0) return true;

    bool dir_changed = false;

    /* X */
    {
        uint8_t nd = (dx >= 0) ? 1 : 0;
        s_axes[AXIS_X].target    = x;
        s_axes[AXIS_X].abs_delta = ax;
        s_axes[AXIS_X].err       = 0;
        s_axes[AXIS_X].active    = (ax > 0);
        if (s_axes[AXIS_X].dir != nd) {
            s_axes[AXIS_X].dir = nd;
            axis_set_dir(AXIS_X, nd);
            dir_changed = true;
        }
    }
    /* Y */
    {
        uint8_t nd = (dy >= 0) ? 1 : 0;
        s_axes[AXIS_Y].target    = y;
        s_axes[AXIS_Y].abs_delta = ay;
        s_axes[AXIS_Y].err       = 0;
        s_axes[AXIS_Y].active    = (ay > 0);
        if (s_axes[AXIS_Y].dir != nd) {
            s_axes[AXIS_Y].dir = nd;
            axis_set_dir(AXIS_Y, nd);
            dir_changed = true;
        }
    }
    /* Z */
    {
        uint8_t nd = (dz >= 0) ? 1 : 0;
        s_axes[AXIS_Z].target    = z;
        s_axes[AXIS_Z].abs_delta = az;
        s_axes[AXIS_Z].err       = 0;
        s_axes[AXIS_Z].active    = (az > 0);
        if (s_axes[AXIS_Z].dir != nd) {
            s_axes[AXIS_Z].dir = nd;
            axis_set_dir(AXIS_Z, nd);
            dir_changed = true;
        }
    }

    /* DIR 建立时间(TB6600 要求 ≥5μs,这里给足 1ms) */
    if (dir_changed) HAL_Delay(1);

    s_total_steps    = (uint32_t)maxd;
    s_steps_remaining = s_total_steps;
    s_feed_hz        = feed_hz;
    s_step_accum     = 0;
    s_busy           = true;
    return true;
}

bool Motion_IsBusy(void) { return s_busy; }

void Motion_Stop(void) {
    s_busy = false;
    s_steps_remaining = 0;
}

const AxisState* Motion_GetAxis(AxisId id) {
    if (id > AXIS_Z) return NULL;
    return &s_axes[id];
}

/* ============ TIM6 中断,20kHz ============ */
void Motion_TickHandler(void) {
    /* 1. 先拉低所有 PUL(产生下降沿,结束上一拍脉冲) */
    axis_pulse_low(AXIS_X);
    axis_pulse_low(AXIS_Y);
    axis_pulse_low(AXIS_Z);

    if (!s_busy) return;

    /* 2. 速度控制:每 TICK_RATE_HZ/feed_hz 个 tick 走一步 */
    s_step_accum += s_feed_hz;
    if (s_step_accum < TICK_RATE_HZ) return;
    s_step_accum -= TICK_RATE_HZ;

    /* 3. 三轴 Bresenham:按比例分配步数 */
    if (s_axes[AXIS_X].active) {
        s_axes[AXIS_X].err += s_axes[AXIS_X].abs_delta;
        if (s_axes[AXIS_X].err >= (int32_t)s_total_steps) {
            s_axes[AXIS_X].err -= (int32_t)s_total_steps;
            axis_pulse_high(AXIS_X);
            s_axes[AXIS_X].position += (s_axes[AXIS_X].dir ? 1 : -1);
        }
    }
    if (s_axes[AXIS_Y].active) {
        s_axes[AXIS_Y].err += s_axes[AXIS_Y].abs_delta;
        if (s_axes[AXIS_Y].err >= (int32_t)s_total_steps) {
            s_axes[AXIS_Y].err -= (int32_t)s_total_steps;
            axis_pulse_high(AXIS_Y);
            s_axes[AXIS_Y].position += (s_axes[AXIS_Y].dir ? 1 : -1);
        }
    }
    if (s_axes[AXIS_Z].active) {
        s_axes[AXIS_Z].err += s_axes[AXIS_Z].abs_delta;
        if (s_axes[AXIS_Z].err >= (int32_t)s_total_steps) {
            s_axes[AXIS_Z].err -= (int32_t)s_total_steps;
            axis_pulse_high(AXIS_Z);
            s_axes[AXIS_Z].position += (s_axes[AXIS_Z].dir ? 1 : -1);
        }
    }

    /* 4. 递减剩余步数 */
    s_steps_remaining--;
    if (s_steps_remaining == 0) s_busy = false;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* 判断是不是 TIM6 触发的中断（如果是多个定时器共用，必须有这个判断） */
    if (htim->Instance == TIM6) {
        Motion_TickHandler();    /* 调用我们在文件上面写的时基节拍处理 */
    }
}