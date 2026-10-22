#include "motion.h"
#include "machine_config.h"
#include <string.h>

typedef struct {
    int32_t target[3];
    uint32_t feed_hz;
    uint16_t seq;
} MotionSegment;

static AxisState        s_axes[3];

static MotionSegment    s_queue[MOTION_QUEUE_SIZE];
static volatile uint8_t s_q_head;
static volatile uint8_t s_q_tail;

static MotionDone       s_done[MOTION_DONE_QUEUE_SIZE];
static volatile uint8_t s_d_head;
static volatile uint8_t s_d_tail;

static volatile bool    s_busy;
static uint32_t         s_feed_hz;
static uint32_t         s_step_accum;
static uint32_t         s_total_steps;
static uint32_t         s_steps_remaining;
static uint32_t         s_active_start_tick;
static uint32_t         s_active_steps;
static uint32_t         s_active_feed;
static uint16_t         s_active_seq;

static inline void axis_pulse_high(AxisId id) {
    switch (id) {
    case AXIS_X: HAL_GPIO_WritePin(X_PUL_PORT, X_PUL_PIN, GPIO_PIN_SET); break;
#if !Y_Z_SHADOW
    case AXIS_Y: HAL_GPIO_WritePin(Y_PUL_PORT, Y_PUL_PIN, GPIO_PIN_SET); break;
    case AXIS_Z: HAL_GPIO_WritePin(Z_PUL_PORT, Z_PUL_PIN, GPIO_PIN_SET); break;
#else
    case AXIS_Y: case AXIS_Z: break;
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

static uint32_t motion_critical_enter(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void motion_critical_exit(uint32_t primask) {
    if (primask == 0u) __enable_irq();
}

static uint32_t motion_abs_i32(int32_t value) {
    return (value < 0) ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
}

static uint32_t motion_max_feed(void) {
    uint32_t max_feed = MACHINE_MAX_FEED_HZ;
    if (max_feed > MAX_STEP_HZ) max_feed = MAX_STEP_HZ;
    return max_feed;
}

void Motion_Init(void) {
    memset(s_axes, 0, sizeof(s_axes));
    s_q_head = 0;
    s_q_tail = 0;
    s_d_head = 0;
    s_d_tail = 0;
    s_busy = false;
    s_feed_hz = 1000u;
    s_step_accum = 0;
    s_total_steps = 0;
    s_steps_remaining = 0;
    s_active_start_tick = 0;
    s_active_steps = 0;
    s_active_feed = 0;
    s_active_seq = 0;

    HAL_GPIO_WritePin(X_PUL_PORT, X_PUL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(X_DIR_PORT, X_DIR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(X_ENA_PORT, X_ENA_PIN, GPIO_PIN_RESET);
}

bool Motion_Enqueue(int32_t x, int32_t y, int32_t z, uint32_t feed_hz, uint16_t seq) {
    if (feed_hz == 0u || feed_hz > motion_max_feed()) return false;

    uint32_t primask = motion_critical_enter();
    uint8_t next = (uint8_t)((s_q_head + 1u) % MOTION_QUEUE_SIZE);
    if (next == s_q_tail) {
        motion_critical_exit(primask);
        return false;
    }

    MotionSegment *seg = &s_queue[s_q_head];
    seg->target[AXIS_X] = x;
    seg->target[AXIS_Y] = y;
    seg->target[AXIS_Z] = z;
    seg->feed_hz = feed_hz;
    seg->seq = seq;
    s_q_head = next;

    motion_critical_exit(primask);
    return true;
}

uint8_t Motion_QueueFree(void) {
    uint32_t primask = motion_critical_enter();
    uint8_t free_slots = (uint8_t)((s_q_tail + MOTION_QUEUE_SIZE - s_q_head - 1u) % MOTION_QUEUE_SIZE);
    motion_critical_exit(primask);
    return free_slots;
}

bool Motion_IsBusy(void) {
    uint32_t primask = motion_critical_enter();
    bool busy = s_busy || (s_q_head != s_q_tail);
    motion_critical_exit(primask);
    return busy;
}

bool Motion_PopDone(MotionDone *out) {
    if (!out) return false;

    uint32_t primask = motion_critical_enter();
    if (s_d_head == s_d_tail) {
        motion_critical_exit(primask);
        return false;
    }

    *out = s_done[s_d_tail];
    s_d_tail = (uint8_t)((s_d_tail + 1u) % MOTION_DONE_QUEUE_SIZE);
    motion_critical_exit(primask);
    return true;
}

void Motion_Stop(void) {
    uint32_t primask = motion_critical_enter();
    s_busy = false;
    s_total_steps = 0;
    s_steps_remaining = 0;
    s_step_accum = 0;
    s_q_head = 0;
    s_q_tail = 0;
    s_d_head = 0;
    s_d_tail = 0;
    motion_critical_exit(primask);
}

static void motion_push_done(uint16_t seq, uint32_t feed_hz, uint32_t steps,
                             uint32_t elapsed_ms, uint8_t more) {
    uint8_t next = (uint8_t)((s_d_head + 1u) % MOTION_DONE_QUEUE_SIZE);
    if (next == s_d_tail) return;

    MotionDone *done = &s_done[s_d_head];
    done->seq = seq;
    done->more = more;
    done->steps = steps;
    done->elapsed_ms = elapsed_ms;
    done->feed_hz = feed_hz;
    done->position[AXIS_X] = s_axes[AXIS_X].position;
    done->position[AXIS_Y] = s_axes[AXIS_Y].position;
    done->position[AXIS_Z] = s_axes[AXIS_Z].position;
    s_d_head = next;
}

static bool motion_load_next(void) {
    for (uint8_t guard = 0; guard < 8u; guard++) {
        if (s_q_head == s_q_tail) {
            s_busy = false;
            return false;
        }

        MotionSegment seg = s_queue[s_q_tail];
        s_q_tail = (uint8_t)((s_q_tail + 1u) % MOTION_QUEUE_SIZE);

        int32_t dx = seg.target[AXIS_X] - s_axes[AXIS_X].position;
        int32_t dy = seg.target[AXIS_Y] - s_axes[AXIS_Y].position;
        int32_t dz = seg.target[AXIS_Z] - s_axes[AXIS_Z].position;

        uint32_t ax = motion_abs_i32(dx);
        uint32_t ay = motion_abs_i32(dy);
        uint32_t az = motion_abs_i32(dz);

        uint32_t maxd = ax;
        if (ay > maxd) maxd = ay;
        if (az > maxd) maxd = az;

        uint8_t more = (s_q_head != s_q_tail) ? 1u : 0u;

        if (maxd == 0u) {
            motion_push_done(seg.seq, seg.feed_hz, 0u, 0u, more);
            continue;
        }

        s_axes[AXIS_X].target = seg.target[AXIS_X];
        s_axes[AXIS_X].abs_delta = (int32_t)ax;
        s_axes[AXIS_X].err = 0;
        s_axes[AXIS_X].active = (ax > 0u);
        if (dx != 0) {
            uint8_t nd = (dx > 0) ? 1u : 0u;
            if (s_axes[AXIS_X].dir != nd) {
                s_axes[AXIS_X].dir = nd;
                axis_set_dir(AXIS_X, nd);
            }
        }

        s_axes[AXIS_Y].target = seg.target[AXIS_Y];
        s_axes[AXIS_Y].abs_delta = (int32_t)ay;
        s_axes[AXIS_Y].err = 0;
        s_axes[AXIS_Y].active = (ay > 0u);
        if (dy != 0) {
            uint8_t nd = (dy > 0) ? 1u : 0u;
            if (s_axes[AXIS_Y].dir != nd) {
                s_axes[AXIS_Y].dir = nd;
                axis_set_dir(AXIS_Y, nd);
            }
        }

        s_axes[AXIS_Z].target = seg.target[AXIS_Z];
        s_axes[AXIS_Z].abs_delta = (int32_t)az;
        s_axes[AXIS_Z].err = 0;
        s_axes[AXIS_Z].active = (az > 0u);
        if (dz != 0) {
            uint8_t nd = (dz > 0) ? 1u : 0u;
            if (s_axes[AXIS_Z].dir != nd) {
                s_axes[AXIS_Z].dir = nd;
                axis_set_dir(AXIS_Z, nd);
            }
        }

        s_total_steps = maxd;
        s_steps_remaining = maxd;
        s_feed_hz = seg.feed_hz;
        s_step_accum = 0;
        s_active_start_tick = HAL_GetTick();
        s_active_steps = maxd;
        s_active_feed = seg.feed_hz;
        s_active_seq = seg.seq;
        s_busy = true;
        return true;
    }

    s_busy = false;
    return false;
}

const AxisState* Motion_GetAxis(AxisId id) {
    if (id > AXIS_Z) return NULL;
    return &s_axes[id];
}

void Motion_TickHandler(void) {
    axis_pulse_low(AXIS_X);
    axis_pulse_low(AXIS_Y);
    axis_pulse_low(AXIS_Z);

    if (!s_busy) {
        (void)motion_load_next();
        return;
    }

    s_step_accum += s_feed_hz;
    if (s_step_accum < TICK_RATE_HZ) return;
    s_step_accum -= TICK_RATE_HZ;

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

    s_steps_remaining--;
    if (s_steps_remaining == 0u) {
        uint32_t elapsed_ms = HAL_GetTick() - s_active_start_tick;
        uint8_t more = (s_q_head != s_q_tail) ? 1u : 0u;
        motion_push_done(s_active_seq, s_active_feed, s_active_steps, elapsed_ms, more);
        s_busy = false;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        Motion_TickHandler();
    }
}
