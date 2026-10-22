#ifndef __MACHINE_CONFIG_H
#define __MACHINE_CONFIG_H

#include <stdint.h>

#define MACHINE_MOTOR_FULL_STEPS_PER_REV   200u
#define MACHINE_MICROSTEPS                 4u
#define MACHINE_STEPS_PER_REV              (MACHINE_MOTOR_FULL_STEPS_PER_REV * MACHINE_MICROSTEPS)

#define MACHINE_MAX_FEED_HZ                10000u
#define MACHINE_ACCEL_STEPS_S2             20000u

#define MACHINE_STEPS_PER_MM_X             0.0f
#define MACHINE_STEPS_PER_MM_Y             0.0f
#define MACHINE_STEPS_PER_MM_Z             0.0f

#endif
