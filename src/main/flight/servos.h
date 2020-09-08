/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "pg/pg.h"

#define DEFAULT_SERVO_MIN     1000
#define DEFAULT_SERVO_MAX     2000
#define DEFAULT_SERVO_CENTER  1500

#define SERVO_OVERRIDE_OFF    0
#define SERVO_OVERRIDE_MIN    PWM_SERVO_PULSE_MIN
#define SERVO_OVERRIDE_MAX    PWM_SERVO_PULSE_MAX

typedef struct servoParam_s {
    int16_t min;    // servo min
    int16_t max;    // servo max
    int16_t mid;    // servo midpoint
    int16_t rate;   // range [-1000;+1000] ; can be used to adjust a rate 0-2000%% and a direction
    int16_t freq;   // low pass filter freq
} servoParam_t;

PG_DECLARE_ARRAY(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams);

typedef struct servoConfig_s {
    servoDevConfig_t dev;
} servoConfig_t;

PG_DECLARE(servoConfig_t, servoConfig);

extern int16_t servo[MAX_SUPPORTED_SERVOS];
extern int16_t servoOverride[MAX_SUPPORTED_SERVOS];

void servoInit(void);
void servoUpdate(void);

