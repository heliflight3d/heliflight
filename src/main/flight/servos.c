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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_SERVOS

#include "build/build_config.h"

#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"
#include "config/feature.h"

#include "drivers/pwm_output.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"

#include "flight/servos.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"


FAST_RAM_ZERO_INIT int16_t servo[MAX_SUPPORTED_SERVOS];

FAST_RAM_ZERO_INIT biquadFilter_t servoFilter[MAX_SUPPORTED_SERVOS];

int16_t servoOverride[MAX_SUPPORTED_SERVOS];


PG_REGISTER_WITH_RESET_FN(servoConfig_t, servoConfig, PG_SERVO_CONFIG, 0);

void pgResetFn_servoConfig(servoConfig_t *servoConfig)
{
    servoConfig->dev.servoPwmRate = 50;

    for (unsigned i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servoConfig->dev.ioTags[i] = timerioTagGetByUsage(TIM_USE_SERVO, i);
    }
}

PG_REGISTER_ARRAY_WITH_RESET_FN(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams, PG_SERVO_PARAMS, 0);

void pgResetFn_servoParams(servoParam_t *instance)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        RESET_CONFIG(servoParam_t, &instance[i],
                     .min  = DEFAULT_SERVO_MIN,
                     .max  = DEFAULT_SERVO_MAX,
                     .mid  = DEFAULT_SERVO_CENTER,
                     .rate = 1000,
                     .freq = 0,
        );
    }
}

void servoInit(void)
{
    servoDevInit(&servoConfig()->dev);
    
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servoOverride[i] = SERVO_OVERRIDE_OFF;
        if (servoParams(i)->freq > 0) {
            biquadFilterInitLPF(&servoFilter[i], servoParams(i)->freq, targetPidLooptime);
        }
    }
}

void servoUpdate(void)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        // Convert mixer output -1..1 to PWM 1000..2000
        float pwm = servoParams(i)->mid + (mixerGetServoOutput(i) * servoParams(i)->rate / 2);

        if (servoParams(i)->freq > 0)
            pwm = biquadFilterApply(&servoFilter[i], pwm);

        if (!ARMING_FLAG(ARMED) && servoOverride[i] != SERVO_OVERRIDE_OFF)
            servo[i] = servoOverride[i];
        else
            servo[i] = constrain(pwm, servoParams(i)->min, servoParams(i)->max);

        pwmWriteServo(i, servo[i]);
    }
}

#endif
