/*
 * This file is part of Heliflight 3D.
 *
 * Heliflight 3D is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Heliflight 3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/filter.h"

#include "config/feature.h"
#include "config/config.h"

#include "pg/motor.h"

#include "drivers/pwm_output.h"
#include "drivers/dshot_command.h"
#include "drivers/dshot.h"
#include "drivers/motor.h"
#include "drivers/freq.h"
#include "drivers/time.h"
#include "drivers/io.h"

#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"

#include "io/motors.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/core.h"
#include "fc/rc.h"

#include "flight/motors.h"
#include "flight/servos.h"
#include "flight/mixer.h"
#include "flight/pid.h"


FAST_RAM_ZERO_INIT uint8_t        motorCount;

FAST_RAM_ZERO_INIT float          motor[MAX_SUPPORTED_MOTORS];

FAST_RAM_ZERO_INIT float          motorOutputLow;
FAST_RAM_ZERO_INIT float          motorOutputHigh;
FAST_RAM_ZERO_INIT float          motorOutputStop;
FAST_RAM_ZERO_INIT float          motorOutputRange;
FAST_RAM_ZERO_INIT float          motorOutputDisarmed[MAX_SUPPORTED_MOTORS];

FAST_RAM_ZERO_INIT float          motorRpm[MAX_SUPPORTED_MOTORS];
FAST_RAM_ZERO_INIT uint8_t        motorRpmSource[MAX_SUPPORTED_MOTORS];
FAST_RAM_ZERO_INIT biquadFilter_t motorRpmFilter[MAX_SUPPORTED_MOTORS];


uint8_t getMotorCount(void)
{
    return motorCount;
}

bool isRpmSourceActive(void)
{
    for (int i = 0; i < motorCount; i++)
        if (motorRpmSource[i] == RPM_SRC_NONE)
            return false;

    return true;
}

bool areMotorsRunning(void)
{
    if (ARMING_FLAG(ARMED))
        return true;

    for (int i = 0; i < motorCount; i++)
        if (motorOutputDisarmed[i] != motorOutputStop)
            return true;

    return false;
}

int calcMotorRpm(uint8_t motorNumber, int erpm)
{
    int div = motorConfig()->motorPoleCount[motorNumber] / 2;
    return (erpm * 100) / MAX(div,1);
}

int getMotorRPM(uint8_t motor)
{
    if (motor < motorCount)
        return motorRpm[motor];
    else
        return 0;
}

int getMotorERPM(uint8_t motor)
{
    int erpm;
#ifdef USE_FREQ_SENSOR
    if (motorRpmSource[motor] == RPM_SRC_FREQ_SENSOR) {
        erpm = getFreqSensorRPM(motor);
    }
    else
#endif
#ifdef USE_DSHOT_TELEMETRY
    if (motorRpmSource[motor] == RPM_SRC_DSHOT_TELEM) {
        erpm = getDshotTelemetry(motor);
    }
    else
#endif
#ifdef USE_ESC_SENSOR
    if (motorRpmSource[motor] == RPM_SRC_ESC_SENSOR) {
        erpm = getEscSensorRPM(motor);
    }
    else
#endif
    {
        erpm = 0;
    }
    return erpm;
}

void rpmSourceInit(void)
{
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
#ifdef USE_FREQ_SENSOR
        if (featureIsEnabled(FEATURE_FREQ_SENSOR) && isFreqSensorPortInitialized(i)) {
            motorRpmSource[i] = RPM_SRC_FREQ_SENSOR;
        }
        else
#endif
#ifdef USE_DSHOT_TELEMETRY
        if (isMotorProtocolDshot() && motorConfig()->dev.useDshotTelemetry) {
            motorRpmSource[i] = RPM_SRC_DSHOT_TELEM;
        }
        else
#endif
#ifdef USE_ESC_SENSOR
        if (featureIsEnabled(FEATURE_ESC_SENSOR) && isEscSensorActive()) {
            motorRpmSource[i] = RPM_SRC_ESC_SENSOR;
        }
        else
#endif
        {
            motorRpmSource[i] = RPM_SRC_NONE;
        }

        int freq = constrain(motorConfig()->motorRpmLpf[i], 1, 1000);
        biquadFilterInitLPF(&motorRpmFilter[i], freq, gyro.targetLooptime);
    }
}

void initEscEndpoints(void)
{
    if (isMotorProtocolDshot()) {
        motorOutputLow  = DSHOT_MIN_THROTTLE;
        motorOutputHigh = DSHOT_MAX_THROTTLE;
        motorOutputStop = DSHOT_CMD_MOTOR_STOP;
    }
    else {
        motorOutputLow  = motorConfig()->minthrottle;
        motorOutputHigh = motorConfig()->maxthrottle;
        motorOutputStop = motorConfig()->mincommand;
    }

    motorOutputRange = motorOutputHigh - motorOutputLow;
}

void motorInit(void)
{
    checkMotorProtocol(&motorConfig()->dev);

    initEscEndpoints();
    motorResetDisarmed();

    motorCount = MIN(mixerGetActiveMotors(), MAX_SUPPORTED_MOTORS);

    motorDevInit(&motorConfig()->dev, motorOutputStop, motorCount);
}

void motorUpdate(void)
{
    if (ARMING_FLAG(ARMED)) {
        for (int i = 0; i < motorCount; i++) {
            // Convert mixer motor output 0..1 to motor output
            float out = mixerGetMotorOutput(i) * motorOutputRange + motorOutputLow;
            motor[i] = constrainf(out, motorOutputLow, motorOutputHigh);
        }
        motorWriteAll(motor);
    }
    else {
        motorWriteAll(motorOutputDisarmed);
    }

    for (int i = 0; i < motorCount; i++) {
        motorRpm[i] = biquadFilterApply(&motorRpmFilter[i], calcMotorRpm(i,getMotorERPM(i)));
        DEBUG_SET(DEBUG_RPM_SOURCE, i, motorRpm[i]);
    }
}

void motorStop(void)
{
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        motor[i] = motorOutputStop;

    motorWriteAll(motor);
    delay(50);
}

void motorSetDisarmed(uint8_t motor, uint32_t value)
{
    motorOutputDisarmed[motor] = value;
}

void motorResetDisarmed(void)
{
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        motorOutputDisarmed[i] = motorOutputStop;
}

