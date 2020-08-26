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


FAST_RAM_ZERO_INIT uint8_t        motorRpmSource[MAX_SUPPORTED_MOTORS];
FAST_RAM_ZERO_INIT pt1Filter_t    motorRpmFilter[MAX_SUPPORTED_MOTORS];


bool isRpmSourceActive(void)
{
    for (int i = 0; i < getMotorCount(); i++)
        if (motorRpmSource[i] == RPM_SRC_NONE)
            return false;

    return true;
}

int calcMotorRpm(uint8_t motorNumber, int erpm)
{
    int div = motorConfig()->motorPoleCount[motorNumber] / 2;
    return (erpm * 100) / MAX(div,1);
}

int getMotorRPM(uint8_t motor)
{
    if (motor < getMotorCount())
        return motorRpmFilter[motor].state;
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
        pt1FilterInit(&motorRpmFilter[i], pt1FilterGain(freq, pidGetDT()));
    }
}

void rpmSourceUpdate(void)
{
    for (int i = 0; i < getMotorCount(); i++) {
        pt1FilterApply(&motorRpmFilter[i], calcMotorRpm(i,getMotorERPM(i)));
        DEBUG_SET(DEBUG_RPM_SOURCE, i, motorRpmFilter[i].state);
    }
}

float getHeadSpeed(void)
{
    return 0; // TODO
}

