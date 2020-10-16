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

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/governor.h"

#include "rx/rx.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/gyro.h"


PG_REGISTER_ARRAY(mixer_t, MIXER_RULE_COUNT, mixerRules, PG_HELI_MIXER, 0);

FAST_RAM_ZERO_INIT uint8_t mixerActiveServos;
FAST_RAM_ZERO_INIT uint8_t mixerActiveMotors;

FAST_RAM_ZERO_INIT uint8_t mixerRuleCount;

FAST_RAM_ZERO_INIT mixer_t mixer[MIXER_RULE_COUNT];
FAST_RAM_ZERO_INIT int16_t mixScales[MIXER_INPUT_COUNT];

FAST_RAM_ZERO_INIT float mixerInput[MIXER_INPUT_COUNT];
FAST_RAM_ZERO_INIT float mixerOutput[MIXER_OUTPUT_COUNT];

FAST_RAM_ZERO_INIT int16_t mixerOverride[MIXER_INPUT_COUNT];

FAST_RAM_ZERO_INIT float cyclicTotal;
FAST_RAM_ZERO_INIT float cyclicLimit;

PG_REGISTER_WITH_RESET_FN(mixscale_t, mixerScales, PG_HELI_MIXER_SCALES, 0);

void pgResetFn_mixerScales(mixscale_t *mixerScales)
{
    for (unsigned index = 0; index < MIXER_INPUT_COUNT; index++) {
        mixerScales->scale[index] = 1000;
    }
}

void mixerInit(void)
{
    mixerActiveServos = 0;
    mixerActiveMotors = 0;
    mixerRuleCount = 0;

    cyclicLimit = 1.0;

    for (int i = 0; i < MIXER_RULE_COUNT; i++) {
        const mixer_t *rule = mixerRules(i);

        if (rule->oper == MIXER_OP_NUL)
            break;

        mixer[i].oper    = constrain(rule->oper, 1, MIXER_OP_COUNT - 1);
        mixer[i].input   = constrain(rule->input, 0, MIXER_INPUT_COUNT -1);
        mixer[i].output  = constrain(rule->output, 0, MIXER_OUTPUT_COUNT - 1);
        mixer[i].offset  = constrain(rule->offset, -2000, 2000);
        mixer[i].rate    = constrain(rule->rate, -2000, 2000);
        mixer[i].min     = constrain(rule->min, -2000, 2000);
        mixer[i].max     = constrain(rule->max, mixer[i].min, 2000);

        if (mixer[i].output < MIXER_OUTPUT_MOTORS)
            mixerActiveServos = MAX(mixerActiveServos, mixer[i].output + 1);
        else
            mixerActiveMotors = MAX(mixerActiveMotors, mixer[i].output - MIXER_OUTPUT_MOTORS + 1);

        mixerRuleCount++;
    }

    for (int i = 1; i < MIXER_INPUT_COUNT; i++) {
        mixerOverride[i] = MIXER_OVERRIDE_OFF;
        mixScales[i] = constrain(mixerScales()->scale[i], -2000, 2000);
    }

    mixerInitProfile();
}

void mixerInitProfile(void)
{
    cyclicLimit = currentPidProfile->pidSumLimit * MIXER_PID_SCALING;
}

void mixerUpdate(void)
{
    mixerInput[MIXER_IN_RCCMD_ROLL]       = rcCommand[ROLL]       * MIXER_RC_SCALING;
    mixerInput[MIXER_IN_RCCMD_PITCH]      = rcCommand[PITCH]      * MIXER_RC_SCALING;
    mixerInput[MIXER_IN_RCCMD_YAW]        = rcCommand[YAW]        * MIXER_RC_SCALING;
    mixerInput[MIXER_IN_RCCMD_COLLECTIVE] = rcCommand[COLLECTIVE] * MIXER_RC_SCALING;

    mixerInput[MIXER_IN_RCCMD_THROTTLE]   = (rcCommand[THROTTLE] - MIXER_THR_OFFSET) * MIXER_THR_SCALING;

    if (FLIGHT_MODE(PASSTHRU_MODE)) {
        mixerInput[MIXER_IN_STABILIZED_ROLL]  = mixerInput[MIXER_IN_RCCMD_ROLL];
        mixerInput[MIXER_IN_STABILIZED_PITCH] = mixerInput[MIXER_IN_RCCMD_PITCH];
        mixerInput[MIXER_IN_STABILIZED_YAW]   = mixerInput[MIXER_IN_RCCMD_YAW];
    } else {
        mixerInput[MIXER_IN_STABILIZED_ROLL]  = pidData[FD_ROLL].SumLim  * MIXER_PID_SCALING;
        mixerInput[MIXER_IN_STABILIZED_PITCH] = pidData[FD_PITCH].SumLim * MIXER_PID_SCALING;
        mixerInput[MIXER_IN_STABILIZED_YAW]   = pidData[FD_YAW].SumLim   * MIXER_PID_SCALING;
    }

    mixerInput[MIXER_IN_STABILIZED_THROTTLE]   = mixerInput[MIXER_IN_RCCMD_THROTTLE];
    mixerInput[MIXER_IN_STABILIZED_COLLECTIVE] = mixerInput[MIXER_IN_RCCMD_COLLECTIVE];

    for (int i = 0; i < 16; i++)
        mixerInput[MIXER_IN_RCDATA_0 + i] = (rcData[i] - rxConfig()->midrc) * MIXER_RC_SCALING;

    governorUpdate();

    mixerInput[MIXER_IN_GOVERNOR_MAIN] = getGovernorOutput(0);
    mixerInput[MIXER_IN_GOVERNOR_TAIL] = getGovernorOutput(1);

    // Current cyclic deflection
    cyclicTotal = sqrtf(mixerInput[MIXER_IN_STABILIZED_ROLL] * mixerInput[MIXER_IN_STABILIZED_ROLL] +
                        mixerInput[MIXER_IN_STABILIZED_PITCH] * mixerInput[MIXER_IN_STABILIZED_PITCH]);

    // Cyclic ring limit reached
    if (cyclicTotal > cyclicLimit) {
        mixerInput[MIXER_IN_STABILIZED_ROLL]  *= cyclicLimit / cyclicTotal;
        mixerInput[MIXER_IN_STABILIZED_PITCH] *= cyclicLimit / cyclicTotal;
    }

    // Input override
    if (!ARMING_FLAG(ARMED)) {
        for (int i = 1; i < MIXER_INPUT_COUNT; i++) {
            if (mixerOverride[i] >= MIXER_OVERRIDE_MIN && mixerOverride[i] <= MIXER_OVERRIDE_MAX)
                mixerInput[i] = mixerOverride[i] / 1000.0f;
        }
    }

    // Reset outputs
    for (int i = 0; i < MIXER_OUTPUT_COUNT; i++) {
        mixerOutput[i] = 0;
    }

    // Calculate mixer outputs
    for (int i = 0; i < mixerRuleCount; i++) {
        int src = mixer[i].input;
        int dst = mixer[i].output;
        float val = constrainf(mixer[i].offset + mixerInput[src] * mixer[i].rate * mixScales[src]/1000.0f, mixer[i].min, mixer[i].max) / 1000.0f;

        switch (mixer[i].oper)
        {
            case MIXER_OP_SET:
                mixerOutput[dst] = val;
                break;
            case MIXER_OP_ADD:
                mixerOutput[dst] += val;
                break;
            case MIXER_OP_MUL:
                mixerOutput[dst] *= val;
                break;
        }
    }
}


float mixerGetInput(uint8_t i)
{
    return mixerInput[i];
}

float mixerGetServoOutput(uint8_t i)
{
    return mixerOutput[i];
}

float mixerGetMotorOutput(uint8_t i)
{
    return mixerOutput[i + MIXER_OUTPUT_MOTORS];
}

float getCyclicDeflection(void)
{
    return MIN(cyclicTotal / cyclicLimit, 1.0f);
}

