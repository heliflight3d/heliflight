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

#pragma once

#include "pg/pg.h"

#include "drivers/io_types.h"
#include "drivers/pwm_output.h"

#include "flight/servos.h"
#include "flight/motors.h"
#include "flight/governor.h"
#include "flight/rpm_filter.h"


enum {
    MIXER_IN_NONE = 0,
    MIXER_IN_GOVERNOR_MAIN,
    MIXER_IN_GOVERNOR_TAIL,
    MIXER_IN_STABILIZED_ROLL,
    MIXER_IN_STABILIZED_PITCH,
    MIXER_IN_STABILIZED_YAW,
    MIXER_IN_STABILIZED_THROTTLE,
    MIXER_IN_STABILIZED_COLLECTIVE,
    MIXER_IN_RCCMD_ROLL,
    MIXER_IN_RCCMD_PITCH,
    MIXER_IN_RCCMD_YAW,
    MIXER_IN_RCCMD_THROTTLE,
    MIXER_IN_RCCMD_COLLECTIVE,
    MIXER_IN_RCDATA_0,
    MIXER_IN_RCDATA_1,
    MIXER_IN_RCDATA_2,
    MIXER_IN_RCDATA_3,
    MIXER_IN_RCDATA_4,
    MIXER_IN_RCDATA_5,
    MIXER_IN_RCDATA_6,
    MIXER_IN_RCDATA_7,
    MIXER_IN_RCDATA_8,
    MIXER_IN_RCDATA_9,
    MIXER_IN_RCDATA_10,
    MIXER_IN_RCDATA_11,
    MIXER_IN_RCDATA_12,
    MIXER_IN_RCDATA_13,
    MIXER_IN_RCDATA_14,
    MIXER_IN_RCDATA_15,
    MIXER_IN_RCDATA_16,
    MIXER_IN_RCDATA_17,
    MIXER_IN_COUNT
};

enum {
    MIXER_OP_NUL = 0,
    MIXER_OP_SET,
    MIXER_OP_ADD,
    MIXER_OP_MUL,
    MIXER_OP_COUNT
};


#define MIXER_RULE_COUNT      32

#define MIXER_INPUT_COUNT     MIXER_IN_COUNT
#define MIXER_OUTPUT_COUNT    (MAX_SUPPORTED_SERVOS + MAX_SUPPORTED_MOTORS)
#define MIXER_OUTPUT_MOTORS   MAX_SUPPORTED_SERVOS

#define MIXER_OVERRIDE_MIN   -1250
#define MIXER_OVERRIDE_MAX    1250
#define MIXER_OVERRIDE_OFF    (MIXER_OVERRIDE_MAX + 1)

#define MIXER_RC_SCALING      (1.0f / 500)
#define MIXER_PID_SCALING     (1.0f / 500)
#define MIXER_THR_SCALING     (1.0f / (PWM_RANGE_MAX - PWM_RANGE_MIN))
#define MIXER_THR_OFFSET      PWM_RANGE_MIN

#define MIXER_CUSTOM          23


typedef struct mixer_s
{
    uint8_t oper;              // rule operation
    uint8_t input;             // input channel
    uint8_t output;            // output channel
    int16_t offset;            // output offset -2000..2000%%
    int16_t rate;              // range [-2000;+2000] ; can be used to adjust rate 0-2000%% and direction
    int16_t min;               // lower bound of rule range -1000..1000%%
    int16_t max;               // lower bound of rule range -1000..1000%%
} mixer_t;

typedef struct mixscale_s
{
    int16_t scale[MIXER_INPUT_COUNT];         // lower bound of rule range -1000..1000%%
} mixscale_t;

PG_DECLARE_ARRAY(mixer_t, MIXER_RULE_COUNT, mixerRules);
PG_DECLARE(mixscale_t, mixerScales);

extern FAST_RAM_ZERO_INIT uint8_t mixerActiveServos;
extern FAST_RAM_ZERO_INIT uint8_t mixerActiveMotors;

extern FAST_RAM_ZERO_INIT int16_t mixerOverride[MIXER_INPUT_COUNT];
extern FAST_RAM_ZERO_INIT int16_t mixScales[MIXER_INPUT_COUNT];

extern FAST_RAM_ZERO_INIT bool    mixerInputSaturated[MIXER_INPUT_COUNT];

void mixerInit(void);
void mixerInitProfile(void);

void mixerUpdate(void);

void mixerSetOutputSaturated(uint8_t out);

float mixerGetInput(uint8_t i);
float mixerGetServoOutput(uint8_t i);
float mixerGetMotorOutput(uint8_t i);
float getCyclicDeflection(void);

static inline uint8_t mixerGetActiveServos(void) { return mixerActiveServos; }
static inline uint8_t mixerGetActiveMotors(void) { return mixerActiveMotors; }

static inline bool  mixerSaturated(uint8_t i) { return mixerInputSaturated[i]; }

static inline void  mixerSetServoOutputSaturated(uint8_t i) { mixerSetOutputSaturated(i); }
static inline void  mixerSetMotorOutputSaturated(uint8_t i) { mixerSetOutputSaturated(i + MIXER_OUTPUT_MOTORS); }

static inline float mixerGetThrottle() { return mixerGetInput(MIXER_IN_STABILIZED_THROTTLE); }

