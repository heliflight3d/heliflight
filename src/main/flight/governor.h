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

#include "platform.h"

#include "pg/pg.h"

enum {
    GOV_MAIN = 0,
    GOV_TAIL = 1,
};

typedef enum {
    GM_STANDARD,
    GM_MODEL1,
    GM_MODEL2,
    GM_MODEL3,
} govMode_e;

typedef enum {
    GS_THROTTLE_OFF,
    GS_PASSTHROUGH_SPOOLING_UP,
    GS_PASSTHROUGH_ACTIVE,
    GS_PASSTHROUGH_LOST_THROTTLE,
    GS_PASSTHROUGH_LOST_HEADSPEED,
    GS_GOVERNOR_SPOOLING_UP,
    GS_GOVERNOR_ACTIVE,
    GS_GOVERNOR_LOST_THROTTLE,
    GS_GOVERNOR_LOST_HEADSPEED,
    GS_AUTOROTATION_CLASSIC,
    GS_AUTOROTATION_ASSIST,
    GS_AUTOROTATION_BAILOUT,
} govState_e;

typedef struct governorConfig_s {
    uint8_t  gov_mode;
    uint16_t gov_max_headspeed;
    uint16_t gov_spoolup_time;
    uint16_t gov_gear_ratio;
    uint16_t gov_p_gain;
    uint16_t gov_i_gain;
    uint16_t gov_cyclic_ff_gain;
    uint16_t gov_collective_ff_gain;
    uint16_t gov_collective_ff_impulse_gain;
    uint16_t gov_tailmotor_assist_gain;
} governorConfig_t;

PG_DECLARE(governorConfig_t, governorConfig);


extern FAST_RAM_ZERO_INIT uint8_t govMode;
extern FAST_RAM_ZERO_INIT uint8_t govState;

extern FAST_RAM_ZERO_INIT float govHeadSpeed;
extern FAST_RAM_ZERO_INIT float govGearRatio;

extern FAST_RAM_ZERO_INIT float govOutput[MAX_SUPPORTED_MOTORS];


void governorInit();
void governorUpdate();

void governorInitStandard();
void governorUpdateStandard();

bool isHeliSpooledUp(void);

float getHeadSpeed(void);
float getGovernorOutput(uint8_t motor);

uint8_t getGovernorMode();
uint8_t getGovernorState();

