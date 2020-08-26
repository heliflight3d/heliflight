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


typedef enum {
    RPM_SRC_NONE = 0,
    RPM_SRC_DSHOT_TELEM,
    RPM_SRC_FREQ_SENSOR,
    RPM_SRC_ESC_SENSOR,
} rpmSource_e;


bool isRpmSourceActive(void);

int getMotorRPM(uint8_t motor);
int calcMotorRpm(uint8_t motor, int erpm);

void rpmSourceInit(void);
void rpmSourceUpdate(void);

float getHeadSpeed(void);
