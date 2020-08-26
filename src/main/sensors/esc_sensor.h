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

#include "common/time.h"

#define ESCHW4_V_REF 3.3
#define ESCHW4_DIFFAMP_GAIN 13.6
#define ESCHW4_DIFFAMP_SHUNT 0.25 / 1000.0
#define ESCHW4_ADC_RESOLUTION 4096.0
#define ESCHW4_NTC_BETA 3950.0
#define ESCHW4_NTC_R1 10000.0
#define ESCHW4_NTC_R_REF 47000.0

#define ESC_DATA_INVALID 255
#define ESC_BATTERY_AGE_MAX 10
#define ESC_SENSOR_COMBINED 255

typedef enum {
    ESC_SENSOR_PROTOCOL_KISS = 0,
    ESC_SENSOR_PROTOCOL_HOBBYWINGV4,
} escSensorProtocols_e;

typedef struct escSensorConfig_s {
    uint8_t halfDuplex;             // Set to false to listen on the TX pin for telemetry data
    uint16_t offset;                // offset consumed by the flight controller / VTX / cam / ... in milliamperes
    uint8_t escSensorProtocol;      // ESC telemetry protocol selection
    uint16_t esc_sensor_hobbywing_curroffset;    // HobbyWing V4 raw current offset (depends on specific HWV4 ESC)
    uint8_t esc_sensor_hobbywing_voltagedivisor; // HobbyWing V4 voltage divisor (11 for LV ESCs, 21 for HV ESCs)
    uint8_t esc_sensor_hobbywing_currscale;      // HobbyWing V4 current scaling (100 = 1.0 times the default scaling)
} escSensorConfig_t;

PG_DECLARE(escSensorConfig_t, escSensorConfig);

typedef struct {
    uint8_t dataAge;
    int8_t temperature;  // C degrees
    int16_t voltage;     // 0.01V
    int32_t current;     // 0.01A
    int32_t consumption; // mAh
    int16_t rpm;         // 0.01erpm
} escSensorData_t;

bool escSensorInit(void);
void escSensorProcess(timeUs_t currentTime);

escSensorData_t *getEscSensorData(uint8_t motorNumber);

void startEscDataRead(uint8_t *frameBuffer, uint8_t frameLength);
uint8_t getNumberEscBytesRead(void);

uint8_t calculateCrc8(const uint8_t *Buf, const uint8_t BufLen);

bool isEscSensorActive(void);
uint16_t getEscSensorRPM(uint8_t motorNumber);

