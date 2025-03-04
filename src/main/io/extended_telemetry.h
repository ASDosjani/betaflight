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

#include <ctype.h>
#include <math.h>

#include "common/time.h"
void extendedTelemetryInit();
void extendedTelemetryUpdate(timeUs_t currentTimeUs);

typedef struct extendedTelemetryConfig_s {
    uint8_t extended_telemetry_uart_number;
    uint8_t extended_telemetry_frequency;
    uint8_t extended_telemetry_forwarded_rc_channel;
} extendedTelemetryConfig_t;

PG_DECLARE(extendedTelemetryConfig_t, extendedTelemetryConfig);
