#include "extended_telemetry.h"
#include "../common/streambuf.h"
#include "../sensors/acceleration.h"
#include "../sensors/gyro_init.h"
#include "../sensors/compass.h"
#include "../fc/runtime_config.h"
#include "../rx/rx.h"
#include "../io/gps.h"
#include "msp/msp_protocol_v2_betaflight.h"
#include "msp/msp.h"
#include "msp/msp_serial.h"
#include "pg/pg_ids.h"

void extendedTelemetryUpdate(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);
    //54 bytes payload
    uint8_t telemetryPayloadArr[54] = {0};
    sbuf_t telemetryPayload;
    telemetryPayload.ptr = &telemetryPayloadArr[0];
    telemetryPayload.end = telemetryPayload.ptr + sizeof(telemetryPayloadArr);
    sbuf_t *dst = &telemetryPayload;
    
        //Unfiltered IMU data 9 DoF
        for (int i = 0; i < 3; i++) {
#if defined(USE_ACC)
            sbufWriteU16(dst, lrintf(acc.accADCUnfiltered.v[i] * acc.dev.acc_1G_rec*10000)); //scaled
#else
            sbufWriteU16(dst, 0);
#endif
        }
        for (int i = 0; i < 3; i++) {
            sbufWriteU16(dst, gyroRateDpsUnfiltered(i));
        }
        for (int i = 0; i < 3; i++) {
#if defined(USE_MAG)
            sbufWriteU16(dst, lrintf(mag.magADC.v[i]));
#else
            sbufWriteU16(dst, 0);
#endif
        }

    //Filtered IMU data 6 DoF
        for (int i = 0; i < 3; i++) {
#if defined(USE_ACC)
            sbufWriteU16(dst, lrintf(acc.accADC.v[i] * acc.dev.acc_1G_rec*10000)); //scaled
#else
            sbufWriteU16(dst, 0);
#endif
        }
        for (int i = 0; i < 3; i++) {
            sbufWriteU16(dst, gyroRateDps(i));
        }
    //1 RC channel forwarding
        sbufWriteU16(dst, rcData[extendedTelemetryConfig()->extended_telemetry_forwarded_rc_channel]);
#ifdef USE_GPS
    // GPS data with latency
        sbufWriteU8(dst, STATE(GPS_FIX));
        sbufWriteU8(dst, gpsSol.numSat);
        sbufWriteU32(dst, gpsSol.llh.lat);
        sbufWriteU32(dst, gpsSol.llh.lon);
        sbufWriteU16(dst, (uint16_t)constrain(gpsSol.llh.altCm / 100, 0, UINT16_MAX)); // alt changed from 1m to 0.01m per lsb since MSP API 1.39 by RTH. To maintain backwards compatibility compensate to 1m per lsb in MSP again.
        sbufWriteU16(dst, gpsSol.groundSpeed);
        sbufWriteU16(dst, gpsSol.groundCourse);
        // Added in API version 1.44
        sbufWriteU16(dst, gpsSol.dop.pdop);
        uint32_t latency = millis() - gpsData.lastNavMessage;
        sbufWriteU32(dst, latency);
#endif

        /*uint32_t time = millis()/1000;
        telemetryPayloadArr[0] = time & 0xFF;
        telemetryPayloadArr[1] = (time >> 8) & 0xFF;
        telemetryPayloadArr[3] = (time >> 16) & 0xFF;
        telemetryPayloadArr[4] = (time >> 24) & 0xFF;*/
        mspSerialPush(extendedTelemetryConfig()->extended_telemetry_uart_number+SERIAL_PORT_UART_FIRST-1, MSP2_EXTENDED_TELEMETRY, telemetryPayloadArr, sizeof(telemetryPayloadArr), MSP_DIRECTION_REPLY, MSP_V2_NATIVE); // TODO SERIAL_PORT_UART6 cmd command
}

PG_REGISTER_WITH_RESET_FN(extendedTelemetryConfig_t, extendedTelemetryConfig, PG_EXTENDED_TELEMETRY_CONFIG, 1);

void pgResetFn_extendedTelemetryConfig(extendedTelemetryConfig_t *extendedTelemetryConfig)
{
    extendedTelemetryConfig->extended_telemetry_uart_number = 10;
    extendedTelemetryConfig->extended_telemetry_frequency = 100;
    extendedTelemetryConfig->extended_telemetry_forwarded_rc_channel = 6;
}