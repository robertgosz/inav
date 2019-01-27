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

#include "platform.h"

#if defined(USE_TELEMETRY_SRXL)

#include "build/version.h"

#include "common/crc.h"
#include "common/streambuf.h"
#include "common/utils.h"

#include "config/feature.h"

#include "io/gps.h"
#include "io/serial.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"

#include "io/gps.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "sensors/battery.h"

#include "telemetry/telemetry.h"
#include "telemetry/srxl.h"

#define SRXL_ADDRESS_FIRST          0xA5
#define SRXL_ADDRESS_SECOND         0x80
#define SRXL_PACKET_LENGTH          0x15

#define SRXL_FRAMETYPE_TELE_QOS     0x7F
#define SRXL_FRAMETYPE_TELE_RPM     0x7E
#define SRXL_FRAMETYPE_POWERBOX     0x0A
#define SRXL_FRAMETYPE_TELE_FP_MAH  0x34
#define TELE_DEVICE_VTX             0x0D   // Video Transmitter Status
#define SRXL_FRAMETYPE_SID          0x00
#define SRXL_FRAMETYPE_GPS_LOC      0x16   // GPS Location Data (Eagle Tree)
#define SRXL_FRAMETYPE_GPS_STAT     0x17   

static bool srxlTelemetryEnabled;
static uint8_t srxlFrame[SRXL_FRAME_SIZE_MAX];

static void srxlInitializeFrame(sbuf_t *dst)
{
    dst->ptr = srxlFrame;
    dst->end = ARRAYEND(srxlFrame);

    sbufWriteU8(dst, SRXL_ADDRESS_FIRST);
    sbufWriteU8(dst, SRXL_ADDRESS_SECOND);
    sbufWriteU8(dst, SRXL_PACKET_LENGTH);
}

static void srxlFinalize(sbuf_t *dst)
{
    crc16_ccitt_sbuf_append(dst, &srxlFrame[3]); // start at byte 3, since CRC does not include device address and packet length
    sbufSwitchToReader(dst, srxlFrame);
    // write the telemetry frame to the receiver.
    srxlRxWriteTelemetryData(sbufPtr(dst), sbufBytesRemaining(dst));
}

/*
SRXL frame has the structure:
<0xA5><0x80><Length><16-byte telemetry packet><2 Byte CRC of payload>
The <Length> shall be 0x15 (length of the 16-byte telemetry packet + overhead).
*/

/*
typedef struct
{
    UINT8 identifier; // Source device = 0x7F
    UINT8 sID; // Secondary ID
    UINT16 A;
    UINT16 B;
    UINT16 L;
    UINT16 R;
    UINT16 F;
    UINT16 H;
    UINT16 rxVoltage; // Volts, 0.01V increments
} STRU_TELE_QOS;
*/

#define STRU_TELE_QOS_EMPTY_REPORT_COUNT 14

bool srxlFrameQos(sbuf_t *dst, timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    sbufWriteU8(dst, SRXL_FRAMETYPE_TELE_QOS);
    sbufWriteU8(dst, SRXL_FRAMETYPE_SID);

    sbufFill(dst, 0xFF, STRU_TELE_QOS_EMPTY_REPORT_COUNT); // Clear remainder

    return true;
}

/*
typedef struct
{
    UINT8 identifier; // Source device = 0x7E
    UINT8 sID; // Secondary ID
    UINT16 microseconds; // microseconds between pulse leading edges
    UINT16 volts; // 0.01V increments
    INT16 temperature; // degrees F
    INT8 dBm_A, // Average signal for A antenna in dBm
    INT8 dBm_B; // Average signal for B antenna in dBm.
    // If only 1 antenna, set B = A
} STRU_TELE_RPM;
*/

#define STRU_TELE_RPM_EMPTY_FIELDS_COUNT 8

bool srxlFrameRpm(sbuf_t *dst, timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    sbufWriteU8(dst, SRXL_FRAMETYPE_TELE_RPM);
    sbufWriteU8(dst, SRXL_FRAMETYPE_SID);
    sbufWriteU16BigEndian(dst, 0xFFFF);                     // pulse leading edges
    sbufWriteU16BigEndian(dst, getBatteryVoltage() * 10);   // vbat is in units of 0.1V
    sbufWriteU16BigEndian(dst, 0x7FFF);                     // temperature

    sbufFill(dst, 0xFF, STRU_TELE_RPM_EMPTY_FIELDS_COUNT);
    return true;
}

#if defined(USE_GPS) 

// From Frsky implementation
static void GPStoDDDMM_MMMM(int32_t mwiigps, gpsCoordinateDDDMMmmmm_t *result)
{
    int32_t absgps, deg, min;
    absgps = ABS(mwiigps);
    deg = absgps / GPS_DEGREES_DIVIDER;
    absgps = (absgps - deg * GPS_DEGREES_DIVIDER) * 60;     // absgps = Minutes left * 10^7
    min = absgps / GPS_DEGREES_DIVIDER;                     // minutes left
    result->dddmm = deg * 100 + min;
    result->mmmm = (absgps - min * GPS_DEGREES_DIVIDER) / 1000;
}

// BCD conversion
static uint32_t dec2bcd(uint16_t dec)
{
    uint32_t result = 0;
    uint8_t counter = 0;

    while (dec) {
        result |= (dec % 10) << counter * 4;
        counter++;
        dec /= 10;
    }
    return result;
}

/*
typedef struct
{
    UINT8    identifier;    // Source device = 0x16
    UINT8    sID;           // Secondary ID
    UINT16   altitudeLow;   // BCD, meters, format 3.1 (Low order of altitude)
    UINT32   latitude;      // BCD, format 4.4, Degrees * 100 + minutes, less than 100 degrees
    UINT32   longitude;     // BCD, format 4.4 , Degrees * 100 + minutes, flag indicates > 99 degrees
    UINT16   course;        // BCD, 3.1
    UINT8    HDOP;          // BCD, format 1.1
    UINT8    GPSflags;      // see definitions below
} STRU_TELE_GPS_LOC;
*/

// GPS flags definitions
#define GPS_FLAGS_IS_NORTH_BIT              0x01
#define GPS_FLAGS_IS_EAST_BIT               0x02
#define GPS_FLAGS_LONGITUDE_GREATER_99_BIT  0x04
#define GPS_FLAGS_GPS_FIX_VALID_BIT         0x08
#define GPS_FLAGS_GPS_DATA_RECEIVED_BIT     0x10
#define GPS_FLAGS_3D_FIX_BIT                0x20
#define GPS_FLAGS_NEGATIVE_ALT_BIT          0x80

bool srxlFrameGpsLoc(sbuf_t *dst, timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    gpsCoordinateDDDMMmmmm_t coordinate;
    uint32_t latitudeBcd, longitudeBcd, altitudeLo;
    uint16_t altitudeLoBcd, groundCourseBcd, hdop;
    uint8_t hdopBcd, gpsFlags;

    if (!feature(FEATURE_GPS) || !STATE(GPS_FIX) || gpsSol.numSat < 6) {
        return false;
    }

    // lattitude
    GPStoDDDMM_MMMM(gpsSol.llh.lat, &coordinate);
    latitudeBcd  = (dec2bcd(coordinate.dddmm) << 16) | dec2bcd(coordinate.mmmm);

    // longitude
    GPStoDDDMM_MMMM(gpsSol.llh.lon, &coordinate);
    longitudeBcd = (dec2bcd(coordinate.dddmm) << 16) | dec2bcd(coordinate.mmmm);

    // altitude (low order)
    altitudeLo = ABS(gpsSol.llh.alt) / 10;
    altitudeLoBcd = dec2bcd(altitudeLo % 100000);

    // Ground course
    groundCourseBcd = dec2bcd(gpsSol.groundCourse);

    // HDOP
    hdop = gpsSol.hdop / 10;
    hdop = (hdop > 99) ? 99 : hdop;
    hdopBcd = dec2bcd(hdop);

    // flags
    gpsFlags = GPS_FLAGS_GPS_DATA_RECEIVED_BIT | GPS_FLAGS_GPS_FIX_VALID_BIT | GPS_FLAGS_3D_FIX_BIT;
    gpsFlags |= (gpsSol.llh.lat > 0) ? GPS_FLAGS_IS_NORTH_BIT : 0;
    gpsFlags |= (gpsSol.llh.lon > 0) ? GPS_FLAGS_IS_EAST_BIT : 0;
    gpsFlags |= (gpsSol.llh.alt < 0) ? GPS_FLAGS_NEGATIVE_ALT_BIT : 0;
    gpsFlags |= (gpsSol.llh.lon / GPS_DEGREES_DIVIDER > 99) ? GPS_FLAGS_LONGITUDE_GREATER_99_BIT : 0;

    // SRXL frame
    sbufWriteU8(dst, SRXL_FRAMETYPE_GPS_LOC);
    sbufWriteU8(dst, SRXL_FRAMETYPE_SID);
    sbufWriteU16(dst, altitudeLoBcd);
    sbufWriteU32(dst, latitudeBcd);
    sbufWriteU32(dst, longitudeBcd);
    sbufWriteU16(dst, groundCourseBcd);
    sbufWriteU8(dst, hdopBcd);
    sbufWriteU8(dst, gpsFlags);

    return true;
}

/*
typedef struct
{
   UINT8   identifier;                      // Source device = 0x17
   UINT8   sID;                             // Secondary ID
   UINT16  speed;                           // BCD, knots, format 3.1
   UINT32  UTC;                             // BCD, format HH:MM:SS.S, format 6.1
   UINT8   numSats;                         // BCD, 0-99
   UINT8   altitudeHigh;                    // BCD, meters, format 2.0 (High bits alt)
} STRU_TELE_GPS_STAT;
*/

#define STRU_TELE_GPS_STAT_EMPTY_FIELDS_COUNT 6
#define SPEKTRUM_TIME_UNKNOWN 0xFFFFFFFF

bool srxlFrameGpsStat(sbuf_t *dst, timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    uint32_t timeBcd;
    uint16_t speedKnotsBcd, speedTmp;
    uint8_t numSatBcd, altitudeHighBcd;
    dateTime_t dt;
    bool timeProvided = false;

    if (!feature(FEATURE_GPS) || !STATE(GPS_FIX) || gpsSol.numSat < 6) {
        return false;
    }

    // Number of sats and altitude (high bits)
    numSatBcd = (gpsSol.numSat > 99) ? dec2bcd(99) : dec2bcd(gpsSol.numSat);
    altitudeHighBcd = dec2bcd(gpsSol.llh.alt / 100000);

    // Speed (knots)
    speedTmp = gpsSol.groundSpeed * 1944 / 1000;
    speedKnotsBcd = (speedTmp > 9999) ? dec2bcd(9999) : dec2bcd(speedTmp);

    // RTC
    if (rtcHasTime()) {
        rtcGetDateTime(&dt);
        timeBcd = dec2bcd(dt.hours);
        timeBcd = timeBcd << 8;
        timeBcd = timeBcd | dec2bcd(dt.minutes);
        timeBcd = timeBcd << 8;
        timeBcd = timeBcd | dec2bcd(dt.seconds);
        timeBcd = timeBcd << 4;
        timeBcd = timeBcd | dec2bcd(dt.millis / 100);
        timeProvided = true;
    }
    timeBcd = (timeProvided) ? timeBcd : SPEKTRUM_TIME_UNKNOWN;

    // SRXL frame
    sbufWriteU8(dst, SRXL_FRAMETYPE_GPS_STAT);
    sbufWriteU8(dst, SRXL_FRAMETYPE_SID);
    sbufWriteU16(dst, speedKnotsBcd);
    sbufWriteU32(dst, timeBcd);
    sbufWriteU8(dst, numSatBcd);
    sbufWriteU8(dst, altitudeHighBcd);
    sbufFill(dst, 0xFF, STRU_TELE_GPS_STAT_EMPTY_FIELDS_COUNT);

    return true;
}

#endif

/*
typedef struct
{
    UINT8   identifier;     // Source device = 0x34
    UINT8   sID;            // Secondary ID
    INT16   current_A;      // Instantaneous current, 0.1A (0-3276.8A)
    INT16   chargeUsed_A;   // Integrated mAh used, 1mAh (0-32.766Ah)
    UINT16  temp_A;         // Temperature, 0.1C (0-150C, 0x7FFF indicates not populated)
    INT16   current_B;      // Instantaneous current, 0.1A (0-3276.8A)
    INT16   chargeUsed_B;   // Integrated mAh used, 1mAh (0-32.766Ah)
    UINT16  temp_B;         // Temperature, 0.1C (0-150C, 0x7FFF indicates not populated)
    UINT16  spare;          // Not used
} STRU_TELE_FP_MAH;
*/

#define FP_MAH_KEEPALIVE_TIME_OUT 2000000 // 2s

bool srxlFrameFlightPackCurrent(sbuf_t *dst, timeUs_t currentTimeUs)
{
    uint16_t amps = getAmperage() / 10;
    uint16_t mah  = getMAhDrawn();
    static uint16_t sentAmps;
    static uint16_t sentMah;
    static timeUs_t lastTimeSentFPmAh = 0;

    timeUs_t keepAlive = currentTimeUs - lastTimeSentFPmAh;

    if ( (amps != sentAmps) || (mah != sentMah) ||
         keepAlive > FP_MAH_KEEPALIVE_TIME_OUT ) {
        sbufWriteU8(dst, SRXL_FRAMETYPE_TELE_FP_MAH);
        sbufWriteU8(dst, SRXL_FRAMETYPE_SID);
        sbufWriteU16(dst, amps);
        sbufWriteU16(dst, mah);
        sbufWriteU16(dst, 0x7fff);            // temp A
        sbufWriteU16(dst, 0x7fff);            // Amps B
        sbufWriteU16(dst, 0x7fff);            // mAH B
        sbufWriteU16(dst, 0x7fff);            // temp B
        sbufWriteU16(dst, 0xffff);

        sentAmps = amps;
        sentMah = mah;
        lastTimeSentFPmAh = currentTimeUs;
        return true;
    }
    return false;
}

// Schedule array to decide how often each type of frame is sent
// The frames are scheduled in sets of 3 frames, 2 mandatory and 1 user frame.
// The user frame type is cycled for each set.
// Example. QOS, RPM,.CURRENT, QOS, RPM, TEXT. QOS, RPM, CURRENT, etc etc

#define SRXL_SCHEDULE_MANDATORY_COUNT  2 // Mandatory QOS and RPM sensors

#define SRXL_FP_MAH_COUNT   1

#if defined(USE_GPS)
#define SRXL_GPS_LOC_COUNT  1
#define SRXL_GPS_STAT_COUNT 1
#else
#define SRXL_GPS_LOC_COUNT  0
#define SRXL_GPS_STAT_COUNT 0
#endif

#define SRXL_SCHEDULE_USER_COUNT (SRXL_FP_MAH_COUNT + SRXL_GPS_LOC_COUNT + SRXL_GPS_STAT_COUNT)
#define SRXL_SCHEDULE_COUNT_MAX  (SRXL_SCHEDULE_MANDATORY_COUNT + 1)
#define SRXL_TOTAL_COUNT         (SRXL_SCHEDULE_MANDATORY_COUNT + SRXL_SCHEDULE_USER_COUNT)

typedef bool (*srxlScheduleFnPtr)(sbuf_t *dst, timeUs_t currentTimeUs);

const srxlScheduleFnPtr srxlScheduleFuncs[SRXL_TOTAL_COUNT] = {
    /* must send srxlFrameQos, Rpm and then alternating items of our own */
    srxlFrameQos,
    srxlFrameRpm,
    srxlFrameFlightPackCurrent,
#if defined(USE_GPS)
    srxlFrameGpsStat,
    srxlFrameGpsLoc,
#endif
};

static void processSrxl(timeUs_t currentTimeUs)
{
    static uint8_t srxlScheduleIndex = 0;
    static uint8_t srxlScheduleUserIndex = 0;

    sbuf_t srxlPayloadBuf;
    sbuf_t *dst = &srxlPayloadBuf;
    srxlScheduleFnPtr srxlFnPtr;

    if (srxlScheduleIndex < SRXL_SCHEDULE_MANDATORY_COUNT) {
        srxlFnPtr = srxlScheduleFuncs[srxlScheduleIndex];
    } else {
        srxlFnPtr = srxlScheduleFuncs[srxlScheduleIndex + srxlScheduleUserIndex];
        srxlScheduleUserIndex = (srxlScheduleUserIndex + 1) % SRXL_SCHEDULE_USER_COUNT;
    }

    if (srxlFnPtr) {
        srxlInitializeFrame(dst);
        if (srxlFnPtr(dst, currentTimeUs)) {
            srxlFinalize(dst);
        }
    }
    srxlScheduleIndex = (srxlScheduleIndex + 1) % SRXL_SCHEDULE_COUNT_MAX;
}

void initSrxlTelemetry(void)
{
    // check if there is a serial port open for SRXL telemetry (ie opened by the SRXL RX)
    // and feature is enabled, if so, set SRXL telemetry enabled
    srxlTelemetryEnabled = srxlRxIsActive();
 }

bool checkSrxlTelemetryState(void)
{
    return srxlTelemetryEnabled;
}

/*
 * Called periodically by the scheduler
 */
void handleSrxlTelemetry(timeUs_t currentTimeUs)
{
  if (srxlTelemetryBufferEmpty()) {
      processSrxl(currentTimeUs);
  }
}
#endif
