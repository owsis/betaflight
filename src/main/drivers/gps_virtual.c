/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_GPS

#include "common/axis.h"
#include "common/utils.h"
#include "common/maths.h"

#include "drivers/time.h"

#include "io/gps.h"

// Virtual GPS state
static bool gpsDataReady = false;
static uint32_t lastUpdateTime = 0;

void virtualGpsSet(int32_t lat, int32_t lon, int32_t alt, 
                   uint16_t groundSpeed, uint16_t groundCourse,
                   uint8_t numSat, uint8_t fixType,
                   int16_t velNED_N, int16_t velNED_E, int16_t velNED_D)
{
    // Directly populate gpsSol like other GPS providers do
    gpsSol.llh.lat = lat;          // degrees * 1e7
    gpsSol.llh.lon = lon;          // degrees * 1e7
    gpsSol.llh.altCm = alt;        // altitude in cm
    
    gpsSol.groundSpeed = groundSpeed;      // cm/s
    gpsSol.groundCourse = groundCourse;    // degrees * 10
    gpsSol.numSat = numSat;
    
    // Calculate 3D speed from NED velocity
    float speed3d_cms = sqrtf((float)(velNED_N * velNED_N + velNED_E * velNED_E + velNED_D * velNED_D));
    gpsSol.speed3d = (uint16_t)speed3d_cms;
    
    // Set fix status (fixType: 0=none, 2=2D, 3=3D)
    if (fixType >= 3) {  // 3D fix
        gpsSetFixState(true);
        gpsSol.dop.hdop = 100;  // HDOP 1.0 (good fix)
    } else {
        gpsSetFixState(false);
        gpsSol.dop.hdop = 9999;
    }
    
    gpsSol.dop.pdop = 150;  // PDOP 1.5 (reasonable)
    
    // Set accuracy estimates (better than defaults)
    gpsSol.acc.hAcc = 100;  // 1m horizontal accuracy
    gpsSol.acc.vAcc = 150;  // 1.5m vertical accuracy
    gpsSol.acc.sAcc = 50;   // 0.5 m/s speed accuracy
    
    lastUpdateTime = millis();
    gpsDataReady = true;
}

bool virtualGpsNewFrame(void)
{
    if (gpsDataReady) {
        gpsDataReady = false;
        return true;
    }
    return false;
}

bool virtualGpsIsHealthy(void)
{
    // Consider GPS healthy if we received data in last 2 seconds
    return (millis() - lastUpdateTime) < 2000;
}

#endif // USE_GPS
