/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software.
 */

#pragma once

void virtualGpsSet(int32_t lat, int32_t lon, int32_t alt, 
                   uint16_t groundSpeed, uint16_t groundCourse,
                   uint8_t numSat, uint8_t fixType,
                   int16_t velNED_N, int16_t velNED_E, int16_t velNED_D);

bool virtualGpsNewFrame(void);
bool virtualGpsIsHealthy(void);
