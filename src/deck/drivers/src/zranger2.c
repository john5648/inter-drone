/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * vl53l0x.c: Time-of-flight distance sensor driver
 */

#define DEBUG_MODULE "ZR2"

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "range.h"
#include "static_mem.h"

#include "i2cdev.h"
#include "zranger2.h"
#include "vl53l1x.h"

#include "cf_math.h"

#include "commander.h"

// Measurement noise model
static const float expPointA = 2.5f;
static const float expStdA = 0.0025f; // STD at elevation expPointA [m]
static const float expPointB = 4.0f;
static const float expStdB = 0.2f;    // STD at elevation expPointB [m]
static float expCoeff;

#define RANGE_OUTLIER_LIMIT 5000 // the measured range is in [mm]

static uint16_t range_last = 0;

static bool isInit;

NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t dev;

#define ABS(X) ((X) < 0 ? -(X) : (X))
static setpoint_t setpoint;
static state_t state;
bool ondesiredheight = false;
bool outofrange = false;
static float offset_range = 0;
static float history_range = 0;
static float collect_range = 0;

static uint16_t zRanger2GetMeasurementAndRestart(VL53L1_Dev_t *dev)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;
    VL53L1_RangingMeasurementData_t rangingData;
    uint8_t dataReady = 0;
    uint16_t range;

    while (dataReady == 0)
    {
        status = VL53L1_GetMeasurementDataReady(dev, &dataReady);
        vTaskDelay(M2T(1));
    }

    status = VL53L1_GetRangingMeasurementData(dev, &rangingData);
    range = rangingData.RangeMilliMeter;

    VL53L1_StopMeasurement(dev);
    status = VL53L1_StartMeasurement(dev);
    status = status;

    return range;
}

void zRanger2Init(DeckInfo* info)
{
  if (isInit)
    return;

  if (vl53l1xInit(&dev, I2C1_DEV))
  {
      DEBUG_PRINT("Z-down sensor [OK]\n");
  }
  else
  {
    DEBUG_PRINT("Z-down sensor [FAIL]\n");
    return;
  }

  xTaskCreate(zRanger2Task, ZRANGER2_TASK_NAME, ZRANGER2_TASK_STACKSIZE, NULL, ZRANGER2_TASK_PRI, NULL);

  // pre-compute constant in the measurement noise model for kalman
  expCoeff = logf(expStdB / expStdA) / (expPointB - expPointA);

  isInit = true;
}

bool zRanger2Test(void)
{
  if (!isInit)
    return false;

  return true;
}

void zRanger2Task(void* arg)
{
  TickType_t lastWakeTime;

  systemWaitStart();

  // Restart sensor
  VL53L1_StopMeasurement(&dev);
  VL53L1_SetDistanceMode(&dev, VL53L1_DISTANCEMODE_MEDIUM);
  VL53L1_SetMeasurementTimingBudgetMicroSeconds(&dev, 25000);

  VL53L1_StartMeasurement(&dev);

  lastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&lastWakeTime, M2T(25));

    range_last = zRanger2GetMeasurementAndRestart(&dev);
    rangeSet(rangeDown, range_last / 1000.0f);

    // check if range is feasible and push into the estimator
    // the sensor should not be able to measure >5 [m], and outliers typically
    // occur as >8 [m] measurements
    if (range_last < RANGE_OUTLIER_LIMIT){
      float distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
      // range_last is the measurement of z in mm (uint_16)
      // setpoint.position.z is in meter
      commanderGetSetpoint(&setpoint, &state);
      // check if server has send setpoint and drone is on desired height
      if (setpoint.position.z >= 0.1f && ondesiredheight == false){
        if(distance >= (setpoint.position.z-0.02f)){
          ondesiredheight = true;
          history_range = distance;
        }
      }else if(setpoint.position.z <= 0.1f && ondesiredheight == true){
        ondesiredheight = false;
        offset_range = 0.0f;
      }else if(ondesiredheight == true){
        if (distance>=setpoint.position.z-0.02f){
          offset_range = 0.0f;
        }else if(ABS(setpoint.position.z-offset_range - distance) >= 0.03f){
          if (history_range-distance>=0){
            offset_range = setpoint.position.z - (distance - (history_range-distance)*0.3f);
          }
          else{
            offset_range = setpoint.position.z - distance;
          }
          history_range = distance;
        }
      }
      distance = distance + offset_range;
      collect_range = distance;
      float stdDev = expStdA * (1.0f  + expf( expCoeff * (distance - expPointA)));
      rangeEnqueueDownRangeInEstimator(distance, stdDev, xTaskGetTickCount());
    }
  }
}

static const DeckDriver zranger2_deck = {
  .vid = 0xBC,
  .pid = 0x0E,
  .name = "bcZRanger2",
  .usedGpio = 0x0C,

  .init = zRanger2Init,
  .test = zRanger2Test,
};

DECK_DRIVER(zranger2_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcZRanger2, &isInit)
PARAM_GROUP_STOP(deck)

LOG_GROUP_START(zranging)
// LOG_ADD(LOG_FLOAT, criterion, &takeoffing)
LOG_ADD(LOG_FLOAT, history, &history_range)
LOG_ADD(LOG_FLOAT, collect, &collect_range)
LOG_GROUP_STOP(zranging) 