/*
 * counters.c
 *
 * Copyright © Weber Stephen Products LLC., 2022 All Rights Reserved No part of this file may be
 * distributed, reproduced, or used by entities other than Stephen Products LLC. without express
 * written permission. All Rights Reserved UNPUBLISHED, LICENSED SOFTWARE IS CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Weber Stephen Products LLC.
 *
 *  Created on: May 16, 2022
 *      Author: ronso
 */

#include "common.h"

#define MAX_DOWN_COUNTER_VALUE 0xFFFF
#define MS_TO_SEC_MULTIPLIER 1000

uint16_t dwnCounter[DWN_CNTR_END_OF_LIST];


/* Clear all the down counters
 */
void initializeDownCounters(void)
{
    for(uint8_t cntr = 0; cntr < (uint8_t) DWN_CNTR_END_OF_LIST; cntr++)
    {
        dwnCounter[cntr] = 0;
    }
}


/*
 * This function is called to increment  a global variable "uwTick"
 *  used as application time base. This variable is incremented each 1ms
 *  in SysTick ISR.
 *  Also decrements down counters used by tasks.
 *
 */
void HAL_IncTick(void)
{
  uwTick += (uint32_t)uwTickFreq;

  for(uint8_t cntr = 0; cntr < (uint8_t) DWN_CNTR_END_OF_LIST; cntr++)
  {
      if( dwnCounter[cntr] > 0)
      {
          dwnCounter[cntr]--;
      }
  }
}

/* Clear the indicated down counter
 */
void resetDownCounter(downCounters cntr)
{
    if(cntr < DWN_CNTR_END_OF_LIST)
    {
        dwnCounter[cntr] = 0;
    }
}

/* Start a down counter by loading it with a value in milliseconds
 *   Return true if successful
 */
bool startDownCounterMs(downCounters cntr, uint16_t value)
{
    bool retVal = false;
    if(cntr < DWN_CNTR_END_OF_LIST)
    {
        dwnCounter[cntr] = value;
        retVal = true;
    }
    return(retVal);
}

/* Start a down counter by loading it with a value in seconds
 *   If the requested value is greater than the max allowable, use the max
 *   Return true if successful
 */
bool startDownCounterSec(downCounters cntr, uint16_t value)
{
    bool retVal;
    uint16_t newValue;

    if((value * MS_TO_SEC_MULTIPLIER) > MAX_DOWN_COUNTER_VALUE)
    {
        newValue = MAX_DOWN_COUNTER_VALUE;
    }
    else
    {
        newValue = value * MS_TO_SEC_MULTIPLIER;
    }
    retVal = startDownCounterMs(cntr, newValue);

    return(retVal);
}

/* Check a down counter to see if it expired (Counted down to zero)
 *   Return true if expired.
 */
bool hasDownCounterExpired(downCounters cntr)
{
    bool retVal = false;
    if(dwnCounter[cntr] == 0)
    {
        retVal = true;
    }
    return(retVal);
}

