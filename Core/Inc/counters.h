/*
 * counters.h
 *
 * Copyright © Weber Stephen Products LLC., 2022 All Rights Reserved No part of this file may be
 * distributed, reproduced, or used by entities other than Stephen Products LLC. without express
 * written permission. All Rights Reserved UNPUBLISHED, LICENSED SOFTWARE IS CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Weber Stephen Products LLC.
 *
 *  Created on: May 16, 2022
 *      Author: ronso
 */

#ifndef INC_COUNTERS_H_
#define INC_COUNTERS_H_

typedef enum
{
    DWN_CNTR_MODBUS,
    DWN_CNTR_IPC,
    DWN_CNTR_END_OF_LIST
} downCounters;

void initializeDownCounters(void);
bool startDownCounterMs(downCounters, uint16_t);
bool startDownCounterSec(downCounters, uint16_t);
bool hasDownCounterExpired(downCounters);
void resetDownCounter(downCounters);

#endif /* INC_COUNTERS_H_ */
