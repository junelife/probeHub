/*
 * common.h
 *
 * Copyright © Weber Stephen Products LLC., 2022 All Rights Reserved No part of this file may be
 * distributed, reproduced, or used by entities other than Stephen Products LLC. without express
 * written permission. All Rights Reserved UNPUBLISHED, LICENSED SOFTWARE IS CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Weber Stephen Products LLC.
 *
 *  Created on: Mar 16, 2022
 *      Author: ronso
 *
 *  This file contains headers, and variables that should be included in all most source files
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

//#define DEBUG_STATE

#define UPPER_BYTE(int_x) ((uint8_t) ((int_x >> 8) & 0xff));
#define LOWER_BYTE(int_x) ((uint8_t) (int_x & 0xff));


#define FW_VERSION  06
#define HW_VERSION  "0001"
#define DEVICE_ID   02

#include "stdbool.h"
#include "stdint.h"
#include "ctype.h"
#include "gpioXlate.h"
#include "counters.h"
#include "stm32g0xx.h"      //This file is included here because it includes headers for STM32 supplied code. Although it isn't
                            //needed for all files, it contains includes that if included in the wrong order, can cause hard to
                            //find compiler errors.




#endif /* INC_COMMON_H_ */
