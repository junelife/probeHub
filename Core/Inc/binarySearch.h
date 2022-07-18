/*
 * binarySearch.h
 *
 * Copyright © Weber Stephen Products LLC., 2022 All Rights Reserved No part of this file may be
 * distributed, reproduced, or used by entities other than Stephen Products LLC. without express
 * written permission. All Rights Reserved UNPUBLISHED, LICENSED SOFTWARE IS CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Weber Stephen Products LLC.
 *
 *  Created on: July 14, 2022
 *  	Author: Bold Erdene Chinbat
 */

#ifndef INC_BINARYSEARCH_H_
#define INC_BINARYSEARCH_H_

#include "main.h"

int BinarySearch_Gap_Finder_Linear_Interpolation_I16(const int16_t* table, int tableSize, int target,
                                                     int indexCoefficent, int indexOffset);
int BinarySearch_Gap_Finder_Linear_Interpolation_U16(const uint16_t* table, int tableSize, int target,
                                                     int indexCoefficent, int indexOffset);

#endif /* INC_BINARYSEARCH_H_ */
