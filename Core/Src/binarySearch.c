/*
 * binarySearch.c
 *
 * Copyright © Weber Stephen Products LLC., 2022 All Rights Reserved No part of this file may be
 * distributed, reproduced, or used by entities other than Stephen Products LLC. without express
 * written permission. All Rights Reserved UNPUBLISHED, LICENSED SOFTWARE IS CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Weber Stephen Products LLC.
 *
 *  Created on: July 14, 2022
 *  	Author: Bold Erdene Chinbat
 */

// Local includes
#include "binarySearch.h"

#include "main.h"

// Binary Search for TC (table sort: Ascending)
int BinarySearch_Gap_Finder_Linear_Interpolation_I16(const int16_t* table, int tableSize, int target,
                                                     int indexCoefficent, int indexOffset) {
    int left = 0;
    int right = tableSize - 1;
    int mid;

    while (left + 1 < right) {
        mid = (left + right) >> 1;
        if (table[mid] > target) {
            right = mid;
        } else if (table[mid] < target) {
            left = mid;
        } else {
            return indexCoefficent * (mid + indexOffset);
        }
    }

    return indexCoefficent * (target - table[left]) / (table[right] - table[left]) +
           indexCoefficent * (left + indexOffset);
}

// Binary Search for NTC (table sort: Descending)
int BinarySearch_Gap_Finder_Linear_Interpolation_U16(const uint16_t* table, int tableSize, int target,
                                                     int indexCoefficent, int indexOffset) {
    int left = 0;
    int right = tableSize - 1;
    int mid;

    while (left + 1 < right) {
        mid = (left + right) >> 1;
        if (table[mid] > target) {
            left = mid;
        } else if (table[mid] < target) {
            right = mid;
        } else {
            return indexCoefficent * (mid + indexOffset);
        }
    }

    return indexCoefficent * (target - table[left]) / (table[right] - table[left]) +
           indexCoefficent * (left + indexOffset);
}
