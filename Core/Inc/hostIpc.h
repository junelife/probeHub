/*
 * hostIpc.h
 *
 * Copyright © Weber Stephen Products LLC., 2022 All Rights Reserved No part of this file may be
 * distributed, reproduced, or used by entities other than Stephen Products LLC. without express
 * written permission. All Rights Reserved UNPUBLISHED, LICENSED SOFTWARE IS CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Weber Stephen Products LLC.
 *
 *  Created on: May 24, 2022
 *      Author: ronso
 *
 *   This file contains defines and prototypes used by the Grill controller interface
 */


#ifndef INC_HOSTIPC_H_
#define INC_HOSTIPC_H_

#include "modbus.h"

void initializeHostIpcTask(void);
bool hostIpcTask(void);
void setMbusPktReceived(t_modbus_packet_struct *, bool);

#endif /* INC_HOSTIPC_H_ */
