/*
 * hostAlert.c
 *
 * Copyright © Weber Stephen Products LLC., 2022 All Rights Reserved No part of this file may be
 * distributed, reproduced, or used by entities other than Stephen Products LLC. without express
 * written permission. All Rights Reserved UNPUBLISHED, LICENSED SOFTWARE IS CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Weber Stephen Products LLC.
 *
 *
 *  Created on: Aug 11, 2022
 *      Author: ronso
 *
 *  This file handles the MODBUS alerts to the host. The host initiates all MODBUS transactions. If the child device
 *  needs to communicate an alert to the host, the approbate read function will be inserted into the next MODBUS packet
 *  notifying the host that there is information that it needs to read.
 */

#include "common.h"
#include "modbusShare.h"

#define HOST_EVENT_BUF_SIZE 8

struct
{
	uint8_t buf[HOST_EVENT_BUF_SIZE];
	uint8_t inNdx;
	uint8_t outNdx;
	uint8_t size;
}hostEvent;


void addHostReadEvent(mBusReadCodes event)
{
	if(hostEvent.size < HOST_EVENT_BUF_SIZE)
	{
		hostEvent.buf[hostEvent.inNdx++] = event;
		hostEvent.inNdx &= (HOST_EVENT_BUF_SIZE - 1);
		hostEvent.size++;
	}
}


mBusReadCodes getHostReadEvent(void)
{
	mBusReadCodes retVal = MBUS_READ_NONE;

	if(hostEvent.size > 0)
	{
	    retVal = hostEvent.buf[hostEvent.outNdx++];
	    hostEvent.outNdx &= (HOST_EVENT_BUF_SIZE - 1);
	    hostEvent.size--;
	}
	return(retVal);
}
