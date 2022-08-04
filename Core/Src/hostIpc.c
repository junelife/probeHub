/*
 * hostIpc.c
 *
 * Copyright © Weber Stephen Products LLC., 2022 All Rights Reserved No part of this file may be
 * distributed, reproduced, or used by entities other than Stephen Products LLC. without express
 * written permission. All Rights Reserved UNPUBLISHED, LICENSED SOFTWARE IS CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Weber Stephen Products LLC.
 *
 *  Created on: May 24, 2022
 *      Author: ronso
 *
 *   This file contains code used to communicate with the grill controller using MODBUS
 */

#include <stdio.h>
#include <string.h>
#include "common.h"
#include "hostIpc.h"
#include "modbus.h"
#include "tim.h"

static void mbFunctRead(void);
static void mbCmdReset(void);
static void mBusCmdWrite(void);
static void adjustLedBrightness(void);

#define IPC_TIME_BETWEEN_PACKETS_MSEC 2 //Wait 2 milliseconds between MODBUS packets
#define MODBUS_TX_MSG_BUF_SIZE 20

struct ipc_vars
{
	t_modbus_packet_struct *pkt;
	uint8_t  msgBuf[MODBUS_TX_MSG_BUF_SIZE];
	uint8_t  msgLen;
	uint8_t  mbPktToSend :1;
	uint8_t  mbPktSent   :1;
	uint8_t  broadCast   :1;
} ipcVars;


void initializeHostIpcTask(void)
{
	ipcVars.mbPktToSend = false;
	ipcVars.mbPktSent = false;
}

/* Host IPC task
 *   The communications between the UGC and devices is handled by MODBUS commands over RS-485
 *   This task task sends packets to the UGC when ipcVars.mbPktToSend indicates
 *   that there is a packet to send
 */
bool hostIpcTask(void)
{
	bool busy = false;

	if(ipcVars.mbPktToSend && hasDownCounterExpired(DWN_CNTR_IPC))
	{
		switch(ipcVars.pkt->fCode)
		{
		    case MBUS_FUNCT_READ:
		    	if(ipcVars.broadCast == false)
		    	{
		    		mbFunctRead();
		    	}
		    	break;
		    case MBUS_FUNCT_RESET:
		    	mbCmdReset();
		    	ipcVars.mbPktSent = true;
                break;
		    case MBUS_FUNCTION_WRITE:
		    	mBusCmdWrite();
		    	break;
		    default:
		    	break;
		}
		if(ipcVars.mbPktSent)
		{
			ipcVars.mbPktToSend = false;
			ipcVars.mbPktSent = false;
		}
	}
	return(busy);
}

/* NOTE- THIS CODE IS NOT THE FINAL CODE
 *    This code will be replaced when we know the actual MODBUS read commands the devices will have
 */
void mbFunctRead(void)
{
	ipcVars.msgLen = 0;
	ipcVars.msgBuf[ipcVars.msgLen++] = ipcVars.pkt->data[0];
	switch(ipcVars.pkt->data[0])
	{
	    case MBUS_READ_ID:
	    	ipcVars.msgBuf[ipcVars.msgLen++] = 0;
	    	ipcVars.msgBuf[ipcVars.msgLen++] = 0x16;
	    	ipcVars.mbPktSent = buildModbusPacket(ipcVars.pkt->mbAdd, ipcVars.pkt->fCode, ipcVars.msgBuf, ipcVars.msgLen);
            break;
	    case MBUS_READ_TYPE:
	    	ipcVars.msgBuf[ipcVars.msgLen++] = (uint8_t) MBUS_DEV_TYPE_PROBE;
	    	ipcVars.msgBuf[ipcVars.msgLen++] = 0x16;
	    	ipcVars.msgBuf[ipcVars.msgLen++] = (uint8_t) FW_VERSION;
	    	ipcVars.mbPktSent = buildModbusPacket(ipcVars.pkt->mbAdd, ipcVars.pkt->fCode, ipcVars.msgBuf, ipcVars.msgLen);
            break;
	    case MBUS_READ_HELLO:
	        ipcVars.msgLen = sprintf((char *) ipcVars.msgBuf, "Hello from %d\n",  getDeviceAddress());
	    	ipcVars.mbPktSent = buildModbusPacket(ipcVars.pkt->mbAdd, ipcVars.pkt->fCode, ipcVars.msgBuf, ipcVars.msgLen);
            break;
		case MBUS_READ_TEMPERATURE:
			adjustLedBrightness();
			ipcVars.msgBuf[ipcVars.msgLen++] = 0;
			ipcVars.msgBuf[ipcVars.msgLen++] = 120;
			ipcVars.msgBuf[ipcVars.msgLen++] = 0;
			ipcVars.msgBuf[ipcVars.msgLen++] = 140;
			ipcVars.mbPktSent = buildModbusPacket(getDeviceAddress(), ipcVars.pkt->fCode, ipcVars.msgBuf, ipcVars.msgLen);
			break;
		case MBUS_READ_FUEL_STATUS:
			ipcVars.msgBuf[ipcVars.msgLen++] = 0;
			ipcVars.msgBuf[ipcVars.msgLen++] = 120;
			ipcVars.msgBuf[ipcVars.msgLen++] = 0;
			ipcVars.msgBuf[ipcVars.msgLen++] = 140;
			ipcVars.mbPktSent = buildModbusPacket(getDeviceAddress(), ipcVars.pkt->fCode, ipcVars.msgBuf, ipcVars.msgLen);
			break;

	    case MBUS_READ_UPGRADE_STATUS:
	    	break;
	}
}


/* This function handles the various MODBUS reset commands send by the UGC.
 */
void mbCmdReset(void)
{
	switch(ipcVars.pkt->data[0])
	{
	    case MBUS_RESET_ADD_IF_NOT_INITIALIZED:
	        // Reset the device if it hasn't been fully initialized
	    	if(!isDeviceAddressInitialized())
	    	{
	    		createRandomModbusAddress();
	    	}
	    	break;
	    case MBUS_RESET_ADD:
	    	createRandomModbusAddress();
	        break;
	    case MBUS_RESET_DEVICE:
	    	NVIC_SystemReset();
	    	break;
	    case MBUS_RESET_UPGRADE:
	    	if(didUpgradeAlreadyExecute())
	    	{
	    	    ipcVars.msgLen = 0;
	    		ipcVars.msgBuf[ipcVars.msgLen++] = MBUS_RESET_UPGRADE;
	    		ipcVars.msgBuf[ipcVars.msgLen++] = ipcVars.pkt->crcCheck;
		    	ipcVars.mbPktSent = buildModbusPacket(getDeviceAddress(), ipcVars.pkt->fCode, ipcVars.msgBuf, ipcVars.msgLen);
	    	}
	    	else if(ipcVars.pkt->data[1] == getDeviceAddress())
	    	{
	    		setUpgradeDataLen(((uint16_t) ipcVars.pkt->data[2] << 8) + ((uint16_t) ipcVars.pkt->data[3]));
	    	}
	    	else
	    	{
	    		disableModbusRx();
	    	}
	    	break;
	    case MBUS_RESET_ENABLE:
	    	if(strncmp(MBUS_DEVICE_ENABLE_STRING, (char *) &ipcVars.pkt->data[1], (uint16_t) MBUS_DEVICE_ENABLE_STRING_LEN) == 0)
	    	{
	    		enableModbusRx();
	    	}

		    break;
	    default:
	    	break;
	}
}

void mBusCmdWrite(void)
{
	ipcVars.msgLen = 0;
	ipcVars.msgBuf[ipcVars.msgLen++] = ipcVars.pkt->data[0];
	switch(ipcVars.pkt->data[0])
	{
	   case MBUS_WRITE_ADD:
		   setDeviceAddress(ipcVars.pkt->data[1]);
		   ipcVars.mbPktSent = buildModbusPacket(getDeviceAddress(), ipcVars.pkt->fCode, ipcVars.msgBuf, ipcVars.msgLen);
           break;
	   case MBUS_WRITE_LED:
		   adjustLedBrightness();
		   ipcVars.msgBuf[ipcVars.msgLen++] = 1;
		   ipcVars.mbPktSent = buildModbusPacket(getDeviceAddress(), ipcVars.pkt->fCode, ipcVars.msgBuf, ipcVars.msgLen);
		   break;
	   default:
		   ipcVars.mbPktSent = true;
	       break;
	}
}

static void adjustLedBrightness(void)
{
	uint16_t brightness = ((uint16_t) ipcVars.pkt->data[1]) << 8;
	brightness |= (uint16_t) ipcVars.pkt->data[2];
	LED1_Brightness_Set(brightness);
}

void setMbusPktReceived(t_modbus_packet_struct *pkt, bool broadcast)
{
	ipcVars.pkt = pkt;
	ipcVars.mbPktToSend = true;
	ipcVars.broadCast = broadcast;
	startDownCounterMs(DWN_CNTR_IPC, IPC_TIME_BETWEEN_PACKETS_MSEC);
}
