/*
 * modbus.c
 *
 * Copyright © Weber Stephen Products LLC., 2022 All Rights Reserved No part of this file may be
 * distributed, reproduced, or used by entities other than Stephen Products LLC. without express
 * written permission. All Rights Reserved UNPUBLISHED, LICENSED SOFTWARE IS CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Weber Stephen Products LLC.
 *
 *  Created on: Apr 14, 2022
 *      Author: ronso
 *
 *   This file handles the assembly, transmission, and reception of Modbus packets over
 *   RS-485 serial
 */

#include <stdio.h>
#include <stdlib.h>
#include "common.h"
#include "rs485.h"
#include "modbus.h"
#include "hostIpc.h"

uint16_t generateCRC16(uint8_t*, uint16_t);
void startCrc(void);
void iterateOneByteForCrc(uint8_t);
void reStartModbusTask(void);

#define MODBUS_TIME_OUT_SEC                3 // 3 seconds, Allow MODBUS this time MAX to collect a packet
#define MODBUS_UPGRADE_START_TIME_OUT_SEC 10 // 10 Seconds Allow MODBUS this time MAX to start receiving data for an upgrade

uint16_t nCrc;

t_modbus_packet_struct modbusStruct;



typedef enum modbus_states
{
    MB_STATE_INITIALIZE = 0,
    MB_STATE_IDLE,
    MB_STATE_ADDRESS,
    MB_STATE_FUNCTION,
    MB_STATE_DATA_LENGTH,
    MB_STATE_DATA_COLLECT,
    MB_STATE_DATA_CRC_HI,
    MB_STATE_DATA_CRC_LOW
} mbStates;

modbusMsgs mbMsg[] =
{
   //Message Name     Device Address  Function   data                       TxLen    RxLen
   //--------------   ---------------  ------   --------------------------  -------- --------
    { MB_MSG_GET_TEMP,MODBUS_ADD_TEMP,   4,    (uint8_t*) "\x00\x01\x00\x01",   4,    MODBUS_GET_RX_LEN_FROM_DEVICE },
    { MB_MSG_END,     MB_MSG_END,        0,    (uint8_t*) "",                   0,    0 },
};

struct modbus_task_vars
{
    mbStates state;
    uint16_t upgradeLen;
    uint8_t  mbAddress;
    uint8_t  upgrade :1;
    uint8_t  enable  :1;
} mbVars;


/* This is test code to allow the blue button on the development board to
 * Enable/disable MODBUS
 */
void changeStateModbusTask(void)
{
    if (mbVars.enable)
    {
        mbVars.enable = false;
    }
    else
    {
        mbVars.enable = true;
    }
}

/* Enable MODBUS communication with the controller.
 */
void enableModbusRx(void)
{
    mbVars.enable = true;
}

/* Disable MODBUS communication with the controller.
 */
void disableModbusRx(void)
{
    mbVars.enable = false;
}


/* Set the data length for the upgrade.
 *   This function sets the number of bytes we expect to get from the controller
 *   on the next transfer. It also configures the state machine to start receiving
 *   a stream of raw data.
 */
void setUpgradeDataLen(uint16_t dLen)
{
    mbVars.upgradeLen = dLen;
    mbVars.upgrade = true;
    startCrc();
    startDownCounterSec(DWN_CNTR_MODBUS, MODBUS_UPGRADE_START_TIME_OUT_SEC);
    mbVars.state = MB_STATE_DATA_COLLECT;
}

/* check to see if the an upgrade just ran, and clear the "just ran" flag
 *
 */
bool didUpgradeAlreadyExecute(void)
{
    bool retVal = mbVars.upgrade;
    mbVars.upgrade = false;
    return (retVal);
}

void initializeModbusTask(void)
{
    mbVars.state = MB_STATE_INITIALIZE;
    mbVars.mbAddress = MODBUS_UNITIALIZED_ADDRESS;
    mbVars.enable = true;
    mbVars.upgrade = false;
}

/* Re-start the MODBUS task (call me if the state machine gets in trouble)
 */
void reStartModbusTask(void)
{
    mbVars.state = MB_STATE_IDLE;
    mbVars.upgrade = false;
}

/* Traverse the rs485 receive buffer and parse Modbus packets
 *   Calculate the CRC and check it, if the CRC checks out, and
 *   it is the address of this device, notify the IPC task.
 *
 */
bool modbusTask(void)
{
    bool busy = true;
    switch (mbVars.state)
    {
        case MB_STATE_INITIALIZE:
            // Remove any spurious characters from the receive buffer before we start
            while ((getRs485RxbufSize() > 0))
            {
                getNextCharFromRs485rxBuf();
            }
            mbVars.state = MB_STATE_IDLE;
            break;
        case MB_STATE_IDLE:
            // Wait here for the start of a packet
            if (getRs485RxbufSize() > 0)
            {
                startCrc();
                startDownCounterSec(DWN_CNTR_MODBUS, MODBUS_TIME_OUT_SEC);
                mbVars.state = MB_STATE_ADDRESS;
            }
            else
            {
                busy = false;
            }
            break;
        case MB_STATE_ADDRESS:
            // We got a character
            modbusStruct.mbAdd = getNextCharFromRs485rxBuf();
            if (mbVars.enable || (modbusStruct.mbAdd == MODBUS_BROADCAST_ADDRESS))
            {   // The only command that is allowed if modbus is disabled, is the broadcast modbus enable command.
                // so verify that MODBUS rx is enabled, or this is the broadcast address.
                if (modbusStruct.mbAdd > MODBUS_MAX_CHILD_ADDRESS)
                {
                    mbVars.state = MB_STATE_INITIALIZE;
                }
                else
                {
                    iterateOneByteForCrc(modbusStruct.mbAdd);
                    mbVars.state = MB_STATE_FUNCTION;
                }
                break;
            }
            else
            {
                mbVars.state = MB_STATE_INITIALIZE;
            }
        case MB_STATE_FUNCTION:
            if (getRs485RxbufSize() > 0)
            {
                modbusStruct.fCode = getNextCharFromRs485rxBuf();
                if (mbVars.enable || (modbusStruct.fCode == MBUS_FUNCT_RESET))
                {
                    iterateOneByteForCrc(modbusStruct.fCode);
                    mbVars.state = MB_STATE_DATA_LENGTH;
                }
            }
            break;
        case MB_STATE_DATA_LENGTH:
            if (getRs485RxbufSize() > 0)
            {
                modbusStruct.dataLen = getNextCharFromRs485rxBuf();
                iterateOneByteForCrc(modbusStruct.dataLen);
                if (modbusStruct.dataLen > 0)
                {
                    if (modbusStruct.dataLen < MODBUS_MAX_DATA_LENGTH)
                    {
                        mbVars.state = MB_STATE_DATA_COLLECT;
                    } else
                    {   // Ill-formed packet, start over
                        mbVars.state = MB_STATE_INITIALIZE;
                    }
                } else
                {
                    mbVars.state = MB_STATE_DATA_CRC_HI;
                }
            }
            break;
        case MB_STATE_DATA_COLLECT:
            if (mbVars.upgrade)
            {
                if (mbVars.upgradeLen > 0)
                {
                    if (getRs485RxbufSize() > 0)
                    {
                        iterateOneByteForCrc(getNextCharFromRs485rxBuf());
                        mbVars.upgradeLen--;
                    }
                } else
                {
                    mbVars.state = MB_STATE_DATA_CRC_HI;
                }
            }
            else
            {
                if (getRs485RxbufSize() >= modbusStruct.dataLen)
                {
                    for (int i = 0; i < modbusStruct.dataLen; i++)
                    {
                        modbusStruct.data[i] = getNextCharFromRs485rxBuf();
                        iterateOneByteForCrc(modbusStruct.data[i]);
                    }
                    if (mbVars.enable || (modbusStruct.data[0] == MBUS_RESET_ENABLE))
                    {
                        mbVars.state = MB_STATE_DATA_CRC_HI;
                    }
                    else
                    {
                    	mbVars.state = MB_STATE_INITIALIZE;
                    }
                }
            }
            break;
        case MB_STATE_DATA_CRC_HI:
            if (getRs485RxbufSize() > 0)
            {
                modbusStruct.crc = ((uint16_t) getNextCharFromRs485rxBuf()) & 0x00FF;
                mbVars.state = MB_STATE_DATA_CRC_LOW;
            }
            break;
        case MB_STATE_DATA_CRC_LOW:
            if (getRs485RxbufSize() > 0)
            {
                modbusStruct.crc |= (((uint16_t) getNextCharFromRs485rxBuf()) << 8) & 0xFF00;
                modbusStruct.crcCheck = false;
                if (modbusStruct.crc == nCrc)
                {
                    modbusStruct.crcCheck = true;
                    if (modbusStruct.mbAdd == MODBUS_BROADCAST_ADDRESS)
                    {
                        setMbusPktReceived(&modbusStruct, true); // Good packet (BRODCAST), let IPC know about it.
                    }
                    else if (modbusStruct.mbAdd == mbVars.mbAddress)
                    {
                        setMbusPktReceived(&modbusStruct, false); // Good packet (this device address), let IPC know about it.
                    }
                }
                else
                {
                    // If the CRC isn't correct, we need to clear the upgrade flag here, otherwise
                    // it gets cleared, if and when, the upgrade gets handled.
                    mbVars.upgrade = false;
                }
                mbVars.state = MB_STATE_IDLE;
            }
            break;
        default:
            mbVars.state = MB_STATE_INITIALIZE;
            break;
    }
    if (mbVars.state > MB_STATE_IDLE)
    { // If the MODBUS packet is taking to long to finish, restart the MODBUS task.
        if (hasDownCounterExpired(DWN_CNTR_MODBUS))
        {
            mbVars.state = MB_STATE_IDLE;
            mbVars.upgrade = false;
            busy = false;
        }
    }
    return (busy);
}

/* Send a Modbus message from the table entry indicated by msg
 *
 */
bool sendModbusMessage(mbMsgName msg)
{
    bool retVal = false;
    uint8_t msgNdx = 0;

    while (mbMsg[msgNdx].name != MB_MSG_END)
    {
        if (mbMsg[msgNdx].name == msg)
        {
            buildModbusPacket(mbMsg[msgNdx].address, mbMsg[msgNdx].function,
                    mbMsg[msgNdx].data, mbMsg[msgNdx].txDataLen);
            msgNdx++;
            retVal = true;
            break;
        }
    }
    return (retVal);
}

/**********************************************************************
 * Function Name: build_modbus_packet
 ***********************************************************************
 * Summary: This function builds the referenced packet structure, given parameters below.
 *
 * Parameters: Pointer to packet to be built, child address, function code, data, and length of the data.
 *
 * Return: none
 *
 **********************************************************************/
bool buildModbusPacket(uint8_t childAdd, uint8_t fCode, uint8_t *data, uint8_t dataLen)
{
    bool retVal = false;

    uint8_t mBusNdx = 0;
    uint8_t modbusBuf[40];
    uint16_t crcTemp;

    modbusBuf[mBusNdx++] = childAdd;
    modbusBuf[mBusNdx++] = fCode;
    modbusBuf[mBusNdx++] = dataLen;
    if (dataLen > 0)
    {
        for (int i = 0; i < dataLen; i++)
        {
            modbusBuf[mBusNdx++] = data[i];
        }
    }
    crcTemp = generateCRC16(modbusBuf, mBusNdx);
    modbusBuf[mBusNdx++] = (uint8_t) (crcTemp & 0xFF);
    modbusBuf[mBusNdx++] = (uint8_t) ((crcTemp >> 8) & 0xFF);

    reStartModbusTask();

    if (UART_Transmit_IT(&rs485uart, modbusBuf, mBusNdx) == HAL_OK)
   //if (HAL_UART_Transmit_IT(&rs485uart, modbusBuf, mBusNdx) == HAL_OK)
    {
        retVal = true;
    }
    return (retVal);
}

/* Create a random address within a range to use for connecting
 *  to the controller.
 */
void createRandomModbusAddress(void)
{
    srand(uwTick);
    uint8_t newAdd = (uint8_t) (rand() % (MODBUS_MAX_NON_RANDOM_ADDRESS +1));
    mbVars.mbAddress = newAdd + MODBUS_MAX_NON_RANDOM_ADDRESS;
}

/* Set the MODBUS address for this device.
 */
void setDeviceAddress(uint8_t add)
{
    mbVars.mbAddress = add;
}

/* Get the MODBUS address for this device.
 */
uint8_t getDeviceAddress(void)
{
    return (mbVars.mbAddress);
}


/* Check to see if this device has been configured with an address from the controller
 *   A configured address will be between 1 and MODBUS_MAX_NON_RANDOM_ADDRESS
 */
bool isDeviceAddressInitialized(void)
{
    bool retVal = true;
    if ((mbVars.mbAddress == MODBUS_UNITIALIZED_ADDRESS) || (mbVars.mbAddress >= MODBUS_MAX_NON_RANDOM_ADDRESS))
    {
        retVal = false;
    }
    return (retVal);
}

/* Generate a 16 bit MODBUS CRC
 *
 */
uint16_t generateCRC16(uint8_t *buf, uint16_t length)
{
    startCrc();
    for (uint16_t pos = 0; pos < length; pos++)
    {
        iterateOneByteForCrc(buf[pos]);
    }
    // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
    return nCrc;
}

/* Start the CRC calculation
 *  For tasks that need to calculate the CRC one character at a time, Call
 *  this function to to start the process
 */
void startCrc(void)
{
    nCrc = 0xFFFF;
}

/* Call this function to process only one byte of the data
 *   for the CRC.
 */
void iterateOneByteForCrc(uint8_t nByte)
{
    nCrc ^= (uint16_t) nByte;   // XOR byte into least sig. byte of crc

    for (uint8_t bit = 8; bit != 0; bit--)
    {    // Loop over each bit
        if ((nCrc & 0x0001) != 0)
        {   // If the LSB is set
            nCrc >>= 1;                    // Shift right and XOR 0xA001
            nCrc ^= 0xA001;
        } else
        {   // Else LSB is not set Just shift right
            nCrc >>= 1;
        }
    }
}

