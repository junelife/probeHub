/*
 * modbus.h
 *
 * Copyright © Weber Stephen Products LLC., 2022 All Rights Reserved No part of this file may be
 * distributed, reproduced, or used by entities other than Stephen Products LLC. without express
 * written permission. All Rights Reserved UNPUBLISHED, LICENSED SOFTWARE IS CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Weber Stephen Products LLC.
 *
 *  Created on: Apr 14, 2022
 *      Author: ronso
 */

#ifndef INC_MODBUS_H_
#define INC_MODBUS_H_
#include "../Shared/modbusShare.h"

#define MODBUS_MAX_DATA_LENGTH 40
#define MODBUS_MAX_CHILD_ADDRESS 0x60
#define MODBUS_GET_RX_LEN_FROM_DEVICE 0xFF


#define MODBUS_ADD_TEMP 1

typedef enum modbus_message_names
{
    MB_MSG_GET_TEMP = 0,
    MB_MSG_END
} mbMsgName;

typedef struct modbus_messages
{
    mbMsgName name;
    uint8_t    address;
    uint8_t    function;
    uint8_t    *data;
    uint8_t    txDataLen;
    uint8_t    rxDataLen;
} modbusMsgs;


typedef struct
{
    uint8_t  mbAdd;
    uint8_t  fCode;
    uint8_t  data[MODBUS_MAX_DATA_LENGTH];
    uint8_t  dataLen;
    uint16_t crc;
    uint16_t cmdNdx;
    uint8_t  crcCheck           :1;
    uint8_t  getRxLenFromDevice :1;
} t_modbus_packet_struct;

extern t_modbus_packet_struct modbusStruct;

bool modbusTask(void);
void initializeModbusTask(void);
bool buildModbusPacket(uint8_t, uint8_t, uint8_t *, uint8_t);
bool sendModbusMessage(mbMsgName);
void createRandomModbusAddress(void);
void setDeviceAddress(uint8_t);
uint8_t getDeviceAddress(void);
bool isDeviceAddressInitialized(void);
void changeStateModbusTask(void);
void enableModbusRx(void);
void disableModbusRx(void);
void setUpgradeDataLen(uint16_t);
bool didUpgradeAlreadyExecute(void);

#endif /* INC_MODBUS_H_ */
