/*
 * Serial_Communication.c
 *
 *  Created on: Jul 25, 2017
 *      Author: steven
 */

/*
 * Copyright (C) C. DW
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/decawave/Serial/Serial_Communication.h"
 * @author S. vd H, C. DW
 *
 */

#include "modules/decawave/Serial/Serial_Communication.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/radio_control.h"
#include "state.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "mcu_periph/uart.h"
#include <stdio.h>


PRINT_CONFIG_VAR(SERIAL_UART)
PRINT_CONFIG_VAR(SERIAL_BAUD)

// define coms link for stereocam
#define SERIAL_PORT   (&((SERIAL_UART).device))
struct link_device *xdev = SERIAL_PORT;

#define SerialGetch() SERIAL_PORT ->get_byte(SERIAL_PORT->periph)
#define SerialSend1(c) SERIAL_PORT->put_byte(SERIAL_PORT->periph, 0, c)
#define SerialUartSend1(c) SerialSend1(c)
#define SerialSend(_dat,_len) { for (uint8_t i = 0; i< (_len); i++) SerialSend1(_dat[i]); };
#define SerialUartSetBaudrate(_b) uart_periph_set_baudrate(SERIAL_PORT, _b);
#define SerialChAvailable()(xdev->char_available(xdev->periph))
#define SerialSendNow() uart_send_message(SERIAL_PORT->periph,0)

struct nodeState{
	uint8_t nodeAddress;
	float vx;
	float vy;
	float z;
	float r;
	bool stateUpdated[NODE_STATE_SIZE];
};

struct MessageIn{
	uint8_t type;
	uint8_t nodeAddress;
	uint8_t msg[IN_MESSAGE_SIZE];
};

static uint8_t _bytesRecvd = 0;
static uint8_t _dataSentNum = 0;
static uint8_t _dataRecvCount = 0;


static uint8_t _tempBuffer[MAX_MESSAGE];
static uint8_t _tempBuffer2[MAX_MESSAGE];
static uint8_t _recvBuffer[FLOAT_SIZE];

static uint8_t _dataSendCount = 0;
static uint8_t _dataTotalSend = 0;

static bool _inProgress = false;
static bool _startFound = false;
static bool _allReceived = false;

static uint8_t _varByte = 0;

static bool _bigEndian = false;



static struct nodeState _states[DIST_NUM_NODES];

struct NedCoor_f current_pos;
struct NedCoor_f current_speed;
struct NedCoor_f current_accel;
struct FloatEulers current_angles;

float range_float = 0.0;


static void decodeHighBytes(void);
static void encodeHighBytes(uint8_t* sendData, uint8_t msgSize);
static void checkBigEndian(void);
static void send_range_pos(struct transport_tx *trans, struct link_device *dev);
static void testSend();
static void testReceive();
static void handleNewStateValue(uint8_t nodeIndex, uint8_t msgType, float value);
static void setNodeStatesFalse();
static void checkStatesUpdated();



void decawave_serial_init(void)
{
	setNodeStatesFalse();
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RANGE_POS, send_range_pos);
	//SerialUartSetBaudrate(SERIAL_BAUD);
	//uart_periph_set_baudrate(&uart1,B9600);
}

void decawave_serial_periodic(void)
{

	current_speed = *stateGetSpeedNed_f();
	current_pos = *stateGetPositionNed_f();

	sendFloat(VX,current_speed.x);
	sendFloat(VY,current_speed.y);
	sendFloat(Z,current_pos.z);

}

void decawave_serial_event(void){
	getSerialData();
	checkStatesUpdated();

}

static void setNodeStatesFalse(){
	for (uint8_t i = 0; i < DIST_NUM_NODES; i++){
		for (uint8_t j = 0; j < NODE_STATE_SIZE; j++){
			_states[i].stateUpdated[j] = false;
		}
	}
}

/**
 * This function checks if all the states of all the distant nodes have at least once been updated.
 * If all the states are updated, then do something with it! AKA CALLBACK TO MARIO
 */
static void checkStatesUpdated(){
	bool checkbool = true;
	for (uint8_t i = 0; i < DIST_NUM_NODES; i++){
		for (uint8_t j = 0; j < NODE_STATE_SIZE; j++){
			checkbool = checkbool && _states[i].stateUpdated[j];
		}
	}
	if (checkbool){
		// ------- CALLBACK TO MARIO ------------------//
		printf("All states were updated!");
		setNodeStatesFalse();
	}
}

void testSend(){
	//float testerfloat = 3.5;
	//uint8_t testerbyte[4] = {2,3,4,5};
	//memcpy(&testerbyte,&testerfloat,sizeof(float));
	/*
	if (testervar>254){
		testervar = 0;
	}
	*/
	/*
	for(int i=0;i<4;i++){
		uart_put_byte(&SERIAL_UART,0,testerbyte[i]);
		SerialSend1(testerbyte[i]);
	}*/
	//unsigned char tosend[2] = {1,2};

	SerialSend1(1);
	SerialSend1(2);
	SerialSend1(3);
	SerialSend1(4);
	//SerialSend(tosend,2);
	//uart_put_buffer(&SERIAL_UART,0,&tosend,2);
/*	SerialSend1(tosend);
	SerialSend1(tosend);
	SerialSend1(tosend);
	SerialSend1(tosend);*/
}

void testReceive(){
	while (SerialChAvailable()){
		printf("Received byte: %u\n",SerialGetch());
	}
}


static void send_range_pos(struct transport_tx *trans, struct link_device *dev){
	current_pos = *stateGetPositionNed_f();
	current_speed = *stateGetSpeedNed_f();
	current_accel = *stateGetAccelNed_f();
	current_angles = *stateGetNedToBodyEulers_f();
	//printf("range, x, y, z: %f, %f, %f, %f\n",range_float,current_pos.x,current_pos.y,current_pos.z);
	pprz_msg_send_RANGE_POS(trans,dev,AC_ID,&range_float,&current_pos.x,&current_pos.y,&current_pos.z,&current_speed.x,&current_speed.y,&current_speed.z,&current_accel.x,&current_accel.y,&current_accel.z,&current_angles.phi,&current_angles.theta,&current_angles.psi);
}


/**
 * Function for receiving serial data.
 * Only receives serial data that is between the start and end markers. Discards all other data.
 * Stores the received data in _tempBuffer, and after decodes the high bytes and copies the final
 * message to the corresponding message in _messages.
 */
void getSerialData(void){
	while (SerialChAvailable()){
		_varByte = SerialGetch();
		if (_varByte == START_MARKER){
			_bytesRecvd = 0;
			_inProgress = true;
		}

		if (_inProgress){
			_tempBuffer[_bytesRecvd] = _varByte;
			_bytesRecvd++;
		}

		if (_varByte == END_MARKER){
			_inProgress = false;
			_allReceived = true;

			decodeHighBytes();
		}
	}

}

/**
 * Function for decoding the high bytes of received serial data and saving the message.
 * Since the start and end marker could also be regular payload bytes (since they are simply the values
 * 254 and 255, which could also be payload data) the payload values 254 and 255 have been encoded
 * as byte pairs 253 1 and 253 2 respectively. Value 253 itself is encoded as 253 0.
 *  This function will decode these back into values the original payload values.
 */
static void decodeHighBytes(void){
	_dataRecvCount = 0;
	float tempfloat;
	uint8_t thisAddress = _tempBuffer[1];
	uint8_t msgFrom = _tempBuffer[2];
	uint8_t msgType = _tempBuffer[3];
	uint8_t nodeIndex = msgFrom -1 - (uint8_t)(thisAddress<msgFrom);
	for (uint8_t i = 4; i<_bytesRecvd-1; i++){ // Skip the begin marker (0), this address (1), remote address (2), message type (3), and end marker (_bytesRecvd-1)
		_varByte = _tempBuffer[i];
		if (_varByte == SPECIAL_BYTE){
			i++;
			_varByte = _varByte + _tempBuffer[i];
		}
		if(_dataRecvCount<=FLOAT_SIZE){
			_recvBuffer[_dataRecvCount] = _varByte;
		}
		_dataRecvCount++;
	}
	if(_dataRecvCount==FLOAT_SIZE){
		memcpy(&tempfloat,&_recvBuffer,FLOAT_SIZE);
		handleNewStateValue(nodeIndex,msgType,tempfloat);
	}
}

/**
 * Function that is called when over the serial a new state value from a remote node is received
 */
static void handleNewStateValue(uint8_t nodeIndex, uint8_t msgType, float value){
	struct nodeState node = _states[nodeIndex];
	switch(msgType){
	case VX : node.vx=value; node.stateUpdated[VX] = true; break;
	case VY : node.vy=value; node.stateUpdated[VY] = true; break;
	case Z : node.z=value; node.stateUpdated[Z] = true; break;
	case R : node.r=value; node.stateUpdated[R] = true; break;
	}

}



/**
 * Function that will send a float over serial. The actual message that will be sent will have
 * a start marker, the message type, 4 bytes for the float, and the end marker.
 */
void sendFloat(uint8_t msgtype, float outfloat){

	uint8_t floatbyte[4];
	memcpy(floatbyte,&outfloat,4);
	encodeHighBytes(floatbyte,4);
	SerialSend1(START_MARKER);
	SerialSend1(msgtype);
	SerialSend(_tempBuffer2,_dataTotalSend);
	SerialSend1(END_MARKER);

	/*
	SerialSend1(0);
	SerialSend1(0);
	SerialSend1(96);
	SerialSend1(64);
	SerialSend1(END_MARKER);
	SerialSendNow();
	printf("tried to send float\n");


	uart_put_byte(&SERIAL_UART,0,START_MARKER);
	uart_put_byte(&SERIAL_UART,0,msgtype);
	uart_put_byte(&SERIAL_UART,0,0);
	uart_put_byte(&SERIAL_UART,0,0);
	uart_put_byte(&SERIAL_UART,0,96);
	uart_put_byte(&SERIAL_UART,0,64);
	uart_put_byte(&SERIAL_UART,0,END_MARKER);
	printf("tried to send float\n");
	*/


}

/**
 * Function that encodes the high bytes of the serial data to be sent.
 * Start and end markers are reserved values 254 and 255. In order to be able to send these values,
 * the payload values 253, 254, and 255 are encoded as 2 bytes, respectively 253 0, 253 1, and 253 2.
 */
static void encodeHighBytes(uint8_t* sendData, uint8_t msgSize){
	_dataSendCount = msgSize;
	_dataTotalSend = 0;
	for (uint8_t i = 0; i < _dataSendCount; i++){
		if (sendData[i] >= SPECIAL_BYTE){
			_tempBuffer2[_dataTotalSend] = SPECIAL_BYTE;
			_dataTotalSend++;
			_tempBuffer2[_dataTotalSend] = sendData[i] - SPECIAL_BYTE;
		}
		else{
			_tempBuffer2[_dataTotalSend] = sendData[i];
		}
		_dataTotalSend++;
	}
}

/**
 * Function to check the endianness of the system
 */
static void checkBigEndian(void)
{
    union {
        uint32_t i;
        char c[4];
    } un = {0x01020304};

    _bigEndian = un.c[0] == 1;
}





