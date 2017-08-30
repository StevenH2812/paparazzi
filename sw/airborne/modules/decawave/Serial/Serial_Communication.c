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



struct MessageIn{
	uint8_t type;
	uint8_t msg[IN_MESSAGE_SIZE];
};

static uint8_t _bytesRecvd = 0;
static uint8_t _dataSentNum = 0;
static uint8_t _dataRecvCount = 0;

//static uint8_t _dataRecvd[MAX_MESSAGE];
static uint8_t _dataSend[MAX_MESSAGE];
static uint8_t _tempBuffer[MAX_MESSAGE];

static uint8_t _dataSendCount = 0;
static uint8_t _dataTotalSend = 0;

static bool _inProgress = false;
static bool _startFound = false;
static bool _allReceived = false;

static uint8_t _varByte = 0;

static bool _bigEndian = false;


static struct MessageIn _receiveMessages[IN_MESSAGES];
//static struct MessageOut _sendMessages[OUT_MESSAGES];
struct NedCoor_f current_pos;
struct NedCoor_f current_speed;
struct NedCoor_f current_accel;
struct FloatEulers current_angles;

float vx = 0.0;
float vy = 10.1;
float z = 20.17;
float range_float = 0.0;


static void decodeHighBytes(void);
static void encodeHighBytes(uint8_t* sendData, uint8_t msgSize);
static void checkBigEndian(void);
static int serial_parse(uint8_t c);
static void send_range_pos(struct transport_tx *trans, struct link_device *dev);
static void testSend();
static void testReceive();

static uint8_t testervar = 0;

void decawave_serial_init(void)
{
	  //register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RANGE_POS, send_range_pos);
	  //SerialUartSetBaudrate(SERIAL_BAUD);
	  //uart_periph_set_baudrate(&uart1,B9600);
}
void decawave_serial_periodic(void)
{
	testSend();
	testReceive();

/*
	sendFloat(VX,vx);
	sendFloat(VY,vy);
	sendFloat(Z,z);
	vx += 1;
	vy += 2;
	z += 3;
	/*
	current_pos = *stateGetPositionNed_f();
	current_speed = *stateGetSpeedNed_f();
	float rfloat;
	rfloat = 112.0;
	for (int i = 0; i<IN_MESSAGES;i++){
		rfloat = receiveFloat(i);
		printf("Received float is: %f\n",rfloat);
	}

	float rfloat;
	for (int i = 0; i<IN_MESSAGES;i++){
		rfloat = receiveFloat(i);
		printf("Received float %i is: %f\n",i,rfloat);
	}
	*/


}

void decawave_serial_event(void){
	//getSerialData();

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
	SerialSend1(1);
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
	uint8_t msgType = _tempBuffer[1];
	for (uint8_t i = 2; i<_bytesRecvd-1; i++){ // Skip the begin marker (0), message type (1), and end marker (_bytesRecvd-1)
		_varByte = _tempBuffer[i];
		if (_varByte == SPECIAL_BYTE){
			i++;
			_varByte = _varByte + _tempBuffer[i];
		}
		_receiveMessages[msgType].msg[_dataRecvCount] = _varByte;
		_dataRecvCount++;
	}
}

/**
 * Function used to receive a float with a certain message ID
 */
float receiveFloat(uint8_t msgtype){
	float tempfloat;
	memcpy(&tempfloat,&_receiveMessages[msgtype].msg,4);
	return tempfloat;
}

/**
 * Function that will send a float over serial. The actual message that will be sent will have
 * a start marker, the message type, 4 bytes for the float, and the end marker.
 */
void sendFloat(uint8_t msgtype, float outfloat){
	/*
	//uint8_t floatbyte[4];
	//memcpy(floatbyte,&outfloat,4);
	//encodeHighBytes(floatbyte,4);
	SerialSend1(START_MARKER);
	SerialSend1(msgtype);
	//SerialSend(_tempBuffer,_dataTotalSend);
	SerialSend1(0);
	SerialSend1(0);
	SerialSend1(96);
	SerialSend1(64);
	SerialSend1(END_MARKER);
	SerialSendNow();
	//printf("tried to send float\n");
	 */

	uart_put_byte(&SERIAL_UART,0,START_MARKER);
	uart_put_byte(&SERIAL_UART,0,msgtype);
	uart_put_byte(&SERIAL_UART,0,0);
	uart_put_byte(&SERIAL_UART,0,0);
	uart_put_byte(&SERIAL_UART,0,96);
	uart_put_byte(&SERIAL_UART,0,64);
	uart_put_byte(&SERIAL_UART,0,END_MARKER);
	//printf("tried to send float\n");


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
			_tempBuffer[_dataTotalSend] = SPECIAL_BYTE;
			_dataTotalSend++;
			_tempBuffer[_dataTotalSend] = sendData[i] - SPECIAL_BYTE;
		}
		else{
			_tempBuffer[_dataTotalSend] = sendData[i];
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





