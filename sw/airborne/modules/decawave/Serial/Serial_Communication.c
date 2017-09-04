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

/*
struct MessageIn{
	uint8_t type;
	uint8_t nodeAddress;
	uint8_t msg[IN_MESSAGE_SIZE];
};*/

static uint8_t _bytesRecvd = 0;
//static uint8_t _dataSentNum = 0;
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
static void handleNewStateValue(uint8_t nodeIndex, uint8_t msgType, float value);
static void setNodeStatesFalse(uint8_t index);
static void setAllNodeStatesFalse();
static void checkStatesUpdated();
static void uwbStateUpdateCallback(uint8_t sender_id __attribute__((unused)),
	uint8_t ac_id, int8_t source_strength, int8_t rssi);
//static void initNodes();



/**
 * Initialization function. Initializes nodes (which contain data of other bebops) and registers a periodic message.
 */
void decawave_serial_init(void)
{
	//initNodes();
	setAllNodeStatesFalse();
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RANGE_POS, send_range_pos);
	//SerialUartSetBaudrate(SERIAL_BAUD);
	//uart_periph_set_baudrate(&uart1,B9600);
}

/**
 * This function periodically sends state data over the serial (which is received by the arduino)
 */
void decawave_serial_periodic(void)
{

	current_speed = *stateGetSpeedNed_f();
	current_pos = *stateGetPositionNed_f();

	sendFloat(VX,current_speed.x);
	sendFloat(VY,current_speed.y);
	sendFloat(Z,current_pos.z);




}

/**
 * Event function currently checks for serial data and whether an update of states is available for a distant drone.
 * If these cases are true, then actions are taken.
 */
void decawave_serial_event(void){
	getSerialData();
	checkStatesUpdated();


}

/**
 * Helper function that sets the boolean that tells whether a remote drone has a new state update to false.
 */
static void setNodeStatesFalse(uint8_t index){
	for (uint8_t j = 0; j < NODE_STATE_SIZE; j++){
		_states[index].stateUpdated[j] = false;
	}
}

/**
 * Helper function that sets the booleans to false for all the remote drones (DIST_NUM_NODES)
 */
static void setAllNodeStatesFalse(){
	for (uint8_t i = 0; i < DIST_NUM_NODES; i++){
		setNodeStatesFalse(i);
	}
}

/**
 * This function checks if all the states of all the distant nodes have at least once been updated.
 * If all the states are updated, then do something with it! AKA CALLBACK TO MARIO
 */
static void checkStatesUpdated(){
	bool checkbool;
	for (uint8_t i = 0; i < DIST_NUM_NODES; i++){
		checkbool = true;
		for (uint8_t j = 0; j < NODE_STATE_SIZE; j++){
			checkbool = checkbool && _states[i].stateUpdated[j];
		}
		if (checkbool){
			// ------- CALLBACK TO MARIO ------------------//
			printf("States for drone %i: r = %f, vx = %f, vy = %f, z = %f \n",i,_states[i].r,_states[i].vx,_states[i].vy,_states[i].z);
			setNodeStatesFalse(i);
		}
	}

}


static void send_range_pos(struct transport_tx *trans, struct link_device *dev){
	current_pos = *stateGetPositionNed_f();
	current_speed = *stateGetSpeedNed_f();
	current_accel = *stateGetAccelNed_f();
	current_angles = *stateGetNedToBodyEulers_f();
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
	struct nodeState *node = &_states[nodeIndex];
	switch(msgType){
	case VX : node->vx=value; node->stateUpdated[VX] = true; break;
	case VY : node->vy=value; node->stateUpdated[VY] = true; break;
	case Z  : node->z=value ; node->stateUpdated[Z]  = true; break;
	case R  : node->r=value ; node->stateUpdated[R]  = true; break;
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



static void uwbStateUpdateCallback(uint8_t sender_id __attribute__((unused)),float vx, float vy, float range){
	int i = -1; // Initialize the index of all tracked drones (-1 for null assumption of no drone found).

	// Check if a new aircraft ID is present, if it's a new ID we start a new EKF for it.
	if (( !array_find_int(NUAVS-1, IDarray, ac_id, &i))  // If yes, a new drone is found.
		   && (nf < NUAVS-1))  // If yes, the amount of drones does not exceed the maximum.
	{
		IDarray[nf] = ac_id; 				// Store ID in an array (logging purposes)
		ekf_filter_new(&ekf[nf]); 			// Initialize an EKF filter for the newfound drone

		// Set up the Q and R matrices and all the rest
		// Weights are based on:
		// Coppola et al, "On-board Communication-based Relative Localization for Collision Avoidance in Micro Air Vehicle teams", 2017
		fmat_scal_mult(EKF_N,EKF_N, ekf[nf].Q, pow(0.5,2.0), ekf[nf].Q);
		fmat_scal_mult(EKF_M,EKF_M, ekf[nf].R, pow(SPEEDNOISE,2.0), ekf[nf].R);
		ekf[nf].Q[0]   	   = 0.01; // Reccomended 0.01 to give this process a high level of trust
		ekf[nf].Q[EKF_N+1] = 0.01;
		ekf[nf].R[0]   = pow(RSSINOISE,2.0);

		// Initialize the states
		// Initial position cannot be zero or the filter will divide by zero on initialization
		ekf[i].X[0] = 1.0; // Relative position North
		ekf[i].X[1] = 1.0; // Relative position East
		// The other variables can be initialized at 0
		ekf[i].X[2] = 0.0; // Own Velocity North
		ekf[i].X[3] = 0.0; // Own Velocity East
		ekf[i].X[4] = 0.0; // Relative velocity North
		ekf[i].X[5] = 0.0; // Relative velocity East
		ekf[i].X[6] = 0.0; // Height difference

		ekf[nf].dt       = 0.2;  // Initial assumption for time difference between messages (STDMA code runs at 5Hz)
		model[nf].Pn     = -63;  // Expected RSSI at 1m (based on experience)
		model[nf].gammal = 2.0;	 // Expected Space-loss parameter (based on free space assumption)
		nf++; 					 // Number of filter is present is increased
	}
	// Else, if we do recognize the ID, then we can update the measurement message data
	else if ((i != -1) || (nf == (NUAVS-1)) )
	{
		RSSIarray[i] = (float)rssi; // Store RSSI in array (for logging purposes)

		// Get own velocities
		float ownVx = stateGetSpeedEnu_f()->y; // Velocity North in NED
		float ownVy = stateGetSpeedEnu_f()->x; // Velocity East in NED
		// Bind to realistic amounts to avoid occasional spikes/NaN/inf errors
		keepbounded(&ownVx,-2.0,2.0);
		keepbounded(&ownVy,-2.0,2.0);

		// Make the filter only in Guided mode (flight).
		// This is because it is best for the filter should only start once the drones are in motion,
		// otherwise it might diverge while drones are not moving.
		if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED)
		{
			ekf[i].dt = (get_sys_time_usec() - now_ts[i])/pow(10,6); // Update the time between messages

			// Get the velocity in NED for the tracked aircraft
			float trackedVx, trackedVy;
			polar2cart(acInfoGetGspeed(ac_id), acInfoGetCourse(ac_id), &trackedVx, &trackedVy); // get North and East velocities (m/s)
			// As for own velocity, bind to realistic amounts to avoid occasional spikes/NaN/inf errors
			keepbounded(&trackedVx,-2.0,2.0);
			keepbounded(&trackedVy,-2.0,2.0);

			// Construct measurement vector Y for EKF using the latest data obtained.
			// Y = [RSSI owvVx ownVy trackedVx trackedVy dh], EKF_M = 6 as defined in discreteekf.h
			float Y[EKF_M];
			Y[0] = (float)rssi; 	//RSSI measurement
			Y[1] = ownVx; 	   		// Own velocity North (NED frame)
			Y[2] = ownVy;			// Own velocity East  (NED frame)
			Y[3] = trackedVx;  		// Velocity of other drone Norht (NED frame)
			Y[4] = trackedVy;		// Velocity of other drone East  (NED frame)
			Y[5] = acInfoGetPositionUtm_f(ac_id)->alt - stateGetPositionEnu_f()->z;  // Height difference

			// Run the steps of the EKF, but only if velocity difference is significant (to filter out minimal noise)
			if (  sqrt( pow(Y[1]-Y[3],2) + pow(Y[2]-Y[4],2) ) > 0.05 )
			{
				ekf_filter_predict(&ekf[i], &model[i]); // Prediction step of the EKF
				ekf_filter_update(&ekf[i], Y);	// Update step of the EKF
			}
		}

		now_ts[i] = get_sys_time_usec();  // Store latest time

	}
};

