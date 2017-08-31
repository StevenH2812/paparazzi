/*
 * Serial_Communication.h
 *
 *  Created on: Jul 25, 2017
 *      Author: steven
 */

#ifndef SW_AIRBORNE_MODULES_DECAWAVE_SERIAL_SERIAL_COMMUNICATION_H_
#define SW_AIRBORNE_MODULES_DECAWAVE_SERIAL_SERIAL_COMMUNICATION_H_

#include <stdint.h>
#include <stdbool.h>

#define MAX_MESSAGE 10
#define IN_MESSAGE_SIZE 4
#define OUT_MESSAGE_SIZE 7
#define END_MARKER 255
#define SPECIAL_BYTE 253
#define START_MARKER 254
#define IN_MESSAGES 3

#define MAX_NODES 5

// Message types
#define VX 0
#define VY 1
#define Z 2
#define R 3


extern void decawave_serial_init(void);
extern void decawave_serial_periodic(void);
extern void decawave_serial_event(void);

extern void getSerialData(void);
extern void sendFloat(uint8_t msgtype, float outfloat);
extern float receiveFloat(uint8_t msgtype);





#endif /* SW_AIRBORNE_MODULES_DECAWAVE_SERIAL_SERIAL_COMMUNICATION_H_ */
