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

#include <stdio.h>

// Serial Port
#include "mcu_periph/uart.h"
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






// Downlink
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

#include "led.h"


struct NedCoor_f current_pos;
struct NedCoor_f current_speed;
struct NedCoor_f current_accel;
struct FloatEulers current_angles;

int32_t globalcounter = 0;

// Module data
struct DataStruct {
	//  uint8_t mode; ///< 0 = straight, 1 =  right, 2 = left, ...
	uint8_t decode_cnt;
	uint8_t bin[10];
	uint8_t timeout;
};

struct DataStruct serial_data;




static float range_float = 0.0;

static void send_range_pos(struct transport_tx *trans, struct link_device *dev);
static void send_range_pos(struct transport_tx *trans, struct link_device *dev){
	current_pos = *stateGetPositionNed_f();
	current_speed = *stateGetSpeedNed_f();
	current_accel = *stateGetAccelNed_f();
	current_angles = *stateGetNedToBodyEulers_f();
	//printf("range, x, y, z: %f, %f, %f, %f\n",range_float,current_pos.x,current_pos.y,current_pos.z);
	pprz_msg_send_RANGE_POS(trans,dev,AC_ID,&range_float,&current_pos.x,&current_pos.y,&current_pos.z,&current_speed.x,&current_speed.y,&current_speed.z,&current_accel.x,&current_accel.y,&current_accel.z,&current_angles.phi,&current_angles.theta,&current_angles.psi);
}


static int serial_parse(uint8_t c);
static int serial_parse(uint8_t c)
{


	//return -1;

    serial_data.timeout = 20;

    //printf("%i: %i\n",globalcounter,c);
    //globalcounter++;
    //return -1;

	// arduino: printf("%05d\n"); in mm
	if (serial_data.decode_cnt > 9)
	{
		serial_data.decode_cnt = 0;
		return -1;
	}

  // Protocol is one byte only: store last instance
  serial_data.bin[serial_data.decode_cnt] = c;
  serial_data.decode_cnt++;

  if ((char)c == '\n')
  {
	  // decode: in de buffer staan nu 5 ASCII character van de afstand, bijvoorbeeld '0','0','0','2','2'
	  //Strings zijn char arrays die eindigen met 0 in C
	  /*serial_data.bin[5] = 0;
	  range_measurement = atoi((char*)serial_data.bin);*/

	  serial_data.bin[serial_data.decode_cnt-1] = 0;
	  range_float = atof((char*)serial_data.bin);
	  //printf("%f\n",range_float);
	  serial_data.decode_cnt = 0;


	  return 1;

  }


  return 0;
}


void decawave_serial_init(void)
{
  // Do nothing
	  serial_data.decode_cnt = 0;
	  serial_data.timeout = 0;
	  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RANGE_POS, send_range_pos);
	  //SerialUartSetBaudrate(SERIAL_BAUD);
	  //uart_periph_set_baudrate(&uart1,B9600);
}
void decawave_serial_periodic(void)
{
	//printf("hello world\n");
	int newrange = 0;

	// Read Serial
	while (SerialChAvailable()) {
		if (serial_parse(SerialGetch()) > 0)
			newrange++;
	}

	if (serial_data.timeout <= 0)
		return;

	if (newrange > 0)
	{
		float a,b;

		a = range_float;
		b = 0.0;

		// Results
		//DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice, strlen(buf), (uint8_t*) buf);
		//DOWNLINK_SEND_ESTIMATOR(DefaultChannel, DefaultDevice, &a, &b);
	}

	serial_data.timeout --;


}





