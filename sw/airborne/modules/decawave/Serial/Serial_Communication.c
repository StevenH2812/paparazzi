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
 * @file "modules/stereocam/droplet/stereocam_droplet.c"
 * @author C. DW
 *
 */

#include "modules/decawave/Serial/Serial_Communication.h"


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





// Module data
struct DataStruct {
	//  uint8_t mode; ///< 0 = straight, 1 =  right, 2 = left, ...
	uint8_t decode_cnt;
	uint8_t bin[8];
	uint8_t timeout;
};

struct DataStruct serial_data;



static int range_measurement = 0;



static int stereo_parse(uint8_t c);
static int stereo_parse(uint8_t c)
{

	//printf("%c",c);
	//return;
    serial_data.timeout = 20;

	// arduino: printf("%05d\n"); in mm
	if ((c == '\n') || (serial_data.decode_cnt > 7))
	{
		serial_data.decode_cnt = 0;
		return -1;
	}
  // Protocol is one byte only: store last instance
  serial_data.bin[serial_data.decode_cnt] = c;
  serial_data.decode_cnt++;

  if (serial_data.decode_cnt == 5)
  {
	  // decode: in de buffer staan nu 5 ASCII character van de afstand, bijvoorbeeld '0','0','0','2','2'
	  //Strings zijn char arrays die eindigen met 0 in C
	  serial_data.bin[5] = 0;
	  range_measurement = atoi((char*)serial_data.bin);
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

	  //uart_periph_set_baudrate(&uart1,B9600);
}
void decawave_serial_periodic(void)
{
	int newrange = 0;

	// Read Serial
	while (SerialChAvailable()) {
		if (stereo_parse(SerialGetch()) > 0)
			newrange++;
	}

	if (serial_data.timeout <= 0)
		return;

	if (newrange > 0)
	{
		uint8_t a,b;
		char buf[256];
		sprintf(buf,"Range = %d", range_measurement);

		a = range_measurement;
		b=0;
		// Results
		//DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice, strlen(buf), (uint8_t*) buf);
		DOWNLINK_SEND_DEBUG_MCU_LINK(DefaultChannel, DefaultDevice, &a, &b,&b);
	}

	serial_data.timeout --;


}






