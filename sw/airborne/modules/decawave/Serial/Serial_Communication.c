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
PRINT_CONFIG_VAR(STEREO_UART)
PRINT_CONFIG_VAR(STEREO_BAUD)

// define coms link for stereocam
#define STEREO_PORT   (&((STEREO_UART).device))
struct link_device *xdev = STEREO_PORT;

#define StereoGetch() STEREO_PORT ->get_byte(STEREO_PORT->periph)
#define StereoSend1(c) STEREO_PORT->put_byte(STEREO_PORT->periph, 0, c)
#define StereoUartSend1(c) StereoSend1(c)
#define StereoSend(_dat,_len) { for (uint8_t i = 0; i< (_len); i++) StereoSend1(_dat[i]); };
#define StereoUartSetBaudrate(_b) uart_periph_set_baudrate(STEREO_PORT, _b);
#define StereoChAvailable()(xdev->char_available(xdev->periph))






// Downlink
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

#include "led.h"





// Module data
struct AvoidNavigationStruct {
	//  uint8_t mode; ///< 0 = straight, 1 =  right, 2 = left, ...
	uint8_t decode_cnt;
	uint8_t stereo_bin[8];
	uint8_t timeout;
};

struct AvoidNavigationStruct avoid_navigation_data;



static int stereo_range_measurement = 0;



static int stereo_parse(uint8_t c);
static int stereo_parse(uint8_t c)
{

	//printf("%c",c);
	//return;
    avoid_navigation_data.timeout = 20;

	// arduino: printf("%05d\n"); in mm
	if ((c == '\n') || (avoid_navigation_data.decode_cnt > 7))
	{
		avoid_navigation_data.decode_cnt = 0;
		return -1;
	}
  // Protocol is one byte only: store last instance
  avoid_navigation_data.stereo_bin[avoid_navigation_data.decode_cnt] = c;
  avoid_navigation_data.decode_cnt++;

  if (avoid_navigation_data.decode_cnt == 5)
  {
	  // decode: in de buffer staan nu 5 ASCII character van de afstand, bijvoorbeeld '0','0','0','2','2'
	  //Strings zijn char arrays die eindigen met 0 in C
	  avoid_navigation_data.stereo_bin[5] = 0;
	  stereo_range_measurement = atoi((char*)avoid_navigation_data.stereo_bin);
	  avoid_navigation_data.decode_cnt = 0;
	  return 1;

  }


  return 0;
}


void stereocam_droplet_init(void)
{
  // Do nothing
	  avoid_navigation_data.decode_cnt = 0;
	  avoid_navigation_data.timeout = 0;

	  //uart_periph_set_baudrate(&uart1,B9600);
}
void stereocam_droplet_periodic(void)
{
	int newrange = 0;

	// Read Serial
	while (StereoChAvailable()) {
		if (stereo_parse(StereoGetch()) > 0)
			newrange++;
	}

	if (avoid_navigation_data.timeout <= 0)
		return;

	if (newrange > 0)
	{
		uint8_t a,b;
		char buf[256];
		sprintf(buf,"Range = %d", stereo_range_measurement);

		a = stereo_range_measurement;
		b=0;
		// Results
		//DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice, strlen(buf), (uint8_t*) buf);
		DOWNLINK_SEND_DEBUG_MCU_LINK(DefaultChannel, DefaultDevice, &a, &b,&b);
	}

	avoid_navigation_data.timeout --;


}






