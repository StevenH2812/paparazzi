/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/*
 * Steven_guided_ctrl.h
 *
 *  Created on: Sep 11, 2017
 *      Author: steven <stevenhelm@live.nl>
 */

#include <math.h>
#include "modules/relativelocalizationfilter/relativelocalizationfilter.h"
#include "math/pprz_algebra_int.h"
#include "navigation.h"
#include "autopilot.h"
#include "../../firmwares/rotorcraft/guidance/guidance_h.h"
#include "../../firmwares/rotorcraft/guidance/guidance_v.h"
#include "generated/airframe.h"
#include "std.h"
#include "Steven_guided_ctrl.h"
// // ABI messages
#include "subsystems/abi.h"

static abi_event uwb_ev;

static float rec_velx = 0.0;
static float rec_vely = 0.0;
static float rec_range = 10.0;
static float relxcom = 0.0;
static float relycom = 3.0;
static float pgain = 0.5;

static pthread_mutex_t ekf_mutex;

static void keepBounded(float bound);
static void uwb_cb(uint8_t sender_id __attribute__((unused)),
		uint8_t ac_id, float range, float trackedVx, float trackedVy, float trackedh);


static void uwb_cb(uint8_t sender_id __attribute__((unused)),
		uint8_t ac_id, float range, float trackedVx, float trackedVy, float trackedh){
	rec_velx = trackedVx;
	rec_vely = trackedVy;
	rec_range = range;

}

void guided_ctrl_init(void){
	AbiBindMsgUWB(ABI_BROADCAST, &uwb_ev, uwb_cb); // Subscribe to the ABI RSSI messages
}

void guided_ctrl_per(void){

}

bool checkVicinity(void){
	return rec_range<1;
}

bool trackRelPos(void){

	bool temp = true;
	temp &= guidance_v_set_guided_z(-1.0);
	pthread_mutex_lock(&ekf_mutex);
	float relx = ekf[0].X[0];
	float rely = ekf[0].X[1];
	pthread_mutex_unlock(&ekf_mutex);
	float relxerr = relx-relxcom; //positive error means VX must increase
	float relyerr = rely-relycom; // positive error means VY must increase
	float vxcommand = pgain*relxerr;
	float vycommand = pgain*relyerr;
	temp &= guidance_h_set_guided_vel(vxcommand,vycommand);

}



bool trackVelocity(void){
	keepBounded(2.0);
	bool temp = true;
	temp &= guidance_v_set_guided_z(-1.0);
	temp &= guidance_h_set_guided_vel(rec_velx,rec_vely);
	return !temp; // Returning FALSE means in the flight plan that the function executed successfully.
}

bool hoverGuided(void){
	bool temp = true;
	temp &= guidance_v_set_guided_z(-1.0);
	temp &= guidance_h_set_guided_vel(0.0,0.0);
	return !temp; // Returning FALSE means in the flight plan that the function executed successfully.
}

bool setForwardVelocity(float velx){
	bool temp = true;
	temp &= guidance_v_set_guided_z(-1.0);
	temp &= guidance_h_set_guided_vel(velx,0);
	return !temp;
}

bool stopFlying(void){
	bool temp = true;
	temp &= guidance_v_set_guided_z(-1.0);
	temp &= guidance_h_set_guided_vel(0.0,0.0);
	return !temp;
}

bool goLand(void){
	bool temp = true;
	temp &= guidance_v_set_guided_vz(0.05);
	temp &= guidance_v_set_guided_z(0.0);
	temp &= guidance_h_set_guided_vel(0.0,0.0);
	return !temp;
}

static void keepBounded(float bound){
	if (abs(rec_velx)>bound){
		rec_velx = rec_velx / (abs(rec_velx)*bound);
	}
	if (abs(rec_vely)>bound){
		rec_vely = rec_vely / (abs(rec_vely)*bound);
	}
}
