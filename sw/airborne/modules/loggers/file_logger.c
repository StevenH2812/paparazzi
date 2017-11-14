/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/loggers/file_logger.c
 *  @brief File logger for Linux based autopilots
 */

#include "file_logger.h"

#include <stdio.h>
#include "std.h"

#include "subsystems/imu.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "state.h"

bool loggingStartedFnip = false;

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/video/usb
#endif

/** The file pointer */
static FILE *file_logger = NULL;

uint32_t logCounter = 0;

struct NedCoor_f posNED;
struct NedCoor_f speedNED;
struct NedCoor_f accelNED;
struct FloatEulers eulerAngles;
struct FloatRates bodyRates;
float speedDir;
float speedNorm;

/** Start the file logger and open a new file */
void file_logger_start(void)
{
  logCounter = 0;
  char filename[512];

  // Check for available files
  sprintf(filename, "%s/%05d.txt", STRINGIFY(FILE_LOGGER_PATH), logCounter);
  while ((file_logger = fopen(filename, "r"))) {
    fclose(file_logger);

    logCounter++;
    sprintf(filename, "%s/%05d.txt", STRINGIFY(FILE_LOGGER_PATH), logCounter);
  }

  file_logger = fopen(filename, "w");

  if (file_logger != NULL) {
	  /*
    fprintf(
      file_logger,
      "counter,gyro_unscaled_p,gyro_unscaled_q,gyro_unscaled_r,accel_unscaled_x,accel_unscaled_y,accel_unscaled_z,mag_unscaled_x,mag_unscaled_y,mag_unscaled_z,COMMAND_THRUST,COMMAND_ROLL,COMMAND_PITCH,COMMAND_YAW,qi,qx,qy,qz\n"
    );*/
	  fprintf(file_logger,
			  "counter,time,posNEDx,posNEDy,posNEDz,speedNEDx,speedNEDy,speedNEDz,speedDir,speedNorm,accelNEDx,accelNEDy,accelNEDz,eulerPhi,eulerTheta,eulerPsi,ratep,rateq,rater,scaledp,scaledq,scaledr,scaledaccx,scaledaccy,scaledaccz,scaledmagx,caledmagy,scaledmagz\n");
  }
}

/** Stop the logger an nicely close the file */
void file_logger_stop(void)
{
  if (file_logger != NULL) {
    fclose(file_logger);
    file_logger = NULL;
  }
}

/** Log the values to a csv file */
void file_logger_periodic(void)
{
	posNED = *stateGetPositionNed_f();
	speedNED = *stateGetSpeedNed_f();
	accelNED = *stateGetAccelNed_f();
	speedDir = stateGetHorizontalSpeedDir_f();
	speedNorm = stateGetHorizontalSpeedNorm_f();
	eulerAngles = *stateGetNedToBodyEulers_f();
	bodyRates = *stateGetBodyRates_f();
	//printf("speedNED is %f, %f, %f, norm, dir is %f, %f, %f\n",speedNED.x,speedNED.y,speedNED.z,speedNorm,speedDir,DegOfRad(speedDir));
  if (file_logger == NULL || !loggingStartedFnip) {
    return;
  }
/*  struct Int32Quat *quat = stateGetNedToBodyQuat_i();

  fprintf(file_logger, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
          counter,

          imu.gyro_unscaled.p,
          imu.gyro_unscaled.q,
          imu.gyro_unscaled.r,
          imu.accel_unscaled.x,
          imu.accel_unscaled.y,
          imu.accel_unscaled.z,
          imu.mag_unscaled.x,
          imu.mag_unscaled.y,,

          imu.mag_unscaled.z,
          stabilization_cmd[COMMAND_THRUST],
          stabilization_cmd[COMMAND_ROLL],
          stabilization_cmd[COMMAND_PITCH],
          stabilization_cmd[COMMAND_YAW],
          quat->qi,
          quat->qx,
          quat->qy,
          quat->qz
         );*/
  fprintf(file_logger,"%i,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
		  logCounter,
		  (float)(get_sys_time_usec()/pow(10,6)),
		  posNED.x,
		  posNED.y,
		  posNED.z,
		  speedNED.x,
		  speedNED.y,
		  speedNED.z,
		  speedDir,
		  speedNorm,
		  accelNED.x,
		  accelNED.y,
		  accelNED.z,
		  eulerAngles.phi,
		  eulerAngles.theta,
		  eulerAngles.psi,
		  bodyRates.p,
		  bodyRates.q,
		  bodyRates.r,
		  RATE_FLOAT_OF_BFP(imu.gyro.p),
		  RATE_FLOAT_OF_BFP(imu.gyro.q),
		  RATE_FLOAT_OF_BFP(imu.gyro.r),
		  ACCEL_FLOAT_OF_BFP(imu.accel.x),
		  ACCEL_FLOAT_OF_BFP(imu.accel.y),
		  ACCEL_FLOAT_OF_BFP(imu.accel.z),
		  MAG_FLOAT_OF_BFP(imu.mag.x),
		  MAG_FLOAT_OF_BFP(imu.mag.y),
		  MAG_FLOAT_OF_BFP(imu.mag.z));
  logCounter++;
}
