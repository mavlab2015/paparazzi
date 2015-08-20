/*
 * Copyright (C) 2014 Hann Woei Ho
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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

/**
 * @file modules/computer_vision/opticflow/stabilization_opticflow.c
 * @brief Optical-flow based control for Linux based systems
 *
 * Control loops for optic flow based hovering.
 * Computes setpoint for the lower level attitude stabilization to control horizontal velocity.
 */

// Own Header
#include "stabilization_opticflow_IMAV2015.h"

// Stabilization
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "autopilot.h"
#include "subsystems/datalink/downlink.h"

#include "subsystems/radio_control.h"

// Math
#include <math.h>

#define CMD_OF_SAT  800 // 40 deg = 2859.1851

#ifndef VISION_PHI_PGAIN
#define VISION_PHI_PGAIN 400
#endif
PRINT_CONFIG_VAR(VISION_PHI_PGAIN)

#ifndef VISION_PHI_IGAIN
#define VISION_PHI_IGAIN 20
#endif
PRINT_CONFIG_VAR(VISION_PHI_IGAIN)

#ifndef VISION_THETA_PGAIN
#define VISION_THETA_PGAIN 400
#endif
PRINT_CONFIG_VAR(VISION_THETA_PGAIN)

#ifndef VISION_THETA_IGAIN
#define VISION_THETA_IGAIN 20
#endif
PRINT_CONFIG_VAR(VISION_THETA_IGAIN)

#ifndef VISION_DESIRED_VX
#define VISION_DESIRED_VX 0
#endif
PRINT_CONFIG_VAR(VISION_DESIRED_VX)

#ifndef VISION_DESIRED_VY
#define VISION_DESIRED_VY 0
#endif
PRINT_CONFIG_VAR(VISION_DESIRED_VY)







/* Check the control gains */
#if (VISION_PHI_PGAIN < 0)      ||  \
  (VISION_PHI_IGAIN < 0)        ||  \
  (VISION_THETA_PGAIN < 0)      ||  \
  (VISION_THETA_IGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif

/* Initialize the default gains and settings */
struct opticflow_stab_t opticflow_stab = {
  .phi_pgain = VISION_PHI_PGAIN,
  .phi_igain = VISION_PHI_IGAIN,
  .phi_dgain = VISION_PHI_DGAIN,
  .theta_pgain = VISION_THETA_PGAIN,
  .theta_igain = VISION_THETA_IGAIN,
  .theta_dgain = VISION_THETA_DGAIN,
  .desired_vx = VISION_DESIRED_VX,
  .desired_vy = VISION_DESIRED_VY
};

/* Initialize the default gains and settings */
struct visionhover_stab_t visionhover_stab = {
  .phi_pgain = VH_PHI_PGAIN,
  .phi_igain = VH_PHI_IGAIN,
  .phi_dgain = VH_PHI_DGAIN,
  .theta_pgain = VH_THETA_PGAIN,
  .theta_igain = VH_THETA_IGAIN,
  .theta_dgain = VH_THETA_DGAIN,
};

float pre_err_x, pre_err_y;
float des_vx, des_vy;

float pre_err_vx, pre_err_vy;
int32_t testcount, v_control;

/**
 * Horizontal guidance mode enter resets the errors
 * and starts the controller.
 */
void guidance_h_module_enter(void)
{
  /* Reset the integrated errors */
  opticflow_stab.err_vx_int = 0;
  opticflow_stab.err_vy_int = 0;
  visionhover_stab.err_x_int = 0;
  visionhover_stab.err_y_int = 0;
  
  /*  Reset the differential errors*/
  pre_err_vx = 0;
  pre_err_vy = 0;
  testcount = 0;
  
  opticflow_stab.alt_reached = 0;
  
  v_control = 1;
  
  /*Initialize the boolean to check if marker is detected*/
  visionhover_stab.marker_detected = 0;
  
  

  /* Set rool/pitch to 0 degrees and psi to current heading */
  opticflow_stab.cmd.phi = 0;
  opticflow_stab.cmd.theta = 0;
  opticflow_stab.cmd.psi = stateGetNedToBodyEulers_i()->psi;
}

/**
 * Read the RC commands
 */
void guidance_h_module_read_rc(void)
{
  // TODO: change the desired vx/vy
}

/**
 * Main guidance loop
 * @param[in] in_flight Whether we are in flight or not
 */
void guidance_h_module_run(bool_t in_flight)
{
  /* Update the setpoint */
  stabilization_attitude_set_rpy_setpoint_i(&opticflow_stab.cmd);

  /* Run the default attitude stabilization */
  stabilization_attitude_run(in_flight);
}

/**
 * Update the controls based on a vision result
 * @param[in] *result The opticflow calculation result used for control
 */
void stabilization_opticflow_update(struct opticflow_result_t *result, struct opticflow_state_t *mystate)
{
  /* Check if we are in the correct AP_MODE before setting commands */
  if (autopilot_mode != AP_MODE_MODULE) {
    return;
  }
  
  if (mystate->agl > 1)
  {
  	opticflow_stab.alt_reached = 1;
  }
  
  if (opticflow_stab.alt_reached < 1)
  {
  	return;
  }
  


  if (testcount < 10) 
  {
  	  testcount += 1;
    	opticflow_stab.err_vx_int = 0 ;
  	opticflow_stab.err_vy_int = 0 ;
  	opticflow_stab.cmd.phi = 0;
  	opticflow_stab.cmd.theta = 0;
  	opticflow_stab.cmd.psi = stateGetNedToBodyEulers_i()->psi;
  	return;
  }
  
  if (opticflow_stab.desired_vx !=0 || opticflow_stab.desired_vy !=0)
  	v_control = 1;
  
  if (opticflow_stab.desired_vx ==0 && opticflow_stab.desired_vy == 0 && v_control == 1)
  {
  	opticflow_stab.err_vx_int = 0 ;
  	opticflow_stab.err_vy_int = 0 ;
  	v_control = 0;
  }
  
  
  	/* Calculate the vision hover error if we have enough inliers */
  	float err_x = 0;
	float err_y = 0;
	
	if (result->inlier > 0)
	{
		visionhover_stab.marker_detected = 1;
		err_x = mystate->agl * result->deviation_x;
		err_y = mystate->agl * result->deviation_y;
	}
	   
	/* Calculate the error if we have enough flow */
	float err_vx = 0;
	float err_vy = 0;
	
	if (result->tracked_cnt > 0) 
	{
		if (visionhover_stab.marker_detected > 0)
		{
			err_vx = (visionhover_stab.phi_pgain * err_x)/100 - result->vel_x;
			err_vy = (visionhover_stab.theta_pgain * err_y)/100 - result->vel_y;
		}
		else
		{
			err_vx = opticflow_stab.desired_vx - result->vel_x;
			err_vy = opticflow_stab.desired_vy - result->vel_y;
		}
	}
	/* Calculate the integrated errors (TODO: bound??) */
	opticflow_stab.err_vx_int += err_vx;
	opticflow_stab.err_vy_int += err_vy;
	  
	/* Calculate the differential errors (TODO: bound??) */

	opticflow_stab.err_vx_diff = (err_vx - pre_err_vx);
	opticflow_stab.err_vy_diff = (err_vy - pre_err_vy);
	pre_err_vx = err_vx;
	pre_err_vy = err_vy;
  

	/* Calculate the commands */
	opticflow_stab.cmd.phi   = (opticflow_stab.phi_pgain * err_vx
		                     + opticflow_stab.phi_igain * opticflow_stab.err_vx_int
		                     + opticflow_stab.phi_dgain * opticflow_stab.err_vx_diff * 10)/100;
	opticflow_stab.cmd.theta = -(opticflow_stab.theta_pgain * err_vy
		                       + opticflow_stab.theta_igain * opticflow_stab.err_vy_int
		                       + opticflow_stab.theta_dgain * opticflow_stab.err_vy_diff * 10)/100;

	/* Bound the roll and pitch commands */
	BoundAbs(opticflow_stab.cmd.phi, CMD_OF_SAT);
	BoundAbs(opticflow_stab.cmd.theta, CMD_OF_SAT);
	  
	//printf("marker detected = %i\n", visionhover_stab.marker_detected);
}
