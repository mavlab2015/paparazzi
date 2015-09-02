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

// FROM guidance_v.c
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "generated/airframe.h"
#include "firmwares/rotorcraft/guidance/guidance_module.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include "math/pprz_algebra_int.h"


#define CMD_OF_SAT  500 // 40 deg = 2859.1851

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

#ifndef VH_LANDING_MARKER
#define VH_LANDING_MARKER FALSE
#endif
PRINT_CONFIG_VAR(LANDING_MARKER)

#define ALT_FIRST -200
#define ALT_SECOND -700
#define HOVER_COUNT_FIRST 1000
#define MARKER_COUNT 200
#define NO_MARKER_COUNT 200
#define LANDING_COUNT 4000
#define DEVIATION_SAT 80




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
  .landing_marker = VH_LANDING_MARKER
};

float pre_err_x, pre_err_y;
float des_vx, des_vy;

float pre_err_vx, pre_err_vy;
float past_err_vx, past_err_vy;

int32_t test_count, hover_count_first, return_count;

int32_t forever_hover;

int8_t v_control, marker_detected;
int8_t alt_reached_first, alt_reached_second;
int8_t init_done;

int8_t inlier_sum;
int8_t forever_hover_init;
int8_t return_start;

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
  
  /*  Reset the differential errors and others*/
  pre_err_vx = 0;
  pre_err_vy = 0;
  pre_err_x = 0;
  pre_err_y = 0;
  test_count = 0;
  
  alt_reached_first = 0;
  marker_detected = 0;
  v_control = 1;
  forever_hover = 0;
  forever_hover_init = 0;
  
  opticflow_stab.marker_count = 0;
  opticflow_stab.no_marker_count = 0;
  
  /* Set rool/pitch to 0 degrees and psi to current heading */
  opticflow_stab.cmd.phi = 0;
  opticflow_stab.cmd.theta = 0;
  opticflow_stab.cmd.psi = stateGetNedToBodyEulers_i()->psi;
}





///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

#define GuidanceVSetRef(_pos, _speed, _accel) { \
    gv_set_ref(_pos, _speed, _accel);        \
    guidance_v_z_ref = _pos;             \
    guidance_v_zd_ref = _speed;          \
    guidance_v_zdd_ref = _accel;             \
  }
  
void guidance_v_module_enter(void)
{
      alt_reached_first = 0;
      alt_reached_second = 0;
      init_done = 0;
      hover_count_first = 0;
      return_start = 0;
      return_count = 0;
      opticflow_stab.landing_count = 0;
      guidance_v_z_sp = ALT_FIRST;
      
      
     
}

void guidance_v_module_run(bool_t in_flight)
{
	if (forever_hover == 0)
	{
		if (init_done == 0)
		{			
			if (stabilization_cmd[COMMAND_THRUST] < 3500)   //3000 (3500) for AR Drone 2 (heavy bat), 2000 for bebop
			{
				stabilization_cmd[COMMAND_THRUST] += 2;
			} 
			else
				init_done = 1;
		}
		else
		{
			if (stateGetPositionNed_i()->z < ALT_FIRST*0.2)  // -256 means 1m
			{
				alt_reached_first = 1;
			}
		
			if (alt_reached_second == 0)
			{
				if (guidance_v_z_sp > ALT_SECOND)
				{	
					if (alt_reached_first == 0)
						hover_count_first = 0;
						
					if (hover_count_first < HOVER_COUNT_FIRST)
					{
						hover_count_first += 1;
						guidance_v_z_sp = ALT_FIRST;
					}
					else					
					{
						guidance_v_z_sp -= 1;
					}
				}
				else
				{
					alt_reached_second = 1;
				}
			}
			
					
	      		guidance_v_z_sum_err = 0;
	      		GuidanceVSetRef(guidance_v_z_sp, 0, 0);
			guidance_v_zd_sp = 0;
			gv_update_ref_from_z_sp(guidance_v_z_sp);
			run_hover_loop(in_flight);
	
			if (alt_reached_first == 1)
			{
				stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
			}
			else
			stabilization_cmd[COMMAND_THRUST] = 8300; // 8000 (8300) for AR Drone 2 (heavy bat), 6500 for bebop
		}
	}
	if (forever_hover == 1)
	{
		if (!visionhover_stab.landing_marker)
		{
			guidance_v_z_sp = ALT_SECOND;
			opticflow_stab.landing_count += 1;
			
			if (opticflow_stab.landing_count < 2*LANDING_COUNT) 
			{
				if (stateGetPositionNed_i()->z > ALT_SECOND)
					guidance_v_z_sp = stateGetPositionNed_i()->z;
				else
					guidance_v_z_sp = ALT_SECOND;
			}
			else
			{	
				if (opticflow_stab.landing_count < 2.5*LANDING_COUNT)
				{
					if (stateGetPositionNed_i()->z > ALT_FIRST*0.65)
						guidance_v_z_sp = stateGetPositionNed_i()->z;
					else
						guidance_v_z_sp = ALT_FIRST*0.65;
				}
				else
				{
					if (opticflow_stab.landing_count < 3.5*LANDING_COUNT)
					{
						if (stateGetPositionNed_i()->z < ALT_FIRST*0.65)
							guidance_v_z_sp = stateGetPositionNed_i()->z;
						else
							guidance_v_z_sp = ALT_FIRST*0.65;
					}
					else
					{
						return_start = 1;
						guidance_v_z_sp = ALT_SECOND;
						if (opticflow_stab.landing_count > 3.5*LANDING_COUNT + 2000)
						{
							return_count += 1;
							guidance_v_z_sp += return_count;
							if (stateGetPositionNed_i()->z > -30)
							{
								stabilization_cmd[COMMAND_THRUST] = 0;
								return;
							}
						}
					}
				}
			}
			
			
			
		}
		else
		{
			opticflow_stab.landing_count += 1;
			if (opticflow_stab.landing_count < LANDING_COUNT) 
			{
				guidance_v_z_sp = ALT_SECOND;
			}
			else
			{
				guidance_v_z_sp += 1 ;
				if (stateGetPositionNed_i()->z > -30)
				{
					stabilization_cmd[COMMAND_THRUST] = 0;
					return;
				}
				
			}
		}
		
		guidance_v_z_sum_err = 0;
	      	GuidanceVSetRef(guidance_v_z_sp, 0, 0);
		guidance_v_zd_sp = 0;
		gv_update_ref_from_z_sp(guidance_v_z_sp);
		run_hover_loop(in_flight);
	
		stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////









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
  
  if (alt_reached_first < 1)
  {
  	return;
  }


  if (test_count < 10) 
  {
  	test_count += 1;
    	opticflow_stab.err_vx_int = 0 ;
  	opticflow_stab.err_vy_int = 0 ;
  	visionhover_stab.err_x_int = 0;
  	visionhover_stab.err_y_int = 0;
  	opticflow_stab.cmd.phi = 0;
  	opticflow_stab.cmd.theta = 0;
  	opticflow_stab.cmd.psi = stateGetNedToBodyEulers_i()->psi;
  	inlier_sum = 0;
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
		marker_detected = 1;
		err_x = mystate->agl * result->deviation_x;
		err_y = mystate->agl * (result->deviation_y) - 20; //-20 offset because the camera is closer to the tail..
		
		BoundAbs(err_x, DEVIATION_SAT);
		BoundAbs(err_y, DEVIATION_SAT);
	}
	/* Calculate the integrated errors */
	
	visionhover_stab.err_x_int += err_x;
  	visionhover_stab.err_y_int += err_y;   
  
  	/* Calculate the differential errors*/
  	visionhover_stab.err_x_diff = (err_x - pre_err_x);
	visionhover_stab.err_y_diff = (err_y - pre_err_y);
	pre_err_x = err_x;
	pre_err_y = err_y;
	
	/* Calculate the error if we have enough flow */
	float err_vx = 0;
	float err_vy = 0;
	
	
	
	if (result->tracked_cnt > 0) 
	{
		if (forever_hover == 0) // When forever_hover is 0 (Not TRUE)
		{		  	
			if (opticflow_stab.marker_count < MARKER_COUNT)
			{
				err_vx = (visionhover_stab.phi_pgain * err_x/1000 
					+ visionhover_stab.phi_igain * visionhover_stab.err_x_int/100000
					+ visionhover_stab.phi_dgain * visionhover_stab.err_x_diff/1000) - result->vel_x;
				err_vy = (visionhover_stab.theta_pgain * err_y/1000 
					+ visionhover_stab.theta_igain * visionhover_stab.err_y_int/100000
					+ visionhover_stab.theta_dgain * visionhover_stab.err_y_diff/1000) - result->vel_y;
				
				if (alt_reached_second == 1)	
					opticflow_stab.marker_count += 1;
			}
			else
			{		
				err_vx = opticflow_stab.desired_vx*50 - result->vel_x;
				err_vy = opticflow_stab.desired_vy*50 - result->vel_y;
				       
				if (opticflow_stab.marker_count > MARKER_COUNT - 1)
				{
					opticflow_stab.no_marker_count += 1;
				}
			}
		
			if (result->inlier > 0 && opticflow_stab.no_marker_count > NO_MARKER_COUNT)
			{
				inlier_sum += result->inlier;
				if (inlier_sum > 10)
					forever_hover = 1;
			}
		}
		else // when forever_hover is 1 (TRUE)
		{
			if (forever_hover_init == 0)
			{
				opticflow_stab.err_vx_int = 0 ;
			  	opticflow_stab.err_vy_int = 0 ;
			  	visionhover_stab.err_x_int = 0;
			  	visionhover_stab.err_y_int = 0;
			  	opticflow_stab.cmd.phi = 0;
			  	opticflow_stab.cmd.theta = 0;
			  	opticflow_stab.cmd.psi = stateGetNedToBodyEulers_i()->psi;
			  	forever_hover_init = 1;
			  	return;
		  	}
			if (return_start == 0)
			{
				if (result->inlier > 0)
				{
					if (opticflow_stab.landing_count > LANDING_COUNT)
					{
						err_vx = (visionhover_stab.phi_pgain * err_x/1000 
						+ visionhover_stab.phi_igain * visionhover_stab.err_x_int/100000
						+ visionhover_stab.phi_dgain * visionhover_stab.err_x_diff/1000) - result->vel_x;
						err_vy = (visionhover_stab.theta_pgain * err_y/1000 
						+ visionhover_stab.theta_igain * visionhover_stab.err_y_int/100000
						+ visionhover_stab.theta_dgain * visionhover_stab.err_y_diff/1000) - result->vel_y;
					}
					else
					{
						err_vx = 0 - result->vel_x;
						err_vy = 0 - result->vel_y;
					}
				}
				else
				{
					err_vx = 0 - result->vel_x;
					err_vy = 0 - result->vel_y;
				}
			}
			else
			{
				err_vx = 0 - result->vel_x;
				err_vy = -opticflow_stab.desired_vy*50 - result->vel_y;
				//printf("<Drop_Paintball_Now1 \n"); // print only ONCE !!!
				
			}
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
		                     + opticflow_stab.phi_dgain * opticflow_stab.err_vx_diff)/100;
	opticflow_stab.cmd.theta = -(opticflow_stab.theta_pgain * err_vy
		                       + opticflow_stab.theta_igain * opticflow_stab.err_vy_int
		                       + opticflow_stab.theta_dgain * opticflow_stab.err_vy_diff)/100;

	/* Bound the roll and pitch commands */
	BoundAbs(opticflow_stab.cmd.phi, CMD_OF_SAT);
	BoundAbs(opticflow_stab.cmd.theta, CMD_OF_SAT);
	  
	//printf("marker detected = %i\n", visionhover_stab.marker_detected);
}




