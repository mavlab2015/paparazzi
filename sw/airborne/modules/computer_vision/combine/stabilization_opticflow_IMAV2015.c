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
 
#include <stdio.h>

// Own Header
#include "stabilization_opticflow_IMAV2015.h"

// Stabilization
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "autopilot.h"
#include "subsystems/datalink/downlink.h"

#include "subsystems/radio_control.h"
#include "modules/com/uart_drop.h"

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
#define VISION_PHI_PGAIN 300
#endif
PRINT_CONFIG_VAR(VISION_PHI_PGAIN)

#ifndef VISION_PHI_IGAIN
#define VISION_PHI_IGAIN 20
#endif
PRINT_CONFIG_VAR(VISION_PHI_IGAIN)

#ifndef VISION_PHI_DGAIN
#define VISION_PHI_DGAIN 30
#endif
PRINT_CONFIG_VAR(VISION_PHI_DGAIN)

#ifndef VISION_THETA_PGAIN
#define VISION_THETA_PGAIN 300
#endif
PRINT_CONFIG_VAR(VISION_THETA_PGAIN)

#ifndef VISION_THETA_IGAIN
#define VISION_THETA_IGAIN 20
#endif
PRINT_CONFIG_VAR(VISION_THETA_IGAIN)

#ifndef VISION_THETA_DGAIN
#define VISION_THETA_DGAIN 30
#endif
PRINT_CONFIG_VAR(VISION_THETA_DGAIN)

#ifndef VISION_DESIRED_VX
#define VISION_DESIRED_VX 0
#endif
PRINT_CONFIG_VAR(VISION_DESIRED_VX)

#ifndef VISION_DESIRED_VY
#define VISION_DESIRED_VY 0
#endif
PRINT_CONFIG_VAR(VISION_DESIRED_VY)

#ifndef VH_PHI_PGAIN
#define VH_PHI_PGAIN 600
#endif
PRINT_CONFIG_VAR(VH_PHI_PGAIN)

#ifndef VH_PHI_IGAIN
#define VH_PHI_IGAIN 100
#endif
PRINT_CONFIG_VAR(VH_PHI_IGAIN)

#ifndef VH_PHI_DGAIN
#define VH_PHI_DGAIN 100
#endif
PRINT_CONFIG_VAR(VH_PHI_DGAIN)

#ifndef VH_THETA_PGAIN
#define VH_THETA_PGAIN 400
#endif
PRINT_CONFIG_VAR(VH_THETA_PGAIN)

#ifndef VH_THETA_IGAIN
#define VH_THETA_IGAIN 100
#endif
PRINT_CONFIG_VAR(VH_THETA_IGAIN)


#ifndef VH_THETA_DGAIN
#define VH_THETA_DGAIN 100
#endif
PRINT_CONFIG_VAR(VH_THETA_DGAIN)

#ifndef VH_ALPHA
#define VH_ALPHA 1
#endif
PRINT_CONFIG_VAR(VH_ALPHA)

#ifndef VH_POST_LEFT
#define VH_POST_LEFT 1
#endif
PRINT_CONFIG_VAR(VH_POST_LEFT)

#ifndef VH_POST_MIDDLE
#define VH_POST_MIDDLE 2
#endif
PRINT_CONFIG_VAR(VH_POST_MIDDLE)

#ifndef VH_POST_RIGHT
#define VH_POST_RIGHT 3
#endif
PRINT_CONFIG_VAR(VH_POST_RIGHT)

#ifndef VH_LANDING_MARKER
#define VH_LANDING_MARKER FALSE
#endif
PRINT_CONFIG_VAR(VH_LANDING_MARKER)

#ifndef VH_HEAT_ENTER
#define VH_HEAT_ENTER 0
#endif
PRINT_CONFIG_VAR(VH_HEAT_ENTER)

#ifndef VH_DROP
#define VH_DROP FALSE
#endif
PRINT_CONFIG_VAR(VH_DROP)

#ifndef VH_DESCENT_RATE
#define VH_DESCENT_RATE 0.1 // Descent rate for descent from ALT_SECOND to ALT_FIRST
#endif
PRINT_CONFIG_VAR(VH_DESCENT_RATE)

#ifndef VH_ERR
#define VH_ERR 10
#endif
PRINT_CONFIG_VAR(VH_ERR)

#ifndef VH_VEL_SAT
#define VH_VEL_SAT 15
#endif
PRINT_CONFIG_VAR(VH_VEL_SAT)

#ifndef VH_LINE_FOLLOW
#define VH_LINE_FOLLOW FALSE
#endif
PRINT_CONFIG_VAR(VH_LINE_FOLLOW)

#ifndef VH_LINE_PHI_PGAIN
#define VH_LINE_PHI_PGAIN 400
#endif
PRINT_CONFIG_VAR(VH_PHI_PGAIN)

#ifndef VH_LINE_THETA_PGAIN
#define VH_LINE_THETA_PGAIN 400
#endif
PRINT_CONFIG_VAR(VH_THETA_PGAIN)

#ifndef VH_LINE_VEL_X_SAT
#define VH_LINE_VEL_X_SAT 40
#endif
PRINT_CONFIG_VAR(VH_LINE_VEL_X_SAT)

#ifndef VH_LINE_VEL_Y_SAT
#define VH_LINE_VEL_Y_SAT 20
#endif
PRINT_CONFIG_VAR(VH_LINE_VEL_Y_SAT)

#ifndef VH_LINE_CMD_SAT
#define VH_LINE_CMD_SAT 248
#endif
PRINT_CONFIG_VAR(VH_LINE_CMD_SAT)

#ifndef VH_ALT_SECOND
#define VH_ALT_SECOND -600
#endif
PRINT_CONFIG_VAR(VH_ALT_SECOND)

#ifndef VH_LINE_ALT_SECOND
#define VH_LINE_ALT_SECOND -400
#endif
PRINT_CONFIG_VAR(VH_LINE_ALT_SECOND)

#ifndef VH_LINE_NO_COUNT
#define VH_LINE_NO_COUNT 100
#endif
PRINT_CONFIG_VAR(VH_LINE_ALT_SECOND)

#ifndef VH_NO_MARKER_COUNT
#define VH_NO_MARKER_COUNT 120
#endif
PRINT_CONFIG_VAR(VH_LINE_NO_MARKER_COUNT)

#ifndef VH_FACTOR_ALT
#define VH_FACTOR_ALT 1.2         // ALT_FIRST * FACTOR_ALT = Hovering & Dropping altitude on marker B
#endif 
PRINT_CONFIG_VAR(VH_FACTOR_ALT)

#ifndef VH_LANDING_COUNT_A
#define VH_LANDING_COUNT_A 1000
#endif
PRINT_CONFIG_VAR(VH_LANDING_COUNT_A)

#ifndef VH_LANDING_COUNT_B
#define VH_LANDING_COUNT_B 6000
#endif
PRINT_CONFIG_VAR(VH_LANDING_COUNT_B)

#ifndef VH_LANDING_COUNT_C
#define VH_LANDING_COUNT_C 2000
#endif
PRINT_CONFIG_VAR(VH_LANDING_COUNT_C)

#ifndef VH_LANDING_COUNT_D
#define VH_LANDING_COUNT_D 2000
#endif
PRINT_CONFIG_VAR(VH_LANDING_COUNT_D)

#ifndef VH_FLOWER_MODE
#define VH_FLOWER_MODE FALSE
#endif
PRINT_CONFIG_VAR(VH_FLOWER_MODE)

#ifndef VH_FLOWER_NO_MARKER_COUNT
#define VH_FLOWER_NO_MARKER_COUNT 30
#endif
PRINT_CONFIG_VAR(VH_FLOWER_NO_MARKER_COUNT)

#ifndef VH_RIGHT_FORWARD_VX
#define VH_RIGHT_FORWARD_VX 0
#endif
PRINT_CONFIG_VAR(VH_RIGHT_FORWARD_VX)

#ifndef VH_RIGHT_FORWARD_VY
#define VH_RIGHT_FORWARD_VY 0
#endif
PRINT_CONFIG_VAR(VH_RIGHT_FORWARD_VY)

#ifndef VH_LEFT_FORWARD_VX
#define VH_LEFT_FORWARD_VX 0
#endif
PRINT_CONFIG_VAR(VH_LEFT_FORWARD_VX)

#ifndef VH_LEFT_FORWARD_VY
#define VH_LEFT_FORWARD_VY 0
#endif
PRINT_CONFIG_VAR(VH_LEFT_FORWARD_VY)

#ifndef VH_SERVO_COUNT
#define VH_SERVO_COUNT 1000
#endif
PRINT_CONFIG_VAR(VH_SERVO_COUNT)

#define ALT_FIRST -200 // -256 means 1m

#define HOVER_COUNT_FIRST 1000
#define MARKER_COUNT 150
//#define MARKER_COUNT 30000


#define VH_OFFSET 30





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
  .alpha = VH_ALPHA,
  .post_left = VH_POST_LEFT,
  .post_middle = VH_POST_MIDDLE,
  .post_right = VH_POST_RIGHT,
  .landing_marker = VH_LANDING_MARKER,
  .heat_enter = VH_HEAT_ENTER,
  .drop = VH_DROP,
  .alt_second = VH_ALT_SECOND,
  .factor_alt = VH_FACTOR_ALT,
  .no_marker_count = VH_NO_MARKER_COUNT,
  .descent_rate = VH_DESCENT_RATE,
  .landing_count_A = VH_LANDING_COUNT_A,
  .landing_count_B = VH_LANDING_COUNT_B,
  .landing_count_C = VH_LANDING_COUNT_C,
  .landing_count_D = VH_LANDING_COUNT_D,
  .vel_sat = VH_VEL_SAT,
  .err = VH_ERR,
  
  .line_follow = VH_LINE_FOLLOW,
  .line_phi_pgain = VH_LINE_PHI_PGAIN,
  .line_theta_pgain = VH_LINE_THETA_PGAIN,
  .line_vel_x_sat = VH_LINE_VEL_X_SAT,
  .line_vel_y_sat = VH_LINE_VEL_Y_SAT,
  .line_cmd_sat = VH_LINE_CMD_SAT,
  .line_alt_second = VH_LINE_ALT_SECOND,
  .line_no_count = VH_LINE_NO_COUNT,
  
  .flower_mode = VH_FLOWER_MODE,
  .flower_no_marker_count = VH_FLOWER_NO_MARKER_COUNT,
  .right_forward_vx = VH_RIGHT_FORWARD_VX,
  .right_forward_vy = VH_RIGHT_FORWARD_VY,
  .left_forward_vx = VH_LEFT_FORWARD_VX,
  .left_forward_vy = VH_LEFT_FORWARD_VY,
  .servo_count = VH_SERVO_COUNT
};



float pre_err_x, pre_err_y;
float des_vx, des_vy;

float pre_err_vx, pre_err_vy;
float past_err_vx, past_err_vy;

int32_t test_count, hover_count_first, return_count;

int32_t forever_hover;

int8_t v_control, marker_detected;
float alt_second_chosen;
int8_t alt_reached_first, alt_reached_second;
int8_t init_done;

int8_t inlier_sum;
int8_t return_start;
int8_t descent;
int8_t already_heated;
int8_t already_dropped;
float delta_z;

int8_t no_rope_land_here;

int8_t flower_phase;
int8_t servo_command;
uint16_t servo_count;





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
      delta_z = 0;
      no_rope_land_here = 0;
      flower_phase = 0;
      servo_command = 0;
      servo_count = 0;
      
        if (visionhover_stab.line_follow)
        {	
        	guidance_v_z_sp = visionhover_stab.line_alt_second;
  		alt_second_chosen = visionhover_stab.line_alt_second;
  	}
  	else
  	{	
  		guidance_v_z_sp = ALT_FIRST;	
  		alt_second_chosen = visionhover_stab.alt_second;
  	}
        
        
        if (visionhover_stab.heat_enter == 1)
	{	
		drop_ball(1);
	}
	if (visionhover_stab.heat_enter == 2 )
	{	
		drop_ball(2);
	}
	if (visionhover_stab.heat_enter == 3)
	{	
		drop_ball(3);
	}
	if (visionhover_stab.heat_enter == 4 )
	{	
		drop_ball(4);
	}
}

void guidance_v_module_run(bool_t in_flight)
{
	
	if (forever_hover == 0)
	{
		if (init_done == 0)
		{			
			if (stabilization_cmd[COMMAND_THRUST] < 3000)   //3000 (3500) for AR Drone 2 (heavy bat), 2000 for bebop
			{
				stabilization_cmd[COMMAND_THRUST] += 2;
			} 
			else
			{
				init_done = 1;
			}
		}
		else
		{
			if (stateGetPositionNed_i()->z < ALT_FIRST*0.4) 
			{
				alt_reached_first = 1;
			}
		
			if (alt_reached_second == 0)
			{
				if (guidance_v_z_sp > alt_second_chosen)
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
			
			if (no_rope_land_here == 1 || flower_phase == 3)
			{
				if (guidance_v_z_sp < -10)
				{
					if (no_rope_land_here == 1)
						delta_z += visionhover_stab.descent_rate/10000;
					if (flower_phase == 3)
						delta_z += visionhover_stab.descent_rate;	
						
					guidance_v_z_sp += delta_z;
				}
				if (stateGetPositionNed_i()->z > -30)
				{
					stabilization_cmd[COMMAND_THRUST] = 4000;
					if (stateGetPositionNed_i()->z > -5)
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
	
			if (alt_reached_first == 1)
			{
				stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
			}
			else
			stabilization_cmd[COMMAND_THRUST] = 8000; // 8000 for AR Drone 2, 6500 for bebop
		}
	}
	if (forever_hover == 1) // When you see the (a) second marker, or (b) reached the end of the rope.
	{
		if (!visionhover_stab.landing_marker)
		{
			guidance_v_z_sp = alt_second_chosen;
		
			if (descent == 0)
				opticflow_stab.landing_count += 1;

			if (opticflow_stab.landing_count < visionhover_stab.landing_count_A + visionhover_stab.landing_count_B) 
			{
				if (stateGetPositionNed_i()->z > alt_second_chosen)
					guidance_v_z_sp = stateGetPositionNed_i()->z;
				else
					guidance_v_z_sp = alt_second_chosen;
			}
			else
			{	
				if (opticflow_stab.landing_count == visionhover_stab.landing_count_A +
									 visionhover_stab.landing_count_B)
					descent =1;
			
				if (descent == 1)
				{
					if (guidance_v_z_sp < ALT_FIRST*visionhover_stab.factor_alt
						&& stateGetPositionNed_i()->z < ALT_FIRST*visionhover_stab.factor_alt)
					{	
						delta_z += visionhover_stab.descent_rate;
						guidance_v_z_sp += delta_z;
					
						if (guidance_v_z_sp < stateGetPositionNed_i()->z)
						{
							guidance_v_z_sp = stateGetPositionNed_i()->z;
						}
					}
					else
					{
						descent = 0;
					}	
				}
				else // When the descent maneuver is over
				{	
					delta_z = 0;
					if (opticflow_stab.landing_count < visionhover_stab.landing_count_A +
									 visionhover_stab.landing_count_B +
									  visionhover_stab.landing_count_C+
									  visionhover_stab.landing_count_D)
					{
						guidance_v_z_sp = ALT_FIRST*visionhover_stab.factor_alt;
					}
					else
					{
						if (!visionhover_stab.flower_mode) // When it's not the flower mode --> RETURN
						{
							return_start = 1;
							guidance_v_z_sp = ALT_FIRST * 1.5;
							if (opticflow_stab.landing_count > visionhover_stab.landing_count_A +
											 visionhover_stab.landing_count_B +
											  visionhover_stab.landing_count_C + 
											  visionhover_stab.landing_count_D + 1500)
							{
								return_count += 1;
								guidance_v_z_sp += return_count;
								if (stateGetPositionNed_i()->z > -30)
								{
									stabilization_cmd[COMMAND_THRUST] = 8000;
									if (stateGetPositionNed_i()->z > -10)
									{
										stabilization_cmd[COMMAND_THRUST] = 0;
										return;
									}
							
								}
							}
						}
						else // When it's the flower mode, then instead of return, do this:
						{
							flower_phase += 1;
							guidance_v_z_sp = alt_second_chosen;
							forever_hover = 0;
							opticflow_stab.no_marker_count = 0;
							opticflow_stab.landing_count = 0;
						}
					}
				}
			}
		}
		else // When it's the landing mission
		{
			opticflow_stab.landing_count += 1;
			if (opticflow_stab.landing_count < visionhover_stab.landing_count_A +
										 visionhover_stab.landing_count_B) 
			{
				guidance_v_z_sp = alt_second_chosen;
			}
			else
			{
				if (stateGetPositionNed_i()->z > -30)
				{
					stabilization_cmd[COMMAND_THRUST] = 8000;
					if (stateGetPositionNed_i()->z > -10)
					{
						stabilization_cmd[COMMAND_THRUST] = 0;
						return;
					}
				
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
  test_count = 0;
  
  alt_reached_first = 0;
  marker_detected = 0;
  v_control = 1;
  forever_hover = 0;
  descent = 0;
  already_dropped = 0;
  
  opticflow_stab.marker_count = 0;
  opticflow_stab.no_marker_count = 0;
  
  
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
  if (autopilot_mode != AP_MODE_MODULE) 
  {
  	return;
  }
  
  	
  if (visionhover_stab.flower_mode)
  {	
  	
  	if (result->flower_type == 1 && (servo_command == 0 || servo_command == 6))
	{
		drop_ball(1);
		servo_command = 1;
		servo_count = 0;
	}
	if (servo_command == 1)
	{
		servo_count += 1;
		if (servo_count > visionhover_stab.servo_count)
			servo_command = 2;
		
	}
	if (servo_command == 2)
	{
		drop_ball(2);
		servo_command = 3;
	}
	if (result->flower_type == 3 && (servo_command == 0|| servo_command == 3))
	{
		drop_ball(3);
		servo_command = 4;
		servo_count = 0;
	}
	if (servo_command == 4)
	{
		servo_count += 1;
		if (servo_count > visionhover_stab.servo_count)
			servo_command = 5;
		
	}
	if (servo_command == 5)
	{
		drop_ball(4);
		servo_command = 6;
	}	
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
  /*
  if (opticflow_stab.desired_vx !=0 || opticflow_stab.desired_vy !=0)
  	v_control = 1;
  
  if (opticflow_stab.desired_vx ==0 && opticflow_stab.desired_vy == 0 && v_control == 1)
  {
  	opticflow_stab.err_vx_int = 0 ;
  	opticflow_stab.err_vy_int = 0 ;
  	v_control = 0;
  }
  */
  
  float err_x = 0;
  float err_y = 0;
  float lp_pre_err_x = 0;
  float lp_pre_err_y = 0;
  
  float err_vx = 0;
  float err_vy = 0;
  
  float vh_desired_vx = 0;
  float vh_desired_vy = 0;
  
  float OF_desired_vx = opticflow_stab.desired_vx;
  float OF_desired_vy = opticflow_stab.desired_vy;
  
  	if (visionhover_stab.post_left == result->qr_result)
	{
		OF_desired_vx = visionhover_stab.left_forward_vx;
		OF_desired_vy = visionhover_stab.left_forward_vy;
	}
	if (visionhover_stab.post_right == result->qr_result)
	{
		OF_desired_vx = visionhover_stab.right_forward_vx;
		OF_desired_vy = visionhover_stab.right_forward_vy;
	}

  
 
  

	/* Calculate the vision hover error if we have enough inliers */
	if (result->inlier > 0)
	{
		marker_detected = 1;
		err_x = mystate->agl * result->deviation_x;
		err_y = mystate->agl * (result->deviation_y); 
	}	
	
	
	/* Calculate the desired OF velocity setpoint */
	if (!visionhover_stab.line_follow)
	{
		visionhover_stab.err_x_int += err_x;
		visionhover_stab.err_y_int += err_y;
		
		/* Calculate the differential errors (TODO: bound??) */
		visionhover_stab.err_x_diff = (err_x - pre_err_x);
		visionhover_stab.err_y_diff = (err_y - pre_err_y);
		pre_err_x = err_x;
		pre_err_y = err_y;
		   
		/* Applying low-pass filter for the p-gain? */;
		err_x = visionhover_stab.alpha * err_x + (1-visionhover_stab.alpha)*lp_pre_err_x;
		lp_pre_err_x = err_x;
		err_y = visionhover_stab.alpha * err_y + (1-visionhover_stab.alpha)*lp_pre_err_y;
		lp_pre_err_y = err_y;
		
		vh_desired_vx = (visionhover_stab.phi_pgain * err_x
				+ visionhover_stab.phi_igain * visionhover_stab.err_x_int
				+ visionhover_stab.phi_dgain * visionhover_stab.err_x_diff)/100;
		
		vh_desired_vy = (visionhover_stab.theta_pgain * err_y
				+ visionhover_stab.theta_igain * visionhover_stab.err_y_int
				+ visionhover_stab.theta_dgain * visionhover_stab.err_y_diff)/100;
		
		BoundAbs(vh_desired_vx, visionhover_stab.vel_sat);
		BoundAbs(vh_desired_vy, visionhover_stab.vel_sat);	
	}		
	else
	{
		if (alt_reached_second == 1 && result->deviation_x == 0 && result->deviation_y == 0)
			opticflow_stab.landing_count += 1;
		if (opticflow_stab.landing_count > visionhover_stab.line_no_count)
			no_rope_land_here = 1;
		
		vh_desired_vx = (visionhover_stab.line_phi_pgain * err_x)/1000;
	
		vh_desired_vy = (visionhover_stab.line_theta_pgain * err_y)/1000;

	
		BoundAbs(vh_desired_vx, visionhover_stab.line_vel_x_sat);
		BoundAbs(vh_desired_vy, visionhover_stab.line_vel_y_sat);	
		

	}
	/* Calculate the error if we have enough flow */	
	if (result->tracked_cnt > 0) 
	{
		if (forever_hover == 0) // When forever_hover is 0 (Not TRUE)
		{		  	
			if (opticflow_stab.marker_count < MARKER_COUNT)
			{
				if (result->deviation_x  > -visionhover_stab.err
							&& result->deviation_x < visionhover_stab.err)
				{
					err_vx = 0 - result->vel_x;
					visionhover_stab.err_x_int = 0;	
					lp_pre_err_x = 0;
				}
				else
				{
					err_vx = vh_desired_vx - result->vel_x;
				}
				
				if (!visionhover_stab.line_follow)
				{
					if (result->deviation_y > VH_OFFSET-visionhover_stab.err
						&& result->deviation_y < VH_OFFSET +visionhover_stab.err)
					{
						err_vy = 0 - result->vel_y;
						visionhover_stab.err_y_int = 0;	
						lp_pre_err_y = 0;
					}
					else
					{
						err_vy = vh_desired_vy - result->vel_y;
					}
				}
				else
				{
					if (result->deviation_y > -visionhover_stab.err
						&& result->deviation_y < +visionhover_stab.err)
					{
						err_vy = 0 - result->vel_y;
						visionhover_stab.err_y_int = 0;	
					}
					else
					{
						err_vy = vh_desired_vy - result->vel_y;
					}
				}
				if (!visionhover_stab.line_follow && alt_reached_second == 1 && result->inlier > 0)
					opticflow_stab.marker_count += 1;
				
			}
			else // When marker_count is large enough
			{	
				err_vx = OF_desired_vx*50 - result->vel_x;
				err_vy = OF_desired_vy*50 - result->vel_y;
				
				
				if (flower_phase == 1)
				{
						err_vx = visionhover_stab.right_forward_vx*50 - result->vel_x;
						err_vy = visionhover_stab.right_forward_vy*50 - result->vel_y;
				}	
				if (flower_phase == 2)
				{
						err_vx = visionhover_stab.left_forward_vx*50 - result->vel_x;
						err_vy = visionhover_stab.left_forward_vy*50 - result->vel_y;
				}
				       
				if (opticflow_stab.marker_count > MARKER_COUNT - 1)
				{
					opticflow_stab.no_marker_count += 1;
				}	
			}
	
			

			if (result->inlier > 0 )
			{	
				if (flower_phase > 0 && opticflow_stab.no_marker_count > visionhover_stab.flower_no_marker_count)
				{
					forever_hover = 1;
				}
				if (opticflow_stab.no_marker_count > visionhover_stab.no_marker_count)
				{
					inlier_sum += result->inlier;
					if (inlier_sum > 10)
					{
						forever_hover = 1;
					}
				}
				
			}

			
		}
		else // when forever_hover is 1 (TRUE)
		{
			if (return_start == 0)
			{
				if (result->inlier > 0)
				{
					if (opticflow_stab.landing_count > visionhover_stab.landing_count_A)
					{
						if (result->deviation_x  > -visionhover_stab.err 
							&& result->deviation_x < visionhover_stab.err)
						{
							err_vx = 0 - result->vel_x;
							visionhover_stab.err_x_int = 0;	
						}
						else
						{
							err_vx = vh_desired_vx - result->vel_x;
						
						}
			
						if (result->deviation_y > VH_OFFSET-visionhover_stab.err 
							&& result->deviation_y < VH_OFFSET +visionhover_stab.err)
						{
							err_vy = 0 - result->vel_y;
							visionhover_stab.err_y_int = 0;	
						}
						else
						{
							err_vy = vh_desired_vy - result->vel_y;
						}

						if (opticflow_stab.landing_count == visionhover_stab.landing_count_A +
										 visionhover_stab.landing_count_B +1) 
						{
							err_vx = 0 - result->vel_x;
							err_vy = 0 - result->vel_y;
							visionhover_stab.err_x_int = 0;
					  		visionhover_stab.err_y_int = 0;
					  		vh_desired_vx = 0;
					  		vh_desired_vy = 0;
						}
						
						if (opticflow_stab.landing_count > visionhover_stab.landing_count_A +
										 visionhover_stab.landing_count_B +
										  visionhover_stab.landing_count_C
							&& visionhover_stab.drop && already_dropped == 0) 
						{
							drop_ball(3);
							already_dropped = 1;
						}
					}
					else
					{
						err_vx = 0 - result->vel_x;
						err_vy = 0 - result->vel_y;
						visionhover_stab.err_x_int = 0;
			  			visionhover_stab.err_y_int = 0;
			  			vh_desired_vx = 0;
			  			vh_desired_vy = 0;
					}
				}
				else
				{
					err_vx = 0 - result->vel_x;
					err_vy = 0 - result->vel_y;
					visionhover_stab.err_x_int = 0;
			  		visionhover_stab.err_y_int = 0;
			  		vh_desired_vx = 0;
			  		vh_desired_vy = 0;
				}
			} // If the return start is TRUE
			else
			{
				err_vx = 0 - result->vel_x;
				err_vy = -opticflow_stab.desired_vy*50 - result->vel_y;
			
			}
		}
	
	}
  
	
	if (no_rope_land_here == 1)
	{
		err_vx = 0 - result->vel_x;
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
	if (!visionhover_stab.line_follow)
	{	
		BoundAbs(opticflow_stab.cmd.phi, CMD_OF_SAT);
		BoundAbs(opticflow_stab.cmd.theta, CMD_OF_SAT);
	}
	else
	{
		BoundAbs(opticflow_stab.cmd.phi, visionhover_stab.line_cmd_sat);
		BoundAbs(opticflow_stab.cmd.theta, visionhover_stab.line_cmd_sat);
	} 
	
	if (no_rope_land_here == 1)
	{
		opticflow_stab.cmd.theta = -500;
	}
};




