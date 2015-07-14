/*
 * Copyright (C) 2014 Hann Woei Ho
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2015 IMAV2015 Masters Group
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
 * @brief Vision based control for Linux based systems
 *
 * Control loops for optic flow based hovering.
 * Computes setpoint for the lower level attitude stabilization to hover.
 */

// Own Header
#include "IMAV2015_stabilization.h"

// Stabilization
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "autopilot.h"
#include "subsystems/datalink/downlink.h"

#define CMD_OF_SAT  1500 // 40 deg = 2859.1851

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


/* Check the control gains */
#if (VISION_PHI_PGAIN < 0)      ||  \
  (VISION_PHI_IGAIN < 0)        ||  \
  (VISION_THETA_PGAIN < 0)      ||  \
  (VISION_THETA_IGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif

/* Initialize the default gains and settings */
struct visionhover_stab_t visionhover_stab = {
  .phi_pgain = VISION_PHI_PGAIN,
  .phi_igain = VISION_PHI_IGAIN,
  .theta_pgain = VISION_THETA_PGAIN,
  .theta_igain = VISION_THETA_IGAIN,
};

/**
 * Horizontal guidance mode enter resets the errors
 * and starts the controller.
 */
void guidance_h_module_enter(void)
{

  /* Set rool/pitch to 0 degrees and psi to current heading */
  visionhover_stab.cmd.phi = 0;
  visionhover_stab.cmd.theta = 0;
  visionhover_stab.cmd.psi = stateGetNedToBodyEulers_i()->psi;
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
  stabilization_attitude_set_rpy_setpoint_i(&visionhover_stab.cmd);

  /* Run the default attitude stabilization */
  stabilization_attitude_run(in_flight);
}

/**
 * Update the controls based on a vision result
 * @param[in] *result The opticflow calculation result used for control
 */
void stabilization_visionhover_update(struct visionhover_result_t *result)
{
  /* Check if we are in the correct AP_MODE before setting commands */
  if (autopilot_mode != AP_MODE_MODULE) {
    return;
  }

  /* Calculate the error if we have enough flow */
  float err_x;
  float err_y;
  err_x = result->deviation_x;
  err_y = result->deviation_y;

  //printf("Phi command = %f, Theta command = %f\n", result->deviation_x, result->deviation_y);
  printf("blabla\n");
  
  /* Calculate the integrated errors (TODO: bound??) */
  visionhover_stab.err_x_int += err_x;
  visionhover_stab.err_y_int += err_y;

  /* Calculate the commands */
  visionhover_stab.cmd.phi   = (visionhover_stab.phi_pgain * err_x
                             + visionhover_stab.phi_igain * visionhover_stab.err_x_int) / 100;
  visionhover_stab.cmd.theta = -(visionhover_stab.theta_pgain * err_y 
                               + visionhover_stab.theta_igain * visionhover_stab.err_y_int) / 100;

  
  /* Bound the roll and pitch commands */
  BoundAbs(visionhover_stab.cmd.phi, CMD_OF_SAT);
  BoundAbs(visionhover_stab.cmd.theta, CMD_OF_SAT);
  

}
