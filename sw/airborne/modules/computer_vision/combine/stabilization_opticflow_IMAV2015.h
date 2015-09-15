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
 * @file modules/computer_vision/opticflow/stabilization_opticflow.h
 * @brief Optical-flow based control for Linux based systems
 *
 * Control loops for optic flow based hovering.
 * Computes setpoint for the lower level attitude stabilization to control horizontal velocity.
 */

#ifndef CV_STABILIZATION_OPTICFLOW_IMAV2015_H_
#define CV_STABILIZATION_OPTICFLOW_IMAV2015_H_

#include "std.h"
#include "lib/v4l/v4l2.h"
#include "inter_thread_data_IMAV2015.h"
#include "math/pprz_algebra_int.h"

// FROM guidance_v.h
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/guidance/guidance_v_ref.h"
#include "firmwares/rotorcraft/guidance/guidance_v_adapt.h"


/* The opticflow stabilization */
struct opticflow_stab_t {
  int32_t phi_pgain;        ///< The roll P gain on the err_vx
  int32_t phi_igain;        ///< The roll I gain on the err_vx_int
  int32_t phi_dgain;
  int32_t theta_pgain;      ///< The pitch P gain on the err_vy
  int32_t theta_igain;      ///< The pitch I gain on the err_vy_int
  int32_t theta_dgain;
  float desired_vx;         ///< The desired velocity in the x direction (cm/s)
  float desired_vy;         ///< The desired velocity in the y direction (cm/s)

  float err_vx_int;         ///< The integrated velocity error in x direction (m/s)
  float err_vy_int;         ///< The integrated velocity error in y direction (m/s)
  float err_vx_diff;
  float err_vy_diff;
  struct Int32Eulers cmd;   ///< The commands that are send to the hover loop
  
  uint16_t marker_count;
  uint16_t no_marker_count;
  
  uint16_t landing_count;
};
extern struct opticflow_stab_t opticflow_stab;

struct visionhover_stab_t {
  float phi_pgain;        ///< The roll P gain on the err_vx
  float phi_igain;        ///< The roll I gain on the err_vx_int
  float phi_dgain;        ///< The roll D gain on the err_vx_diff
  float theta_pgain;      ///< The pitch P gain on the err_vy
  float theta_igain;      ///< The pitch I gain on the err_vy_int
  float theta_dgain;      ///< The pitch D gain on the err_vy_diff
  float alpha; ///< Low pass filter coefficient for the p-gain error
  uint8_t post_left; ///< The address of the left postbox
  uint8_t post_middle; ///< The address of the middle postbox
  uint8_t post_right; ///< The address of the right postbox
  
  float err_x_int;         ///< The integrated velocity error in x direction (m/s)
  float err_y_int;         ///< The integrated velocity error in y direction (m/s)
  
  float err_x_diff;        ///< The difference in velocity error in x direction (m/s)
  float err_y_diff;        ///< The difference in velocity error in y direction (m/s)
  bool_t landing_marker;    /// Landing on marker enabled
  bool_t drop;    /// Dropping on marker enabled
  int8_t heat_enter;
  
  float alt_second;
  float factor_alt;
  uint32_t landing_count_A;
  uint32_t landing_count_B;
  uint32_t landing_count_C;
  uint32_t landing_count_D;
  int16_t no_marker_count;
  float descent_rate;
  float vel_sat;
  int32_t err;
  
  bool_t line_follow; /// Line following enabled
  float line_alt_second;
  float line_phi_pgain;
  float line_theta_pgain;
  float line_vel_x_sat;
  float line_vel_y_sat;
  int32_t line_cmd_sat;
  int32_t line_no_count;
  
  bool_t flower_mode;
  int16_t flower_no_marker_count;
  float right_forward_vx;
  float right_forward_vy;
  float left_forward_vx;
  float left_forward_vy;
  uint32_t servo_count;
};

extern struct visionhover_stab_t visionhover_stab;


// Implement own Horizontal loops
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool_t in_flight);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool_t in_flight);

// Update the stabiliztion commands based on a vision result
void stabilization_opticflow_update(struct opticflow_result_t *vision, struct opticflow_state_t *mystate);

#endif /* CV_STABILIZATION_OPTICFLOW_H_ */
