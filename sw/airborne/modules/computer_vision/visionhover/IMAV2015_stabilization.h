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
 * @file modules/computer_vision/opticflow/stabilization_opticflow.h
 * @brief Vision based control for Linux based systems
 *
 * Control loops for optic flow based hovering.
 */

#ifndef IMAV2015_STABILIZATION_H_
#define IMAV2015_STABILIZATION_H_

#include "std.h"
#include "lib/v4l/v4l2.h"
#include "IMAV2015_inter_thread_data.h"
#include "math/pprz_algebra_int.h"

/* The opticflow stabilization */
struct visionhover_stab_t {
  /*int32_t phi_pgain;        ///< The roll P gain on the err_vx
  int32_t phi_igain;        ///< The roll I gain on the err_vx_int
  int32_t theta_pgain;      ///< The pitch P gain on the err_vy
  int32_t theta_igain;      ///< The pitch I gain on the err_vy_int*/
  float phi_pgain;        ///< The roll P gain on the err_vx
  float phi_igain;        ///< The roll I gain on the err_vx_int
  float theta_pgain;      ///< The pitch P gain on the err_vy
  float theta_igain;      ///< The pitch I gain on the err_vy_int

  float err_x_int;         ///< The integrated velocity error in x direction (m/s)
  float err_y_int;         ///< The integrated velocity error in y direction (m/s)
  struct Int32Eulers cmd;   ///< The commands that are send to the hover loop
};
extern struct visionhover_stab_t visionhover_stab;


// Implement own Horizontal loops
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool_t in_flight);

// Update the stabiliztion commands based on a vision result
void stabilization_visionhover_update(struct visionhover_result_t *vision);

#endif /* IMAV2015_STABILIZATION_H_ */
