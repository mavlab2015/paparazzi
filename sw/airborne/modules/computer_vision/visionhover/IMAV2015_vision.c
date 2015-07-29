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
 * @file modules/computer_vision/opticflow/opticflow_calculator.c
 * @brief Estimate velocity from optic flow.
 *
 * Using images from a vertical camera and IMU sensor data.
 */

#include "std.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Own Header
#include "IMAV2015_vision.h"

// Computer Vision
#include "lib/vision/image.h"

// Camera parameters (defaults are from an ARDrone 2)
#ifndef OPTICFLOW_FOV_W
#define OPTICFLOW_FOV_W 0.89360857702
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FOV_W)

#ifndef OPTICFLOW_FOV_H
#define OPTICFLOW_FOV_H 0.67020643276
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FOV_H)

#ifndef OPTICFLOW_FX
#define OPTICFLOW_FX 343.1211
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FX)

#ifndef OPTICFLOW_FY
#define OPTICFLOW_FY 348.5053
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FY)


struct centroid_deviation_t centroid_deviation;
struct marker_deviation_t marker_deviation;

/**
 * Run the vision algorithm on a new image frame
 * @param[in] *img The image frame to calculate the optical flow from
 * @param[out] *result The vision result
 */
void visionhover_calc_frame(struct image_t *img, struct visionhover_state_t *state, struct visionhover_result_t *result)
{
  //centroid_deviation = image_centroid(img, img, 160, 255, 0, 255, 0, 255);
  //result->deviation_x = centroid_deviation.x * state->agl;
  //result->deviation_y = centroid_deviation.y * state->agl;
  marker_deviation = marker(&img, &img, 8, 3, 5, 3);
  result->deviation_x = marker_deviation.x * state ->agl;
  result->deviation_y = marker_deviation.y * state ->agl;
}


