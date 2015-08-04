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

// Vision algorithm input parameters
#ifndef VISION_M
#define VISION_M 8
#endif
PRINT_CONFIG_VAR(VISION_M)

#ifndef VISION_m
#define VISION_m 3
#endif
PRINT_CONFIG_VAR(VISION_m)

#ifndef VISION_t
#define VISION_t 15
#endif
PRINT_CONFIG_VAR(VISION_t)

#ifndef VISION_IN
#define VISION_IN 3
#endif
PRINT_CONFIG_VAR(VISION_IN)


struct centroid_deviation_t centroid_deviation;
struct marker_deviation_t marker_deviation;


/* Initialize the default settings for the vision algorithm */
struct visionhover_param_t visionhover_param = {
  .M = VISION_M,
  .m = VISION_m,
  .t = VISION_t,
  .IN = VISION_IN,
};

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
  marker_deviation = marker(img, img, visionhover_param.M, visionhover_param.m, visionhover_param.t, visionhover_param.IN);
  result->deviation_x = marker_deviation.x * state ->agl;
  result->deviation_y = marker_deviation.y * state ->agl;
  result->inlier = marker_deviation.inlier;
}


