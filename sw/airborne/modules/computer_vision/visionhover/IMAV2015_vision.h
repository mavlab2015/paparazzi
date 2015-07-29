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
 * @file modules/computer_vision/opticflow/opticflow_calculator.h
 * @brief Calculate velocity from optic flow.
 *
 * Using images from a vertical camera and IMU sensor data.
 */

#ifndef VISIONHOVER_CALCULATOR_H
#define VISIONHOVER_CALCULATOR_H

#include "std.h"
#include "IMAV2015_inter_thread_data.h"
#include "lib/vision/image.h"
#include "lib/v4l/v4l2.h"

extern struct centroid_deviation_t centroid_dev;
extern struct marker_deviation_t marker_dev;

void visionhover_calc_frame(struct image_t *img, struct visionhover_state_t *state, struct visionhover_result_t *result);

#endif /* VISIONHOVER_CALCULATOR_H */
