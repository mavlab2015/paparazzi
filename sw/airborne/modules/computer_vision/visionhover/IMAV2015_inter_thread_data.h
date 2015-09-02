/*
 * Copyright (C) 2015 The Paparazzi Community
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
 * @file modules/computer_vision/opticflow/inter_thread_data.h
 * @brief Inter-thread data structures.
 *
 * Data structures used to for inter-thread communication via Unix Domain sockets.
 */


#ifndef _IMAV2015_INTER_THREAD_DATA_H
#define _IMAV2015_INTER_THREAD_DATA_H

/* The state of the drone when it took an image */
struct visionhover_state_t {
  float phi;      ///< roll [rad]
  float theta;    ///< pitch [rad]
  float agl;      ///< height above ground [m]
};

/* The result calculated from the vision */

struct visionhover_result_t {
  float deviation_x;      ///< Deviation of centroid from the center in x direction [pixel]
  float deviation_y;      ///< Deviation of centroid from the center in y direction [pixel]
  int8_t inlier;            ///< The number of inliers
};



#endif
