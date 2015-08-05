/*
 * Copyright (C) 2012-2014 The Paparazzi Community
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/viewvideo.h
 *
 * Get live images from a RTP/UDP stream
 * and save pictures on internal memory
 *
 * Works on Linux platforms
 */

#ifndef VIEW_VIDEO_H
#define VIEW_VIDEO_H

#include "std.h"
#include "lib/vision/image.h"
#include "lib/v4l/v4l2.h"

#include "visionhover/IMAV2015_vision.h"


// Main viewvideo structure
struct viewvideo_t {
  volatile bool_t is_streaming;   ///< When the device is streaming
  struct v4l2_device *dev;        ///< The V4L2 device that is used for the video stream
  uint8_t downsize_factor;        ///< Downsize factor during the stream
  uint8_t quality_factor;         ///< Quality factor during the stream
  uint8_t fps;                    ///< The amount of frames per second

  volatile bool_t take_shot;      ///< Wether to take an image
  uint16_t shot_number;           ///< The last shot number
};
extern struct viewvideo_t viewvideo;

/* The vision algorithm parameters */
/*struct visionhover_param_t {
  float M;        ///< The distance between the pixel of interest and farthest neighbor pixel [pixel]
  float m;        ///< The safety margin around the pixel of interest [pixel]
  float t;      ///< Threshold for intensity difference
  float IN;      ///< The number of minimum inliers required
};*/
struct visionhover_param_t visionhover_parameter;


// Module functions
extern void viewvideo_init(void);
extern void viewvideo_periodic(void); ///< A dummy for now
extern void viewvideo_start(void);
extern void viewvideo_stop(void);
extern void viewvideo_take_shot(bool_t take);

#endif /* VIEW_VIDEO_H */

