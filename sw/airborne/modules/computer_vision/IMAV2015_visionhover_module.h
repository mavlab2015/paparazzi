/*
 * Copyright (C) 2014 Hann Woei Ho
 *               2015 IMAV Masters Group
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
 * @file modules/computer_vision/IMAV2015_visionhover_module.h
 * @brief vision based hovering for Parrot AR.Drone 2.0
 *
 * Sensors from vertical camera and IMU of Parrot AR.Drone 2.0
 */

#ifndef IMAV2015_VISIONHOVER_MODULE_H
#define IMAV2015_VISIONHOVER_MODULE_H

// Include opticflow calculator and stabilization loops
#include "visionhover/IMAV2015_vision.h"
#include "visionhover/IMAV2015_stabilization.h"


// Module functions
extern void visionhover_module_init(void);
extern void visionhover_module_run(void);
extern void visionhover_module_start(void);
extern void visionhover_module_stop(void);

#endif /* IMAV2015_VISIONHOVER_MODULE_H */
