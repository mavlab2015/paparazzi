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
#include "opticflow_calculator_IMAV2015.h"

// Computer Vision
#include "lib/vision/image.h"
#include "lib/vision/lucas_kanade.h"
#include "lib/vision/fast_rosten.h"
#include "modules/computer_vision/qrcode/qr_code.h"
#include "modules/computer_vision/combine/cv_function/marker_finder.h"
#include "modules/computer_vision/combine/cv_function/line_finder.h"
#include "modules/computer_vision/combine/cv_function/centroid_finder.h"

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

/* Set the default values */
#ifndef OPTICFLOW_MAX_TRACK_CORNERS
#define OPTICFLOW_MAX_TRACK_CORNERS 25
#endif
PRINT_CONFIG_VAR(OPTICFLOW_MAX_TRACK_CORNERS)

#ifndef OPTICFLOW_WINDOW_SIZE
#define OPTICFLOW_WINDOW_SIZE 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_WINDOW_SIZE)

#ifndef OPTICFLOW_SUBPIXEL_FACTOR
#define OPTICFLOW_SUBPIXEL_FACTOR 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_SUBPIXEL_FACTOR)

#ifndef OPTICFLOW_MAX_ITERATIONS
#define OPTICFLOW_MAX_ITERATIONS 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_MAX_ITERATIONS)

#ifndef OPTICFLOW_THRESHOLD_VEC
#define OPTICFLOW_THRESHOLD_VEC 2
#endif
PRINT_CONFIG_VAR(OPTICFLOW_THRESHOLD_VEC)

#ifndef OPTICFLOW_FAST9_ADAPTIVE
#define OPTICFLOW_FAST9_ADAPTIVE TRUE
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_ADAPTIVE)

#ifndef OPTICFLOW_FAST9_THRESHOLD
#define OPTICFLOW_FAST9_THRESHOLD 20
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_THRESHOLD)

#ifndef OPTICFLOW_FAST9_MIN_DISTANCE
#define OPTICFLOW_FAST9_MIN_DISTANCE 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_MIN_DISTANCE)

// Vision algorithm input parameters
#ifndef VH_M
#define VH_M 8
#endif
PRINT_CONFIG_VAR(VISION_M)

#ifndef VH_m
#define VH_m 3
#endif
PRINT_CONFIG_VAR(VISION_m)

#ifndef VH_t
#define VH_t 15
#endif
PRINT_CONFIG_VAR(VISION_t)

#ifndef VH_RADIUS
#define VH_RADIUS 10
#endif
PRINT_CONFIG_VAR(VISION_RADIUS)

#ifndef VH_IN
#define VH_IN 3
#endif
PRINT_CONFIG_VAR(VISION_IN)

#ifndef VH_LINE_W
#define VH_LINE_W 5
#endif
PRINT_CONFIG_VAR(VISION_LINE_W)

#ifndef VH_LINE_THR
#define VH_LINE_THR 100
#endif
PRINT_CONFIG_VAR(VISION_LINE_THR)

// Flower detection parameters
#ifndef RED_Y_MIN
#define RED_Y_MIN 120
#endif

#ifndef RED_Y_MAX
#define RED_Y_MAX 220
#endif

#ifndef RED_U_MIN
#define RED_U_MIN 0
#endif

#ifndef RED_U_MAX
#define RED_U_MAX 120
#endif

#ifndef RED_V_MIN
#define RED_V_MIN 160
#endif

#ifndef RED_V_MAX
#define RED_V_MAX 255
#endif

#ifndef BLUE_Y_MIN
#define BLUE_Y_MIN 55
#endif

#ifndef BLUE_Y_MAX
#define BLUE_Y_MAX 130
#endif

#ifndef BLUE_U_MIN
#define BLUE_U_MIN 150
#endif

#ifndef BLUE_U_MAX
#define BLUE_U_MAX 255
#endif

#ifndef BLUE_V_MIN
#define BLUE_V_MIN 0
#endif

#ifndef BLUE_V_MAX
#define BLUE_V_MAX 255
#endif

#ifndef COUNTER_TRESH
#define COUNTER_TRESH 2000
#endif

uint8_t flower_color=0;

struct marker_deviation_t marker_deviation;
struct line_deviation_t line_deviation;
struct centroid_deviation_t centroid_deviation;

/* Initialize the default settings for the vision algorithm */
struct visionhover_param_t visionhover_param = {
  .M = VH_M,
  .m = VH_m,
  .t = VH_t,
  .radius = VH_RADIUS,
  .IN = VH_IN,
  .line_w = VH_LINE_W,
  .line_thr = VH_LINE_THR
};
struct visionhover_stab_t visionhover_stab;


/* Initialize the default settings for the flower detect algorithm */
struct flowerdetect_param_t flowerdetect_param = {
  .red_y_m = RED_Y_MIN,
  .red_y_M = RED_Y_MAX,
  .red_u_m = RED_U_MIN,
  .red_u_M = RED_U_MAX,
  .red_v_m = RED_V_MIN,
  .red_v_M = RED_V_MAX,
  
  .blue_y_m = BLUE_Y_MIN,
  .blue_y_M = BLUE_Y_MAX,
  .blue_u_m = BLUE_U_MIN,
  .blue_u_M = BLUE_U_MAX,
  .blue_v_m = BLUE_V_MIN,
  .blue_v_M = BLUE_V_MAX,
  
  .counter_tresh = COUNTER_TRESH
};

/* Functions only used here */
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime);
static int cmp_flow(const void *a, const void *b);

uint8_t QR_read_already;

/**
 * Initialize the opticflow calculator
 * @param[out] *opticflow The new optical flow calculator
 * @param[in] *w The image width
 * @param[in] *h The image height
 */
void opticflow_calc_init(struct opticflow_t *opticflow, uint16_t w, uint16_t h)
{
  /* Create the image buffers */
  image_create(&opticflow->img_gray, w, h, IMAGE_GRAYSCALE);
  image_create(&opticflow->prev_img_gray, w, h, IMAGE_GRAYSCALE);

  /* Set the previous values */
  opticflow->got_first_img = FALSE;
  opticflow->prev_phi = 0.0;
  opticflow->prev_theta = 0.0;
  opticflow->prev_deviation_x = 0.0;
  opticflow->prev_deviation_y = 0.0;
  
  /* Set the default values */
  opticflow->max_track_corners = OPTICFLOW_MAX_TRACK_CORNERS;
  opticflow->window_size = OPTICFLOW_WINDOW_SIZE;
  opticflow->subpixel_factor = OPTICFLOW_SUBPIXEL_FACTOR;
  opticflow->max_iterations = OPTICFLOW_MAX_ITERATIONS;
  opticflow->threshold_vec = OPTICFLOW_THRESHOLD_VEC;

  opticflow->fast9_adaptive = OPTICFLOW_FAST9_ADAPTIVE;
  opticflow->fast9_threshold = OPTICFLOW_FAST9_THRESHOLD;
  opticflow->fast9_min_distance = OPTICFLOW_FAST9_MIN_DISTANCE;
}

/**
 * Run the optical flow on a new image frame
 * @param[in] *opticflow The opticalflow structure that keeps track of previous images
 * @param[in] *state The state of the drone
 * @param[in] *img The image frame to calculate the optical flow from
 * @param[out] *result The optical flow result
 */
void opticflow_calc_frame(struct opticflow_t *opticflow, struct opticflow_state_t *state, struct image_t *img, struct opticflow_result_t *result)
{
  // Update FPS for information
  float dt = (timeval_diff(&opticflow->prev_timestamp, &img->ts) / 1000.);
  if (dt == 0)
  {
  	result->fps = 15;
  }
  else
  {
  	result->fps = 1/dt;
  }
  memcpy(&opticflow->prev_timestamp, &img->ts, sizeof(struct timeval));
  /////////////////////////////////////////////
  ////////  QR CODE TESTING starts       //////
  //////// SEONG                         //////
  /////////////////////////////////////////////
  result->qr_result = qrscan(img);
  /*
  if ((qrscan(img) ==1 || qrscan(img) ==2 || qrscan(img) == 3 ) && QR_read_already < 1)
  {
  	result->qr_result = qrscan(img);
  	QR_read_already = 1;
  }
  */
  
  /////////////////////////////////////////////
  ////////  QR CODE TESTING   ends       //////
  //////// SEONG                         //////
  /////////////////////////////////////////////  
  
  // Convert image to grayscale
  image_to_grayscale(img, &opticflow->img_gray);

  // Copy to previous image if not set
  if (!opticflow->got_first_img) {
    image_copy(&opticflow->img_gray, &opticflow->prev_img_gray);
    opticflow->got_first_img = TRUE;
  }

  // *************************************************************************************
  // Corner detection
  // *************************************************************************************

  // FAST corner detection (TODO: non fixed threashold)
  struct point_t *corners = fast9_detect(img, opticflow->fast9_threshold, opticflow->fast9_min_distance,
                                         20, 20, &result->corner_cnt);

  // Adaptive threshold
  if (opticflow->fast9_adaptive) {

    // Decrease and increase the threshold based on previous values
    if (result->corner_cnt < 40 && opticflow->fast9_threshold > 5) {
      opticflow->fast9_threshold--;
    } else if (result->corner_cnt > 50 && opticflow->fast9_threshold < 60) {
      opticflow->fast9_threshold++;
    }
  }

#if OPTICFLOW_DEBUG && OPTICFLOW_SHOW_CORNERS
  image_show_points(img, corners, result->corner_cnt);
#endif

  // Check if we found some corners to track
  if (result->corner_cnt < 1) {
    free(corners);
    image_copy(&opticflow->img_gray, &opticflow->prev_img_gray);
    return;
  }

  // *************************************************************************************
  // Corner Tracking
  // *************************************************************************************

  // Execute a Lucas Kanade optical flow
  result->tracked_cnt = result->corner_cnt;
  struct flow_t *vectors = opticFlowLK(&opticflow->img_gray, &opticflow->prev_img_gray, corners, &result->tracked_cnt,
                                       opticflow->window_size / 2, opticflow->subpixel_factor, opticflow->max_iterations,
                                       opticflow->threshold_vec, opticflow->max_track_corners);

#if OPTICFLOW_DEBUG && OPTICFLOW_SHOW_FLOW
  image_show_flow(img, vectors, result->tracked_cnt, opticflow->subpixel_factor);
#endif

  // Get the median flow
  qsort(vectors, result->tracked_cnt, sizeof(struct flow_t), cmp_flow);
  if (result->tracked_cnt == 0) {
    // We got no flow
    result->flow_x = 0;
    result->flow_y = 0;
  } else if (result->tracked_cnt > 3) {
    // Take the average of the 3 median points
    result->flow_x = vectors[result->tracked_cnt / 2 - 1].flow_x;
    result->flow_y = vectors[result->tracked_cnt / 2 - 1].flow_y;
    result->flow_x += vectors[result->tracked_cnt / 2].flow_x;
    result->flow_y += vectors[result->tracked_cnt / 2].flow_y;
    result->flow_x += vectors[result->tracked_cnt / 2 + 1].flow_x;
    result->flow_y += vectors[result->tracked_cnt / 2 + 1].flow_y;
    result->flow_x /= 3;
    result->flow_y /= 3;
  } else {
    // Take the median point
    result->flow_x = vectors[result->tracked_cnt / 2].flow_x;
    result->flow_y = vectors[result->tracked_cnt / 2].flow_y;
  }

  // Flow Derotation
  float diff_flow_x = (state->phi - opticflow->prev_phi) * img->w / OPTICFLOW_FOV_W;
  float diff_flow_y = (state->theta - opticflow->prev_theta) * img->h / OPTICFLOW_FOV_H;
  result->flow_der_x = result->flow_x - diff_flow_x * opticflow->subpixel_factor;
  result->flow_der_y = result->flow_y - diff_flow_y * opticflow->subpixel_factor;
  opticflow->prev_phi = state->phi;
  opticflow->prev_theta = state->theta;

  // Velocity calculation
  result->vel_x = -result->flow_der_x * result->fps * state->agl/ opticflow->subpixel_factor * img->w / OPTICFLOW_FX;
  result->vel_y =  result->flow_der_y * result->fps * state->agl/ opticflow->subpixel_factor * img->h / OPTICFLOW_FY;

  // *************************************************************************************
  // Next Loop Preparation
  // *************************************************************************************
  free(corners);
  free(vectors);
  image_switch(&opticflow->img_gray, &opticflow->prev_img_gray);
  
  /////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////
  ////////                                                                                /////////
  ////////               Vision hover algorithm here                                      /////////
  ////////                                                                                /////////
  /////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////
  
  if (!visionhover_stab.line_follow)
  {
	  if (visionhover_stab.flower_mode)
	  {
		  /* flower detection */
		    uint16_t counter_red=image_yuv422_colorfilt_simple(img, flowerdetect_param.red_y_m, flowerdetect_param.red_y_M, flowerdetect_param.red_u_m,
		                        flowerdetect_param.red_u_M, flowerdetect_param.red_v_m, flowerdetect_param.red_v_M);
	      uint16_t counter_blue=image_yuv422_colorfilt_simple(img, flowerdetect_param.blue_y_m, flowerdetect_param.blue_y_M, flowerdetect_param.blue_u_m,
		                        flowerdetect_param.blue_u_M, flowerdetect_param.blue_v_m, flowerdetect_param.blue_v_M);
	      //static uint16_t counter_tresh=2000; // min number of pixels to consider a blob to be a flower                         
		      
		    #if (OPTICFLOW_DEBUG)
            centroid_deviation=image_centroid(img, img, flowerdetect_param.red_y_m, flowerdetect_param.red_y_M, flowerdetect_param.red_u_m,
                                  flowerdetect_param.red_u_M, flowerdetect_param.red_v_m, flowerdetect_param.red_v_M);
            centroid_deviation=image_centroid(img, img, flowerdetect_param.blue_y_m, flowerdetect_param.blue_y_M, flowerdetect_param.blue_u_m,
                                  flowerdetect_param.blue_u_M, flowerdetect_param.blue_v_m, flowerdetect_param.blue_v_M);
        #endif 

	      if (counter_red > counter_blue && counter_red > flowerdetect_param.counter_tresh)
		{
		    result->flower_type=1;
		    //printf("red flower \n");
		}
	      else if (counter_blue > counter_red && counter_blue > flowerdetect_param.counter_tresh)
    {
        result->flower_type=3;
        //printf("blue flower \n");
    }
     //   else if (counter_green > counter_red && counter_green > counter_blue && counter_green > counter_tresh)
		//{
		//    flower_color=2;
		    //printf("green flower \n");
		//}
	      else
		{
		    result->flower_type=0;
		    //printf("no flower \n");
		}
      
      	  //printf("%i, %i, %i \n",counter_red, counter_blue, flower_color);
      	  //result->flower_type = flower_color;
	  }
          else
          {
          	result->flower_type = 0;
          }
    
	  /* marker detection */
	  marker_deviation = marker(img, img, visionhover_param.M, visionhover_param.m, visionhover_param.t, visionhover_param.radius, visionhover_param.IN);
	  result->deviation_x = marker_deviation.x;
	  result->deviation_y = marker_deviation.y;
	  result->inlier = marker_deviation.inlier;
  }
  else
  {
  	  line_deviation = line_follow(img, img, visionhover_param.line_w, visionhover_param.line_thr);
	  result->deviation_x = line_deviation.x;
	  result->deviation_y = line_deviation.y;
	  result->inlier = 111;
  }
  

}

/**
 * Calculate the difference from start till finish
 * @param[in] *starttime The start time to calculate the difference from
 * @param[in] *finishtime The finish time to calculate the difference from
 * @return The difference in milliseconds
 */
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime)
{
  uint32_t msec;
  msec = (finishtime->tv_sec - starttime->tv_sec) * 1000;
  msec += (finishtime->tv_usec - starttime->tv_usec) / 1000;
  return msec;
}

/**
 * Compare two flow vectors based on flow distance
 * Used for sorting.
 * @param[in] *a The first flow vector (should be vect flow_t)
 * @param[in] *b The second flow vector (should be vect flow_t)
 * @return Negative if b has more flow than a, 0 if the same and positive if a has more flow than b
 */
static int cmp_flow(const void *a, const void *b)
{
  const struct flow_t *a_p = (const struct flow_t *)a;
  const struct flow_t *b_p = (const struct flow_t *)b;
  return (a_p->flow_x * a_p->flow_x + a_p->flow_y * a_p->flow_y) - (b_p->flow_x * b_p->flow_x + b_p->flow_y * b_p->flow_y);
}
