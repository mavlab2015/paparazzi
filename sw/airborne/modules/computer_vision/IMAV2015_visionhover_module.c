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
 * @file modules/computer_vision/IMAV2015_visionhover_module.c
 * @brief vision-based hovering for Parrot AR.Drone 2.0
 *
 * Sensors from vertical camera and IMU of Parrot AR.Drone 2.0
 */


#include "IMAV2015_visionhover_module.h"

#include <stdio.h>
#include <pthread.h>
#include "state.h"
#include "subsystems/abi.h"

#include "lib/v4l/v4l2.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"

/* Default sonar/agl to use in visual_estimator */
#ifndef VISIONHOVER_AGL_ID
#define VISIONHOVER_AGL_ID ABI_BROADCAST    ///< Default sonar/agl to use in visual_estimator
#endif
PRINT_CONFIG_VAR(VISIONHOVER_AGL_ID)

/* The video device */
#ifndef VISIONHOVER_DEVICE
#define VISIONHOVER_DEVICE /dev/video2      ///< The video device
#endif
PRINT_CONFIG_VAR(VISIONHOVER_DEVICE)

/* The video device size (width, height) */
#ifndef VISIONHOVER_DEVICE_SIZE
#define VISIONHOVER_DEVICE_SIZE 320,240     ///< The video device size (width, height)
#endif
#define __SIZE_HELPER(x, y) #x", "#y
#define _SIZE_HELPER(x) __SIZE_HELPER(x)
PRINT_CONFIG_MSG("VISIONHOVER_DEVICE_SIZE = " _SIZE_HELPER(VISIONHOVER_DEVICE_SIZE))

/* The video device buffers (the amount of V4L2 buffers) */
#ifndef VISIONHOVER_DEVICE_BUFFERS
#define VISIONHOVER_DEVICE_BUFFERS 15       ///< The video device buffers (the amount of V4L2 buffers)
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_DEVICE_BUFFERS)

/* The main visionhover variables */

static struct visionhover_result_t visionhover_result; ///< The vision result
static struct v4l2_device *visionhover_dev;          ///< The visionhover camera V4L2 device
static struct visionhover_state_t visionhover_state;   ///< State of the drone to communicate with the visionhover
static abi_event visionhover_agl_ev;                 ///< The altitude ABI event
static pthread_t visionhover_calc_thread;            ///< The visionhover flow calculation thread
static bool_t visionhover_got_result;                ///< When we have a vision calculation
static pthread_mutex_t visionhover_mutex;            ///< Mutex lock fo thread safety

/* Static functions */
static void *visionhover_module_calc(void *data);                   ///< The main vision calculation thread
static void visionhover_agl_cb(uint8_t sender_id, float distance);  ///< Callback function of the ground altitude

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
/**
 * Send thetelemetry information
 * @param[in] *trans The transport structure to send the information over
 * @param[in] *dev The link to send the data over
 */
static void visionhover_telem_send(struct transport_tx *trans, struct link_device *dev)
{
  pthread_mutex_lock(&visionhover_mutex);
  pprz_msg_send_VISION_HOVER_EST(trans, dev, AC_ID, &visionhover_result.deviation_x,  &visionhover_result.deviation_y);
  pthread_mutex_unlock(&visionhover_mutex);
}
#endif

/**
 * Initialize the vision-based hover module for the bottom camera
 */
void visionhover_module_init(void)
{

  // Subscribe to the altitude above ground level ABI messages
  AbiBindMsgAGL(VISIONHOVER_AGL_ID, &visionhover_agl_ev, visionhover_agl_cb);
  
  // Initialize the visionhover calculation
  visionhover_got_result = FALSE;
  /* Try to initialize the video device */
  visionhover_dev = v4l2_init(STRINGIFY(VISIONHOVER_DEVICE), VISIONHOVER_DEVICE_SIZE, VISIONHOVER_DEVICE_BUFFERS);
  if (visionhover_dev == NULL) {
    printf("[visionhover_module] Could not initialize the video device\n");
  }
  
  #if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "VISION_HOVER_EST", visionhover_telem_send);
  #endif
}

/**
 * Update the vision state for the calculation thread
 * and update the stabilization loops with the newest result
 */
void visionhover_module_run(void)
{
  pthread_mutex_lock(&visionhover_mutex);
  stabilization_visionhover_update(&visionhover_result);
  //printf("Vision results are: %.0f, %.0f\n", visionhover_result.deviation_x, visionhover_result.deviation_y);
  pthread_mutex_unlock(&visionhover_mutex);
}

/**
 * Start the vision calculation
 */
void visionhover_module_start(void)
{
  // Check if we are not already running
  if (visionhover_calc_thread != 0) {
    printf("[visionhover_module] VisionHover already started!\n");
    return;
  }

  // Create the visionhover calculation thread
  int rc = pthread_create(&visionhover_calc_thread, NULL, visionhover_module_calc, NULL);
  if (rc) {
    printf("[visionhover_module] Could not initialize visionhover thread (return code: %d)\n", rc);
  }
}

/**
 * Stop the visionhover calculation
 */
void visionhover_module_stop(void)
{
  // Stop the capturing
  v4l2_stop_capture(visionhover_dev);

  // TODO: fix thread stop
}

/**
 * The main visionhover flow calculation thread
 */
#include "errno.h"
static void *visionhover_module_calc(void *data __attribute__((unused)))
{
  // Start the streaming on the V4L2 device
  if (!v4l2_start_capture(visionhover_dev)) {
    printf("[visionhover_module] Could not start capture of the camera\n");
    return 0;
  }

#if VISIONHOVER_DEBUG
  // Create a new JPEG image
  struct image_t img_jpeg;
  image_create(&img_jpeg, visionhover_dev->w, visionhover_dev->h, IMAGE_JPEG);
#endif

  /* Main loop of the vision hover calculation */
  while (TRUE) {
    // Try to fetch an image
    struct image_t img;
    v4l2_image_get(visionhover_dev, &img);

     // Copy the state
    pthread_mutex_lock(&visionhover_mutex);
    struct visionhover_state_t temp_state;
    memcpy(&temp_state, &visionhover_state, sizeof(struct visionhover_state_t));
    pthread_mutex_unlock(&visionhover_mutex);
    
    // Do the vision calculation
    struct visionhover_result_t temp_result;
    visionhover_calc_frame(&img, &temp_state, &temp_result);
    //printf("Altitude is: %f, Vision results are: %f, %f\n", temp_state.agl, temp_result.deviation_x, temp_result.deviation_y);
  
    
    // Copy the result if finished
    pthread_mutex_lock(&visionhover_mutex);
    memcpy(&visionhover_result, &temp_result, sizeof(struct visionhover_result_t));
    visionhover_got_result = TRUE;
    pthread_mutex_unlock(&visionhover_mutex);

#if VISIONHOVER_DEBUG
    jpeg_encode_image(&img, &img_jpeg, 70, FALSE);
    rtp_frame_send(
      &VIEWVIDEO_DEV,           // UDP device
      &img_jpeg,
      0,                        // Format 422
      70, // Jpeg-Quality
      0,                        // DRI Header
      0                         // 90kHz time increment
    );
#endif

    // Free the image
    v4l2_image_free(visionhover_dev, &img);
  }

#if VISIONHOVER_DEBUG
  image_free(&img_jpeg);
#endif
}

/**
 * Get the altitude above ground of the drone
 * @param[in] sender_id The id that send the ABI message (unused)
 * @param[in] distance The distance above ground level in meters
 */
static void visionhover_agl_cb(uint8_t sender_id __attribute__((unused)), float distance)
{
  // Update the distance if we got a valid measurement
  if (distance > 0) {
    visionhover_state.agl = distance;
  }
}









////////////////////////////////////////////////////////////////////
/////////                                                 //////////
/////////       Comment from Seong below... 13-07-2015    //////////
/////////                                                 //////////
////////////////////////////////////////////////////////////////////

/*

I tried to printf the vision result using the following code:
	static void *visionhover_module_calc(void *data __attribute__((unused)))
	{
	  ....
	    struct visionhover_result_t temp_result;
	    visionhover_calc_frame(&img, &temp_result);
	    printf("Vision results are: %.0f, %.0f\n", temp_result.deviation_x, temp_result.deviation_y);
	  ....
        }
        
They worked perfectly fine. However, when I tried the similar thing with:
   
	stabilization_visionhover_update(&visionhover_result);
	
by adding an extra output parameter to the function, it doesn't printf correctly. 
Funny thing is that even when I just add printf("blabla\n") in the stabilization_visionhover_update function in IMAV_stabilization.c,
it will not print it !! Note that if you add it in visionhover_calc_frame in IMAV_vision.c, it prints !!

So for the time being, I have successfully verified that vision algorithm works, but I have not confirmed if the control commands are also correctly computed accordingly.....

*/

////////////////////////////////////////////////////////////////////
/////////                                                 //////////
/////////       Comment from Seong below... 17-07-2015    //////////
/////////                                                 //////////
////////////////////////////////////////////////////////////////////

/*

Answer to the previous comment: The reason why it didn't display the printf in the stabilization_visionhover_update function in IMAV_stabilization.c is because of this line:
	  if (autopilot_mode != AP_MODE_MODULE) {
	    return;
	  }
So the autopilot_mode MUST be AP_MODE_MODULE in order to proceed to the lines afterwards.
And We MUST change the autopilot mode by clicking the button on the joystick !! 
See the AUTOPILOT section in our airframe "ardrone2_raw_IMAV2015.xml" !!
*/

