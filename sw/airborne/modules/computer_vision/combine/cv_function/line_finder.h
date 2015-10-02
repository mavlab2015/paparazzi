/*
 * Copyright (C) 2015 Seong Hun Lee
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */
 /**
 * Compute the 2D image waypoint for line-following mission.
 * @param[in] *input The input image
 * @param[out] *output The output image
 * @param[in] w Evaluation width for the pixel (estimated line width in pixel)
 * @param[in] th Threshold for line detection.
 * @return The deviation of the 2D waypoint wrt the center.
 */


#include <stdint.h>
#include "modules/computer_vision/lib/vision/image.h"

struct line_deviation_t
{
	int32_t x;
	int32_t y;
	uint16_t inlier;
};

inline struct line_deviation_t line_follow(struct image_t *input, struct image_t *output, uint8_t w, uint8_t th)
{
  struct line_deviation_t line_deviation;
  
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;
  uint16_t x, y, i, j;
  int32_t integral1[240] = {};
  int32_t integral2[320] = {};
  int32_t integral3[240] = {};
  int32_t integral1_max = 0;
  int32_t integral2_max = 0;
  int32_t integral3_max = 0;
  int32_t integral_max_max = 0;
  uint16_t idx1 = 0;
  uint16_t idx2 = 0;
  uint16_t idx3 = 0;
  int32_t sum1 = 0;
  int32_t sum2 = 0;
  int32_t sum3 = 0;
  uint16_t max_idx1[2][1] = {};
  uint16_t max_idx2[2][1] = {};
  uint16_t max_idx3[2][1] = {};
  uint32_t max_idx[2][1] = {};
  
  uint8_t image[240][320] = {};
  uint16_t counter;

  // Copy the creation timestamp (stays the same)
  memcpy(&output->ts, &input->ts, sizeof(struct timeval));
 
  // Go through all the pixels
  
  counter = 0;
  for (y = 0; y < output->h; y++) 
  {
	  for (x = 0; x < output->w; x+=2) 
	  {
	  	image[y][x] = source[1];
	  	image[y][x+1] = source[3];
	  	// Go to the next 2 pixels
	  	source+=4;
	  	counter +=1;
	  } 
  }
  
  for (i= 0; i < output->h - w+1; i++)
  {
  	sum1 = 0;
  	sum3 = 0;
  	
  	for (j = 0; j < w; j++)
  	{
  		sum1 += image[i+j][0];
  		sum3 += image[i+j][output->w-1];
  	}
  	integral1[i] = sum1;
  	integral3[i] = sum3;
  	
  	if (integral1[idx1] > integral1_max)
  	{
  		integral1_max = integral1[idx1];
  		max_idx1[0][0] = i;
  		max_idx1[1][0] = 0;
  	}
  	idx1 += 1;
  	
  	if (integral3[idx3] > integral3_max)
  	{
  		integral3_max = integral3[idx3];
  		max_idx3[0][0] = i;
  		max_idx3[1][0] = output->w-2;
  	}
  	idx3 += 1;
  	
  }
  
  for (i= 0; i < output->w - w+1; i++)
  {
  	sum2 = 0;
  	for (j = 0; j < w; j++)
  	{
  		sum2 += image[0][i+j];
  	}
  	integral2[i] = sum2;
  	if (integral2[idx2] > integral2_max)
  	{
  		integral2_max = integral2[idx2];
  		max_idx2[0][0] = 0;
  		max_idx2[1][0] = i;
  	}
  	idx2 += 1;
  }

  if (integral1_max > integral_max_max)
  	integral_max_max = integral1_max;
  if (integral2_max > integral_max_max)
  	integral_max_max = integral2_max;
  if (integral3_max > integral_max_max)
  	integral_max_max = integral3_max;
  
  if (integral_max_max < w*th)
  {	
  	max_idx[0][0] = (output->h) / 2;
  	max_idx[1][0] = (output->w) / 2;
  }
  else
  {
  	 if (integral2_max == integral_max_max || integral2_max > w*th)
  	 {
	  	max_idx[0][0] = max_idx2[0][0];
	  	max_idx[1][0] = max_idx2[1][0];
 	 }
 	 else
 	 {
	 	 if (integral1_max > w*th && integral3_max < w*th)
	  	 {
	  		max_idx[0][0] = max_idx1[0][0];
	  		max_idx[1][0] = max_idx1[1][0];
	  	 }
	  	 if (integral3_max > w*th && integral1_max < w*th)
	  	 {
	  		max_idx[0][0] = max_idx3[0][0];
	  		max_idx[1][0] = max_idx3[1][0];
	  	 }
	  	 if (integral1_max > w*th && integral3_max > w*th)
	  	 {
	  		if (max_idx1[0][0] < max_idx3[0][0])
	  		{
	  			max_idx[0][0] = max_idx1[0][0];
	  			max_idx[1][0] = max_idx1[1][0];
	  		}
	  		else
	  		{
	  			max_idx[0][0] = max_idx3[0][0];
	  			max_idx[1][0] = max_idx3[1][0];
	  		}
	  		
	  	 }
 	 }
  }	
 

    
    line_deviation.x = max_idx[1][0] - (output->w)/2;
    line_deviation.y = -max_idx[0][0] + (output->h)/2;
    //line_deviation.inlier = ;
    
    // Display the marker location and center-lines.
    source-=4*counter;
    for (y = 0; y < output->h; y++) 
    {
    	for (x = 0; x < output->w; x+=2) 
    	{
	      	dest[0] = source[0]; 	    //U
		dest[1] = source[1];        //Y
		dest[2] = source[2];        //V
		dest[3] = source[3];        //Y
	 	
	 	if ( y == (output->h)/2 || x == (output->w)/2)
    		{	
	    		dest[0] = 250;       // U
			dest[2] = 60;        // V
      		}
      		if (y == max_idx[0][0] || x == max_idx[1][0])
      		{
	      		dest[0] = 65;        //U
			dest[2] = 255;       //V
      		}
      		dest+=4;
      		source+=4;
    	}
    }
    
    //printf("The number of inliers = %i\n", counter3);
    return line_deviation;
}
