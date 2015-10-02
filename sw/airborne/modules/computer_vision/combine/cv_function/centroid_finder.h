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
 * Create a binary map after filtering colors in an YUV422 image, then calculate the centroid location wrt center.
 * @param[in] *input The input image to filter
 * @param[out] *output The filtered output image
 * @param[in] y_m The Y minimum value
 * @param[in] y_M The Y maximum value
 * @param[in] u_m The U minimum value
 * @param[in] u_M The U maximum value
 * @param[in] v_m The V minimum value
 * @param[in] v_M The V maximum value
 * @return The location of the filtered image centroid.
 */

#include <stdint.h>
#include "modules/computer_vision/lib/vision/image.h"

struct centroid_deviation_t
{
	float x;
	float y;
};

inline struct centroid_deviation_t image_centroid(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M, uint8_t u_m,
                                uint8_t u_M, uint8_t v_m, uint8_t v_M)
{
  struct centroid_deviation_t centroid_deviation;
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;
  uint16_t x, y;
  int bin[240][320];
  int sum_row, moment_row_sum, sum_col, moment_col_sum, image_total = 0;
  int x_centroid, y_centroid;
  int counter;
  

  // Copy the creation timestamp (stays the same)
  memcpy(&output->ts, &input->ts, sizeof(struct timeval));

  // Go trough all the pixels
  counter = 0;
  for (y = 0; y < output->h; y++) 
  {
    for (x = 0; x < output->w; x+=2) 
    {
      // Check if the color is inside the specified values
      if (
        (dest[1] >= y_m)
        && (dest[1] <= y_M)
        && (dest[0] >= u_m)
        && (dest[0] <= u_M)
        && (dest[2] >= v_m)
        && (dest[2] <= v_M)
         )
       {
        // UYVY
        dest[0] = 255;         // U 0
        dest[1] = source[1];  // Y 255
        dest[2] = 255;        // V 0
        dest[3] = source[3];  // Y 255
        bin[y][x] = 1;
        bin[y][x+1] = 1;
       } 
      else {
        // UYVY
        dest[0] = source[0];  // U 
        dest[1] = source[1];  // Y 
        dest[2] = source[2];  // V 
        dest[3] = source[3];  // Y 
        bin[y][x] = 0;
        bin[y][x+1] = 0;
      }

      // Go to the next 2 pixels
      dest+=4;
      source+=4;
      counter+=1;
    }
  }
    
    // Calculate the y-position of the centroid.
    moment_row_sum = 0;
    for (y = 0; y < 240; y++) 
    {
	    sum_row = 0;
	    for (x = 0; x < 320; x++) 
	    {
	     sum_row = sum_row + bin[y][x];
	    }
	    image_total = image_total + sum_row;
	    moment_row_sum = moment_row_sum + sum_row * y;
    }

    
    if (image_total == 0) {y_centroid = 120;}
    else { y_centroid = moment_row_sum/image_total;}
    
    // Calculate the x-position of the centroid.
    moment_col_sum = 0;
    for (x = 0; x < 320; x++) 
    {
    	    sum_col = 0;
	    for (y = 0; y < 240; y++) 
	    {
	     sum_col = sum_col + bin[y][x];
	    }
	    moment_col_sum = moment_col_sum + sum_col * x;
    }
    
    if (image_total == 0) {x_centroid = 160;}
    else { x_centroid = moment_col_sum/image_total;}
    
   
    centroid_deviation.x = x_centroid - (output->w)/2;
    centroid_deviation.y = -y_centroid + (output->h)/2;
    
    
    dest-=4*counter;
    source-=4*counter;
    for (y = 0; y < output->h; y++) 
    {
    	for (x = 0; x < output->w; x+=2) 
    	{
    		dest[1] = source[1]; //Y
    		dest[3] = source[3]; //Y
    		
    		if ( y == (output->h)/2 || x == (output->w)/2)
    		{
	    		dest[0] = 200;       // U
			dest[2] = 60;        // V
      		}
      		if (y == y_centroid || x == x_centroid)
      		{
	      		dest[0] = 60;       // U
			dest[2] = 200;        // V
      		}
         	else
      		{
	      		dest[0] = source[0];    // U
			dest[2] = source[2];    // V
      		}
        	
      		dest+=4;
      		source+=4;
    	}
    }
    
    return centroid_deviation;

}



