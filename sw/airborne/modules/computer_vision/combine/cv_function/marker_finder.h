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
 * Find the marker location.
 * @param[in] *input The input image to filter
 * @param[in] M The distance between the pixel of interest and farthest neighbor pixel [pixel]
 * @param[in] m The safety margin around the pixel of interest [pixel]
 * @param[in] t Threshold for intensity difference
 * @param[in] radius The radius used for inlier detection.
 * @param[in] IN The number of minimum inliers required
 * @return The deviation of the marker location wrt the center.
 */


#include <stdint.h>
#include "modules/computer_vision/lib/vision/image.h"

struct marker_deviation_t
{
	int32_t x;
	int32_t y;
	uint16_t inlier;
};

inline struct marker_deviation_t marker(struct image_t *input, struct image_t *output, uint8_t M, uint8_t m, uint8_t t, uint8_t radius, uint8_t IN )
{
  struct marker_deviation_t marker_deviation;
  
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;
  uint16_t x, y, i, j, k, l, n, o;
  uint8_t image[240][320] = {};
  uint16_t idx[2][200] = {};
  uint32_t counter1, counter2, counter3, counter4;
  int32_t min1, max1, min2, max2, min3, max3, min4, max4;
  uint32_t marker_x = 0;
  uint32_t marker_y = 0;
  uint32_t inlier[3][200] = {};
  uint32_t top_inlier;
  uint32_t inlier_final = 0;

  // Copy the creation timestamp (stays the same)
  memcpy(&output->ts, &input->ts, sizeof(struct timeval));

  // Go trough all the pixels
  counter1 = 0;
  for (y = 0; y < output->h; y++) 
  {
	  for (x = 0; x < output->w; x+=2) 
	  {
	  	image[y][x] = source[1];
	  	image[y][x+1] = source[3];
	  	// Go to the next 2 pixels
	  	source+=4;
	  	counter1+=1;
	  } 
  }
  
  counter2 = 0;
  
  for (i = M; i < output->h -M; i++)
  {
  	for (j = M; j < output->w -M; j++)
  	{
  		
  	 	// 1st stage: FAST-like check	
  	 	if ((image[i][j]<image[i-M][j]-t && image[i][j]<image[i+M][j]-t 
  	  		&& image[i][j]>image[i][j-M]+t && image[i][j]>image[i][j+M]+t) 
		|| 
  	   		(image[i][j]<image[i][j-M]-t && image[i][j]<image[i][j+M]-t 
  	   		&& image[i][j]>image[i-M][j]+t && image[i][j]>image[i+M][j]+t)
  	   	|| 
  	   		(image[i][j]<image[i-M][j-M]-t && image[i][j]<image[i+M][j+M]-t 
  	   		&& image[i][j]>image[i-M][j+M]+t && image[i][j]>image[i+M][j-M]+t)      
  	   	|| 
  	   		(image[i][j]<image[i-M][j+M]-t && image[i][j]<image[i+M][j-M]-t 
  	   		&& image[i][j]>image[i-M][j-M]+t && image[i][j]>image[i+M][j+M]+t))
	   	{	
			min1=999; max1=-1; min2=999; max2=-1; min3=999; max3=-1; min4=999; max4=-1;
			
			// 2nd stage: max & min comparison between different neighbor groups.
			for (k = m; k < M+1; k++)
			{
				if (image[i+k][j] < min1) {min1 = image[i+k][j];}
				if (image[i+k][j] > max1) {max1 = image[i+k][j];}
				if (image[i][j+k] < min2) {min2 = image[i][j+k];}
				if (image[i][j+k] > max2) {max2 = image[i][j+k];}
				if (image[i+k][j+k] < min3) {min3 = image[i+k][j+k];}
				if (image[i+k][j+k] > max3) {max3 = image[i+k][j+k];}
				if (image[i+k][j-k] < min4) {min4 = image[i+k][j-k];}
				if (image[i+k][j-k] > max4) {max4 = image[i+k][j-k];}
				
				if (image[i-k][j] < min1) {min1 = image[i-k][j];}
				if (image[i-k][j] > max1) {max1 = image[i-k][j];}
				if (image[i][j-k] < min2) {min2 = image[i][j-k];}
				if (image[i][j-k] > max2) {max2 = image[i][j-k];}
				if (image[i-k][j-k] < min3) {min3 = image[i-k][j-k];}
				if (image[i-k][j-k] > max3) {max3 = image[i-k][j-k];}
				if (image[i-k][j+k] < min4) {min4 = image[i-k][j+k];}
				if (image[i-k][j+k] > max4) {max4 = image[i-k][j+k];}
			}
	
			if ((image[i][j] > max1+t && image[i][j] < min2-t) || (image[i][j] > max2+t && image[i][j] < min1-t)
			  || (image[i][j] > max3+t && image[i][j] < min4-t) || (image[i][j] > max4+t && image[i][j]<min3-t))
			{
				counter2 = counter2 + 1;
				idx[0][counter2] = i;
				idx[1][counter2] = j;
			}    
		}    
  	}
  }
  
  
    // 3rd stage: Outlier rejection:
    
    counter3 = 0; // Counter for the final inlier index.
    for (l = 1; l < counter2+1; l++)
    {
  	counter4 = 0; // Number of inliers corresponding to idx[][l].
  	for (n = 1; n < counter2+1; n++)
  	{
  		if (sqrt(pow((idx[0][l]-idx[0][n]), 2) + pow((idx[1][l]-idx[1][n]), 2)) < radius)
  		{
  			counter4 += 1;
  			if (counter4 > IN)
  			{
  				counter3 += 1;
      				inlier[0][counter3] = idx[0][l]; // i (row) value of "l"th idx
      				inlier[1][counter3] = idx[1][l]; // j (col) value of "l"th idx
    	  			inlier[2][counter3] = counter4 ; // Number of inliers for "l"th idx
	  		}
	  	}
	}
    }

    // Finally: find the inlier which is located at the top.
    if (counter3 > 0)
    {
  	top_inlier = output->h;
  	for (o = 1; o < counter3+1; o++)
  	{
  		if (inlier[0][o] < top_inlier)
  		{
  			top_inlier = inlier[0][o];
  			marker_x = inlier[1][o];
  			marker_y = inlier[0][o];
  			inlier_final = inlier[2][o];
  		} 
  	}
    }
    else
    {
    	marker_x = (output->w)/2;
    	marker_y = (output->h)/2; 
    	inlier_final = 0;
    }
    
    
    
    marker_deviation.x = marker_x - (output->w)/2;
    marker_deviation.y = -marker_y + (output->h)/2;
    marker_deviation.inlier = inlier_final;
    
    // Display the marker location and center-lines.
    source-=4*counter1;
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
      		if (y == marker_y || x == marker_x)
      		{
	      		dest[0] = 65;        //U
			dest[2] = 255;       //V
      		}
      		dest+=4;
      		source+=4;
    	}
    }
    
    //printf("The number of inliers = %i\n", counter3);
    return marker_deviation;

}
