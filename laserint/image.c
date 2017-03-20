

/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** Welcome!
 *****
 ***** This file is part of the robot control software provided
 ***** by Real World Interface Inc.
 *****
 ***** All copyrights are by Real World Interface Inc., Carnegie
 ***** Mellon University, and the University of Bonn, Germany.
 ***** Removing the copyright information is illegal. Please read
 ***** and make sure you understand the disclaimer below.
 *****
 ***** Contact tyson@rwii.com if you have questions or would
 ***** like to obtain further information.
 *****
 ***** We hope you enjoy working with this software package.
 *****
 *****                      Tyson D. Sawyer and Sebastian Thrun
 *****
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** 
 ***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED
 ***** BY APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING
 ***** THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM
 ***** "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR
 ***** IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 ***** OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE
 ***** ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM
 ***** IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME
 ***** THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
 ***** 
 ***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
 ***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
 ***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
 ***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
 ***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
 ***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO
 ***** LOSS OF DATA OR DATA BEING RENDERED INACCURATE OR LOSSES
 ***** SUSTAINED BY YOU OR THIRD PARTIES OR A FAILURE OF THE PROGRAM
 ***** TO OPERATE WITH ANY OTHER PROGRAMS OR FAILURE TO CONTROL A
 ***** PHYSICAL DEVICE OF ANY TYPE), EVEN IF SUCH HOLDER OR OTHER
 ***** PARTY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 ***** Source file:     $Source: /usr/local/cvs/bee/src/laserint/image.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:34:07 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: image.c,v $
 * Revision 1.1  2002/09/14 15:34:07  rstone
 * *** empty log message ***
 *
 * Revision 1.8  2000/09/26 01:11:32  thrun
 * ,
 *
 * Revision 1.7  2000/03/26 16:32:09  thrun
 * Can write image strips now.
 *
 * Revision 1.6  2000/03/25 04:26:00  thrun
 * improved the assignment of line and co-line.
 *
 * Revision 1.5  2000/03/21 05:25:32  thrun
 * extract line.
 *
 * Revision 1.4  2000/03/21 05:07:22  thrun
 * second line extracted from image
 *
 * Revision 1.3  2000/03/18 22:02:54  thrun
 * Improved version of the segmentation.
 *
 * Revision 1.2  2000/03/14 05:23:57  thrun
 * fixed a few problems with the 3D mapping. This version
 * pruced the results first shown to the DAPRA folks.
 * In particular, it reads the texture entirely from file and
 * constructs a new texture file (laserint.ppm, which has
 * to be converted manyally into laserint.jpg)
 *
 * Revision 1.1  2000/03/13 00:31:12  thrun
 * first version of he texturing module. Still a bit clumpsy.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/




#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include "tcx.h"
#include "tcxP.h"
#include "Application.h"
#include "Net.h"
#include "EZX11.h"
#include "o-graphics.h"
#include "MAP-messages.h"
#include "LASER-messages.h"
#include "BASE-messages.h"
#include "LASERINT.h"
#include "bUtils.h"


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

int decimal = 1;
int to_file = 1;
int print_imageline1 = 1;
int print_imageline2 = 0;
int create_subimage = 0;
int align = 0;
int display_image = 0;

/************************************************************************\
 ************************************************************************
\************************************************************************/

#define RANDOM_MAX ((float)(2147483647))   /* (2**31 - 1) */
#define RAND_PLUS_MINUS_ONE() ((2.0 * ((float)(random()) / ((float) RANDOM_MAX))) - 1.0)
#define RAND_ZERO_TO_ONE() ((float)(random()) / ((float) RANDOM_MAX))


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

image_ptr bg_image = NULL;
image_ptr texture_image = NULL;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

image_ptr 
read_image(char *filename)
{
  int i, k, s_x, s_y, number;
  char text[128];
  unsigned char ch;
  FILE *iop;
  image_ptr image;

  if ((iop = fopen(filename, "r")) == 0){
    fprintf(stderr, "ERROR: Could not open image file %s.\n", filename);
    return NULL;
  }

  if (fscanf(iop, "%s", text) == EOF){
    fprintf(stderr, "ERROR: Couldn't find file fomat in image file %s.\n",
	    filename);    
    fclose(iop);
    return NULL;
  }
  if (strcmp(text, "P6")){
    fprintf(stderr, "ERROR: Unable to handle image format %s.\n",
	    filename);    
    fclose(iop);
    return NULL;
  }

  if (fscanf(iop, "%d %d %d", &s_x, &s_y, &number) == EOF){
    fprintf(stderr, "ERROR: Couldn't find image size in image file %s.\n",
	    filename);    
    fclose(iop);
    return NULL;
  }
  if (s_x <= 0 || s_y <= 0 || s_x > 1000 || s_y > 1000){
    fprintf(stderr, "ERROR: Can't handle image format %d %d in file %s.\n",
	    s_x, s_y, filename);    
    fclose(iop);
    return NULL;
  }
  if (number != 255){
    fprintf(stderr, "ERROR: Can't handle color depth %d in file %s.\n",
	    number, filename);    
    fclose(iop);
    return NULL;
  }
  if (s_x != NORMAL_IMAGE_SIZE_X && s_y != NORMAL_IMAGE_SIZE_Y){
    fprintf(stderr, "ERROR: Currently unable to handle image format %d %d in file %s.\n",
	    s_x, s_y, filename);    
    fclose(iop);
    return NULL;
  }

  
  if (fscanf(iop, "%c", &ch) == EOF){
    fprintf(stderr, "ERROR: Format error A in file %s.\n", filename);
    fclose(iop);
    return NULL;
  }
  if (((int) ch) != 10){
    fprintf(stderr, "ERROR: Format error B in file %s: %d.\n", filename,
	    (int) ch);
    fclose(iop);
    return NULL;
  }


  image = (image_ptr) malloc(sizeof(image_type));
  if (!image){
    fprintf(stderr, "ERROR: Out of memory in read_image()\n");
    exit(-1);
  }
  image->size = s_x * s_y;
  image->size_x = s_x;
  image->size_y = s_y;
  image->name = (char *) malloc(sizeof(char) * (strlen(filename)+1));
  strcpy(image->name, filename);
  image->red   = (unsigned char *) malloc(sizeof(unsigned char) * image->size);
  image->green = (unsigned char *) malloc(sizeof(unsigned char) * image->size);
  image->blue  = (unsigned char *) malloc(sizeof(unsigned char) * image->size);
  if (!(image->red) || !(image->green) || !(image->blue)){
    fprintf(stderr, "ERROR: Out of memory in read_image()\n");
    exit(-1);
  }

  for (i = 0; i < image->size; i++){
    for (k = 0; k < 3; k++){
      
      if (fscanf(iop, "%c", &ch) == EOF){
	fprintf(stderr, "ERROR: Insufficient image data in image file %s.\n",
		filename);    
	free(image->red);
	free(image->green);
	free(image->blue);
	free(image);
	fclose(iop);
	return NULL;
      }
      if (k == 0)
	image->red[i] = ch;
      else if (k == 1)
	image->green[i] = ch;
      else
	image->blue[i] = ch;
    }
  }  

  fprintf(stderr, "Image file %s successfully read (%d pixels).\n", filename,
	  image->size);
  fclose(iop);
  return image;
}


image_ptr 
random_image()
{
  int i;
  image_ptr image;

  image = (image_ptr) malloc(sizeof(image_type));
  if (!image){
    fprintf(stderr, "ERROR: Out of memory in read_image()\n");
    exit(-1);
  }
  image->size_x = NORMAL_IMAGE_SIZE_X;
  image->size_y = NORMAL_IMAGE_SIZE_Y;
  image->size = image->size_x * image->size_y;
  image->name = (char *) malloc(sizeof(char) * 80);
  strcpy(image->name, "unknown");

  image->red   = (unsigned char *) malloc(sizeof(unsigned char) * image->size);
  image->green = (unsigned char *) malloc(sizeof(unsigned char) * image->size);
  image->blue  = (unsigned char *) malloc(sizeof(unsigned char) * image->size);
  if (!(image->red) || !(image->green) || !(image->blue)){
    fprintf(stderr, "ERROR: Out of memory in read_image()\n");
    exit(-1);
  }

  for (i = 0; i < image->size; i++){
    image->red[i] = (unsigned char) (RAND_ZERO_TO_ONE() * 256.0);
    image->green[i] = (unsigned char) (RAND_ZERO_TO_ONE() * 256.0);
    image->blue[i] = (unsigned char) (RAND_ZERO_TO_ONE() * 256.0);
  }  

  fprintf(stderr, "Random image constructed (%d pixels).\n",image->size);
  return image;
}


image_ptr 
make_image(int size_x, int size_y)
{
  int i;
  image_ptr image;

  image = (image_ptr) malloc(sizeof(image_type));
  if (!image){
    fprintf(stderr, "ERROR: Out of memory in read_image()\n");
    exit(-1);
  }
  image->size_x = size_x;
  image->size_y = size_y;
  image->size = image->size_x * image->size_y;
  image->name = (char *) malloc(sizeof(char) * 80);
  strcpy(image->name, "unknown");

  image->red   = (unsigned char *) malloc(sizeof(unsigned char) * image->size);
  image->green = (unsigned char *) malloc(sizeof(unsigned char) * image->size);
  image->blue  = (unsigned char *) malloc(sizeof(unsigned char) * image->size);
  if (!(image->red) || !(image->green) || !(image->blue)){
    fprintf(stderr, "ERROR: Out of memory in read_image()\n");
    exit(-1);
  }

  for (i = 0; i < image->size; i++){
    image->red[i] = 255;
    image->green[i] = 0;
    image->blue[i] = 0;
  }  

  fprintf(stderr, "Image constructed (%d pixels).\n",image->size);
  return image;
}



void
fill_region_fraction(image_ptr image, 
		     float from_x, float to_x, 
		     float from_y, float to_y,
		     unsigned char red, unsigned char green,
		     unsigned char blue)
{
  int x, y, i;
  
  if (from_x < 0.0 || from_x > 1.01 || to_x < 0.0 || to_x > 1.01 ||
      from_x > to_x ||
      from_y < 0.0 || from_y > 1.01 || to_y < 0.0 || to_y > 1.01 ||
      from_y > to_y){
    fprintf(stderr, "Format error in fill_region_fraction(): %g %g %g %g.\n",
	    from_x, to_x, from_y, to_y);
    return;
  }


  for (x = (int) (from_x * ((float) image->size_x));
       x <= (int) (to_x * ((float) image->size_x)); x++)
    if (x >= 0 && x < image->size_x)
      for (y = (int) (from_y * ((float) image->size_y));
	   y <= (int) (to_y * ((float) image->size_y)); y++)
	if (y >= 0 && y < image->size_y){
	  i = PIXEL(x, y);
	  image->red[i] = red;
	  image->green[i] = green;
	  image->blue[i] = blue;
	}
}




void
destroy_image(image_ptr *image)
{
  if (!image)
    return;
  if (!(*image))
    return;
  
  if ((*image)->red)
    free((*image)->red);
  else
    fprintf(stderr, "WARNING: corrupted image (no red) in destroy_image().\n");

  if ((*image)->green)
    free((*image)->green);
  else
    fprintf(stderr, 
	    "WARNING: corrupted image (no green) in destroy_image().\n");

  if ((*image)->blue)
    free((*image)->blue);
  else
    fprintf(stderr, 
	    "WARNING: corrupted image (no blue) in destroy_image().\n");
  
  if ((*image)->name)
    free((*image)->name);

  free(*image);
  *image = NULL;

  fprintf(stderr, "Image destroyed.\n");

}


int
save_image(image_ptr image, char *filename)
{
  FILE *iop;
  int i;

  if (!image){
    fprintf(stderr, "ERROR: Unable to save nonexisting image into file %s.\n",
	    filename);
    return 0;
  }
  if (!(image->red) || !(image->blue)  || !(image->green)){
    fprintf(stderr, "ERROR: Unable to save corrupted image into file %s.\n",
	    filename);
    return 0;
  }

  if ((iop = fopen(filename, "w")) == 0){
    fprintf(stderr, "ERROR: Could not open image file %s.\n", filename);
    return 0;
  }

  fprintf(iop, "P6\n%d %d\n255\n", image->size_x, image->size_y);

  for (i = 0; i < image->size; i++)
    fprintf(iop, "%c%c%c", image->red[i], image->green[i], image->blue[i]);

  fprintf(stderr, "Image file %s successfully written (%d pixels).\n", 
	  filename, image->size);
  fclose(iop);
  return 1;
}



void
shift_image(image_ptr image, int dx, int dy)
{
  int from_x, to_x, delta_x;
  int from_y, to_y, delta_y;
  int x, y;
  
  if (dx >= 0){
    from_x  = image->size_x - 1;
    to_x    = -1;
    delta_x = -1;
  }
  else if (dx < 0){
    from_x  = 0;
    to_x    = image->size_x;
    delta_x = 1;
  }
  
  if (dy >= 0){
    from_y  = image->size_y - 1;
    to_y    = -1;
    delta_y = -1;
  }
  else if (dy < 0){
    from_y  = 0;
    to_y    = image->size_y;
    delta_y = 1;
  }
  

  for (x = from_x; x != to_x; x += delta_x){
    if (x >= 0 && x < image->size_x){
      for (y = from_y; y != to_y; y += delta_y){
	if (y >= 0 && y < image->size_y){
	  int destination = PIXEL(x, y);
	  int source      = PIXEL(x - dx, y - dy);
	  if (x - dx >= 0 && x - dx < image->size_x &&
	      y - dy >= 0 && y - dy < image->size_y){
	    image->red[destination] = image->red[source];
	    image->green[destination] = image->green[source];
	    image->blue[destination] = image->blue[source];
	  }
	  else{
	    image->red[destination] = 0;
	    image->green[destination] = 0;
	    image->blue[destination] = 0;
	  }
	}
	else
	  fprintf(stderr, "Yucc-Y: %d\n", y);
      }
    }
    else
      fprintf(stderr, "Yucc-X: %d\n", x);
  }
}



void
add_ref_line_to_image(image_ptr image)
{
  int line[][2] = {
    {91, 312},
    {112, 275},
    {136, 236},
    {164, 194},
    {204, 154},
    {249, 125},
    {290, 113},
    {329, 101},
    {382, 97},
    {431, 94},
    {480, 101},
    {522, 114},
    {551, 135},
    {575, 163},
    {604, 206},
    {625, 244},
    {646, 289},
    {661, 327},
    /*
    {34, 354},
    {52, 307},
    {77, 267},
    {106, 235},
    {132, 209},
    {158, 191},
    {180, 177},
    {201, 168},
    {216, 160},
    {229, 156},
    {240, 151},
    {250, 147},
    {258, 145},
    {264, 144},
    {273, 142},
    {282, 141},
    {291, 139},
    {300, 138},
    {309, 136},
    {324, 134},
    {339, 134},
    {351, 134},
    {365, 135},
    {379, 136},
    {394, 137},
    {405, 138},
    {420, 140},
    {433, 143},
    {444, 146},
    {454, 150},
    {463, 152},
    {474, 157},
    {486, 162},
    {499, 167},
    {512, 174},
    {525, 182},
    {545, 193},
    {564, 209},
    {585, 227},
    {607, 250},
    {629, 279},
    {649, 314},
    {662, 352},
    */
    {-1, -1}
  };
  int coline[][2] = {
    {70, 369},
    {85, 325},
    {105, 288},
    {128, 255},
    {151, 233},
    {172, 215},
    {191, 199},
    {209, 188},
    {224, 180},
    {235, 174},
    {245, 168},
    {255, 163},
    {263, 160},
    {269, 158},
    {278, 156},
    {285, 155},
    {294, 153},
    {302, 150},
    {311, 148},
    {326, 146},
    {340, 145},
    {352, 145},
    {364, 145},
    {378, 145},
    {393, 146},
    {403, 149},
    {417, 151},
    {430, 154},
    {441, 159},
    {451, 164},
    {459, 166},
    {469, 173},
    {481, 177},
    {491, 185},
    {503, 193},
    {515, 203},
    {530, 215},
    {547, 232},
    {564, 251},
    {581, 272},
    {599, 299},
    {613, 333},
    {625, 367},
    {-1, -1}
  };

  
  int i, n, k, l, index;
  int prev_x = -1, prev_y = -1, x, y, x2, y2, x0, y0, y3;
  int dist;
  int count = 0;
  image_ptr small_image;

  if (print_imageline1){

    for (k = 0; k < 2; k++){
      if (k == 1)
	fprintf(stderr, "#IMAGELINE %d: ", count);
      for (i = 1; line[i][0] != -1; i++)
	for (n = 0; n <= 1000; n++){
	  x = line[i-1][0] + (int) ((((float) n) / 1000.0) 
				    * ((float) (line[i][0] - line[i-1][0])));
	  y = line[i-1][1] + (int) ((((float) n) / 1000.0) 
				    * ((float) (line[i][1] - line[i-1][1])));
	  dist = (int) sqrt(0.00001 + 
			    (((float) (x - prev_x)) * ((float) (x - prev_x))) +
			    (((float) (y - prev_y)) * ((float) (y - prev_y))));
	  if (dist >= 1){
	    index = PIXEL(x, y);
	    if (k == 1){
	      if (decimal)
		fprintf(stderr, "%d %d %d ", 
			image->red[index], image->green[index], image->blue[index]);
	      else
		fprintf(stderr, "%.2X%.2X%.2X", 
			image->red[index], image->green[index], image->blue[index]);
	    }
	    else
	      count++;
	    prev_x = x;
	    prev_y = y;
	  }
	}
      if (k == 1)
	fprintf(stderr, "\n");
    }
  }
  if (print_imageline2){

    prev_x = -1;
    prev_y = -1;
    count = 0;

    for (k = 0; k < 2; k++){
      if (k == 1)
	fprintf(stderr, "#IMAGELINE2 %d: ", count);
      for (i = 1; coline[i][0] != -1; i++)
	for (n = 0; n <= 1000; n++){
	  x = coline[i-1][0] + (int) ((((float) n) / 1000.0) 
				      * ((float) (coline[i][0] - coline[i-1][0])));
	  y = coline[i-1][1] + (int) ((((float) n) / 1000.0) 
				      * ((float) (coline[i][1] - coline[i-1][1])));
	  dist = (int) sqrt(0.00001 + 
			    (((float) (x - prev_x)) * ((float) (x - prev_x))) +
			    (((float) (y - prev_y)) * ((float) (y - prev_y))));
	  if (dist >= 1){
	    index = PIXEL(x, y);
	    if (k == 1){
	      if (decimal)
		fprintf(stderr, "%d %d %d ", 
			image->red[index], image->green[index], image->blue[index]);
	      else
		fprintf(stderr, "%.2X%.2X%.2X", 
			image->red[index], image->green[index], image->blue[index]);
	    }
	    else
	      count++;
	    prev_x = x;
	    prev_y = y;
	  }
	}
      if (k == 1)
	fprintf(stderr, "\n");
    }
  }


  prev_x = -1;
  prev_y = -1;
  count = 0;


  if (create_subimage)

    for (k = 0; k < 2; k++){
      if (k == 1){
	if (to_file){
	  small_image = make_image(40, count);
	  free (small_image->name);
	  small_image->name = (char *) malloc(sizeof(char)
					      * (strlen(image->name) + 80));
	  sprintf(small_image->name, "strip-%s", image->name);
	}
	else
	  fprintf(stderr, "#SUBIMAGE 40 %d: ", count);
      }
      for (i = 1, y3 = 0; line[i][0] != -1; i++)
	for (n = 0; n <= 1000; n++){
	  x = line[i-1][0] + (int) ((((float) n) / 1000.0) 
				    * ((float) (line[i][0] - line[i-1][0])));
	  y = line[i-1][1] + (int) ((((float) n) / 1000.0) 
				    * ((float) (line[i][1] - line[i-1][1])));
	  x2 = coline[i-1][0] + (int) ((((float) n) / 1000.0) 
				       * ((float) (coline[i][0] 
						   - coline[i-1][0])));
	  y2 = coline[i-1][1] + (int) ((((float) n) / 1000.0) 
				       * ((float) (coline[i][1]
						   - coline[i-1][1])));
	  dist = (int) sqrt(0.00001 + 
			    (((float) (x - prev_x)) * ((float) (x - prev_x))) +
			    (((float) (y - prev_y)) * ((float) (y - prev_y))));
	  if (dist >= 1){
	    if (k == 1){
	      for (l = 0; l < 40; l++){
		x0 = x + (int) (((float) l) / ((float) (40-1))
				* ((float) (x2-x)));
		y0 = y + (int) (((float) l) / ((float) (40-1)) 
				* ((float) (y2-y)));
		index = PIXEL(x0, y0);
		if (to_file){
		  set_pixel(small_image, l, y3, 
			    image->red[index], 
			    image->green[index], image->blue[index], 1);
		}
		else if (decimal)
		  fprintf(stderr, "%d %d %d ", 
			  image->red[index], image->green[index],
			  image->blue[index]);
		else
		  fprintf(stderr, "%.2X%.2X%.2X", 
			  image->red[index], image->green[index], 
			  image->blue[index]);
	      }
	      if (!to_file)
		fprintf(stderr, "  ");
	    }
	    else
	      count++;
	    prev_x = x;
	    prev_y = y;
	    y3++;
	  }
	}
      if (k == 1){
	if (to_file){
	  save_image(small_image, small_image->name);
	  destroy_image(&small_image);
	}
	else{
	  fprintf(stderr, "\n");
	}
      }
    }


 






  /* ********************************************************************** */


  /*
   * fit circle
   */


  if (0){  
    float cx, cy, dist, radius, E;
    int probe;

    cx = cy = 0;
    for (probe = 0; coline[probe][0] != -1; probe++){
      cx += (float) coline[probe][0];
      cy += (float) coline[probe][1];
    }
    cx /= (float) probe;
    cy /= (float) probe;

    dist = 0.0;

    for (probe = 0; coline[probe][1] != -1; probe++){
      float dx = (cx - ((float) coline[probe][0]));
      float dy = (cy - ((float) coline[probe][1]));
      float dd = (dx * dx) + (dy * dy);
      dist += sqrt((float) dd);
    }
  
    dist /= ((float) probe);
    radius = dist;

  
    set_pixel(image, (int) cx, (int) cy, 0, 255, 0, 3);
    
    for (n = 0; n < 10000; n++){
      float nf = ((float) n) / ((float) (10000)) * 2.0 * M_PI;
      float x1 = cx + radius * cos(nf);
      float y1 = cy + 0.91 * radius * sin(nf);
      set_pixel(image, ((int) x1), ((int) y1), 0, 255, 0, 1);
    }

    /*
     * iterate
     */


    for (k = 0; k < 1000; k++){
      float d_cx = 0.0, d_cy = 0.0, d_radius = 0.0, E=0.0;
      
      for (probe = 0; coline[probe][1] != -1; probe++){
	float dx = (cx - ((float) coline[probe][0]));
	float dy = (cy - ((float) coline[probe][1])) / 0.80;
	float dd = (dx * dx) + (dy * dy);
	float d_dd__d_cx = 2.0 * dx;
	float d_dd__d_cy = 2.0 * dy / 0.80;
	float dist = sqrt(dd) - radius;
	float d_dist__d_cx = 0.5 / sqrt(dd) * d_dd__d_cx;
	float d_dist__d_cy = 0.5 / sqrt(dd) * d_dd__d_cy;
	float d_dist__d_radius = -1.0;
	float dist2 = dist * dist;
	float d_dist2__d_cx = 2.0 * dist * d_dist__d_cx;
	float d_dist2__d_cy = 2.0 * dist * d_dist__d_cy;
	float d_dist2__d_radius = 2.0 * dist * d_dist__d_radius;
	d_cx -= 0.01 * d_dist2__d_cx;
	d_cy -= 0.01 * d_dist2__d_cy;
	d_radius -= 0.01 * d_dist2__d_radius;
	// d_cx -= 0.01 * 2.0 * dx * dist / 


	E += dist2;
      }
    
      // fprintf(stderr, "%g %g %g -> %g\n", cx, cy, radius, E);

      cx += d_cx;
      cy += d_cy;
      radius += d_radius;
    
      /*
	set_pixel(image, (int) cx, (int) cy, 255, 0, 0, 3);
      
      for (n = 0; n < 10000; n++){
	float nf = ((float) n) / ((float) (10000)) * 2.0 * M_PI;
	float x1 = cx + radius * cos(nf);
	float y1 = cy + 0.80 * radius * sin(nf);
	set_pixel(image, ((int) x1), ((int) y1), 255, 0, 0, 1);
      }
      */

    }



    /*
     * display
     */

    set_pixel(image, (int) cx, (int) cy, 0, 0, 255, 3);
  
    for (n = 0; n < 10000; n++){
      float nf = ((float) n) / ((float) (10000)) * 2.0 * M_PI;
      float x1 = cx + radius * cos(nf);
      float y1 = cy + 0.80 * radius * sin(nf);
      set_pixel(image, ((int) x1), ((int) y1), 0, 0, 255, 1);
    }
  }

  /* ********************************************************************** */

  if (print_imageline1){
    for (i = 1; line[i][0] != -1; i++)
      for (n = 0; n <= 1000; n++)
	set_pixel(image, 
		  line[i-1][0] + (int) ((((float) n) / 1000.0) 
					* ((float) (line[i][0] - line[i-1][0]))),
		  line[i-1][1] + (int) ((((float) n) / 1000.0) 
					* ((float) (line[i][1] - line[i-1][1]))),
		  255, 255, 255, 1);
    for (i = 0; line[i][0] != -1; i++)
      set_pixel(image, line[i][0], line[i][1], 255, 0, 0, 2);
  }

  if (print_imageline2){
    for (i = 1; coline[i][0] != -1; i++)
      for (n = 0; n <= 1000; n++)
	set_pixel(image, 
		  coline[i-1][0] + (int) ((((float) n) / 1000.0) 
					  * ((float) (coline[i][0] -
						      coline[i-1][0]))),
		  coline[i-1][1] + (int) ((((float) n) / 1000.0) 
					  * ((float) (coline[i][1] -
						      coline[i-1][1]))),
		  255, 255, 255, 1);
    for (i = 0; coline[i][0] != -1; i++)
      set_pixel(image, coline[i][0], coline[i][1], 255, 0, 0, 2);
  }
  if (print_imageline1 && print_imageline2){
    
    for (i = 0; line[i][0] != -1; i++)
      for (l = 0; l < 40; l++){
	x = line[i][0] + (int) (((float) l) / ((float) (40-1))
				* ((float) (coline[i][0]-line[i][0])));
	y = line[i][1] + (int) (((float) l) / ((float) (40-1))
				* ((float) (coline[i][1]-line[i][1])));
	index = PIXEL(x, y);
	image->red[index] = 255;
	image->green[index] = 255;
	image->blue[index] = 0;
      }
  }
}



#define NUM_KERNELS 120
#define KERNEL_SIZE 13
int ***kernel = NULL;
float *kernel_angle = NULL;



void
make_kernels()
{
  int i, j, n;
  float x, y, dist, angle;
  int v = 0;
  

  kernel = (int ***) malloc(sizeof(int **) * NUM_KERNELS);
  kernel_angle = (float *) malloc(sizeof(float) * NUM_KERNELS);

  for (n = 0; n < NUM_KERNELS; n++){
    
    kernel[n] = (int **) malloc(sizeof(int *) * KERNEL_SIZE);

    angle = 360.0 * ((float) n) / ((float) (NUM_KERNELS));
    kernel_angle[n] = angle;
    if (v) fprintf(stderr, "Angle= %g kernel=%d\n", angle, n);
    
    for (i = 0; i < KERNEL_SIZE; i++){
      kernel[n][i] = (int *) malloc(sizeof(int) * KERNEL_SIZE);

      for (j = 0; j < KERNEL_SIZE; j++){
	x = 2.0 * (((float) i) / ((float) (KERNEL_SIZE-1))) - 1.0;
	y = 2.0 * (((float) j) / ((float) (KERNEL_SIZE-1))) - 1.0;
	dist = (sin(angle/180.0*M_PI) * x) - (cos(angle/180.0*M_PI) * y);

	if (fabs(dist) < 0.05 || fabs(dist) > 0.5){
	  if (v) fprintf(stderr, "  ");
	  kernel[n][i][j] = 0;
	}
	else if (dist < 0.0){
	  if (v) fprintf(stderr, "--");
	  kernel[n][i][j] = -1;
	}
	else{
	  if (v) fprintf(stderr, "++");
	  kernel[n][i][j] = 1;
	}
      }
      if (v) fprintf(stderr, "\n");
    }
  }
}




int
find_kernel(float angle)
{
  float diff;
  float min_diff = 100000.0;
  float min_i = -1;
  int i;

  for (i = 0; i < NUM_KERNELS; i++){
    diff = angle - kernel_angle[i];
    for (; diff < -180.0; ) diff += 360.0;
    for (; diff >= 180.0; ) diff -= 360.0;
    diff = fabs(diff);
    // fprintf(stderr, "??? %g %g %g\n", angle, kernel_angle[i], diff);
    if (diff < min_diff){
      min_diff = diff;
      min_i = i;
    }
  }
  return min_i;
}



int
apply_kernel(image_ptr image, int x, int y, int kernel_nr)
{
  int i, j, ii, jj;
  int r = 0, g = 0, b = 0;
  /*  int tr = 0, tg = 0, tb = 0; */


  
  if (kernel_nr < 0 || kernel_nr >= NUM_KERNELS){
    fprintf(stderr, "ERROR: unknown kernel: %d\n", kernel_nr);
    exit(-1);
  }


  for (i = 0; i < KERNEL_SIZE; i++){
    ii = x + i - (KERNEL_SIZE / 2);
    for (j = 0; j < KERNEL_SIZE; j++){
      jj = y + j - (KERNEL_SIZE / 2);
      if (ii >= 0 && ii < image->size_x &&
	  jj >= 0 && jj < image->size_y){
	if (kernel[kernel_nr][i][j] != 0){
	  r += (((int) image->red  [PIXEL(ii, jj)]) * kernel[kernel_nr][i][j]);
	  g += (((int) image->green[PIXEL(ii, jj)]) * kernel[kernel_nr][i][j]);
	  b += (((int) image->blue [PIXEL(ii, jj)]) * kernel[kernel_nr][i][j]);
	  /*  tr += ((int) image->red  [PIXEL(ii, jj)]); */
	  /* tb += ((int) image->red  [PIXEL(ii, jj)]); */
	  /* tg += ((int) image->red  [PIXEL(ii, jj)]); */
	}
      }
    }
  }
  // fprintf(stderr, " (%d %d %d -> %d) ", x, y, kernel_nr, (abs(r) + abs(g) + abs(b)));
  //return ((int) (10000.0 * ((float) (abs(r) + abs(g) + abs(b))) / ((float) tr+tb+tg)));
  if (r+g+b > 0)
    return (abs(r) + abs(g) + abs(b));
  else
    return -(abs(r) + abs(g) + abs(b));
}








void
set_pixel(image_ptr image, int x, int y, 
	  unsigned char r, unsigned char g, unsigned char b, int size)
{
  int i, j;


  for (i = 1 - size; i < size; i++)
    if (x + i >= 0 && x + i < image->size_x)
      for (j = 1 - size; j < size; j++)
	if (y + j >= 0 && y + j < image->size_y){
	  image->red  [(((y+j) * (image->size_x)) + (x+i))] = r;
	  image->green[(((y+j) * (image->size_x)) + (x+i))] = g;
	  image->blue [(((y+j) * (image->size_x)) + (x+i))] = b;
	}
}










typedef struct
{
  int first_x, first_y;
  int delta_x, delta_y;
  int num_steps;
  float min_expected_angle, max_expected_angle;
  int threshold;
  int x, y;
  int multiplier;
} probe_type, *probe_ptr;


#define NUM_PROBES 4

probe_type probes[NUM_PROBES] =
{
  {65, 65,  1, 1,  200,  135.0, 170.0, 4000, 0, 0, -1},
  {15, NORMAL_IMAGE_SIZE_Y-16,  1, -1,  200,  10.0, 45.0, 1500, 0, 0, 1},
  {NORMAL_IMAGE_SIZE_X-86, 85,  -1, +1,  200, 30.0, 45.0, 4400, 0, 0, 1},
  {NORMAL_IMAGE_SIZE_X-16, NORMAL_IMAGE_SIZE_Y-16, -1, -1,  200,  135.0, 170.0, 1000, 0, 0, -1}
};



#define NUM_LOCAL_PATH_ITEMS (((NORMAL_IMAGE_SIZE_X)+(NORMAL_IMAGE_SIZE_Y)) * 100)


void
find_bound(image_ptr image)
{
  int i, j, k, n, x, y, d, best_d;
  int min_kernel, max_kernel;
  int sum, count, flag, probe;
  int path[NUM_LOCAL_PATH_ITEMS][3];
  int num_path = 0;
  float dist, radius;
  float cx, cy;

  if (!align)
    return;

  for (probe = 0; probe < NUM_PROBES; probe++){
    min_kernel = find_kernel(probes[probe].min_expected_angle);
    max_kernel = find_kernel(probes[probe].max_expected_angle);
    sum = count = flag = 0;
    
    // fprintf(stderr, "\n### Probe: %d, Angle: %g - %g Kernels: %d - %d\n", probe, probes[probe].min_expected_angle, probes[probe].max_expected_angle, min_kernel, max_kernel);
    
    /*
     * find the boundary
     */

    
    for (i = 0, x = probes[probe].first_x, y = probes[probe].first_y; 
	 i < probes[probe].num_steps && !flag;
	 x += probes[probe].delta_x, y += probes[probe].delta_y, i++){

      best_d = 0;
      for (k = min_kernel; k <= max_kernel; k++){
	d = apply_kernel(image, x, y, k);
	d = d * probes[probe].multiplier;
	if (d > best_d)
	  best_d = d;
      }
      
	
      sum += best_d;
      count++;

      // fprintf(stderr, "%d: d=%d sum=%d count=%d\n", i, best_d, sum, count);
      // fprintf(stderr, " %d ", best_d);
	
      if (best_d > probes[probe].threshold)
	flag = 1;

      if (num_path < NUM_LOCAL_PATH_ITEMS){
	path[num_path][0] = x;
	path[num_path][1] = y;
	path[num_path][2] = 1;
	num_path++;
      }
    }

    probes[probe].x = x;
    probes[probe].y = y;
    
    if (num_path < NUM_LOCAL_PATH_ITEMS){
      path[num_path][0] = x;
      path[num_path][1] = y;
      path[num_path][2] = 3;
      num_path++;
    }
  }

  if (!flag)
    fprintf(stderr, "WARNING: Alignment failed.\n");
  
 
  for (i = 0; i < num_path; i++)
    set_pixel(image, path[i][0], path[i][1], 255, 255, 255, path[i][2]);

  /*
   * fit circle
   */


  
  cx = cy = 0;
  for (probe = 0; probe < NUM_PROBES; probe++){
    cx += (float) probes[probe].x;
    cy += (float) probes[probe].y;
  }
  cx /= (float) NUM_PROBES;
  cy /= (float) NUM_PROBES;

  dist = 0.0;

  for (probe = 0; probe < NUM_PROBES; probe++){
    float dx = (cx - ((float) probes[probe].x));
    float dy = (cy - ((float) probes[probe].y));
    float dd = (dx * dx) + (dy * dy);
    dist += sqrt((float) dd);
  }
  
  dist /= ((float) NUM_PROBES);
  radius = dist;

  
  /*
   * iterate
   */


  for (k = 0; k < 100; k++){
    float d_cx = 0.0, d_cy = 0.0, d_radius = 0.0, E=0.0;
    
    for (probe = 0; probe < NUM_PROBES; probe++){
      float dx = (cx - ((float) probes[probe].x));
      float dy = (cy - ((float) probes[probe].y)) / 0.91;
      float dd = (dx * dx) + (dy * dy);
      float d_dd__d_cx = 2.0 * dx;
      float d_dd__d_cy = 2.0 * dy / 0.91;
      float dist = sqrt(dd) - radius;
      float d_dist__d_cx = 0.5 / sqrt(dd) * d_dd__d_cx;
      float d_dist__d_cy = 0.5 / sqrt(dd) * d_dd__d_cy;
      float d_dist__d_radius = -1.0;
      float dist2 = dist * dist;
      float d_dist2__d_cx = 2.0 * dist * d_dist__d_cx;
      float d_dist2__d_cy = 2.0 * dist * d_dist__d_cy;
      float d_dist2__d_radius = 2.0 * dist * d_dist__d_radius;
      d_cx -= 0.1 * d_dist2__d_cx;
      d_cy -= 0.1 * d_dist2__d_cy;
      d_radius -= 0.1 * d_dist2__d_radius;
      // d_cx -= 0.01 * 2.0 * dx * dist / 


      E += dist2;
    }
    
    // fprintf(stderr, "%g %g %g -> %g\n", cx, cy, radius, E);

    cx += d_cx;
     cy += d_cy;
     radius += d_radius;
    
    
     /*
      set_pixel(image, (int) cx, (int) cy, 255, 0, 0, 3);
    
      for (n = 0; n < 10000; n++){
      float nf = ((float) n) / ((float) (10000)) * 2.0 * M_PI;
      float x1 = cx + radius * cos(nf);
      float y1 = cy + 0.91 * radius * sin(nf);
      set_pixel(image, ((int) x1), ((int) y1), 255, 0, 0, 1);
    }

     */

  }



  /*
   * display
   */

  set_pixel(image, (int) cx, (int) cy, 0, 0, 255, 3);
  
  for (n = 0; n < 10000; n++){
    float nf = ((float) n) / ((float) (10000)) * 2.0 * M_PI;
    float x1 = cx + radius * cos(nf);
    float y1 = cy + 0.91 * radius * sin(nf);
    set_pixel(image, ((int) x1), ((int) y1), 0, 0, 255, 1);
  }

  fprintf(stderr, "Center: %g %g radius %g\n", cx, cy, radius);


  shift_image(image, 345-cx, 288-cy);


  /*
  for (y = 0; y < image->size_y; y++)
    for (x = 0, flag = 0; x < image->size_x && !flag; x++){
	
      i = PIXEL(x, y);
           
      if (image->red[i] == 0 && image->green[i] == 0 && image->blue[i] == 255)
	flag = 1;
      else if (image->red[i] == 255 && image->green[i] == 255 
	       && image->blue[i] == 255)
	  ;
      else{
	image->red[i] = 0;
	image->green[i] = 0;
	image->blue[i] = 0;
      }
    }

  for (y = 0; y < image->size_y; y++)
    for (x = image->size_x-1, flag = 0; x >= 0  && !flag; x--){
	
      i = PIXEL(x, y);
           
      if (image->red[i] == 0 && image->green[i] == 0 && image->blue[i] == 255)
	flag = 1;
      else if (image->red[i] == 255 && image->green[i] == 255 
	       && image->blue[i] == 255)
	  ;
      else{
	image->red[i] = 0;
	image->green[i] = 0;
	image->blue[i] = 0;
      }
    }
  */
  
}




void
process_image(char *filename)
{
  image_ptr image = NULL;
  image = read_image(filename);
  if (align) find_bound(image);
  add_ref_line_to_image(image);

  if (display_image){
    save_image(image, "dump.ppm"); 
    system("xv dump.ppm -poll -drift 0 0 -geometry +0+0");
  }

  destroy_image(&image);
}

  
  

void
process_images()
{
  if (program_state->preprocess_images)
    image_loop();
}

void
image_loop()
{
  char text[256];
  unsigned char ch;
  FILE *iop;
  int flag = 0;

  make_kernels();

  // process_image("frm12000203.ppm");  exit(-1);

  system("ls -1 frm*ppm > .laserint.frmppm");
  if ((iop = fopen(".laserint.frmppm", "r")) == 0){
    fprintf(stderr, "ERROR: Can't manipulate file system.\n");
    exit(-1);
  }
  for (flag = 0; !flag; ){
    if (fscanf(iop, "%s", text) == EOF)
      flag = 1;
    else{
      fprintf(stderr, "File: %s\n", text);
      process_image(text);
    }
  }
  fclose(iop);

  exit(-1);
  
}



/*

laserint.script.ceilingtexture

begin: 310
end:   3580
total: 3270

@SENS 08-03-00 16:48:07.758437
@SENS 08-03-00 16:49:56.562746
which is exactly 3270 frames ;-)

Scripts for calculating image frame numbers:


cat laserint.script | gawk '{if ($1 == "@SENS") a = $3 ; m=substr(a,4,2); s=substr(a,7,99); t=(m-48) * 60 + (s-7.7); f = t*30+310  ; if ($1 == "#LASER") print f "."}' | gawk '{print "000" substr($0,1,index($0,".")-1)}' | gawk '{print "frm" substr($0,length($0)-3,99) ".ppm"}' | sort

echo "" | gawk '{for (i = 0; i < 660; i++) print 310 + i * 3270 / 660 "."}' | gawk  '{print "000" substr($0,1,index($0,".")-1)}' | gawk '{print "frm" substr($0,length($0)-3,99) ".ppm"}'

echo "" | gawk '{for (i = 0; i < 3270; i++){ t=i/3270*661 ; print "IMAGE" ; if (t >= tp + 1) {print "LASER"; tp = tp + 1;}}}'




cd /usr8/thrun/images
~/laserint > & ~/dr
cd /usr9/thrun/images
~/laserint >> & ~/dr
cd /usr7/thrun/images
~/laserint >> & ~/dr

cd /usr8/thrun/images
~/laserint-hex > & ~/strips
cd /usr9/thrun/images
~/laserint-hex >> & ~/strips
cd /usr7/thrun/images
~/laserint-hex >> & ~/strips


*/
