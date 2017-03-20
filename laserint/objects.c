

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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/laserint/objects.c,v $
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
 * $Log: objects.c,v $
 * Revision 1.1  2002/09/14 15:34:07  rstone
 * *** empty log message ***
 *
 * Revision 1.4  2000/03/11 23:16:52  thrun
 * latest version (don't remember the exact changes, but
 * should only affect the development code)
 *
 * Revision 1.3  2000/01/02 16:58:02  thrun
 * intermediate version with 2 objects
 *
 * Revision 1.2  1999/12/27 04:12:12  thrun
 * Intermediate version, with dww and dlw (might not work).
 *
 * Revision 1.1  1999/12/27 03:44:24  thrun
 * Magic: Gradient descent for adapting
 * wall parameters xw yw and aw seems to work!!!
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/




#ifdef VMS
#include "vms.h"
#include "vmstime.h"
#endif

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

/**********************
 *
 * THE FOLLOWING IS PORTED FROM MATHEMATICA 
 *
 *     xw, yw = hinge point for the wall
 *     lw, ww = length and width of wall
 *     aw     = angle of wall
 *   
 *     xr, yr, or = robot's pose
 *     dr, ar     = distance and angle of measurement  
 *   
 *     distexponent: determines type of distance
 *                   strictly speaking, we need to set it to 1 for 
 *                   Euclidean distance
 *   
 *   
 */



/*
 * 
 * Coordinates of the wall end points
 *
 */

float
wall1x1(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  *dxw = 1.0;
  *dyw = 0.0;
  *dlw = 0.0;
  *dww = 0.0;
  *daw = 0.0;

  return xw;
}




float
wall1y1(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  *dxw = 0.0;
  *dyw = 1.0;
  *dlw = 0.0;
  *dww = 0.0;
  *daw = 0.0;

  return yw;
}




float
wall1x2(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  *dxw = 1.0;
  *dyw = 0.0;
  *dlw = cos(aw * M_PI / 180.0);
  *dww = 0.0;
  *daw = - sin(aw * M_PI / 180.0) * lw * M_PI / 180.0;

  return xw + (cos(aw * M_PI / 180.0) * lw);
}




float
wall1y2(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  *dxw = 0.0;
  *dyw = 1.0;
  *dlw = sin(aw * M_PI / 180.0);
  *dww = 0.0;
  *daw = cos(aw * M_PI / 180.0) * lw * M_PI / 180.0;

  return yw + (sin(aw * M_PI / 180.0) * lw);
}




float
wall1a(float xw, float yw, float lw, float ww, float aw,
	     float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  *dxw = 0.0;
  *dyw = 0.0;
  *dlw = 0.0;
  *dww = 0.0;
  *daw = 1.0;

  return aw;
}




float
wall2x1(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  return wall1x2(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
}




float
wall2y1(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  return wall1y2(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
}




float
wall2x2(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  float retvalue = wall1x2(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw) 
    - sin(aw * M_PI / 180.0) * ww;
  *daw -= (cos(aw * M_PI / 180.0) * ww * M_PI / 180.0);
  *dww -= sin(aw * M_PI / 180.0);

  return retvalue;
}




float
wall2y2(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  float retvalue = wall1y2(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw) 
    + cos(aw * M_PI / 180.0) * ww;
  *daw -= (sin(aw * M_PI / 180.0) * ww * M_PI / 180.0);
  *dww += cos(aw * M_PI / 180.0);

  return retvalue;
}




float
wall2a(float xw, float yw, float lw, float ww, float aw,
	     float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  *dxw = 0.0;
  *dyw = 0.0;
  *dlw = 0.0;
  *dww = 0.0;
  *daw = 1.0;

  return aw + M_PI/2;
}





float
wall3x1(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  return wall2x2(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
}




float
wall3y1(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  return wall2y2(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
}




float
wall3x2(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  *dxw = 1.0;
  *dyw = 0.0;
  *dlw = 0.0;
  *dww = - sin(aw * M_PI / 180.0);
  *daw = cos(aw * M_PI / 180.0) * ww * M_PI / 180.0;

  return xw - sin(aw * M_PI / 180.0) * ww;
}




float
wall3y2(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  *dxw = 0.0;
  *dyw = 1.0;
  *dlw = 0.0;
  *dww = cos(aw * M_PI / 180.0);
  *daw = - sin(aw * M_PI / 180.0) * ww * M_PI / 180.0;

  return yw + cos(aw * M_PI / 180.0) * ww;
}




float
wall3a(float xw, float yw, float lw, float ww, float aw,
	     float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  *dxw = 0.0;
  *dyw = 0.0;
  *dlw = 0.0;
  *dww = 0.0;
  *daw = 1.0;

  return aw + M_PI;
}





float
wall4x1(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  return wall3x2(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
}




float
wall4y1(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  return wall3y2(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
}




float
wall4x2(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  *dxw = 1.0;
  *dyw = 0.0;
  *dlw = 0.0;
  *dww = 0.0;
  *daw = 0.0;

  return xw;
}




float
wall4y2(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  *dxw = 0.0;
  *dyw = 1.0;
  *dlw = 0.0;
  *dww = 0.0;
  *daw = 0.0;

  return yw;
}




float
wall4a(float xw, float yw, float lw, float ww, float aw,
	     float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  *dxw = 0.0;
  *dyw = 0.0;
  *dlw = 0.0;
  *dww = 0.0;
  *daw = 1.0;

  return aw + (1.5 * M_PI);
}





/*  nr is wall number  */

float
wallx1(int nr, float xw, float yw, float lw, float ww, float aw,
	     float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  switch(nr){
  case 1:
    return wall1x1(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
    break;
  case 2:
    return wall2x1(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
    break;
  case 3:
    return wall3x1(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
    break;
  case 4:
    return wall4x1(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
    break;
  default:
    fprintf(stderr, "Error in wallx1()\n");
    return 99999.9;
  }
}




float
wallx2(int nr, float xw, float yw, float lw, float ww, float aw,
	     float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  switch(nr){
  case 1:
    return wall1x2(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
    break;
  case 2:
    return wall2x2(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
    break;
  case 3:
    return wall3x2(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
    break;
  case 4:
    return wall4x2(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
    break;
  default:
    fprintf(stderr, "Error in wallx2()\n");
    return 99999.9;
  }
}





float
wally1(int nr, float xw, float yw, float lw, float ww, float aw,
	     float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  switch(nr){
  case 1:
    return wall1y1(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
    break;
  case 2:
    return wall2y1(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
    break;
  case 3:
    return wall3y1(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
    break;
  case 4:
    return wall4y1(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
    break;
  default:
    fprintf(stderr, "Error in wally1()\n");
    return 99999.9;
  }
}






float
wally2(int nr, float xw, float yw, float lw, float ww, float aw,
	     float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  switch(nr){
  case 1:
    return wall1y2(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
    break;
  case 2:
    return wall2y2(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
    break;
  case 3:
    return wall3y2(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
    break;
  case 4:
    return wall4y2(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
    break;
  default:
    fprintf(stderr, "Error in wally2()\n");
    return 99999.9;
  }
}





float
walla(int nr, float xw, float yw, float lw, float ww, float aw,
	    float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  switch(nr){
  case 1:
    return wall1a(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
    break;
  case 2:
    return wall2a(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
    break;
  case 3:
    return wall3a(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
    break;
  case 4:
    return wall4a(xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);
    break;
  default:
    fprintf(stderr, "Error in walla()\n");
    return 99999.9;
  }
}






/* 
  *  
  *  Coordinates of the robot sensor measurement
  * 
  */

float
robotsize = 30.0;

float
sensx(float xr, float yr, float or, float dr, float ar)
{
  return xr + ((dr + robotsize) * cos((or + ar) * M_PI / 180.0));
}




float
sensy(float xr, float yr, float or, float dr, float ar)
{
  return yr + ((dr + robotsize) * sin((or + ar) * M_PI / 180.0));
}




float
sensa(float xr, float yr, float or, float dr, float ar)
{
  return or + ar;
}






/* 
 *  
 *  Compute distance of point to line
 * 
 */



float
distPointLineAux(float x, float y, float x1, float y1, float x2, float y2,
		       float *dx1, float *dy1, float *dx2, float *dy2)
{
  float d, quotient, sq_quotient;
  float dd_dx1, dd_dy1, dd_dx2, dd_dy2;

  d = ((y2 - y1) * (x - x1)) - ((x2 - x1) * (y - y1));

  dd_dx1 = - (y2 - y1) + (y - y1);
  dd_dy1 = - (x - x1)  + (x2 - x1);
  dd_dx2 = - (y - y1);
  dd_dy2 = (x - x1);

  if (d < 0.0){
    d = -d;
    dd_dx1 *= -1.0;
    dd_dy1 *= -1.0;
    dd_dx2 *= -1.0;
    dd_dy2 *= -1.0;
  }

  sq_quotient = ((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1));
  quotient = sqrt(sq_quotient);

  *dx1 = ((dd_dx1 * quotient) - (- d / quotient * (x2 - x1))) / sq_quotient;
  *dy1 = ((dd_dy1 * quotient) - (- d / quotient * (y2 - y1))) / sq_quotient;
  *dx2 = ((dd_dx2 * quotient) - (  d / quotient * (x2 - x1))) / sq_quotient;
  *dy2 = ((dd_dy2 * quotient) - (  d / quotient * (y2 - y1))) / sq_quotient;

  d = d / quotient;
  return d;
}







float
distPointLine(float x, float y, float x1, float y1, float x2, float y2,
		    float *dx1, float *dy1, float *dx2, float *dy2)
{
  /* ^distexponent was here */
  float retvalue;

  if (((x2 - x1) * (x - x1)) + ((y2 - y1) * (y - y1)) > 0.0){
    if (((x1 - x2) * (x - x2)) + ((y1 - y2) * (y - y2)) > 0.0)

      return distPointLineAux(x, y, x1, y1, x2, y2, dx1, dy1, dx2, dy2);

    else{
      retvalue = sqrt(((x - x2) * (x - x2)) + ((y - y2) * (y - y2)));
      *dx1 = 0.0;
      *dy1 = 0.0;
      *dx2 = (x2 - x) / retvalue;
      *dy2 = (y2 - y) / retvalue;

      return retvalue;
    }
  }

  else{
    retvalue = sqrt(((x - x1) * (x - x1)) + ((y - y1) * (y - y1)));
    *dx1 = (x1 - x) / retvalue;
    *dy1 = (y1 - y) / retvalue;
    *dx2 = 0.0;
    *dy2 = 0.0;

    return retvalue;
  }
}






/* 
  *  
  *  Compute the distance of point to object
  * 
  */

float
distPointWall(float x, float y, int nr, 
		    float xw, float yw, float lw, float ww, float aw,
		    float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  float dx1, dy1, dx2, dy2, retvalue;

  float wall_x1, dxw_x1, dyw_x1, dlw_x1, dww_x1, daw_x1;
  float wall_y1, dxw_y1, dyw_y1, dlw_y1, dww_y1, daw_y1;
  float wall_x2, dxw_x2, dyw_x2, dlw_x2, dww_x2, daw_x2;
  float wall_y2, dxw_y2, dyw_y2, dlw_y2, dww_y2, daw_y2;

  wall_x1 = wallx1(nr, xw, yw, lw, ww, aw,
		   &dxw_x1, &dyw_x1, &dlw_x1, &dww_x1, &daw_x1);
  wall_y1 = wally1(nr, xw, yw, lw, ww, aw,
		   &dxw_y1, &dyw_y1, &dlw_y1, &dww_y1, &daw_y1);
  wall_x2 = wallx2(nr, xw, yw, lw, ww, aw, 
		   &dxw_x2, &dyw_x2, &dlw_x2, &dww_x2, &daw_x2);
  wall_y2 = wally2(nr, xw, yw, lw, ww, aw,
		   &dxw_y2, &dyw_y2, &dlw_y2, &dww_y2, &daw_y2);

  
  retvalue = distPointLine(x, y, wall_x1, wall_y1, wall_x2, wall_y2,
			   &dx1, &dy1, &dx2, &dy2);

  *dxw = (dx1 * dxw_x1) + (dy1 * dxw_y1) + (dx2 * dxw_x2) + (dy2 * dxw_y2);
  *dyw = (dx1 * dyw_x1) + (dy1 * dyw_y1) + (dx2 * dyw_x2) + (dy2 * dyw_y2);
  *dlw = (dx1 * dlw_x1) + (dy1 * dlw_y1) + (dx2 * dlw_x2) + (dy2 * dlw_y2);
  *dww = (dx1 * dww_x1) + (dy1 * dww_y1) + (dx2 * dww_x2) + (dy2 * dww_y2);
  *daw = (dx1 * daw_x1) + (dy1 * daw_y1) + (dx2 * daw_x2) + (dy2 * daw_y2);

  return retvalue;
}





float
distPointWallWithAngle(float x, float y, int nr,
			     float xw, float yw, float lw, float ww, float aw,
			     float xr, float yr, float or, float dr, float ar,
			     float *dxw, float *dyw, float *dlw, float *dww,
			     float *daw)
{
  /* check if the sensor beam points in the right direction
   * for this wall */

  static int msg[5] = {0, 0, 0, 0, 0};

  if((wally2(nr, xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw) 
      - wally1(nr, xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw)) 
     * (sensx(xr, yr, or, dr, ar) - xr) -
     (wallx2(nr, xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw) 
      - wallx1(nr, xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw))
     * (sensy(xr, yr, or, dr, ar) - yr) <= 0.0)

    return distPointWall(x, y, nr, xw, yw, lw, ww, aw, dxw, dyw, dlw, dww, daw);

  else{
    *dxw = 0.0;
    *dyw = 0.0;
    *dlw = 0.0;
    *dww = 0.0;
    *daw = 0.0;    
    return 99999999999.9;
  }
}






float
distPointToObj(float x, float y, 
		     float xw, float yw, float lw, float ww, float aw,
		     float xr, float yr, float or, float dr, float ar,
		     float *dxw, float *dyw, float *dlw, float *dww, 
		     float *daw)
{
  float min = 99999999999.9;
  float dist;
  int nr;
  float aux_dxw, aux_dyw, aux_dlw, aux_dww, aux_daw;

  for (nr = 1; nr <= 4; nr++){
    dist = distPointWallWithAngle(x, y, nr, xw, yw, lw, ww, aw, 
				  xr, yr, or, dr, ar, 
				  &aux_dxw, &aux_dyw,
				  &aux_dlw, &aux_dww, &aux_daw);
    if (dist < min){
      min = dist;
      *dxw = aux_dxw;
      *dyw = aux_dyw;
      *dlw = aux_dlw;
      *dww = aux_dww;
      *daw = aux_daw;
    }
  }

  return min;
}





float
distSensorToObj(float xw, float yw, float lw, float ww, float aw, 
		      float xr, float yr, float or, float dr, float ar,
		      float *dxw, float *dyw, float *dlw, float *dww, 
		      float *daw)
{
  /* ignore robot diameter here */
  float x = xr + (sin((or + ar) * M_PI / 180.0) * dr);
  float y = yr + (cos((or + ar) * M_PI / 180.0) * dr);
  return distPointToObj(x, y, xw, yw, lw, ww, aw, xr, yr, or, dr, ar, 
			dxw, dyw, dlw, dww, daw);
}


float 
sqError(float dist,
	float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  *dxw = 2.0 * *dxw * dist;
  *dyw = 2.0 * *dyw * dist;
  *dlw = 2.0 * *dlw * dist;
  *dww = 2.0 * *dww * dist;
  *daw = 2.0 * *daw * dist;
  return dist * dist;
}

#define MAX_D 100.0
float 
saturationError(float dist,
		float *dxw, float *dyw, float *dlw, float *dww, float *daw)
{
  if (dist > MAX_D)
    return saturationError(MAX_D, dxw, dyw, dlw, dww, daw);

  else{
    float deriv = 3.0 * (MAX_D * dist - (dist * dist)) / MAX_D;
    *dxw = *dxw * deriv;
    *dyw = *dyw * deriv;
    *dlw = *dlw * deriv;
    *dww = *dww * deriv;
    *daw = *daw * deriv;
    return dist * dist * ((1.5 * MAX_D) - dist) / MAX_D;
  }
}

float
NonlinDistPointToObj(float x, float y, 
		     float xw, float yw, float lw, float ww, float aw,
		     float xr, float yr, float or, float dr, float ar,
		     float *dxw, float *dyw, float *dlw, float *dww, 
		     float *daw)
{
  float dist, aux_dxw, aux_dyw, aux_dlw, aux_dww, aux_daw;

  dist = distPointToObj(x, y, xw, yw, lw, ww, aw, xr, yr, or, dr, ar,
			dxw, dyw, dlw, dww, daw);

  dist = saturationError(dist, dxw, dyw, dlw, dww, daw);
  
  return dist;

}
