
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** File:                        rectangle.cc
 *****
 ***** Part of:                     RHINO SOFTWARE
 *****
 ***** Creator:                     Dirk Schulz, University of Bonn
 *****
 ***** Date of creation:            July 1996
 *****
 *****
 *****
 *****
 ***** $Source: /usr/local/cvs/bee/src/simulator2/rectangle.cc,v $
 *****
 ***** $Revision: 1.1 $
 *****
 ***** $Date: 2002/09/14 16:11:07 $
 *****
 ***** $Author: rstone $
 *****
 *****
 *****
 ***** Contact thrun@carbon.cs.bonn.edu or thrun@cs.cmu.edu.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** 
 ***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED BY
 ***** APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING THE
 ***** COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM "AS IS"
 ***** WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED,
 ***** INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 ***** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE ENTIRE
 ***** RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU.
 ***** SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL
 ***** NECESSARY SERVICING, REPAIR OR CORRECTION.
 ***** 
 ***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
 ***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
 ***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
 ***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
 ***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
 ***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO LOSS OF
 ***** DATA OR DATA BEING RENDERED INACCURATE OR LOSSES SUSTAINED BY YOU
 ***** OR THIRD PARTIES OR A FAILURE OF THE PROGRAM TO OPERATE WITH ANY
 ***** OTHER PROGRAMS), EVEN IF SUCH HOLDER OR OTHER PARTY HAS BEEN
 ***** ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: rectangle.cc,v $
 * Revision 1.1  2002/09/14 16:11:07  rstone
 * *** empty log message ***
 *
 * Revision 1.20  1999/09/29 08:35:09  schulz
 * activated height check again
 *
 * Revision 1.19  1999/09/27 10:46:57  schulz
 * Initial bounding box was not computed for rectangles; fixed
 *
 * Revision 1.18  1997/12/11 14:51:38  schulz
 * added missing case in inhalfplane()
 *
 * Revision 1.17  1997/04/02 12:36:56  fox
 * Fixed a bug in tag handling.
 *
 * Revision 1.16  1997/04/01 12:15:26  fox
 * Don't dump tags of unknown objects.
 *
 * Revision 1.15  1997/04/01 11:11:31  fox
 * Added comment handling for simulator maps. Comment lines must start with #.
 * Added the possibility to set a tag name for the different objects.
 *
 * Revision 1.14  1997/03/26 13:04:30  schulz
 * fixed a bug, we have to invalidate absolute_points after obstacles have
 * been moved
 *
 * Revision 1.13  1997/03/20 10:27:53  schulz
 * no comment
 *
 * Revision 1.12  1997/03/20 09:57:21  schulz
 * temporarily we are displaying dots at the front of obstascles for
 * debugging purposes
 *
 * Revision 1.11  1997/03/11 11:02:19  schulz
 * Removed the distinction between RECTANGLEs and CUBEs  when
 * saving the playground. We always save CUBEs now.
 * Dito for CIRCLE <-> CYLINDER
 *
 * Revision 1.10  1997/02/27 15:43:01  schulz
 * fixed handling of doors and some other bugs
 *
 * Revision 1.9  1997/02/11 16:39:47  schulz
 * we do intersection tests only for the two visable lines of a rectangle know
 *
 * Revision 1.8  1997/01/27 15:13:30  schulz
 * added added multiple sensor planes to the sensor simulation stuff
 * I hope this does not introduce to many new bugs.
 *
 * Revision 1.7  1996/12/09 13:30:32  schulz
 * -- added orintation to rectangles
 * -- additional minor fixes
 *
 * Revision 1.6  1996/12/05 14:59:07  schulz
 * forgot to set color in t_circle
 *
 * Revision 1.5  1996/10/29 16:05:11  ws
 * changed some class names
 *
 * Revision 1.4  1996/10/18 13:59:59  ws
 * obstacles can be toggled on/off now using the right mouse button.
 *
 * Revision 1.3  1996/10/14 12:19:10  ws
 * added support for different surfaces (not yet complete) and sonar error
 *
 * Revision 1.2  1996/10/09 13:27:41  schulz
 * reorganized obstacle store and fixed some more bugs.
 *
// Revision 1.1.1.1  1996/09/30  16:17:45  schulz
// reimport on new source tree
//
 * Revision 1.3  1996/09/20 07:56:21  schulz
 * Added saving of maps. E.g. a filename requester and a save method
 * for each obstacle type.
 *
 * Revision 1.2  1996/09/11 14:03:49  schulz
 * - Speed up sonar and laser simulation by a factor of 20.
 *   We precalculate the set of obstacles which can be hit by a
 *   laser/sonar beam and check only for these few obstacles, if they are hit.
 *   Switched to S.Thruns version of line and circle intersection routines,
 *   these are a little bit faster then the old ones.
 * - Included laser visualisation  (to slow an some machines)
 *
 * Revision 1.1  1996/08/27 15:23:09  schulz
 * Some files forgotten last time
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <math.h>
#include <values.h>
#include "rectangle.hh"
#include "simxw.h"
#include "linalg.h"
#include "trigofkt.h"

extern Boolean mark_option;
extern t_surface *default_surface;

void
t_rectangle::newBounds(float *xi1, float *yi1, float *xi2, float *yi2)
{
  float min_x = MAXFLOAT;
  float max_x = MINFLOAT;
  float min_y = MAXFLOAT;
  float max_y = MINFLOAT;
  absolute_points();
  for(int i = 0; i< 3; i++) {
    if( x[i] < min_x) min_x = x[i];
    if( x[i] > max_x) max_x = x[i];
    if( y[i] < min_y) min_y = y[i];
    if( y[i] > max_y) max_y = y[i];
  }
  last_ori = ori;
  *xi1 = x1 = min_x;
  *yi1 = y1 = min_y;
  *xi2 = x2 = max_x;
  *yi2 = y2 = max_y;
}

void
t_rectangle::bounds(float *xi1, float *yi1, float *xi2, float *yi2)
{
  if( ori != last_ori) {
    newBounds(&x1, &y1, &x2, &y2);
  }
  *xi1 = x1;
  *xi2 = x2;
  *yi1 = y1;
  *yi2 = y2;
}


void
t_rectangle::move(float dx, float dy)
{
  cx += dx;
  cy += dy;
  last_ori  = abs_ori = ori - 1;
}

static void
rotpnt(float angle, float &x, float &y) 
{ 
  float t = x;
  x = x * cos(angle) - y * sin(angle);
  y = t  * sin(angle) + y * cos(angle); 
}

void t_rectangle::absolute_points()
{
  if(ori == abs_ori) return;
    x[0] = w/2;
    y[0] = -d/2;
    rotpnt(ori,x[0],y[0]);
    x[1] = w/2;
    y[1] = +d/2;
    rotpnt(ori,x[1],y[1]);
    x[2] = -w/2;
    y[2] = d/2;
    rotpnt(ori,x[2],y[2]);
    x[3] = -w/2;
    y[3] = -d/2;
    rotpnt(ori,x[3],y[3]);
    for(int i = 0; i <4; i++) {
	x[i] += cx;
	y[i] += cy;
    }
  abs_ori = ori;
}

Boolean t_rectangle::inside(float xi, float yi)
{
    int i;
    absolute_points();
    for(i = 0; i < 4; i++) {
	if(!inhalfplane(xi,yi,x[i],y[i],x[(i+1)%4],y[(i+1)%4]))
	    return FALSE;
    }
    return TRUE;
}

void t_rectangle::expose()
{
    absolute_points();
    myfig = expose_polygon(4,x,y,color);
    if(mark_option)
      place_dot(cx, cy, d, ori);
}

void t_rectangle::toggle()
{
    if(enabled) {
	RemoveFigure(myfig);
	absolute_points();
	myfig = expose_disabled_polygon(4,x,y,color);
	enabled = FALSE;
    }
    else {
	RemoveFigure(myfig);
	absolute_points();
	myfig = expose_polygon(4,x,y,color);
	enabled = TRUE;
    }
}

void t_rectangle::save(FILE *fd)
{
  if ( strcmp( tag, dummyTag))
    fprintf(fd, "CUBE %f %f %f %f %f %f %f %s\n",cx,cy,cz,w,d,h,ori,tag);
  else
    fprintf(fd, "CUBE %f %f %f %f %f %f %f\n",cx,cy,cz,w,d,h,ori);
}

void t_rectangle::redraw()
{
    absolute_points();
    if(enabled)
	myfig = redraw_polygon(myfig,4,x,y);
    else
	myfig = redraw_disabled_polygon(myfig,4,x,y);
}

t_rectangle::t_rectangle()
{
    my_surface = default_surface;
    enabled = TRUE;
    active = FALSE;
}

t_rectangle::t_rectangle( char* tagName, float cxi, float cyi, float czi,
			  float wi, float di, float hi, float orii)
{
  type = 0;
  tag = new char[strlen(tagName)+1];
  strcpy( tag, tagName);
  w = fabs(wi);
  d = fabs(di);
  h = fabs(hi);
  cx = cxi;
  cy = cyi;
  cz = czi;
  ori = orii;
  abs_ori = ori + 1;
  newBounds(&x1, &y1, &x2, &y2);
  color = WALLCOLOR;
  my_surface = default_surface;
  enabled = TRUE;
  active = FALSE;
}

Boolean t_rectangle::distance(float xr, float yr, float zr, float open_angle,
			      float xe, float ye,
			   float *dist, float *angle, t_surface **surface)
{
    Boolean Hit = FALSE,C_Hit = FALSE;
    float c_dist,c_angle;
    absolute_points();
    for(int i = 0; i < 4; i++) {
    //check if this wall is seen by the sensor at (xr,yr)	
      if( ! (point_on_left_side(cx, cy, x[i],y[i], x[(i+1)%4],y[(i+1)%4]) ==
	     point_on_left_side(xr, yr, x[i],y[i], x[(i+1)%4],y[(i+1)%4]))) {
      C_Hit = cut_line_and_line(xr,yr,
				xe, ye,
				x[i],y[i],
				x[(i+1)%4],y[(i+1)%4],
				&c_dist, &c_angle);
      }
      else {
	C_Hit = FALSE;
      }
      
      if(C_Hit) {                                      
	// check if we still hit the obstacle if it is not directly in front
	// of the sensor
	if(!( cz-h/2 <= zr && cz+h/2 >= zr)) {
	  if(c_dist == 0.0) {
	    C_Hit = FALSE;
	  }
	  else {
	    float dz, ta;
	    if(cz+h/2 < zr) 
	      dz = zr - (cz+h/2);
	    else
	      dz = (cz-h/2) - zr;
	    ta = dz/c_dist;
	    if(ta > tan(open_angle)) {
	      C_Hit = FALSE;
	    }
	    else {
	      c_dist = sqrt(dz*dz+c_dist*c_dist);
	    }
	  }
	}
      }
      if(C_Hit) {
	if(Hit) {
	  if (c_dist < *dist) {
	    *dist = c_dist;
	    *angle = c_angle;
	  }
	}
	else {
	  *dist = c_dist;
	  *angle = c_angle;
	  Hit = TRUE;
	}
      }
      
    }
    *surface = my_surface;
    return Hit;   
}

float t_rectangle::min_distance(float xi, float yi)
{
    float dist, tmp;
    dist = MAXFLOAT;
    absolute_points();
    for(int i = 0; i < 4; i++) {
	tmp = line_min_distance(x[i],y[i],x[(i+1)%4],y[(i+1)%4],xi,yi);
	if(tmp < dist) dist = tmp;
    }
    return dist;
}    
