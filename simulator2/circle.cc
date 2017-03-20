
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** File:                        circle.cc
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
 ***** $Source: /usr/local/cvs/bee/src/simulator2/circle.cc,v $
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
 * $Log: circle.cc,v $
 * Revision 1.1  2002/09/14 16:11:07  rstone
 * *** empty log message ***
 *
 * Revision 1.17  1997/04/02 12:36:55  fox
 * Fixed a bug in tag handling.
 *
 * Revision 1.16  1997/04/01 12:15:25  fox
 * Don't dump tags of unknown objects.
 *
 * Revision 1.15  1997/04/01 11:11:29  fox
 * Added comment handling for simulator maps. Comment lines must start with #.
 * Added the possibility to set a tag name for the different objects.
 *
 * Revision 1.14  1997/03/25 08:54:13  schulz
 * no comment ...
 *
 * Revision 1.13  1997/03/24 18:41:38  schulz
 * added a newline after cylinder saving
 *
 * Revision 1.12  1997/03/11 11:02:19  schulz
 * Removed the distinction between RECTANGLEs and CUBEs  when
 * saving the playground. We always save CUBEs now.
 * Dito for CIRCLE <-> CYLINDER
 *
 * Revision 1.11  1997/02/26 09:21:36  schulz
 * Fixed BeamRobot3()
 *
 * Revision 1.10  1997/01/27 15:13:28  schulz
 * added added multiple sensor planes to the sensor simulation stuff
 * I hope this does not introduce to many new bugs.
 *
 * Revision 1.9  1996/12/05 14:59:07  schulz
 * forgot to set color in t_circle
 *
 * Revision 1.8  1996/12/04 14:25:27  schulz
 * Human obstacles completed (hopefully)
 *
 * Revision 1.7  1996/12/03 15:18:28  schulz
 * -- some fixes
 * -- added humans as obstacles (still buggy)
 *
 * Revision 1.6  1996/10/29 16:05:09  ws
 * changed some class names
 *
 * Revision 1.5  1996/10/18 13:59:58  ws
 * obstacles can be toggled on/off now using the right mouse button.
 *
 * Revision 1.4  1996/10/15 13:58:52  ws
 * - new features: deletion of obstacles (Shift+ right mouse button)
 *               usage information (-h or -help option)
 * - added -laser option
 * - Now the simulator waits for BASE to come up before it starts
 *   sending laserReports.
 *
 * Revision 1.3  1996/10/14 12:19:09  ws
 * added support for different surfaces (not yet complete) and sonar error
 *
 * Revision 1.2  1996/10/09 13:27:34  schulz
 * reorganized obstacle store and fixed some more bugs.
 *
// Revision 1.1.1.1  1996/09/30  16:17:44  schulz
// reimport on new source tree
//
 * Revision 1.3  1996/09/20 07:56:20  schulz
 * Added saving of maps. E.g. a filename requester and a save method
 * for each obstacle type.
 *
 * Revision 1.2  1996/09/11 14:03:48  schulz
 * - Speed up sonar and laser simulation by a factor of 20.
 *   We precalculate the set of obstacles which can be hit by a
 *   laser/sonar beam and check only for these few obstacles, if they are hit.
 *   Switched to S.Thruns version of line and circle intersection routines,
 *   these are a little bit faster then the old ones.
 * - Included laser visualisation  (to slow an some machines)
 *
 * Revision 1.1  1996/08/27 15:23:08  schulz
 * Some files forgotten last time
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <math.h>
#include "circle.hh"
#include "linalg.h"
#include "simxw.h"

extern t_surface *default_surface;

void
t_circle::bounds(float *xi1, float *yi1, float *xi2, float *yi2)
{
  *xi1 = cx - r;
  *yi1 = cy - r;
  *xi2 = cx + r;
  *yi2 = cy + r;
}

Boolean
t_circle::inside(float x, float y)
{
  float dx = cx - x;
  float dy = cy - y;
  return( (dx*dx + dy*dy) <= r*r );
}

void
t_circle::expose()
{
  myfig = expose_circle(cx,cy,r, color);  
}

void
t_circle::toggle()
{
    if(enabled) {
	RemoveFigure(myfig);
	myfig = expose_disabled_circle(cx,cy,r,color);
	enabled = FALSE;
    }
    else {
	RemoveFigure(myfig);
	myfig = expose_circle(cx,cy,r,color);
	enabled = TRUE;
    }
}

void
t_circle::redraw()
{
  redraw_circle(myfig,cx,cy,r);           
}

void
t_circle::save(FILE *fd)
{
  if ( strcmp( tag, dummyTag))
    fprintf(fd, "CYLINDER %f %f %f %f %f %s\n",cx,cy,cz,r,h,tag);
  else
    fprintf(fd, "CYLINDER %f %f %f %f %f\n",cx,cy,cz,r,h);
}

t_circle::t_circle(char* tagName, float xi1, float yi1, float zi, float ir, float hi)
{
  type = 1;
  tag = new char[strlen(tagName)+1];
  strcpy( tag, tagName);
  cx = xi1;
  cy = yi1;
  cz = zi;
  r = ir;
  h = hi;
  my_surface = default_surface;
  color = WALLCOLOR;
  enabled = TRUE;
  active = FALSE;
}

Boolean
t_circle::distance(float xs, float ys, float zs, float open_angle,
			   float xe, float ye,
			   float *dist, float *angle, t_surface **surface)
{
    Boolean C_HIT;
    *surface = my_surface;
    C_HIT = cut_circle_and_line(cx,cy,r,xs,ys,xe,ye,dist,angle);
    if(C_HIT) {                                      
	if(!( cz-h/2 <= zs && cz+h/2 >= zs)) {
	    if(*dist == 0.0) {
		C_HIT = FALSE;
	    }
	    else {
		float dz, ta;
		if(cz+h/2 < zs) 
		    dz = zs - (cz+h/2);
		else
		    dz = (cz-h/2) - zs;
		ta = dz / *dist;
		if(ta > tan(open_angle)) {
		    C_HIT = FALSE;
		}
		else {
		    *dist = sqrt(dz*dz+ *dist * *dist);
		}
	    }
	}
    }
    return C_HIT;
}

float
t_circle::min_distance(float xp, float yp)
{
  return 3000;
}



