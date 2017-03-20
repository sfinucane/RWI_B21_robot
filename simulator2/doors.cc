
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** File:                        doors.cc
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
 ***** $Source: /usr/local/cvs/bee/src/simulator2/doors.cc,v $
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
 * $Log: doors.cc,v $
 * Revision 1.1  2002/09/14 16:11:07  rstone
 * *** empty log message ***
 *
 * Revision 1.12  1997/04/20 12:32:26  schulz
 * -- Switched ACTIVE back to false for doors.
 * -- baseServer opcodes are read from baseOpcodes.h now
 *
 * Revision 1.11  1997/04/02 12:36:55  fox
 * Fixed a bug in tag handling.
 *
 * Revision 1.10  1997/04/01 12:15:26  fox
 * Don't dump tags of unknown objects.
 *
 * Revision 1.9  1997/04/01 11:11:30  fox
 * Added comment handling for simulator maps. Comment lines must start with #.
 * Added the possibility to set a tag name for the different objects.
 *
 * Revision 1.8  1997/02/27 15:43:00  schulz
 * fixed handling of doors and some other bugs
 *
 * Revision 1.7  1997/01/27 15:13:28  schulz
 * added added multiple sensor planes to the sensor simulation stuff
 * I hope this does not introduce to many new bugs.
 *
 * Revision 1.6  1996/12/13 12:19:52  schulz
 * added code to adjust the playground to the positive quadrant before
 * saving
 *
 * Revision 1.5  1996/12/04 14:25:28  schulz
 * Human obstacles completed (hopefully)
 *
 * Revision 1.4  1996/12/02 17:01:55  schulz
 * new user interface for obstacle insertion
 * some fixes
 *
 * Revision 1.3  1996/10/29 16:05:10  ws
 * changed some class names
 *
 * Revision 1.2  1996/10/09 13:27:36  schulz
 * reorganized obstacle store and fixed some more bugs.
 *
// Revision 1.1.1.1  1996/09/30  16:17:44  schulz
// reimport on new source tree
//
 * Revision 1.2  1996/09/20 07:56:20  schulz
 * Added saving of maps. E.g. a filename requester and a save method
 * for each obstacle type.
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

#include "doors.hh"
#include "simxw.h"
#include "linalg.h"

t_door::t_door(char* tagName, float wi, float di, float hi, float apxi, float apyi, float apzi, float anglei)
{
    type = 2;
    tag = new char[strlen(tagName)+1];
    tag = strcpy( tag, tagName);
    w = fabs(wi);
    d = di;
    h = fabs(hi);
    apx = apxi;
    apy = apyi;
    apz = apzi;
    angle = anglei;
    cz = apzi;
    absolute_points();
    color = "green"; 
    //    active = TRUE;
}

void
t_door::move(float diffX, float diffY)
{
    apx += diffX;
    apy += diffY;
}


  
#define rotpnt(angle,x,y) \
{ \
      float t = x; \
    x = x * cos(angle) - y * sin(angle); \
    y = t  * sin(angle) + y * cos(angle); \
					      }
void t_door::absolute_points()
{
    if(d >= 0) {
	x[0] = w;
	y[0] = 0;
	rotpnt(angle,x[0],y[0]);
	x[1] = w;
	y[1] = d;
	rotpnt(angle,x[1],y[1]);
	x[2] = 0;
	y[2] = d;
	rotpnt(angle,x[2],y[2]);
	x[3] = 0;
	y[3] = 0;
	rotpnt(angle,x[3],y[3]);
    }
    else {
	x[0] = w;
	y[0] = d;
	rotpnt(angle,x[0],y[0]);
	x[1] = w;
	y[1] = 0;
	rotpnt(angle,x[1],y[1]);
	x[2] = 0;
	y[2] = 0;
	rotpnt(angle,x[2],y[2]);
	x[3] = 0;
	y[3] = d;
	rotpnt(angle,x[3],y[3]);
    }
	for(int i = 0; i <4; i++) {
	x[i] += apx;
	y[i] += apy;
    }
}


void t_door::save(FILE *fd)
{
  if ( strcmp( tag, dummyTag)) {
    if(cz == DEF_Z && h == DEF_H)
      fprintf(fd, "Door %f %f %f %f %f %s\n", apx, apy, w, d, angle, tag);
    else
      fprintf(fd, "3D-Door %f %f %f %f %f %f %f %s\n", apx, apy, apz, w, d, h, angle,tag);
  }
  else {
    if(cz == DEF_Z && h == DEF_H)
      fprintf(fd, "Door %f %f %f %f %f\n", apx, apy, w, d, angle);
    else
      fprintf(fd, "3D-Door %f %f %f %f %f %f %f\n", apx, apy, apz, w, d, h, angle);
  }
}

void t_door::mouse_action(float x, float y)
{
    float m;
    last_angle = angle;    
    if( x == apx ) {
	angle = -M_PI_2;
        if(y > apy) angle *= -1;
    }
    else {
	m = (apy - y) / (apx-x);
	angle = atan(m);
	if(x < apx) angle += M_PI;
    }
    redraw();
}
