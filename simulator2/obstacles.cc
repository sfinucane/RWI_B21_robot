
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** File:                        obstacles.cc
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
 ***** $Source: /usr/local/cvs/bee/src/simulator2/obstacles.cc,v $
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
 * $Log: obstacles.cc,v $
 * Revision 1.1  2002/09/14 16:11:07  rstone
 * *** empty log message ***
 *
 * Revision 1.12  1997/04/02 12:36:56  fox
 * Fixed a bug in tag handling.
 *
 * Revision 1.11  1997/04/01 11:11:30  fox
 * Added comment handling for simulator maps. Comment lines must start with #.
 * Added the possibility to set a tag name for the different objects.
 *
 * Revision 1.10  1997/01/27 15:13:29  schulz
 * added added multiple sensor planes to the sensor simulation stuff
 * I hope this does not introduce to many new bugs.
 *
 * Revision 1.9  1996/12/13 12:19:53  schulz
 * added code to adjust the playground to the positive quadrant before
 * saving
 *
 * Revision 1.8  1996/12/09 13:30:31  schulz
 * -- added orintation to rectangles
 * -- additional minor fixes
 *
 * Revision 1.7  1996/12/04 14:48:41  schulz
 * Just one more bug
 *
 * Revision 1.6  1996/12/04 14:25:28  schulz
 * Human obstacles completed (hopefully)
 *
 * Revision 1.5  1996/12/03 15:18:31  schulz
 * -- some fixes
 * -- added humans as obstacles (still buggy)
 *
 * Revision 1.4  1996/10/29 16:05:11  ws
 * changed some class names
 *
 * Revision 1.3  1996/10/15 13:58:53  ws
 * - new features: deletion of obstacles (Shift+ right mouse button)
 *               usage information (-h or -help option)
 * - added -laser option
 * - Now the simulator waits for BASE to come up before it starts
 *   sending laserReports.
 *
 * Revision 1.2  1996/10/09 13:27:39  schulz
 * reorganized obstacle store and fixed some more bugs.
 *
// Revision 1.1.1.1  1996/09/30  16:17:45  schulz
// reimport on new source tree
//
 * Revision 1.1  1996/08/27 15:23:09  schulz
 * Some files forgotten last time
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include "simxw.h"

#include "obstacles.hh"
#include "rectangle.hh"
#include "circle.hh"
#include "doors.hh"
#include "human.hh"

#define N_OBSTACLE_TYPES 7
char* o_types[N_OBSTACLE_TYPES] ={
    "RECTANGLE",
    "CIRCLE",
    "Door",
    "Human",
    "CUBE",
    "CYLINDER",
    "3D-Door"
};

#define MAX_LINE_LENGTH 255
char* dummyTag = "UNKNOWN";
char commentSign = '#';

t_obst* install_obstacle(int type, FILE* mapfd)
{
    int k;
    float x,y,z,w,d,h,r,angle;
    char line[MAX_LINE_LENGTH];
    char tagString[MAX_LINE_LENGTH];
    char* tagName = tagString;

    // Read in the whole line. 
    fgets( line, sizeof( line), mapfd);
    switch(type) {
    case 0:			// RECTANGLE
	k = sscanf( line, "%f %f %f %f %f %s",&x,&y,&w,&d,&angle, tagName);
	if(k < 4) { printf("mapfile corrupted\n"); exit(0);}
	if(k < 6 || *tagName == commentSign) tagName = dummyTag;
	return new t_rectangle(tagName,x,y,DEF_Z,w,d,DEF_H);
    case 1:			// CIRCLE
	k = sscanf( line, "%f %f %f %s", &x,&y,&r,tagName);
	if(k < 3) { printf("mapfile corrupted\n"); exit(0);}
	if(k < 4 || *tagName == commentSign) tagName = dummyTag;
	return new t_circle(tagName,x,y,DEF_Z,r,DEF_H);
    case 2:
    {
	k = sscanf( line, "%f %f %f %f %f %s", &x, &y, &w, &d, &angle, tagName);
	if(k < 5) { printf("mapfile corrupted\n"); exit(0);}
	if(k < 6 || *tagName == commentSign) tagName = dummyTag;
	return new t_door(tagName,w,d,DEF_H,x,y,DEF_Z,angle);
    }
    case 3:
    {
	float s,a;
	k = sscanf( line, "%f %f %f %f %s", &x, &y, &s, &a, tagName);
	if(k < 4) { printf("mapfile corrupted\n"); exit(0);}
	if(k < 5 || *tagName == commentSign) tagName = dummyTag;
	return new t_human(tagName,x,y,s,a);
    }
    case 4:			// CUBE
      k = sscanf( line, "%f %f %f %f %f %f %f %s",&x,&y,&z,&w,&d,&h,&angle,tagName);
      if(k < 6) { printf("mapfile corrupted\n"); exit(0);}
      if(k < 8 || *tagName == commentSign) tagName = dummyTag;
      if(k >= 7)
	return new t_rectangle(tagName,x,y,z,w,d,h,angle);
      else
	return new t_rectangle(tagName,x,y,z,w,d,h);
    case 5:			// CYLINDER
	k = sscanf( line, "%f %f %f %f %f %s", &x,&y,&z,&r,&h, tagName);
	if(k < 5) { printf("mapfile corrupted\n"); exit(0);}
	if(k < 6 || *tagName == commentSign) tagName = dummyTag;
	return new t_circle(tagName,x,y,z,r,h);
    case 6:                     // 3D-DOOR
    {
	float w,h,angle;
	k = sscanf( line, "%f %f %f %f %f %s", &x, &y, &z, &w, &d, &h, &angle, tagName);
	if(k < 5) { printf("mapfile corrupted\n"); exit(0);}
	if(k < 6 || *tagName == commentSign) tagName = dummyTag;
	return new t_door(tagName,w,d,h,x,y,z,angle);
    }
    }
    return NULL;
}

void t_obst::new_color(char *color)
{
  change_color(myfig, color);  
}

t_obst::t_obst()
{
  active = FALSE;
}

t_obst::~t_obst()
{
    RemoveFigure(myfig);
}

void t_obst::mouse_action(float x, float y)
{
    return;
}

void t_obst::update(struct timeval* now)
{
  return;
}

void
t_obst::move(float diffX, float diffY)
{
    cx += diffX;
    cy += diffY;
}    
