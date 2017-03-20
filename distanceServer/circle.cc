// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/circle.cc,v $
//
//  Created by:      $Author: rstone $
//
//  Revision:        $Revision: 1.1 $
//
//  Date of revision $Date: 2002/09/14 16:52:31 $
//
//  This file is part of the Robotic Telelabor Project.
//
// ============================================================================
//
//                 ------------- HISTORY -----------------
//
//  $Log: circle.cc,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.4  1999/09/29 13:02:18  schulz
//  Several changes, most important, added a scan line algorithm for
//  camera simulation
//
//  Revision 1.3  1998/10/12 13:31:41  schulz
//  This is a complete new version
//
//  Revision 1.2  1998/09/30 15:07:11  schulz
//  Several changes to make things compile under gcc 2.7.2
//
//  Revision 1.1  1998/06/19 13:57:36  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.11  1997/11/15 16:15:39  schulz
//  latest version with collision avoidance support (to disable start RTLClient with -tcx). There are still some race conditions on startup!
//
//  Revision 1.10  1997/09/08 14:00:18  schulz
//  intermediate update, wont run
//
//  Revision 1.9  1997/08/01 12:02:56  schulz
//  many changes
//
//  Revision 1.8  1997/07/24 15:29:02  schulz
//  robots are derived from obstaclesets instead of circles now.
//  So you can give them any shape.
//
//  fixed some bugs
//
//  Revision 1.7  1997/06/20 09:16:22  schulz
//  added basic support for obstacle communication
//
//  Revision 1.6  1997/06/09 13:30:21  schulz
//  database modifications
//
//  Revision 1.5  1997/05/16 14:57:50  schulz
//  source restructioring in progress
//  things won't compile for now
//
//  Revision 1.2  1997/03/04 17:15:00  schulz
//  Added file header and revision history
//
//
// ----------------------------------------------------------------------------
// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/circle.cc,v $
//
//  Created by:      $Author: rstone $
//
//  Revision:        $Revision: 1.1 $
//
//  Date of revision $Date: 2002/09/14 16:52:31 $
//
//  This file is part of the Robotic Telelabor Project.
//
// ============================================================================
//
//                 ------------- HISTORY -----------------
//
//  $Log: circle.cc,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.4  1999/09/29 13:02:18  schulz
//  Several changes, most important, added a scan line algorithm for
//  camera simulation
//
//  Revision 1.3  1998/10/12 13:31:41  schulz
//  This is a complete new version
//
//  Revision 1.2  1998/09/30 15:07:11  schulz
//  Several changes to make things compile under gcc 2.7.2
//
//  Revision 1.1  1998/06/19 13:57:36  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.11  1997/11/15 16:15:39  schulz
//  latest version with collision avoidance support (to disable start RTLClient with -tcx). There are still some race conditions on startup!
//
//  Revision 1.10  1997/09/08 14:00:18  schulz
//  intermediate update, wont run
//
//  Revision 1.9  1997/08/01 12:02:56  schulz
//  many changes
//
//  Revision 1.8  1997/07/24 15:29:02  schulz
//  robots are derived from obstaclesets instead of circles now.
//  So you can give them any shape.
//
//  fixed some bugs
//
//  Revision 1.7  1997/06/20 09:16:22  schulz
//  added basic support for obstacle communication
//
//  Revision 1.6  1997/06/09 13:30:21  schulz
//  database modifications
//
//  Revision 1.5  1997/05/16 14:57:50  schulz
//  source restructioring in progress
//  things won't compile for now
//
//  Revision 1.2  1997/03/04 17:15:00  schulz
//  Added file header and revision history
//
//
// ----------------------------------------------------------------------------

#define _XOPEN_SOURCE
#include <math.h>
#undef _XOPEN_SOURCE
#include "circle.hh"
#include "linalg.h"

extern t_surface *default_surface;

char
t_Circle::bounds(float *xi1, float *yi1, float *xi2, float *yi2)
{
  t_Vector3f p = getPosition();
  *xi1 = p.x() - r/2;
  *yi1 = p.y() - r/2;
  *xi2 = p.x() + r/2;
  *yi2 = p.y() + r/2;
  return TRUE;
}


Boolean
t_Circle::inside(const t_Vector3f &point)
{
  t_Vector3f p = getPosition();
  float dx = p.x() - point.x();
  float dy = p.y() - point.y();
  return( (dx*dx + dy*dy) <= r*r && 
	  point.z() >= p.z()-h/2 && 
	  point.z() <= p.z()+h/2);
}

void
t_Circle::save(FILE *fd)
{
  t_Vector3f p = getPosition();
  fprintf(fd, "CYLINDER %s %f %f %f %f %f",label(), p.x(),p.y(),p.z(),r,h);
}

t_Circle::t_Circle(float xi1, float yi1, float zi, float ir, float hi,
		   const t_Color_3f &rgb)
{
  r = ir;
  h = hi;
  assign_color(WALLCOLOR);
  assign_rgb(rgb);
  move(t_Vector3f(xi1, yi1, zi));
  enable();
}


bool
t_Circle::distance(const t_Vector3f &origin, 
		   const t_Vector3f &direction,
		   float maxLength,
		   float &dist, 
		   float &angle,
		   int *face_id)
{
  Boolean C_HIT;
  
  if(face_id != NULL) *face_id = 0;

  t_Vector3f p = getPosition();
  t_Vector3f dest = origin + maxLength*direction;
  //cout << "origin: " << origin << endl;
  //cout << "dest:   " << dest   << endl;
  //cout << "pos:    " << p << " h: " << h << endl; 
  C_HIT = cut_circle_and_line(p.x(),p.y(),r,
			      origin.x(),origin.y(),
			      dest.x(),dest.y(),&dist,&angle);
  if(C_HIT) {
    if( (p.z()-h/2) < dest.z() && (p.z()+h/2) > dest.z()) {
      return TRUE;
    }
  }
  return FALSE;
}

float
t_Circle::min_distance(float xp, float yp)
{
  float dx,dy;
  t_Vector3f p = getPosition();
  dx = p.x() - xp;
  dy = p.y() - yp;
  return sqrt(dx*dx+dy*dy) - r;
}
