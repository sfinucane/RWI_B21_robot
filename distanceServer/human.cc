// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/human.cc,v $
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
//  $Log: human.cc,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.2  1999/11/18 12:49:01  schulz
//  Fixed framebuffer, added stuff for humans
//
//  Revision 1.1  1998/10/12 13:31:50  schulz
//  This is a complete new version
//
//  Revision 1.2  1998/09/30 15:07:15  schulz
//  Several changes to make things compile under gcc 2.7.2
//
//  Revision 1.1  1998/06/19 13:57:44  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.4  1997/07/20 10:11:51  schulz
//  Many changes sice last update:
//  -- RTLServer/-Client and -Simulator databases are synchronized
//     over IP/Multicasting now.
//  -- multiple robots are allowed in one scene
//  -- robots are treated like ordinary active obstacles
//  -- added simple 3D visualization for robots
//  -- added a classes netlink and typeInfo, which implement
//  -- a fairly convenient way of transfering object states across the network
//
//  Revision 1.3  1997/05/16 14:57:55  schulz
//  source restructioring in progress
//  things won't compile for now
//
//  Revision 1.2  1997/03/04 17:15:09  schulz
//  Added file header and revision history
//
//
// ----------------------------------------------------------------------------

#define _XOPEN_SOURCE
#include <math.h>
#undef _XOPEN_SOURCE
#include <sys/time.h>

#include "human.hh"
#include "store.hh"

extern t_ObstacleGrid *obstore;

#define HUMAN_H 180.0
#define HUMAN_Z (HUMAN_H/2.0)

t_Human::t_Human(float px, float py, float pz,
		 float ir, float ih, float s, float a)
    : t_Circle(px, py, HUMAN_Z, HUMAN_RADIUS, HUMAN_H)
{
  speed = s;
  angle = a;
  assign_color("red");
  start_pos_x = px;
  start_pos_y = py;
  r = ir;
  h = ih;
  move(t_Vector3f(px,py,pz));  
  stopped = 0;
  gettimeofday(&last_update, NULL);
  activate();
  cerr << "human_position: " << getPosition() << endl;
}

void
t_Human::update()
{
  struct timeval tdiff;
  float secs, trans;
  float x,y,z;
  float nx,ny, ex,ey,dist;
  struct timeval now;
  gettimeofday(&now, NULL);
  time_subtract(&tdiff, &now, &last_update);
  secs = (float) timeval_to_usec(&tdiff) / 1000000.0;
  trans = secs * speed;
  t_Vector3f p = getPosition();

  nx = p.x() + direction*trans*cos(angle);
  ny = p.y() + direction*trans*sin(angle);
  ex = nx + direction*HUMAN_RADIUS*cos(angle);
  ey = ny + direction*HUMAN_RADIUS*sin(angle);
  if(obstore->get_distance(t_Vector3f(nx,ny,p.z()),
			   t_Vector3f::unitX.rotate(t_Vector3f(0,0,angle)),
			   HUMAN_RADIUS, &dist)) {
    nx = p.x()+direction*(HUMAN_RADIUS-dist)*cos(angle);
    ny = p.y()+direction*(HUMAN_RADIUS-dist)*sin(angle);
    direction *= -1;
  }

  last_update.tv_sec = now.tv_sec;
  last_update.tv_usec = now.tv_usec; 
  move(t_Vector3f(nx,ny,p.z()));
}

void
t_Human::save(FILE *fd)
{
  t_Vector3f p = getPosition();
  fprintf(fd, "Human %f %f %f %f\n", p.x(), p.y(), speed, angle);
}
