// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/door.cc,v $
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
//  $Log: door.cc,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.3  1999/10/28 09:19:25  schulz
//  -- first running version of framebuffer
//  -- bugfixes in linalg stuff
//  -- bugfixes in door.*
//
//  Revision 1.2  1999/05/20 13:33:55  schulz
//  Changed grid implementation and renamed a method in t_Camera.
//
//  Revision 1.1  1998/10/12 13:31:45  schulz
//  This is a complete new version
//
//  Revision 1.2  1998/09/30 15:07:14  schulz
//  Several changes to make things compile under gcc 2.7.2
//
//  Revision 1.1  1998/06/19 13:57:41  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.7  1997/10/21 12:51:39  schulz
//  still cleaningup for first release
//
//  Revision 1.6  1997/10/02 10:52:14  schulz
//  minor changes mostly to get doormatch running
//
//  Revision 1.5  1997/09/29 16:02:51  schulz
//  Made source files SUN C++ compatible
//
//  Revision 1.4  1997/09/25 15:15:05  schulz
//  so on
//
//  Revision 1.3  1997/09/25 12:16:47  schulz
//  made constructor sequence for rectangles explicit (initialize method)
//
//  Revision 1.2  1997/09/18 08:16:13  schulz
//  Many changes, making RTL compatible with MRT-VR to some extend
//
//  Revision 1.1  1997/05/16 14:57:52  schulz
//  source restructioring in progress
//  things won't compile for now
//
//  Revision 1.2  1997/03/04 17:15:07  schulz
//  Added file header and revision history
//
//
// ----------------------------------------------------------------------------

#define _XOPEN_SOURCE
#include <math.h>
#undef _XOPEN_SOURCE
#include "door.hh"
#include "linalg.h"

t_Door::t_Door(float wi, float di, float hi, float apxi, float apyi, float apzi, float anglei, const t_Color_3f &rgb)
{
  initialize(wi, di, hi, apxi, apyi, apzi, anglei, rgb);
  initialize_typeinfo_array();
}

void
t_Door::initialize_typeinfo_array()
{
  INITIALIZE_COMMON_FIELDS;
  type_info[NUMBER_OF_COMMON_FIELDS+0] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "apx", &apx);
  type_info[NUMBER_OF_COMMON_FIELDS+1] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "apz", &apz);
  type_info[NUMBER_OF_COMMON_FIELDS+2] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "apy", &apy);
  type_info[NUMBER_OF_COMMON_FIELDS+3] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "width", &w);
  type_info[NUMBER_OF_COMMON_FIELDS+4] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "height", &h);
  type_info[NUMBER_OF_COMMON_FIELDS+5] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "depth", &d);
  type_info[NUMBER_OF_COMMON_FIELDS+6] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "angle", &angle);
  type_info[NUMBER_OF_COMMON_FIELDS+7] = TI_END_ATTR;  
}

void
t_Door::initialize(float wi, float di, float hi, float apxi, float apyi, float apzi, float anglei, const t_Color_3f &rgb)
{
  t_Rectangle::initialize(apxi - wi/2, apyi - di/2, apzi, wi, di, hi, anglei, rgb);
  type = DOOR;
  apx = apxi;
  apy = apyi;
  apz = apzi;
  angle = anglei;
  rotate(angle);
  last_angle = angle;
  assign_color("green"); 
}

void
t_Door::absolute_points()
{
  rotate(angle);
  t_Rectangle::absolute_points();
}


void
t_Door::rotate(float orii, float yroti)
{
  float tx,ty,tz;
  angle = orii;
  tx = apx + w/2 * cos(angle) - d/2 * sin(angle);
  ty = apy + w/2 * sin(angle) + d/2 * cos(angle);
  tz = apz;
  move(t_Vector3f(tx,ty,tz));
  t_Obstacle::rotate(angle);  
}

t_Vector3f 
t_Door::getHingePosition() const
{
  return t_Vector3f(apx, apy, apz);
}

void
t_Door::save(FILE *fd)
{
  fprintf(fd, "DOOR %f %f %f %f %f %f %f\n", apx, apy, apz, w, d, h, angle);
}

t_AttribArray
t_Door::netAttrs()
{
  return (t_AttribArray) type_info;
}
