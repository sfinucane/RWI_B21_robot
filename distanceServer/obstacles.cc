// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/obstacles.cc,v $
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
//  $Log: obstacles.cc,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.5  2000/03/09 08:47:45  schulz
//  some minor changes
//
//  Revision 1.4  2000/02/25 14:32:04  schulz
//  dont know
//
//  Revision 1.3  1999/09/29 13:02:18  schulz
//  Several changes, most important, added a scan line algorithm for
//  camera simulation
//
//  Revision 1.2  1999/05/20 13:33:57  schulz
//  Changed grid implementation and renamed a method in t_Camera.
//
//  Revision 1.1  1998/10/12 13:32:00  schulz
//  This is a complete new version
//
//  Revision 1.4  1998/10/12 08:32:32  schulz
//  several changes
//
//  Revision 1.3  1998/09/30 15:07:16  schulz
//  Several changes to make things compile under gcc 2.7.2
//
//  Revision 1.2  1998/06/29 13:43:18  schulz
//  added lookForRobots sensor
//
//  Revision 1.1  1998/06/19 13:57:52  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//
// ----------------------------------------------------------------------------

#include <strings.h>
#define _XOPEN_SOURCE
#include <math.h>
#undef _XOPEN_SOURCE
#include "obstacles.hh"
#include "comdef.h"

extern t_surface *default_surface;

u_long t_Obstacle::number_of_obstacles = 0;

stack<t_Vector3f, vector<t_Vector3f> > t_Obstacle::off_trans[MAX_NUMBER_OF_THREADS];
stack<t_Vector3f, vector<t_Vector3f> > t_Obstacle::off_ori[MAX_NUMBER_OF_THREADS];

u_short t_Obstacle::hierarchy = 0;

t_ObstacleRecord::t_ObstacleRecord()
{
  label = NULL;
  texture = NULL;
  type = UNKNOWN;
}

void
t_Obstacle::clear_offsets()
{
  off_trans[myThread()].empty();
  off_ori[myThread()].empty();
}


void
t_Obstacle::toggle()
{
  if(enabled_flag) enabled_flag = FALSE;
  else enabled_flag = TRUE;
}

void
t_Obstacle::touch()
{
  modified_flag = TRUE;
}

void
t_Obstacle::move(const t_Vector3f& p)
{
  center = p;
  touch();
}

void
t_Obstacle::rotate(float orii, float yroti)
{
  orientation = t_Vector3f(0, yroti, orii);
  touch();
}

t_Obstacle::t_Obstacle()
{
  char l[80];
  active_ = FALSE;
  interface_ptr = NULL;
  polygons = NULL;
  modified_flag = FALSE;
  type = UNKNOWN;
  enable();
  assign_surface(default_surface);
  number = number_of_obstacles++;
  access_stamp = 0;
  sprintf(l,"_%d", number);
  label_ = new char[strlen(l)+1];
  strcpy(label_, l);
}

u_long 
t_Obstacle::id() const
{
  return number;
}

int
t_Obstacle::already_checked()
{
  return (access_stamp == current_stamp);
}

u_long t_Obstacle::current_stamp = 1;

void
t_Obstacle::mark_checked()
{
  access_stamp = current_stamp;
}

void
t_Obstacle::new_stamp()
{
  current_stamp++;
}

char*
t_Obstacle::label() const
{
  return label_;
}

void
t_Obstacle::assign_label(const char* l)
{
  if(label_) delete [] label_;
  label_ = new char[strlen(l)+1];
  strcpy(label_, l);
}

void
t_Obstacle::assign_color(const char* c)
{
  color = c;
}

void 
t_Obstacle::assign_surface(const t_surface *surface)
{
  my_surface = surface;
}

const t_surface* 
t_Obstacle::surface() const
{
  return my_surface;
}

t_Obstacle::~t_Obstacle()
{
  if(label_) {
    delete [] label_;
  }
  if(interface_ptr)
    delete interface_ptr;
}

void
t_Obstacle::activate()
{
  active_ = TRUE;
}

void 
t_Obstacle::assign_rgb(const t_Color_3f &rgb_in)
{
  rgb_color = rgb_in;
}

void
t_Obstacle::update()
{
  abs_center = getPosition();
  abs_orientation = getOrientation();
  return;
}

void
t_Obstacle::synchronise()
{
  netRead();
}

t_AttribArray
t_Obstacle::netAttrs() const
{
  return (t_AttribArray) NULL;
}


t_NetLink* t_Obstacle::my_NetLink;


void
t_Obstacle::netWrite()
{
  if(my_NetLink == NULL) return;
  if(!label()) return;
  t_TypedPtr name_info[2];

  char *lenptr = my_NetLink->writePacketHeader(COMM_MODIFY, COMM_NAMEDOBJECT);
  name_info[0] = t_TypedPtr(t_Attrib::TI_STRING, "obstacle name", &label_);
  name_info[1] = TI_END_ATTR;
  my_NetLink->write(name_info);
  u_short len = my_NetLink->write(netAttrs());
  bcopy(&len, lenptr, sizeof(u_short));
  my_NetLink->netWrite();
}

u_short
t_Obstacle::netRead()
{
  touch();
  return my_NetLink->read(netAttrs());
}

void
t_Obstacle::assignNetLink(t_NetLink *link)
{
  my_NetLink = link;
}

t_TypedPtr*
t_Obstacle::typedAttribute(char *name) const
{
  if(!name) return NULL;
  t_TypedPtr* attr = netAttrs();
  if(!attr) return NULL;
  while(attr->type != t_Attrib::TI_END) {
    if(strcasecmp(attr->name, name) == 0) 
      return attr;
    attr++;
  }
  cerr << "WARNING: Unknown attribute " << name << " requested" << endl;
  return NULL;
}

void
t_Obstacle::getAttribute(char *name, int* ival, char** cval) const
{
  if(!name) {
    cerr << "WARNING: getAttribute no attribute name specified" << endl;
    return;
  }
  t_TypedPtr* tptr = typedAttribute(name);
  if(!tptr) {
    return;
  }
  switch(tptr->type) {
  case t_Attrib::TI_LONG: 
  case t_Attrib::TI_FLOAT:
    *ival = *((int*)tptr->value);
    break;
  case t_Attrib::TI_STRING: 
    *cval = *((char**)tptr->value);
    break;
  default:
    cerr << "WARNING: can not handle query for attribute " << name << endl;
    return;
  }
}

void
t_Obstacle::changeAttribute(char *name, int ival, char* cval)
{
  if(!name) return;
  t_TypedPtr* tptr = typedAttribute(name);
  if(!tptr) return;
  switch(tptr->type) {
  case t_Attrib::TI_LONG: 
  case t_Attrib::TI_FLOAT: 
    *((int*)tptr->value) = ival;  
  break;
  case t_Attrib::TI_STRING: {
    char *string = new char[strlen(cval)+1];
    strcpy(string, cval);
    *((char**)tptr->value) = string;
    break;
  }
  default:
    cerr << "WARNING: can not handle update for attribute " << name << endl;
    return;
  }
  touch();
  netWrite();
}

void
t_Obstacle::changeAttribute(char *name, void* val)
{
  if(!name) return;
  t_TypedPtr* tptr = typedAttribute(name);
  if(!tptr) return;
  switch(tptr->type) {
  case t_Attrib::TI_LONG: 
  case t_Attrib::TI_FLOAT: 
    *((int*)tptr->value) = *((int*) val);
  break;
  case t_Attrib::TI_STRING: {
    char *string = new char[strlen((char*)val)+1];
    strcpy(string, (char*) val);
    *((char**)tptr->value) = string;
    break;
  }
  default:
    cerr << "WARNING: can not handle update for attribute " << name << endl;
    return;
  }
  touch();
}

t_Vector3f
t_Obstacle::getAbsPosition() const
{
  return abs_center;
}

t_Vector3f
t_Obstacle::getAbsOrientation() const
{
  return abs_orientation;
}

t_Vector3f
t_Obstacle::getPosition() const
{
  if(off_trans[myThread()].empty()) 
    return center;
  else
    return 
      center.rotate(off_ori[myThread()].top())
      + off_trans[myThread()].top();
}

t_Vector3f
t_Obstacle::getOrientation() const
{
  if(off_ori[myThread()].empty()) 
    return orientation;
  else
    return orientation+off_ori[myThread()].top();
}

const t_surface *
t_Obstacle::getSurface() const
{
  return my_surface;
}

const char*
t_Obstacle::getColor() const
{
  return color;
}

const t_Color_3f& 
t_Obstacle::getRGB() const
{
  return rgb_color;
}
