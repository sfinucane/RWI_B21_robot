// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/obstacleset.cc,v $
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
//  $Log: obstacleset.cc,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.3  1999/09/29 13:02:21  schulz
//  Several changes, most important, added a scan line algorithm for
//  camera simulation
//
//  Revision 1.2  1999/05/20 13:33:58  schulz
//  Changed grid implementation and renamed a method in t_Camera.
//
//  Revision 1.1  1998/10/12 13:32:02  schulz
//  This is a complete new version
//
//  Revision 1.2  1998/09/30 15:07:17  schulz
//  Several changes to make things compile under gcc 2.7.2
//
//  Revision 1.1  1998/06/19 13:57:53  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.13  1997/11/15 16:15:43  schulz
//  latest version with collision avoidance support (to disable start RTLClient with -tcx). There are still some race conditions on startup!
//
//  Revision 1.12  1997/10/25 08:30:04  schulz
//  added initialize method to some classes
//  modified matchTest to estimate position and door state seperately
//
//  Revision 1.11  1997/09/18 08:16:18  schulz
//  Many changes, making RTL compatible with MRT-VR to some extend
//
//  Revision 1.10  1997/09/12 12:07:36  schulz
//  fixed several bugs
//
//  Revision 1.9  1997/09/09 12:57:30  schulz
//  added multiple camera support
//  hopefully fixed destroyed simrtp.hh and sim_rtp.cc again
//
//  Revision 1.8  1997/09/08 14:00:27  schulz
//  intermediate update, wont run
//
//  Revision 1.7  1997/07/24 15:29:06  schulz
//  robots are derived from obstaclesets instead of circles now.
//  So you can give them any shape.
//
//  fixed some bugs
//
//  Revision 1.6  1997/07/09 15:26:28  schulz
//  first attempt to display more then one root in the RTLClient
//
//  Revision 1.5  1997/07/08 15:16:18  schulz
//  Setting up a RTP capable simulator, with no X
//
//  Revision 1.4  1997/06/20 09:16:26  schulz
//  added basic support for obstacle communication
//
//  Revision 1.3  1997/06/19 09:12:47  schulz
//  major filelist cleanup
//
//  Revision 1.2  1997/06/09 13:50:48  schulz
//  added a header file for solaris
//
//  Revision 1.1  1997/06/09 13:44:57  schulz
//  added obstacleSet files
//
//  Revision 1.3  1997/06/07 14:19:10  schulz
//  new obstacle database
//
//  Revision 1.2  1997/03/04 17:15:05  schulz
//  Added file header and revision history
//
//
// ----------------------------------------------------------------------------

#define _XOPEN_SOURCE
#include <math.h>
#undef _XOPEN_SOURCE
#include <values.h>
#ifdef __GNUG__
#include <typeinfo>
#endif
#include "store.hh"

t_ObstacleSet::~t_ObstacleSet()
{
  for(u_long i = 0; i < elements.size(); i++)
    delete elements[i];
}

t_ObstacleSet::t_ObstacleSet(float x, float y, float z, float iori, float iyrot, bool icolor_defined, const t_Color_3f& icolor)
{
  initialize(x,y,z,iori, iyrot, icolor_defined, icolor);
  initialize_typeinfo_array();
}

void
t_ObstacleSet::closeSet()
{
  int count = 0;
  t_PolygonList* plist;
  for(u_long i = 0; i < elements.size(); i++) {
    plist = elements[i]->polygonList();
    if(plist != NULL) {
      count += plist->size();
    }
  }
  polygons = new t_PolygonList(count);
  int j = 0;
  for(u_long i = 0; i < elements.size(); i++) {
    plist = elements[i]->polygonList();
    if(plist != NULL) {
      for(int k = 0; k < plist->size(); k++)
	(*polygons)[j++] = (*plist)[k];  
    }
  }
}

void
t_ObstacleSet::initialize(float x, float y, float z, float iori, float iyrot, bool icolor_defined, const t_Color_3f& icolor)
{
  color_defined = icolor_defined;
  assign_rgb(icolor);
  rotate(iori,iyrot);
  move(t_Vector3f(x,y,z));
  enable();
}

void
t_ObstacleSet::initialize_typeinfo_array()
{
  INITIALIZE_COMMON_FIELDS;
  type_info[NUMBER_OF_COMMON_FIELDS+0] = TI_END_ATTR;  
}

void
t_ObstacleSet::insert(t_Obstacle* new_element)
{
  elements.push_back(new_element);
}

vector<t_Obstacle*>&
t_ObstacleSet::getElements() 
{
  return elements;
}

bool
t_ObstacleSet::distance(const t_Vector3f &from, 
			const t_Vector3f &direction,
			float maxLength,
			float &dist, 
			float &angle,
			int *face_id)
{  
  float tdist, tang;
  int tfid = 0;
  Boolean C_Hit, Hit = FALSE;
  dist = maxLength;
  push_offset();
  for(u_long i = 0; i < elements.size(); i++) {
    C_Hit = elements[i]->distance(from, direction, maxLength,
				  tdist, tang, &tfid);
    if(C_Hit) {
      Hit = TRUE;
      if(tdist < dist) {
	dist = tdist;
	angle = tang;
	if(face_id) *face_id = tfid;
      }
    }
  }
  pop_offset();    
  return Hit;  
}

/*
t_Vector3f 
t_ObstacleSet::memberPosition(t_Obstacle* member)
{
  t_Vector3f memberPos;
  clear_offsets();
  _memberPosition(member, &memberPos);
  return memberPos;
}

int
t_ObstacleSet::_memberPosition(t_Obstacle* member, t_Vector3f *memberPos)
{
  push_offset();
  for(u_long i = 0; i < elements.size(); i++) {
    if(elements[i] == member) {
      *memberPos =  member->getPosition();
      return 1;
    }
    else {
      t_ObstacleSet* memberSet;
      if( (memberSet = dynamic_cast<t_ObstacleSet*>(elements[i]))) {
	if(memberSet->_memberPosition(member, memberPos))
	  return 1;
      }
    }
  }
  pop_offset();
  return 0;
}

t_Vector3f 
t_ObstacleSet::memberOrientation(t_Obstacle* member)
{
  t_Vector3f memberOri;
  clear_offsets();
  _memberPosition(member, &memberOri);
  return memberOri;
}

int
t_ObstacleSet::_memberOrientation(t_Obstacle* member, t_Vector3f *memberOri)
{
  push_offset();
  for(u_long i = 0; i < elements.size(); i++) {
    if(elements[i] == member) {
      *memberOri =  member->getPosition();
      return 1;
    }
    else {
      t_ObstacleSet *memberSet;
      if((memberSet = dynamic_cast<t_ObstacleSet*>(elements[i]))) {
	if(memberSet->_memberPosition(member, memberOri))
	  return 1;
      }
    }
  }
  pop_offset();
  return 0;
}
*/

float t_ObstacleSet::min_distance(float x, float y)
{
  float tdist,dist;
  dist = MAXFLOAT;
  push_offset();
  for(u_long i = 0; i < elements.size(); i++) {
    tdist = elements[i]->min_distance(x,y);
    if(tdist < dist)
      dist = tdist;
  }
  pop_offset();    
  return dist;
}


char
t_ObstacleSet::bounds(float* xo1, float* yo1, float* xo2, float* yo2)
{
  float tx1,ty1,tx2,ty2;
  if(elements.empty()) {
    *xo1 =0;
    *xo2 =0;
    *yo1 =0;
    *yo2 =0;            
    return TRUE;
  }
  *xo1=MAXFLOAT;
  *yo1=MAXFLOAT;
  *xo2=MINFLOAT;
  *yo2=MINFLOAT;
  
  push_offset();
  for(u_long i = 0; i < elements.size(); i++) {
    if(modified()) {
      elements[i]->touch();
    }
    if(elements[i]->bounds(&tx1,&ty1,&tx2,&ty2)) {
      if(tx1 < *xo1)
	*xo1 = tx1;
      if(ty1 < *yo1)
	*yo1 = ty1;
      if(tx2 > *xo2)
	*xo2 = tx2;
      if(ty2 > *yo2)
	*yo2 = ty2;
    }
  }
  pop_offset();
  return TRUE;
}

Boolean
t_ObstacleSet::inside(const t_Vector3f &point)
{
  push_offset();
  for(u_long i = 0; i < elements.size(); i++) {
    if(elements[i]->inside(point))
      return TRUE;
  }
  pop_offset();      
  return FALSE;  
}


void
t_ObstacleSet::update()
{
  if(elements.empty()) {
    return;
  }
  push_offset();
  for(u_long i = 0; i < elements.size(); i++) {
    elements[i]->update();
  }
  pop_offset();  
}

void
t_ObstacleSet::save(FILE* savefd)
{
  t_Vector3f p = getPosition();
  t_Vector3f o = getOrientation();
  fprintf(savefd, "Begin ");
  if(label()) fprintf(savefd, "%s ", label());
  fprintf(savefd, "%g %g %g %g %g\n", p.x(), p.y(), p.z(), o.z(), o.y());
  for(u_long i = 0; i < elements.size(); i++) {
    elements[i]->save(savefd);
  }
  fprintf(savefd, "End\n");
}

t_AttribArray
t_ObstacleSet::netAttrs()
{
  return (t_AttribArray) type_info;
}
