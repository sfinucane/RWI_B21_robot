// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/rectangle.cc,v $
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
//  $Log: rectangle.cc,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.5  1999/09/29 13:02:22  schulz
//  Several changes, most important, added a scan line algorithm for
//  camera simulation
//
//  Revision 1.4  1999/05/20 13:33:58  schulz
//  Changed grid implementation and renamed a method in t_Camera.
//
//  Revision 1.3  1998/10/12 13:32:03  schulz
//  This is a complete new version
//
//  Revision 1.3  1998/09/30 15:07:18  schulz
//  Several changes to make things compile under gcc 2.7.2
//
//  Revision 1.2  1998/08/04 15:59:44  schulz
//  Several bug fixes
//
//  Revision 1.1  1998/06/19 13:57:57  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.40  1997/11/15 16:15:45  schulz
//  latest version with collision avoidance support (to disable start RTLClient with -tcx). There are still some race conditions on startup!
//
//  Revision 1.39  1997/09/29 16:03:01  schulz
//  Made source files SUN C++ compatible
//
//  Revision 1.38  1997/09/25 12:16:48  schulz
//  made constructor sequence for rectangles explicit (initialize method)
//
//  Revision 1.37  1997/09/08 14:00:28  schulz
//  intermediate update, wont run
//
//  Revision 1.36  1997/08/01 12:03:03  schulz
//  many changes
//
//  Revision 1.35  1997/07/20 10:11:58  schulz
//  Many changes sice last update:
//  -- RTLServer/-Client and -Simulator databases are synchronized
//     over IP/Multicasting now.
//  -- multiple robots are allowed in one scene
//  -- robots are treated like ordinary active obstacles
//  -- added simple 3D visualization for robots
//  -- added a classes netlink and typeInfo, which implement
//  -- a fairly convenient way of transfering object states across the network
//
//  Revision 1.34  1997/07/09 15:26:29  schulz
//  first attempt to display more then one root in the RTLClient
//
//  Revision 1.33  1997/07/08 15:16:20  schulz
//  Setting up a RTP capable simulator, with no X
//
//  Revision 1.32  1997/06/27 11:53:33  schulz
//  -- removed debug messages
//
//  Revision 1.31  1997/06/26 12:28:51  schulz
//  serverv setup changed to server camera positions for MRTVR
//
//  Revision 1.30  1997/06/26 09:21:26  schulz
//  Setup server for first test with MRTVR
//
//  Revision 1.29  1997/06/25 10:05:31  schulz
//  debug
//
//  Revision 1.28  1997/06/25 09:59:58  schulz
//  debug
//
//  Revision 1.27  1997/06/25 09:43:14  schulz
//  debug
//
//  Revision 1.26  1997/06/25 09:22:16  schulz
//  debug
//
//  Revision 1.25  1997/06/25 09:16:17  schulz
//  debug
//
//  Revision 1.24  1997/06/25 09:06:26  schulz
//  debug
//
//  Revision 1.23  1997/06/25 08:55:29  schulz
//  debug
//
//  Revision 1.22  1997/06/25 08:50:14  schulz
//  debug
//
//  Revision 1.21  1997/06/25 08:41:40  schulz
//  binary rep stuff
//
//  Revision 1.20  1997/06/24 16:14:43  schulz
//  *** empty log message ***
//
//  Revision 1.19  1997/06/24 11:56:59  schulz
//  *** empty log message ***
//
//  Revision 1.18  1997/06/20 16:03:40  schulz
//  dbg
//
//  Revision 1.17  1997/06/20 15:47:47  schulz
//  dbg
//
//  Revision 1.16  1997/06/20 15:39:13  schulz
//  dbg
//
//  Revision 1.15  1997/06/20 15:25:01  schulz
//  debugging
//
//  Revision 1.14  1997/06/20 14:47:15  schulz
//  debugging continued
//
//  Revision 1.13  1997/06/20 14:36:53  schulz
//  debugging packets formats
//
//  Revision 1.12  1997/06/20 10:12:37  schulz
//  rearrangements
//
//  Revision 1.11  1997/06/20 09:16:27  schulz
//  added basic support for obstacle communication
//
//  Revision 1.10  1997/06/11 13:17:31  schulz
//  format changes
//
//  Revision 1.9  1997/06/09 13:30:24  schulz
//  database modifications
//
//  Revision 1.8  1997/06/07 14:19:15  schulz
//  new obstacle database
//
//  Revision 1.7  1997/05/16 14:57:59  schulz
//  source restructioring in progress
//  things won't compile for now
//
//  Revision 1.3  1997/03/23 10:28:46  schulz
//  changes for 970318 demo
//
//  Revision 1.2  1997/03/04 17:15:21  schulz
//  Added file header and revision history
//
//
// ----------------------------------------------------------------------------

#define _XOPEN_SOURCE
#include <math.h>
#undef _XOPEN_SOURCE
#include <values.h>
#include <strings.h>
#include "rectangle.hh"
#include "linalg.h"

#include "t_ComRepTypes.hh"
#include "comdef.h"

extern t_surface *default_surface;

char
t_Rectangle::bounds(float *xi1, float *yi1, float *xi2, float *yi2)
{
    if( !modified() ) {
	*xi1 = x1;
	*xi2 = x2;
	*yi1 = y1;
	*yi2 = y2;
	return TRUE;
    }
    else {
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
	modified_flag = FALSE;
	*xi1 = x1 = min_x;
	*yi1 = y1 = min_y;
	*xi2 = x2 = max_x;
	*yi2 = y2 = max_y;
    }
    return TRUE;
}


#define rotpnt(angle,x,y) \
{ \
      float t = x; \
    x = x * cos(angle) - y * sin(angle); \
    y = t  * sin(angle) + y * cos(angle); \
					      }

void
t_Rectangle::absolute_points()
{
  if(!modified()) return;

  t_Vector3f abs_rot = getOrientation();
  t_Vector3f pos_ = getPosition();

  vertices[0] = t_Vector3f( -w/2, -d/2, -h/2).rotate(abs_rot) + pos_;
  vertices[1] = t_Vector3f( -w/2,  -d/2, h/2).rotate(abs_rot) + pos_;
  vertices[2] = t_Vector3f( -w/2,  d/2,  h/2).rotate(abs_rot) + pos_;
  vertices[3] = t_Vector3f( -w/2,  d/2, -h/2).rotate(abs_rot) + pos_;
  vertices[4] = t_Vector3f(  w/2, -d/2, -h/2).rotate(abs_rot) + pos_;
  vertices[5] = t_Vector3f(  w/2,  -d/2, h/2).rotate(abs_rot) + pos_;
  vertices[6] = t_Vector3f(  w/2,  d/2,  h/2).rotate(abs_rot) + pos_;
  vertices[7] = t_Vector3f(  w/2,  d/2, -h/2).rotate(abs_rot) + pos_;

  x[0] = vertices[4].x();
  y[0] = vertices[4].y();
  x[1] = vertices[7].x();
  y[1] = vertices[7].y();
  x[2] = vertices[3].x();
  y[2] = vertices[3].y();
  x[3] = vertices[0].x();
  y[3] = vertices[0].y();

  normals[0] = planeNormal(vertices[0], vertices[1], vertices[2]);
  normals[1] = planeNormal(vertices[3], vertices[2], vertices[6]);
  normals[2] = planeNormal(vertices[6], vertices[5], vertices[4]);
  normals[3] = planeNormal(vertices[5], vertices[1], vertices[0]);
  normals[4] = planeNormal(vertices[1], vertices[5], vertices[6]);
  normals[5] = planeNormal(vertices[0], vertices[3], vertices[7]);

  faces[0][0] = vertices[0];
  faces[0][1] = vertices[1];
  faces[0][2] = vertices[2];
  faces[0][3] = vertices[3];

  faces[1][0] = vertices[3];
  faces[1][1] = vertices[2];
  faces[1][2] = vertices[6];
  faces[1][3] = vertices[7];

  faces[2][0] = vertices[7];
  faces[2][1] = vertices[6];
  faces[2][2] = vertices[5];
  faces[2][3] = vertices[4];

  faces[3][0] = vertices[4];
  faces[3][1] = vertices[5];
  faces[3][2] = vertices[1];
  faces[3][3] = vertices[0];

  faces[4][0] = vertices[5];
  faces[4][1] = vertices[6];
  faces[4][2] = vertices[2];
  faces[4][3] = vertices[1];

  faces[5][0] = vertices[7];
  faces[5][1] = vertices[4];
  faces[5][2] = vertices[0];
  faces[5][3] = vertices[3];

  modified_flag = FALSE;
}

Boolean
t_Rectangle::face(int id, t_Face* returned_face)
{
  switch(id) {
  case 0:
    returned_face->width = d;
    returned_face->height = h;
    returned_face->translation = t_Vector3f(-0.5*w, -0.5*d, -0.5*h);
    break;
  case 1:
    returned_face->width = w;
    returned_face->height = h;
    returned_face->translation = t_Vector3f(-0.5*w, -0.5*d, -0.5*h);
    break;
  case 2:
    returned_face->width = d;
    returned_face->height = h;
    returned_face->translation = t_Vector3f(0.5*w, -0.5*d, -0.5*h);
    break;
  case 3:
    returned_face->width = w;
    returned_face->height = h;
    returned_face->translation = t_Vector3f(0.5*w, -0.5*d, -0.5*h);
    break;
  case 4:
    returned_face->width = w;
    returned_face->height = d;
    returned_face->translation = t_Vector3f(-0.5*w, 0.5*d, 0.5*h);
    break;
  case 5:
    returned_face->width = w;
    returned_face->height = d;
    returned_face->translation = t_Vector3f(-0.5*w, -0.5*d, -0.5*h);
    break;
  default:
    break;
  }
  returned_face->face_id = id;
  return TRUE;
} 

void t_Rectangle::save(FILE *fd)
{
  t_Vector3f p = getPosition();
  fprintf(fd, "CUBE %s %f %f %f %f %f %f",label(), p.x(),p.y(),p.z(),w,d,h);
  p = getOrientation();
  fprintf(fd, " %f\n", p.z());
}

void
t_Rectangle::initialize(float cxi, float cyi, float czi,
		      float wi, float di, float hi, float orii,
			     const t_Color_3f &rgb)
{

  w = fabs(wi);
  d = fabs(di);
  h = fabs(hi);

  dimensions.w = w;
  dimensions.d = d;
  dimensions.h = h;

  x1 = cxi - w/2;
  y1 = cyi - d/2;  
  x2 = cxi + w/2;
  y2 = cyi + d/2;  

  assign_color(WALLCOLOR);
  assign_rgb(rgb);
  move(t_Vector3f(cxi,cyi,czi));
  rotate(orii);
  touch();
  enable();
  absolute_points();
  polygons = new t_PolygonList(6);
  t_PolygonList& poly = *polygons;
  for(int i = 0; i < 6; i++) {
    poly[i].type = t_Polygon::RECTANGLE;
    poly[i].vertices = faces[i];
    poly[i].normal = &normals[i];
  }
}


void
t_Rectangle::initialize_typeinfo_array()
{
  INITIALIZE_COMMON_FIELDS;
  type_info[NUMBER_OF_COMMON_FIELDS+0] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "width", &w);
  type_info[NUMBER_OF_COMMON_FIELDS+1] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "height", &h);
  type_info[NUMBER_OF_COMMON_FIELDS+2] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "depth", &d);
  type_info[NUMBER_OF_COMMON_FIELDS+3] = TI_END_ATTR;  
}

t_Rectangle::t_Rectangle(float cxi, float cyi, float czi,
			 float wi, float di, float hi, float orii,
			 const t_Color_3f &rgb)
{
  initialize(cxi, cyi, czi, wi, di, hi, orii, rgb);
  initialize_typeinfo_array();
}

Boolean
t_Rectangle::inside(const t_Vector3f &point)
{
  int i;
  t_Vector3f p = getPosition();
  absolute_points();
  for(i = 0; i < 6; i++) {
    if(!sameHalfspace(normals[i], faces[i][0], point, p))
      return FALSE;
  }
  // cout << "point is inside" << endl;
  return TRUE;
}

bool
t_Rectangle::distance(const t_Vector3f &origin, 
		      const t_Vector3f &direction,
		      float maxLength,
		      float &dist, 
		      float &angle,
		      int *face_id)
{
  t_Vector3f p;
  float face_angle;
  float length; 

  absolute_points();
  
  dist = maxLength;

  for(int i = 0; i < 6; i++) {

    face_angle = scalarProd(normals[i].Vector3f(), direction);
    if( face_angle < 0) {
      length = linePlaneDistance(origin, direction,
				 faces[i][0], normals[i].Vector3f());
      if(length >= 0) {
	p = origin + length * direction;
	if(inside(p)) {
	  if(dist > length) {
	    dist = length;
	    angle = face_angle;
	    if(face_id != NULL) *face_id = i;
	  }
	}
      }
    }
  }
  if(dist < maxLength) {
    angle = acos(angle);
    return TRUE;
  }
  else return FALSE;
}

float
t_Rectangle::min_distance(float xi, float yi)
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

t_AttribArray
t_Rectangle::netAttrs()
{
  return (t_AttribArray) type_info;
}
