// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/camera.cc,v $
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
//  $Log: camera.cc,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.5  1999/11/18 12:49:00  schulz
//  Fixed framebuffer, added stuff for humans
//
//  Revision 1.4  1999/10/28 09:19:24  schulz
//  -- first running version of framebuffer
//  -- bugfixes in linalg stuff
//  -- bugfixes in door.*
//
//  Revision 1.3  1999/09/29 13:02:17  schulz
//  Several changes, most important, added a scan line algorithm for
//  camera simulation
//
//  Revision 1.2  1999/05/20 13:33:53  schulz
//  Changed grid implementation and renamed a method in t_Camera.
//
//  Revision 1.1  1998/10/12 13:31:39  schulz
//  This is a complete new version
//
//  Revision 1.5  1998/10/12 08:32:29  schulz
//  several changes
//
//  Revision 1.4  1998/09/30 15:07:11  schulz
//  Several changes to make things compile under gcc 2.7.2
//
//  Revision 1.3  1998/08/24 03:58:21  schulz
//  Fix for MAXFLOAT bug on ragweed
//
//  Revision 1.2  1998/08/04 15:59:41  schulz
//  Several bug fixes
//
//  Revision 1.1  1998/06/19 13:57:35  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.9  1997/11/15 16:15:36  schulz
//  latest version with collision avoidance support (to disable start RTLClient with -tcx). There are still some race conditions on startup!
//
//  Revision 1.8  1997/11/10 10:23:38  schulz
//  mainly buf fixes
//
//  Revision 1.7  1997/09/29 16:02:45  schulz
//  Made source files SUN C++ compatible
//
//  Revision 1.6  1997/09/24 16:36:51  schulz
//  Workshop97 Demo Version
//
//  Revision 1.5  1997/09/18 09:35:47  schulz
//  Performance tuning
//
//  Revision 1.4  1997/09/18 08:16:05  schulz
//  Many changes, making RTL compatible with MRT-VR to some extend
//
//  Revision 1.3  1997/09/12 12:07:29  schulz
//  fixed several bugs
//
//  Revision 1.2  1997/09/09 12:57:27  schulz
//  added multiple camera support
//  hopefully fixed destroyed simrtp.hh and sim_rtp.cc again
//
//  Revision 1.1  1997/09/09 08:26:47  schulz
//  added new camera files to repository
//
//
// ----------------------------------------------------------------------------

#include <strings.h>
#define _XOPEN_SOURCE
#include <math.h>
#undef _XOPEN_SOURCE

#include "camera.hh"
#include "store.hh"
#include "netlink.hh"
#include "comdef.h"
#include "vector.hh"

#ifndef MAXFLOAT
#define   MAXFLOAT        ((float)3.40282346638528860e+38)
#endif


int server_only = 1;

t_Camera::t_Camera(float widthp, float heightp, float open_anglep,
		   float x, float y, float z, 
		   float orip, float yrotp, t_ObstacleGrid* scenep)
{

  scene = scenep;
  
  width = (int) widthp;
  height = (int) heightp;
  open_angle = open_anglep;
  aspect = ((float) width) / height;
  focus = 1.0/tan(open_angle/2); 

  upvector_x = 0.0;
  upvector_y = 0.0;
  upvector_z = 1.0;  

  move(t_Vector3f(x,y,z));
  rotate(orip, yrotp);
  update();


  fb = new t_FrameBuffer<t_Obstacle*>(getPosition(),
				      getOrientation(),
				      focus, width, height);  
  cerr << "created framebuffer" << " " << (void*) fb << " "  
       << getPosition() << " "
       << getOrientation() << " "
       << focus << " " << width << " " << height << endl;
  // activate();
  initialize_typeinfo_array();
}

void
t_Camera::initialize_typeinfo_array()
{
  //  INITIALIZE_COMMON_FIELDS;
  type_info[0] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "cam_x", &cam_x);
  type_info[1] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "cam_z", &cam_z);
  type_info[2] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "cam_y", &cam_y);
  type_info[3] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "lookat_x", &lookat_x);
  type_info[4] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "lookat_z", &lookat_z);
  type_info[5] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "lookat_y", &lookat_y);
  type_info[6] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "upvector_x", &upvector_x);
  type_info[7] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "upvector_z", &upvector_z);
  type_info[8] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "upvector_y", &upvector_y);
  type_info[9] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "hfov", &hfov);
  type_info[10] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "vfov", &vfov);
  type_info[11] =
    t_TypedPtr(t_Attrib::TI_FLOAT, "netAngle", &netAngle);
  type_info[12] = TI_END_ATTR;

}

static float
Deg2Rad(float angle)
{
  return angle * M_PI / 180;
}

void
t_Camera::update()
{
  float t;
  float yr, zr;
  float X,Y,Z;
  struct timeval now;

  t_Obstacle::update();
  gettimeofday(&now, NULL);

  //if(!modified()) return;

  last_time = now.tv_sec;
  
  pos = getPosition();
  pan_tilt_rot = getOrientation();

  /*
  pos = t_Vector3f(1020,1700,138);
  pan_tilt_rot = t_Vector3f(0.0, Deg2Rad(-45), Deg2Rad(5));
  */

  focus = 1.0/(tan(open_angle/2)); 
  ccd_normal = t_Vector3f::unitX.rotate(pan_tilt_rot);
  ccd_root = pos - focus * ccd_normal;
  corr_root = pos + ccd_normal;

  X = pos.x();
  Y = pos.y();
  Z = pos.z();
  yr = pan_tilt_rot.y();
  zr = pan_tilt_rot.z();
  X /= 100;
  Y *= -1.0;
  Y /= 100;
  Z /= 100;
  
  lookat_x = 1;
  lookat_y = 0;
  lookat_z = 0;

  t = lookat_x;
  lookat_x = t * cos(yr) - lookat_z * sin(yr);
  lookat_z = t * sin(yr) + lookat_z * cos(yr);
  t = lookat_x;
  lookat_x = t * cos(zr) - lookat_y * sin(zr);
  lookat_y = t * sin(zr) + lookat_y * cos(zr);

  lookat_x += X;
  lookat_y = Y - lookat_y;
  lookat_z += Z;
  
  cam_x = X;
  cam_y = Y;
  cam_z = Z;
  cam_zrot = zr;

  //netWrite();
}

void
t_Camera::save(FILE *mapfd)
{
  t_Vector3f p = getPosition();
  t_Vector3f o = getOrientation();
  fprintf(mapfd, "CAMERA %s (%g %g %g) (%g %g %g) %g %g\n",
	  label(),
	  p.x(), p.y(), p.z(),
	  width,height,open_angle,
	  o.z(), o.y());
}

float
t_Camera::min_distance(float x, float y)
{
  return MAXFLOAT;
}


Boolean
t_Camera::inside(const t_Vector3f &point)
{
  return FALSE;
}

char
t_Camera::bounds(float *xmin, float *ymin, float *xmax, float *ymax)
{
  return FALSE;
}

bool
t_Camera::distance(const t_Vector3f &origin, 
		     const t_Vector3f &_ori,
		     float maxLength,
		     float &dist, 
		     float &angle,
		   int* face_id)
{
  return FALSE;
}

t_Vector2f 
t_Camera::getImageDimensions() const
{
  return t_Vector2f(width,height);
}
// ---------------------------------------------------------------------------


// Constants for compensation of lens distortions

#define C2 0.0003938656 
#define C1 0.02554573

inline
float
t_Camera::lens_forward_correction(const t_Vector3f &_pos,
			  const t_Vector3f &dir,
			  float pxySqr)
{
  t_Vector3f projPixel;
  float dist;

  return 1.0;

  if( ! linePlaneIntersection(_pos, dir,
			      corr_root, corr_normal, projPixel)) {
    return -1;
  }
  dist = (projPixel - corr_root).length();
  return sqrt(C1 * pxySqr + C2 * pxySqr * pxySqr) / dist;
}

inline
float
t_Camera::lens_backward_correction(const t_Vector3f &_pos,
				   const t_Vector3f &dir,
				   float pxySqr)
{
  t_Vector3f projPixel;
  float dist;

  return 1.0;

  if( ! linePlaneIntersection(_pos, dir,
			      corr_root, corr_normal, projPixel)) {
    return -1;
  }
  dist = (projPixel - corr_root).length();
  float p_2 = C1/(0.5*C2);
  return -p_2 + sqrt(p_2*p_2 + dist) / pxySqr;
}


t_Obstacle*
t_Camera::obstacleAtPixel(float px, float py, float *dist)
{
  
  float dx = 0.5;    // half a pixel
  float dy = 0.5;
  px /= 0.5*width;       // normalize to -1 .. 1
  py /= 0.5*width;       // assumes width always to be bigger then height
  px -= 1.0;
  py -= 1.0;
  px *= -1.0;       
  py *= -1.0;

  t_Obstacle* obst;
  t_Vector3f pt;
  t_Vector3f dir = t_Vector3f(focus, px, py).rotate(pan_tilt_rot);
  dir = dir / dir.length();

  if(scene->rayObstacleIntersection(pos, dir, 10000.0, pt, &obst)) {
    *dist = ((pt-pos).irotate(pan_tilt_rot)).x();
    return obst;
  }
  return NULL;
}

bool
t_Camera::projectPixel(float px, float py,
		       const t_Vector3f &planePt,
		       const t_Vector3f &planeNormal, t_Vector3f *quad)
{
  
  float dx = 1.0 / width;    // half a pixel
  float dy = 1.0 / width;
  px /= 0.5*width;              // normalize to -1 .. 1
  py /= 0.5*width;
  px -= 1.0;
  py -= 1.0;
  px *= -1.0;       
  py *= -1.0;


  // squared distance from center for lens correction

  float pxySqr = (px*px+py*py) / 10000; 
  float corr;
  
  t_Vector3f dir;
  t_Vector3f tmpVec;
  t_Vector3f centerPixel;
  t_Vector3f rotations = pan_tilt_rot;

  if( ! linePlaneIntersection(pos, ccd_normal,
			      planePt, planeNormal, centerPixel)) {
    return FALSE;
  }
  
  // retrieve the world coordinate of each corner of the pixel

  dir = t_Vector3f(focus, px, py).rotate(rotations);
  dir = dir / dir.length();

  corr = lens_forward_correction(pos, dir, pxySqr);
  if(corr < 0) return FALSE;
  
  if( ! linePlaneIntersection(pos, dir, planePt, planeNormal, quad[0]) )
    return FALSE;
  tmpVec = (quad[0] - centerPixel);
  quad[0] = corr * tmpVec + centerPixel;

  // second corner
  
  dir = t_Vector3f(focus, px-dx, py+dy).rotate(rotations);
  dir = dir / dir.length();
  corr = lens_forward_correction(pos, dir, pxySqr);
  if(corr < 0) return FALSE;

  if( ! linePlaneIntersection(pos, dir, planePt, planeNormal, quad[1]) )
    return FALSE;
  tmpVec = (quad[1]-centerPixel);
  quad[1] = corr * tmpVec + centerPixel;

  // third corner
  
  dir = t_Vector3f(focus, px+dx, py+dy).rotate(rotations);
  dir = dir / dir.length();
  corr = lens_forward_correction(pos, dir, pxySqr);
  if(corr < 0) return FALSE;
  
  if( ! linePlaneIntersection(pos, dir, planePt, planeNormal, quad[2]) )
    return FALSE;
  tmpVec = (quad[2]-centerPixel);
  quad[2] = corr * tmpVec + centerPixel;

  // forth corner
  
  dir = t_Vector3f(focus, px+dx, py-dy).rotate(rotations);
  dir = dir / dir.length();
  corr = lens_forward_correction(pos, dir, pxySqr);
  //if(corr < 0) return FALSE;

  if( ! linePlaneIntersection(pos, dir, planePt, planeNormal, quad[3]) )
    return FALSE;
  tmpVec = (quad[3]-centerPixel);
  quad[3] = corr * tmpVec + centerPixel;
  
  return TRUE;
}

void
t_Camera::renderPolygons(const t_PolygonList& polygons, t_Obstacle* obst) const
{
  cerr << "render fb: " << (void*) fb << endl;
  for(int i = 0; i < polygons.size(); i++) {
    t_Polygon& poly = polygons[i];
    if( 1 || scalarProd(poly.normal->Vector3f(), ccd_normal) <= 0.0 ) {
            fb->renderPolygon(poly, obst);
    }
  }
}

t_AttribArray
t_Camera::netAttrs()
{
  netAngle = cam_zrot;
  hfov = vfov = (open_angle / 2) * (180 / M_PI);
  //  fprintf(stderr, "cam_x:... %f\n", cam_x);
  //  fprintf(stderr, "cam_y:... %f\n", cam_y);
  //  fprintf(stderr, "cam_z:... %f\n", cam_z);
  //  fprintf(stderr, "lookat_x: %f\n", lookat_x);
  //  fprintf(stderr, "lookat_y: %f\n", lookat_y);
  //  fprintf(stderr, "lookat_z: %f\n", lookat_z);
  //  fprintf(stderr, "hfov:.... %f\n", hfov);
  //  fprintf(stderr, "vfov:.... %f\n", vfov);
  
  return (t_AttribArray) type_info;
}

void
t_Camera::netWrite()
{
  return;
  /*
  if(server_only) {
    char *lenptr = my_NetLink->writePacketHeader(COMM_MODIFY, COMM_CAMERA);
    u_short len = my_NetLink->write(netAttrs());
    bcopy(&len, lenptr, sizeof(u_short));
    my_NetLink->netWrite();
  }
  */
}
