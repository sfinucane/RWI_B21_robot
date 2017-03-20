// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/rectangle.hh,v $
//
//  Created by:      $Author: rstone $
//
//  Revision:        $Revision: 1.1 $
//
//  Date of revision $Date: 2002/09/14 16:52:31 $
//
// ============================================================================
//
//                 ------------- HISTORY -----------------
//
//  $Log: rectangle.hh,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.4  1999/05/20 13:33:59  schulz
//  Changed grid implementation and renamed a method in t_Camera.
//
//  Revision 1.3  1998/10/12 13:32:04  schulz
//  This is a complete new version
//
//  Revision 1.2  1998/08/04 15:59:45  schulz
//  Several bug fixes
//
//  Revision 1.1  1998/06/19 13:57:57  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.16  1997/11/15 16:15:46  schulz
//  latest version with collision avoidance support (to disable start RTLClient with -tcx). There are still some race conditions on startup!
//
//  Revision 1.15  1997/10/02 10:52:18  schulz
//  minor changes mostly to get doormatch running
//
//  Revision 1.14  1997/09/25 12:16:49  schulz
//  made constructor sequence for rectangles explicit (initialize method)
//
//  Revision 1.13  1997/09/18 08:16:21  schulz
//  Many changes, making RTL compatible with MRT-VR to some extend
//
//  Revision 1.12  1997/09/08 14:00:28  schulz
//  intermediate update, wont run
//
//  Revision 1.11  1997/07/20 10:11:59  schulz
//  Many changes sice last update:
//  -- RTLServer/-Client and -Simulator databases are synchronized
//     over IP/Multicasting now.
//  -- multiple robots are allowed in one scene
//  -- robots are treated like ordinary active obstacles
//  -- added simple 3D visualization for robots
//  -- added a classes netlink and typeInfo, which implement
//  -- a fairly convenient way of transfering object states across the network
//
//  Revision 1.10  1997/07/09 15:26:31  schulz
//  first attempt to display more then one root in the RTLClient
//
//  Revision 1.9  1997/07/08 15:16:21  schulz
//  Setting up a RTP capable simulator, with no X
//
//  Revision 1.8  1997/06/24 11:57:00  schulz
//  *** empty log message ***
//
//  Revision 1.7  1997/06/20 09:16:28  schulz
//  added basic support for obstacle communication
//
//  Revision 1.6  1997/06/07 14:19:16  schulz
//  new obstacle database
//
//  Revision 1.5  1997/06/03 12:15:38  schulz
//  to many changes to mention
//
//  Revision 1.4  1997/05/16 14:57:59  schulz
//  source restructioring in progress
//  things won't compile for now
//
//  Revision 1.3  1997/03/23 10:28:47  schulz
//  changes for 970318 demo
//
//  Revision 1.2  1997/03/04 17:28:39  schulz
//  Added file header to *.hh files
//  Removed some old "Zombie" files
//
//
//
// ----------------------------------------------------------------------------

#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "obstacles.hh"
#include "surface.hh"

class t_Face
{
public:
  float width;
  float height;
  int face_id;
  t_Vector3f translation;
  t_Vector3f position(float dx, float dy) const {
    switch(face_id) {
    case 0:
      return translation + t_Vector3f(0, dx, dy);
    case 1:
      return translation + t_Vector3f(dx, 0, dy);
    case 2:
      return translation + t_Vector3f(0, dx, dy);
    case 3:
      return translation + t_Vector3f(-dx, 0, dy);
    case 4:
      return translation + t_Vector3f(dx, -dy, 0);
    case 5:
      return translation +  t_Vector3f(dx, dy, 0);
    }
    return t_Vector3f(0,0,0);
  };
};

class t_Rectangle :
  public t_Obstacle
{
public:
  t_Rectangle() {initialize_typeinfo_array();};
  t_Rectangle(float x, float y, float z,
	      float w, float d, float h, float ori = 0,
	      const t_Color_3f &rgb = t_Color_3f(1.0, 1.0, 1.0));
  void initialize(float x, float y, float z,
			  float w, float d, float h, float ori = 0,
			  const t_Color_3f &rgb = t_Color_3f(1.0, 1.0, 1.0));
  // Explanations see obstacle.hh

  virtual bool distance(const t_Vector3f &origin, 
			const t_Vector3f &ori,
			float maxLength,
			float &dist, 
			float &angle,
			int *face_id = NULL);
  virtual float min_distance(float x1, float y1);
  virtual Boolean bounds(float *x1,float *y1, float *x2,float *y2);
  virtual Boolean inside(const t_Vector3f &point);
  virtual void save(FILE *MapFile);
  virtual Boolean face(int id, t_Face* returned_face);
  virtual ~t_Rectangle() {};
protected:


  // Same after last bounding box calculation
  float last_ori;
  float abs_ori;

  // Width, Depth, height
  float w,d,h;
  t_Dimensions dimensions;

  // Points of Bounding box
  float x[4],y[4];    

  // vertex and normal cache for speeding up 3D calculations

  t_Vector3f vertices[8];
  t_Vector4f normals[6];
  t_Vector3f faces[6][4];
  

  // method for calculating the bounding box
  virtual void absolute_points();    

  DECLARE_TYPEINFO_ARRAY(4);
  void initialize_typeinfo_array();

  // fields to be transfered over the net

  float halfx, halfy, halfz;
  float mat[4][3];
  u_short surfaceId;
  char *nametag;
  t_AttribArray netAttrs();
};

#define RECTANGLE_NET_FIELDS \
{t_Attrib::TI_FLOAT, "cx", &cx}, \
{t_Attrib::TI_FLOAT, "cz", &cz}, \
{t_Attrib::TI_FLOAT, "cy", &cy}, \
{t_Attrib::TI_FLOAT, "halfx", &halfx},  \
{t_Attrib::TI_FLOAT, "halfz", &halfz},  \
{t_Attrib::TI_FLOAT, "halfy", &halfy},  \
TRANSFORM_MATRIX_FIELDS,   \
{t_Attrib::TI_SHORT, "surfaceId", &surfaceId}, \
{t_Attrib::TI_STRING, "nametag", &nametag}


#endif

