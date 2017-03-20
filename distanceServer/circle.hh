// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/circle.hh,v $
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
//  $Log: circle.hh,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.4  1999/05/20 13:33:54  schulz
//  Changed grid implementation and renamed a method in t_Camera.
//
//  Revision 1.3  1998/10/12 13:31:42  schulz
//  This is a complete new version
//
//  Revision 1.1  1998/06/19 13:57:37  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.10  1997/11/15 16:15:39  schulz
//  latest version with collision avoidance support (to disable start RTLClient with -tcx). There are still some race conditions on startup!
//
//  Revision 1.9  1997/10/02 10:52:13  schulz
//  minor changes mostly to get doormatch running
//
//  Revision 1.8  1997/09/08 14:00:19  schulz
//  intermediate update, wont run
//
//  Revision 1.7  1997/07/24 15:29:03  schulz
//  robots are derived from obstaclesets instead of circles now.
//  So you can give them any shape.
//
//  fixed some bugs
//
//  Revision 1.6  1997/06/20 09:16:23  schulz
//  added basic support for obstacle communication
//
//  Revision 1.5  1997/06/07 14:19:07  schulz
//  new obstacle database
//
//  Revision 1.4  1997/06/03 12:15:31  schulz
//  to many changes to mention
//
//  Revision 1.3  1997/05/16 14:57:51  schulz
//  source restructioring in progress
//  things won't compile for now
//
//  Revision 1.2  1997/03/04 17:28:21  schulz
//  Added file header to *.hh files
//  Removed some old "Zombie" files
//
//
//
// ----------------------------------------------------------------------------

#ifndef CIRCLE_HH
#define CIRCLE_HH

#include "obstacles.hh"
#include "surface.hh"

class t_Circle :
  public t_Obstacle
{
public:
  t_Circle() {};
  t_Circle(float x, float y, float z, float r, float h,
	   const t_Color_3f &rgb = t_Color_3f(1.0, 1.0, 1.0));

  // explanations see obstacles.hh
  virtual bool distance(const t_Vector3f &origin, 
			const t_Vector3f &direction,
			float maxLength,
			float &dist, 
			float &angle,
			int *face_id);
  virtual float min_distance(float x1, float y1);
  virtual Boolean bounds(float *x1,float *y1,float *x2,float *y2);
  virtual Boolean inside(const t_Vector3f &point);
  virtual void save(FILE *MapFile);
  virtual ~t_Circle() {};
protected:
  // radius, height
  float r,h;     
};

#endif
