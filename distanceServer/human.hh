// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/human.hh,v $
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
//  $Log: human.hh,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.3  1999/11/18 12:49:01  schulz
//  Fixed framebuffer, added stuff for humans
//
//  Revision 1.2  1998/10/12 13:31:51  schulz
//  This is a complete new version
//
//  Revision 1.1  1998/06/19 13:57:44  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.6  1997/07/20 10:11:52  schulz
//  Many changes sice last update:
//  -- RTLServer/-Client and -Simulator databases are synchronized
//     over IP/Multicasting now.
//  -- multiple robots are allowed in one scene
//  -- robots are treated like ordinary active obstacles
//  -- added simple 3D visualization for robots
//  -- added a classes netlink and typeInfo, which implement
//  -- a fairly convenient way of transfering object states across the network
//
//  Revision 1.5  1997/06/03 12:15:34  schulz
//  to many changes to mention
//
//  Revision 1.4  1997/05/19 21:54:07  schulz
//  declared the base obstacle classes to virtual base classes in
//  the X obstacle class definitions
//
//  Revision 1.3  1997/05/16 14:57:55  schulz
//  source restructioring in progress
//  things won't compile for now
//
//  Revision 1.2  1997/03/04 17:28:26  schulz
//  Added file header to *.hh files
//  Removed some old "Zombie" files
//
//
//
// ----------------------------------------------------------------------------

#include "obstacles.hh"
#include "circle.hh"
#include "surface.hh"

#define HUMAN_RADIUS 20

class t_Human : virtual public t_Circle {
public:
  t_Human(float x, float y, float z, 
	  float r, float h, float speed, float angle);
  virtual void update();
  virtual void save(FILE*);
  virtual ~t_Human() {};
private:
  struct timeval last_update;
  int direction;
    int stopped;
  float speed;
  float angle;
  float start_pos_x;
  float start_pos_y;  
};
