// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/door.hh,v $
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
//  $Log: door.hh,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.3  1999/10/28 09:19:26  schulz
//  -- first running version of framebuffer
//  -- bugfixes in linalg stuff
//  -- bugfixes in door.*
//
//  Revision 1.2  1999/05/20 13:33:55  schulz
//  Changed grid implementation and renamed a method in t_Camera.
//
//  Revision 1.1  1998/10/12 13:31:46  schulz
//  This is a complete new version
//
//  Revision 1.1  1998/06/19 13:57:41  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.7  1997/10/21 12:51:41  schulz
//  still cleaningup for first release
//
//  Revision 1.6  1997/09/29 16:02:52  schulz
//  Made source files SUN C++ compatible
//
//  Revision 1.5  1997/09/25 12:16:47  schulz
//  made constructor sequence for rectangles explicit (initialize method)
//
//  Revision 1.4  1997/09/18 08:16:14  schulz
//  Many changes, making RTL compatible with MRT-VR to some extend
//
//  Revision 1.3  1997/06/03 12:15:34  schulz
//  to many changes to mention
//
//  Revision 1.2  1997/05/19 21:54:06  schulz
//  declared the base obstacle classes to virtual base classes in
//  the X obstacle class definitions
//
//  Revision 1.1  1997/05/16 14:57:53  schulz
//  source restructioring in progress
//  things won't compile for now
//
//  Revision 1.2  1997/03/04 17:28:24  schulz
//  Added file header to *.hh files
//  Removed some old "Zombie" files
//
//
//
// ----------------------------------------------------------------------------

#include "rectangle.hh"

class t_Door :
  public t_Rectangle
{
public:
  t_Door() {  initialize_typeinfo_array();};
  t_Door(float w, float d, float h,
	 float hinge_x, float hinge_y, float hinge_z,
	 float Angle, const t_Color_3f& rgb = t_Color_3f(1.0, 1.0, 1.0));
  void initialize(float w, float d, float h,
		  float hinge_x, float hinge_y, float hinge_z,
		  float Angle, const t_Color_3f& rgb = t_Color_3f(1.0, 1.0, 1.0));
  virtual void save(FILE*);
  virtual ~t_Door() {};
  t_Vector3f getHingePosition() const;
  void rotate(float ori, float yrot = 0);
protected:
  // hinge position 
  float apx,apy,apz;
  
  // Angle the door is opened (0 = closed) radians counter clockwise
  float angle;
  
  // same after the last bounding box calculation
  float last_angle;
  
  // bounding box calculation
  // differs from rectangles because of diffrent reference point (apx,apy,apz)
  void absolute_points();
  DECLARE_TYPEINFO_ARRAY(7);
  void initialize_typeinfo_array();
  t_AttribArray netAttrs();  
};

#define DOOR_NET_FIELDS \
{t_Attrib::TI_FLOAT, "apx", &apx}, \
{t_Attrib::TI_FLOAT, "apz", &apz}, \
{t_Attrib::TI_FLOAT, "apy", &apy}, \
{t_Attrib::TI_FLOAT, "width", &w},   \
{t_Attrib::TI_FLOAT, "height", &h},   \
{t_Attrib::TI_FLOAT, "depth", &d},   \
{t_Attrib::TI_FLOAT, "angle", &angle}
