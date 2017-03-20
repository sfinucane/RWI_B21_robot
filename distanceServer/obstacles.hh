// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/obstacles.hh,v $
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
//  $Log: obstacles.hh,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.9  2000/03/09 08:47:45  schulz
//  some minor changes
//
//  Revision 1.8  2000/02/25 14:32:04  schulz
//  dont know
//
//  Revision 1.7  1999/11/18 12:49:01  schulz
//  Fixed framebuffer, added stuff for humans
//
//  Revision 1.6  1999/10/28 09:19:28  schulz
//  -- first running version of framebuffer
//  -- bugfixes in linalg stuff
//  -- bugfixes in door.*
//
//  Revision 1.5  1999/09/29 13:02:19  schulz
//  Several changes, most important, added a scan line algorithm for
//  camera simulation
//
//  Revision 1.4  1999/05/20 13:33:57  schulz
//  Changed grid implementation and renamed a method in t_Camera.
//
//  Revision 1.3  1998/10/12 13:32:00  schulz
//  This is a complete new version
//
//  Revision 1.3  1998/10/12 08:32:33  schulz
//  several changes
//
//  Revision 1.2  1998/09/30 15:07:16  schulz
//  Several changes to make things compile under gcc 2.7.2
//
//  Revision 1.1  1998/06/19 13:57:53  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//
// ----------------------------------------------------------------------------

#ifndef OBSTACLES_H
#define OBSTACLES_H

//#include <X11/Intrinsic.h>
typedef char Boolean;
#define TRUE 1
#define FALSE 0

#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#ifdef USE_PTHREADS
#include <pthread.h>
#include "pthreads_schedule.hh"
#else
#include "schedule.hh"
#endif

#ifdef __GNUG__
#include <typeinfo>
#endif
#include <stack>
#include <vector>


#include "polygons.hh"
#include "interface.hh"
#include "vector.hh"
#include "surface.hh"
#include "netlink.hh"


#ifndef bool
#define bool char
#endif


//defines to be moved elsewhere (TODO)
#define WALLCOLOR "grey50"
#define DEF_H 300.0
#define DEF_Z (DEF_H/2)

// t_Color3f definition to be moved elsewhere (TODO)
class t_Color_3f {
public:
  float r;
  float g;
  float b;
  t_Color_3f(float ri, float gi, float bi) { r = ri; g = gi; b = bi; };
  t_Color_3f() { r = 1.0; g = 1.0; b = 1.0; };
};

class t_Position_4f {
public:
  float x,y,z, ori;

  t_Position_4f() {x=0; y=0; z=0; ori=0;};
};

// Obstacle record type used by mapreader, to fill in an obstacle

enum obstacleType {UNKNOWN = 0, CUBE, CYLINDER, DOOR, HUMAN, CAMERA, BXXROBOT, COMPOUND, COMPOUND_END} ;

class t_ObstacleRecord {
public:
  t_ObstacleRecord();
  obstacleType type;
  char* label; 
  struct {
    float cx;
    float cy;
    float cz; 	    	    
  } position;
  struct {
    float r;
    float w;
    float d;
    float h;	    
  } dimensions;
  float orientation;
  float yrotation;
  t_Color_3f color;
  bool color_defined;
  char *texture;
};


struct obstacleNames {
    const char *token;
    obstacleType id;
};



class t_Obstacle {
friend class t_ObstacleGrid;
public:

  t_Obstacle();
  // this constructor defines default values for 
  // active (FALSE) and enabled (TRUE)

  virtual ~t_Obstacle();

  u_long id() const;
  // unique obstacle id 
  char* label() const; 
  
  void enable() { enabled_flag = TRUE;};
  void disable() { enabled_flag = FALSE;};
  void toggle();
  Boolean enabled() const 
  { 
    return enabled_flag;
  };
  // methods for changing and testing the visability of the obstacle
  Boolean modified() const 
  { 
    return modified_flag;
  };

  t_PolygonList* polygonList() const {
    return polygons;
  };

  void touch();
  // mark obstacle as modified

  virtual bool distance(const t_Vector3f &origin, 
			const t_Vector3f &direction,
			float maxLength,
			float &dist, 
			float &angle,
			int *face_id = NULL)                            = 0;

  virtual float min_distance(float x1, float y1)                        = 0;
  // returns the minimal distance of the obstacle to the point (x1, y1)

  virtual Boolean inside(const t_Vector3f &point)                       = 0;
  
  virtual Boolean bounds(float* x1,float* y1,float* x2,float* y2)       = 0;
  // returns the current bounding box of the obstacle
  // some objects do not have dimensions, for these FALSE is returned

  virtual void save(FILE *MapFile)                                      = 0;
  // write the obstacle record to file MapFile

  virtual void update();
  // update the obstacle record
  // Active Obstacles only
  Boolean active() const
  {
    return active_;
  };

  void move(const t_Vector3f& p);
  virtual void rotate(float ori, float yrot = 0);
  // manipulate position and orientation


  t_Interface* interface() const 
  {
    return interface_ptr;
  };

  void assign_interface(t_Interface* iptr) 
  {
    interface_ptr = iptr;
  };
  // Interface stuff will be replaced by generic C++ 
  // multiple inheritance + templates

  void clear_offsets();

  // next few methods mainly used for synchronisation via network

  
  virtual u_short netRead();
  virtual void netWrite();
  static void assignNetLink(t_NetLink*);
  void changeAttribute(char *, int, char *);
  void getAttribute(char *, int*, char **) const;
  void changeAttribute(char *, void* );
  virtual void synchronise();
  // Re-initialise from Network
  
  t_Vector3f getPosition() const;
  t_Vector3f getOrientation() const;

  // return the absolute position/ori within the worlds coordinate frame
  // note these methods only return correct values, if the object has
  // been properly updated before (at least for dynamic objects)
  t_Vector3f getAbsPosition() const;
  t_Vector3f getAbsOrientation() const;

  const t_surface* getSurface() const;
  const char* getColor() const;
  const t_Color_3f& getRGB() const;
  void assign_label(const char* l);
  int already_checked();
  void mark_checked();
  static void new_stamp();
  void activate();
protected:
  t_TypedPtr* typedAttribute(char *name) const;
  virtual t_AttribArray netAttrs() const;

  void assign_color(const char* c);
  void assign_rgb(const t_Color_3f &rgb);
  void assign_surface(const t_surface *surface);
  const t_surface* surface() const;


  inline void push_offset() {
    off_trans[myThread()].push(getPosition());
    off_ori[myThread()].push(getOrientation());
  }
  inline void pop_offset() {
    off_trans[myThread()].pop();
    off_ori[myThread()].pop();
  }

  inline float offset_x() const 
  { 
    return off_trans[myThread()].top().x();
  };
  inline float offset_y() const 
  { 
    return off_trans[myThread()].top().y();
  };
  inline float offset_z() const 
  { 
    return off_trans[myThread()].top().z();
  };
  inline float offset_ori() const 
  { 
    return off_ori[myThread()].top().z();
  };
  inline float offset_yrot() const 
  { 
    return off_ori[myThread()].top().y();
  };

  float x1,y1,x2,y2;      // static bounding box ensured to be correct 
  Boolean modified_flag;  // has this record been changed, since the last 
                          // computation of cached values
  static u_long number_of_obstacles;        

  static u_short hierarchy;
  
  // Material of this obstacle
  obstacleType type;
  const t_surface *my_surface;
  t_Interface* interface_ptr;
  static t_NetLink *my_NetLink;
  u_short netId;

  t_Vector3f center;      // center of the obstacle e.g. the obstacles position
  t_Vector3f orientation; // rotation in radians around the three axis

  t_Vector3f abs_center;      // center of the obstacle e.g. the obstacles position
  t_Vector3f abs_orientation; // rotation in radians around the three axis

  t_PolygonList* polygons;
  
private:
 
  u_long number;          // The unique number of this obstacle
  char* label_;

  Boolean active_;        // does this obstacle require regular state updates?   

  t_Color_3f rgb_color;   // RGB color used for 3D visualization
  const char* color;      // symbolic color name used in  2D visualization
  


  Boolean enabled_flag;   // is this obstacle currently visable?

  static u_long current_stamp;
  // every obstacle that has been traversed by some test
  // will be stamped with this value.
  
  u_long access_stamp;    // stamp for this obstacle

  static stack<t_Vector3f, vector<t_Vector3f> > off_trans[MAX_NUMBER_OF_THREADS];
  static stack<t_Vector3f, vector<t_Vector3f> > off_ori[MAX_NUMBER_OF_THREADS];

};

#define TRANSFORM_MATRIX_FIELDS \
{t_Attrib::TI_FLOAT, "mat00", &mat[0][0]},  \
{t_Attrib::TI_FLOAT, "mat01", &mat[0][1]},  \
{t_Attrib::TI_FLOAT, "mat02", &mat[0][2]},  \
{t_Attrib::TI_FLOAT, "mat10", &mat[1][0]},  \
{t_Attrib::TI_FLOAT, "mat11", &mat[1][1]},  \
{t_Attrib::TI_FLOAT, "mat12", &mat[1][2]},  \
{t_Attrib::TI_FLOAT, "mat20", &mat[2][0]},  \
{t_Attrib::TI_FLOAT, "mat21", &mat[2][1]},  \
{t_Attrib::TI_FLOAT, "mat22", &mat[2][2]},  \
{t_Attrib::TI_FLOAT, "mat30", &mat[3][0]},  \
{t_Attrib::TI_FLOAT, "mat31", &mat[3][1]},  \
{t_Attrib::TI_FLOAT, "mat32", &mat[3][2]}

#define INITIALIZE_COMMON_FIELDS \
type_info[0] = t_TypedPtr(t_Attrib::TI_SHORT, "hierarchy", &hierarchy); \
type_info[1] = t_TypedPtr(t_Attrib::TI_CHAR, "type", &type);\
type_info[2] = t_TypedPtr(t_Attrib::TI_SHORT, "netId", &netId);\
type_info[3] = t_TypedPtr(t_Attrib::TI_FLOAT, "cx", &center.p[0]);\
type_info[4] = t_TypedPtr(t_Attrib::TI_FLOAT, "cz", &center.p[2]);\
type_info[5] = t_TypedPtr(t_Attrib::TI_FLOAT, "cy", &center.p[1]);\
type_info[6] = t_TypedPtr(t_Attrib::TI_FLOAT, "rot", &orientation.p[2]);
#define NUMBER_OF_COMMON_FIELDS 7

#define DECLARE_TYPEINFO_ARRAY(Number_of_fields) \
t_TypedPtr type_info[NUMBER_OF_COMMON_FIELDS + Number_of_fields + 1]


#endif
