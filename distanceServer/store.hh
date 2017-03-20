/*
// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/store.hh,v $
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
//  $Log: store.hh,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.6  2000/02/25 14:32:04  schulz
//  dont know
//
//  Revision 1.5  1999/10/28 09:19:30  schulz
//  -- first running version of framebuffer
//  -- bugfixes in linalg stuff
//  -- bugfixes in door.*
//
//  Revision 1.4  1999/09/29 13:02:22  schulz
//  Several changes, most important, added a scan line algorithm for
//  camera simulation
//
//  Revision 1.3  1999/05/25 14:43:54  schulz
//  Moved pthreads and locking stuff from RTLClientServer into libstore.a
//  Added C-functions for locking the database from within C-stuff.
//
//  Revision 1.2  1999/05/20 13:34:01  schulz
//  Changed grid implementation and renamed a method in t_Camera.
//
//  Revision 1.1  1998/10/12 13:32:13  schulz
//  This is a complete new version
//
//  Revision 1.4  1998/10/12 08:32:35  schulz
//  several changes
//
//  Revision 1.3  1998/10/05 09:51:49  schulz
//  Added option to include the simulator into the RTLClient
//
//  Revision 1.2  1998/08/04 15:59:47  schulz
//  Several bug fixes
//
//  Revision 1.1  1998/06/19 13:58:05  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//
// ----------------------------------------------------------------------------
*/

#ifndef _STORE_HH
#define _STORE_HH


#ifdef USE_PTHREADS
#include <pthread.h>
#include "locking.hh"
#endif


#ifdef __cplusplus

#include <stdio.h>
#ifdef __GNUG__
#include <typeinfo>
#endif
#include <map>
#include <vector>
#include <list>
#include <stack>
#include <algorithm>
#include "obstacles.hh"
#include "camera.hh"
#include "surface.hh"
#include "mapreader.hh"
#include "netlink.hh"

#define WS_SPAREROOM 200        /* room for N additional obstacles */


typedef list<t_Obstacle*> t_ObstacleList;

struct charp_less {
  bool operator()(const char* p, const char* q) const 
    {
      return strcmp(p,q) < 0;
    }
};

typedef vector<t_Obstacle*> t_ObstacleCache;

#endif

typedef struct {
  char* id;
  int ival;
  char *cval;
} attr_spec;

#ifdef __cplusplus



// =========================================================================
// class t_ObstacleWorkingSet is completely inlined foro performance reasons
// =========================================================================

class t_ObstacleWorkingSet {
public:
  // Size specifies the maximal number of elements the set may store
  t_ObstacleWorkingSet(u_long Size);
  ~t_ObstacleWorkingSet();

  // returns true if Amount < Size 
  bool check_room(u_long Amount);

  void add(t_Obstacle *ElementObstacle);

  // Empty the set
  void clear();

  // return count elements currently in the set
  u_long size() const;

  // return the Elements in the set one at a time
  t_Obstacle* traverse();
private:
  // the set is implemented using an array of pointers to all elements
  t_Obstacle** workingset;

  // we mark an obstacle as member in a boolean array at least as long as the
  // total number of obstacles 
  bool* members;

  // first free position in workingset
  int top;

  // current position for traverse()
  int travptr;

  // length of members
  u_long maxsize;
};

inline
bool
t_ObstacleWorkingSet::check_room(u_long n)
{
    return n < maxsize;
}

inline
t_ObstacleWorkingSet::~t_ObstacleWorkingSet()
{
    delete [] workingset;
    delete [] members;
}

inline
t_ObstacleWorkingSet::t_ObstacleWorkingSet(u_long n)
{
    maxsize = n+WS_SPAREROOM;
    workingset = new t_Obstacle* [maxsize];
    members = new bool[maxsize];
    top = 0;
    travptr = 0;
}

inline
void
t_ObstacleWorkingSet::add(t_Obstacle *obst)
{
  if(obst->id() >= maxsize) {
    fprintf(stderr, "t_ObstacleWorkingSet::add() : set overflow (%d) !\n",obst->id());
    exit(0);
  }
  if(!obst->enabled() || members[obst->id()]) return;
  workingset[top++] = obst;
  members[obst->id()] = TRUE;
}

inline
u_long
t_ObstacleWorkingSet::size() const
{
    return top;
}

inline
void
t_ObstacleWorkingSet::clear()
{
  u_long i;
  for(i = 0; i < (maxsize); i++) members[i] = FALSE;
  top = 0;
  travptr = 0;
}

inline
t_Obstacle*
t_ObstacleWorkingSet::traverse()
{
  if(travptr < top)
    return workingset[travptr++];
  else {
    travptr = 0;
    return NULL;
  }
}

// ---------------------------------------------------------------------------

#define OBSTACLE_INDEX_BUCKETS 1024 

class t_ObstacleIndex {
public:
  t_ObstacleIndex();
  ~t_ObstacleIndex();
  bool insert(t_Obstacle* new_obstacle);
  t_Obstacle* retrieve(char* label);
  void remove(t_Obstacle* old_obstacle);
protected:
  u_long hashValue(char* label);
  struct charp_less {
    bool operator()(const char* p, const char* q) const 
    {
      return strcmp(p,q) < 0;
    }
  };
  map<char*, t_Obstacle*, charp_less> buckets[OBSTACLE_INDEX_BUCKETS];
};

class t_ObstacleSet :
  public t_Obstacle
{
public:
  t_ObstacleSet() {  initialize_typeinfo_array();};
  virtual ~t_ObstacleSet();
  t_ObstacleSet(float, float, float, float, float, bool, const t_Color_3f& = t_Color_3f(1.0,1.0,1.0));
  void initialize(float, float, float, float, float, bool, const t_Color_3f& = t_Color_3f(1.0,1.0,1.0));
  virtual bool distance(const t_Vector3f &origin, 
			const t_Vector3f &direction,
			float maxLength,
			float &dist, 
			float &angle,
			int *face_id = NULL);
  virtual float min_distance(float, float);
  virtual Boolean bounds(float*,float*,float*,float*);
  virtual Boolean inside(const t_Vector3f &point);
  void insert(t_Obstacle* new_element);
  virtual void update();
  virtual void save(FILE*);
  vector<t_Obstacle*>& getElements();
  void closeSet();
protected:
  bool color_defined;
  vector<t_Obstacle*> elements;

  DECLARE_TYPEINFO_ARRAY(0);
  void initialize_typeinfo_array();
  t_AttribArray netAttrs();  
};



#ifdef USE_PTHREADS
extern t_RWlock store_lock;

#define SYNCHRONIZED() t_mutex_guard guard(store_lock)
#else
#define SYNCHRONIZED() 
#endif

class t_ObstacleGrid {
public:

  // (x1, y1) to (x2, y2) bounding box of total space
  t_ObstacleGrid(float x1, float y1, float x2,float y2, int border = 1);
  t_ObstacleGrid(int border);
  virtual ~t_ObstacleGrid();

  // fill the grid with obstacles read from simMap
  virtual int read_obstacles(t_mapReader &simMap);

  // fill the workingset with all obstacles lying within
  // (x-d,y-d) to (x+d), (y+d)
  void fill_ws(float x,float y,float d);

  // fill the workingset with all obstacles lying in
  // grid cells, which are crossed by the line (x1,y1),(x2,y2)
  void fill_ws_ray(float x1, float y1, float x2, float y2);

  // fill the workingset with all obstacles lying in
  // grid cells, which intersect with the triangle (x1, y1), (x2, y2), (x3, y3)
  void fill_ws_tri(float x1,float y1, float x2,float y2, float x3,float y3);

  // unlink the obstacle from any index structure of the ObstacleGrid 
  virtual void unlinkObstacle(t_Obstacle *remove);

  inline void removeFromGrid(t_Obstacle* obst) { remove_from_grid(obst);};
  inline void insertToGrid(t_Obstacle* obst) { insert_grid(obst);};

  // unlink and delete obstacle
  void RemoveObstacle(t_Obstacle *OldObstacle);

  // insert an obstacle into the grid and the list of all obstacles
  t_Obstacle *insert(t_Obstacle*);


  // mark all obstacles invisible
  void disableAll();
  // mark all obstacles visible
  void enableAll();

  t_Obstacle *getObstacle(char *label);
  t_Obstacle *getObstacle(u_long id);


  virtual void update_obstacle_attrs(char* name,
				     int nattrs, attr_spec* attrs);
  virtual void update_obstacle_attrs_by_pointer(t_Obstacle *obst, 
						int nattrs, attr_spec* attrs);
  virtual void update_obstacle(char *name,
			       char *attribute, int ival, char *cval);

  void Update();              // all active obstacles state
  void Update(t_Obstacle*);   // one active obstacles state
  void Synchronise(t_Obstacle*); // one obstacle with network  
  void SaveObstacles(FILE *MapFile);    

  // returns the pointer to the obstacle, point lies in. 
  // Recognizes only enabled obstacles!
  t_Obstacle* InsideObstacle(const t_Vector3f& point);

  // same as above but recoginzes all obstacles
  t_Obstacle* InsideAnyObstacle(const t_Vector3f& point);       

  // Look into obstacles.hh for explanation 

  virtual bool distance(const t_Vector3f &from,
			const t_Vector3f &direction,
			float maxLength,
			float& min_distance,
			float& hit_angle);  

  // fast_distance doesnot measure the distance to an obstacle but
  // the distance to the middle of first nonempty grid cell
  bool fast_distance(const t_Vector3f &origin, 
		     const t_Vector3f &direction,
		     float maxLength,
		     float *dist);

  // same as distance but respects sensor errors
  bool get_distance(const t_Vector3f &from,
		    const t_Vector3f &direction,
		    float maxLength,
		    float *Distance, float *HitAngle,
		    const t_surface **HitSurface);
  bool get_distance(const t_Vector3f& from,
		    const t_Vector3f& direction,
		    float maxLength,
		    float *Distance);
  bool rayObstacleIntersection(const t_Vector3f &from,
			       const t_Vector3f &direction,
			       float maxLength,
			       t_Vector3f &point_hit,
			       t_Obstacle **obstacle_hit);

  bool wsDistance(const t_Vector3f &from,
		  const t_Vector3f &direction,
		  float maxLength,
		  float& min_distance,
		  float& hit_angle);

  void renderCamera(const t_Camera& cam);
  // minimal distance of (x1, y1) to an obstacle
  float min_distance(float, float);
  void install_obstacle(t_ObstacleRecord *new_obst);
  void doUpdateFromNet();
  void assignNetLink(t_NetLink*);

  void mapBounds(float &minX, float &minY, float &maxX, float &maxY) {
    minX = lx;
    minY = ly;
    maxX = hx;
    maxY = hy;
  };

  // retrieve the obstacle list assigned to grid cell (i,j)
  vector<t_Obstacle*>& get(int i,int j) const;

  int align2GridX(float x) const;
  int align2GridY(float y) const;
  float midCellX(int index) const;
  float midCellY(int index) const;

  // number of rows and columns of the grid
  int cols,rows;
protected:

  // offset of the grid to (0,0)
  float xoff,yoff;

  // bounding box of the grid
  float lx,ly,hx,hy;


  vector<t_Obstacle*> all_obstacles;
  vector<t_Obstacle*> active_obstacles;
  vector<t_Obstacle*> all_cameras;
  stack<t_ObstacleSet*, vector<t_ObstacleSet*> > obstacle_set_stack;
  vector<t_Obstacle*>** grid;
  t_ObstacleWorkingSet* ws[MAX_NUMBER_OF_THREADS];
  t_ObstacleIndex obstacle_index;
  
  // assigns ListIJ to grid cell (i,j)
  void set(int i,int j,vector<t_Obstacle*>* ListIJ);


  void fill_ws_rayIntern(float,float,float,float);          

  void remove_from_grid(t_Obstacle*);
  void insert_grid(t_Obstacle*);
  void insert_lists(t_Obstacle*);  
  void remove_from_obstlist(vector<t_Obstacle*>**, vector<t_Obstacle*>**, t_Obstacle*);

  bool off_ground(float, float);
  virtual t_Obstacle* install(t_ObstacleRecord *new_obst);
  t_NetLink* my_NetLink;
  char* netObstacleName();
  int netTransactionType();

  void map_bounds(float &minx, float &miny, float &maxx, float &maxy);
  void setup_grid(float x1, float y1, float x2, float y2);
  int has_border;
  void init(int border);
  void check_read_ok();
};

#endif


#if !defined(__cplusplus) && !defined(BOOL_DEFINED)
typedef char bool;
#define BOOL_DEFINED 1
#endif

#ifdef __cplusplus
extern "C" {
#endif
    
float obstacles_min_distance(float x, float y);
bool get_distance(int sensor_type,
		  float PosX, float PosY, float PosZ,
		  float DirX, float DirY, float DirZ,
		  float maxLength,
		  float *dist);


void
update_obstacle(char* obstacle, char *attribute, int ival, char* cval); 
void
update_obstacle_attrs(char* name, int no_attrs, attr_spec* attrs);
void
update_obstacle_attrs_by_pointer(void *obst, int no_attrs, attr_spec* attrs);
void
query_obstacle_attrs(char* name, int no_attrs, char** attrs_in, attr_spec* attrs_out);
void
query_obstacle_attrs_by_pointer(void *obst, int no_attrs, char** attrs_in, attr_spec* attrs_out);

void lock_store();
void unlock_store();

#ifdef __cplusplus
}
#endif


#endif
