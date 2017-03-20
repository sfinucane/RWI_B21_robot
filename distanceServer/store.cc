// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/store.cc,v $
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
//  $Log: store.cc,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.11  2000/03/13 13:27:11  schulz
//  some changes
//
//  Revision 1.10  2000/02/07 13:09:49  schulz
//  Some fixes regarding network updates
//
//  Revision 1.9  1999/11/18 12:49:01  schulz
//  Fixed framebuffer, added stuff for humans
//
//  Revision 1.8  1999/10/28 09:19:29  schulz
//  -- first running version of framebuffer
//  -- bugfixes in linalg stuff
//  -- bugfixes in door.*
//
//  Revision 1.7  1999/09/29 13:02:22  schulz
//  Several changes, most important, added a scan line algorithm for
//  camera simulation
//
//  Revision 1.6  1999/05/25 14:43:52  schulz
//  Moved pthreads and locking stuff from RTLClientServer into libstore.a
//  Added C-functions for locking the database from within C-stuff.
//
//  Revision 1.5  1999/05/20 13:34:00  schulz
//  Changed grid implementation and renamed a method in t_Camera.
//
//  Revision 1.4  1998/12/16 15:00:42  schulz
//  patch of path
//
//  Revision 1.3  1998/12/16 13:24:27  schulz
//  patched t_WorkSet, which produced array underruns.
//  This class requires a rewrite!
//
//  Revision 1.2  1998/11/23 13:07:29  schulz
//  distanceServer is now also use by RTL. I made the
//  necessary changes, to bring the two modules into sync
//
//  Revision 1.1  1998/10/12 13:32:12  schulz
//  This is a complete new version
//
//  Revision 1.6  1998/10/12 08:32:34  schulz
//  several changes
//
//  Revision 1.5  1998/10/01 11:33:30  schulz
//  Inserted some ifdefs to handle differences between the GNU and the SUN compilers
//
//  Revision 1.4  1998/09/30 15:07:20  schulz
//  Several changes to make things compile under gcc 2.7.2
//
//  Revision 1.3  1998/08/08 23:05:36  schulz
//  Fixed some NULL Pointer problems
//
//  Revision 1.2  1998/08/04 15:59:47  schulz
//  Several bug fixes
//
//  Revision 1.1  1998/06/19 13:58:04  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//
// ----------------------------------------------------------------------------

#include <stdio.h>
#define _XOPEN_SOURCE
#include <math.h>
#undef _XOPEN_SOURCE
#include <values.h>

#if defined(__GNUG__) && __GNUG__ == 2 && __GNUG_MINOR__ <= 7 
#define __PUT_STATIC_DATA_MEMBERS_HERE
#endif

#ifdef __GNUG__
#include <typeinfo>
#endif
#include <map>

#include "sundefines.h"
#include "camera.hh"
#include "obstacles.hh"
#include "rectangle.hh"
#include "circle.hh"
#include "door.hh"
#include "human.hh"

#include "store.hh"
#include "t_ComRepTypes.hh"
#include "comdef.h"

extern t_surface* default_surface;

#define myABS(x)  ((x) < 0 ? -(x) : (x))
#define mySGN(x)  ((x) >= 0 ? (1) : (-1))


#define BORDER_SPAREROOM 1000.0

#define OBST_GRID_SIZE 10      /* size of a grid cell in cm */

#define N_OBSTACLE_TYPES 7
char* o_types[N_OBSTACLE_TYPES] ={
    "RECTANGLE",
    "CIRCLE",
    "Door",
    "Human",
    "CUBE",
    "CYLINDER",
    "3D-Door"
};

#ifdef USE_PTHREADS
t_RWlock store_lock;
#endif

// ---------------------------------------------------------------------------

t_ObstacleIndex::t_ObstacleIndex()
{
}

t_ObstacleIndex::~t_ObstacleIndex()
{
}

u_long
t_ObstacleIndex::hashValue(char* label)
{
  u_long i = 0;
  u_long k = 0;
  while(label[i] != 0) {
    k += label[i];
    i++;
  }
  return k % OBSTACLE_INDEX_BUCKETS;
}

bool
t_ObstacleIndex::insert(t_Obstacle* new_obstacle)
{
  // can not index an anonymous, they are assumed to be static
  if(new_obstacle->label() == NULL) return FALSE;      
  
  u_long hval = hashValue(new_obstacle->label());

  if(buckets[hval].insert(pair<char *const, t_Obstacle*>(new_obstacle->label(),new_obstacle)).second) {
    return TRUE;
  }
  else {
    fprintf(stderr, "WARNING: Name clash for Obstacle %s\n",
	    new_obstacle->label());
    return FALSE;
  }
}

t_Obstacle*
t_ObstacleIndex::retrieve(char* label)
{
  int hval = hashValue(label);
  return buckets[hval][label];
}

void
t_ObstacleIndex::remove(t_Obstacle* old_obstacle)
{
  if(old_obstacle->label() == NULL) return;
  int hval = hashValue(old_obstacle->label());
  buckets[hval].erase(old_obstacle->label());
}

// ---------------------------------------------------------------------------

#define INCREMENT 1
#define DECREMENT 2

class t_WorkSet
{
public:
  t_WorkSet(const t_ObstacleGrid& g, 
	    const t_Vector2f &from_, const t_Vector2f &to_) 
    : matrix(g), 
      current_cell(g.get(0,0)),
      from(from_),
      to(to_) {

    if(matrix.align2GridX(from.x()) == matrix.align2GridX(to.x())) {
      vertical = TRUE;
      horizontal = FALSE;
      index_is_x = FALSE;
      current_x = matrix.align2GridX(from.x());
      current_index = matrix.align2GridY(from.y());
      final_index =  matrix.align2GridY(to.y()); 
      if(to.y() > from.y()) {
	inc_or_dec = INCREMENT;
	if(final_index >= matrix.rows)
	  final_index = matrix.rows - 1;
      }
      else {
	inc_or_dec = DECREMENT;
	if(final_index < 0) final_index = 0;
      }
      stop_index = final_index;
    }
    else if(matrix.align2GridY(from.y()) == matrix.align2GridY(to.y())) {
      horizontal = TRUE;
      vertical = FALSE;
      index_is_x = TRUE;
      current_y = matrix.align2GridY(from.y()); 
      current_index = matrix.align2GridX(from.x());
      final_index = matrix.align2GridX(to.x());
      if(to.x() > from.x()) {
	inc_or_dec = INCREMENT;
	if(final_index >= matrix.cols)
	  final_index = matrix.cols - 1;
      }
      else {
	inc_or_dec = DECREMENT;
	if(final_index < 0) final_index = 0;
      }
      stop_index = final_index;
    }
    else {
      vertical = FALSE;
      horizontal = FALSE;
      m = (to.y() - from.y()) / (to.x() - from.x());
      decide_on_index_variable();
      decide_on_index_direction();
      m = fabs(m);
      
      if(index_is_x) {
	if(increasing) {
	  next_y = (ceil(from.y()/OBST_GRID_SIZE) * OBST_GRID_SIZE);
	}
	else {
	  next_y = (floor(from.y()/OBST_GRID_SIZE) * OBST_GRID_SIZE);
	}
	float inc_to_next_y = fabs((next_y - from.y()) / m);
	if(inc_or_dec == INCREMENT)
	  next_x = from.x() + inc_to_next_y;
	else
	  next_x = from.x() - inc_to_next_y;
	current_y = matrix.align2GridY(from.y()); 
	current_index = matrix.align2GridX(from.x());
	final_index = matrix.align2GridX(to.x());
	stop_index = matrix.align2GridX(next_x);
	if(inc_or_dec == INCREMENT) {
          if(final_index >= matrix.cols)
	    final_index = matrix.cols - 1;
	  if(stop_index > final_index)
            stop_index = final_index;
	}
	else {
         if(final_index < 0) final_index = 0;
         if(stop_index < final_index) 
	   stop_index = final_index;
        }
      }
      else {
	if(increasing)
	  next_x = (ceil(from.x()/OBST_GRID_SIZE) * OBST_GRID_SIZE);
	else
	  next_x = (floor(from.x()/OBST_GRID_SIZE) * OBST_GRID_SIZE);
	float inc_to_next_x = fabs(m * (next_x - from.x()));
	if(inc_or_dec == INCREMENT)
	  next_y = from.y() + inc_to_next_x;
	else
	  next_y = from.y() - inc_to_next_x;
	current_x = matrix.align2GridX(from.x());       
	current_index = matrix.align2GridY(from.y());
	final_index = matrix.align2GridY(to.y());
	stop_index = matrix.align2GridY(next_y);
	if(inc_or_dec == INCREMENT) {
	  if(final_index >= matrix.rows)
	    final_index = matrix.rows - 1;
	  if(stop_index > final_index)
            stop_index = final_index;
	}
	else {
          if(final_index < 0) final_index = 0;
          if(stop_index < final_index) stop_index = final_index;
        }
      }
    }
  };
  
  inline vector<t_Obstacle*>& operator++(int) {
    int new_ranges;

    decide_on_current_cell();
    
    if(inc_or_dec == INCREMENT) {
      current_index++;
      if(current_index > stop_index) {
	new_ranges = TRUE;
      }
      else new_ranges = FALSE;
    }
    else {
      current_index--;
      if(current_index < stop_index) {
	new_ranges = TRUE;
      }
      else new_ranges = FALSE;
    }
    if(new_ranges) {
      if(vertical || horizontal) {
	if(inc_or_dec == INCREMENT) current_index++;
	else current_index--;
      }
      else {
	current_index = 0;
	if(index_is_x) {
	  current_index = matrix.align2GridX(next_x);
	  advance_x();
	  advance_y();
	  stop_index = matrix.align2GridX(next_x);
	  if(inc_or_dec == INCREMENT)
	    if(stop_index > final_index)
	      stop_index = final_index;
	  else
	    if(stop_index < 0)
	      stop_index = 0;
	}
	else {
	  current_index = matrix.align2GridY(next_y);
	  advance_x();
	  advance_y();
	  stop_index = matrix.align2GridY(next_y);	  
	  if(inc_or_dec == INCREMENT)
	    if(stop_index > final_index)
	      stop_index = final_index;
	  else
	    if(stop_index < 0)
	      stop_index = 0;
	}
      }
    }
    /*
    cerr << "indices: " 
	 << current_index << " "
	 << current_x << " "
	 << current_y << " "
	 << stop_index << " "
         << final_index << endl;
    */
    return current_cell;
  }
  
  inline int empty() {
    if(index_is_x) {
      if(inc_or_dec == INCREMENT)
	return
	  (current_index > final_index 
	   || current_index >= matrix.cols
	   || current_y < 0 
	   || current_y >= matrix.rows);
      else
	return
	  (current_index < final_index
	   || current_y < 0 
	   || current_y >= matrix.rows);
    }
    else {
      if(inc_or_dec == INCREMENT)
	return
	  (current_index > final_index 
	   || current_index >= matrix.rows
	   || current_x < 0 
	   || current_x >= matrix.cols);
      else
	return
	  (current_index < final_index
	   || current_x < 0 
	   || current_x >= matrix.cols);
    }
    return FALSE;
  }
  t_Vector2f
  getCellPosition() {
    float x,y;
    if(horizontal) {
      x = matrix.midCellX(current_index);
      y = matrix.midCellY((int)current_y);
    }
    else if(vertical) {
      x = matrix.midCellX((int)current_x);
      y = matrix.midCellY(current_index);
    }
    else if(index_is_x) {
      x = matrix.midCellX(current_index);
      y = matrix.midCellY((int)current_y);
    }
    else {
      x = matrix.midCellX((int)current_x);
      y = matrix.midCellY(current_index);
    }
    return t_Vector2f(x,y);
  }

private:
  inline void
  decide_on_index_variable() {
    if(fabs(m) < 1) {
      index_is_x = TRUE;
    }
    else {
      index_is_x = FALSE;
    }
  }
  
  inline void
  decide_on_index_direction() {
    if(index_is_x)
      if(to.x() > from.x())  inc_or_dec = INCREMENT;
      else inc_or_dec = DECREMENT;
    else
      if(to.y() > from.y()) inc_or_dec = INCREMENT;
      else inc_or_dec = DECREMENT;
    if(inc_or_dec == INCREMENT)
      if(m >= 0) increasing = TRUE;
      else increasing = FALSE;
    else 
      if(m > 0) increasing = FALSE;
      else increasing = TRUE;
  }

  inline void
  decide_on_current_cell() {
    if(horizontal) {
      current_cell = matrix.get((int) current_y, current_index);
      //cerr << "cell (" << current_index << ", " << current_y << ")" << endl;  
    }
    else if(vertical) {
      current_cell = matrix.get(current_index, (int) current_x);
      //cerr << "cell (" << current_x << ", " << current_index << ")" << endl;  
    }
    else if(index_is_x) {
      current_cell = matrix.get((int) current_y, current_index);
      //cerr << "cell (" << current_index << ", " << current_y << ")" << endl;  
    }
    else {
      current_cell = matrix.get(current_index, (int) current_x);
      //cerr << "cell (" << current_x << ", " << current_index << ")" << endl;  
    }
  }


  
  inline void
  advance_y() {
    if(index_is_x == 0) {
      current_index = matrix.align2GridY(next_y);
      if(inc_or_dec == INCREMENT)
	next_y += m * OBST_GRID_SIZE;
      else
	next_y -= m * OBST_GRID_SIZE;
    }
    else {
      if(increasing)
	current_y += 1;
      else
	current_y -= 1;
    }
  }
  
  inline void
  advance_x() {
    if(index_is_x) {
      current_index = matrix.align2GridX(next_x);
      if(inc_or_dec == INCREMENT)
	next_x += OBST_GRID_SIZE/m;
      else
	next_x -= OBST_GRID_SIZE/m;
    }
    else {
      if(increasing)
	current_x += 1;
      else
	current_x -= 1;
    }
  }
  
  vector<t_Obstacle*>& current_cell;
  const t_ObstacleGrid&      matrix; 
  int index_is_x;
  int inc_or_dec;
  int vertical;
  int horizontal;
  int current_index;
  int stop_index;
  int final_index;
  int increasing;
  
  float m;
  float current_x;
  float current_y;
  float next_x;
  float next_y;
  const t_Vector2f from;
  const t_Vector2f to;
};

/* ------------------------------------------------------------------------- */

vector<t_Obstacle*>&
t_ObstacleGrid::get(int i, int j) const
{
#ifdef DEBUG
  if( i < 0 ) printf("t_ObstacleGrid::get row_underflow: %d %d\n", i, j);
  if( j < 0 ) printf("t_ObstacleGrid::get col_underflow: %d %d\n", i, j);
  if( i >= rows ) printf("t_ObstacleGrid::get row_overflow: %d %d\n", i, j);
  if( j >= cols ) printf("t_ObstacleGrid::get col_overflow: %d %d\n", i, j);
#endif
  //  return grid[i*cols+j];
  return grid[i][j];
}

bool
t_ObstacleGrid::off_ground(float x, float y)
{
    if(x < lx || y < ly || x > hx || y > hy)
	return TRUE;
    else return FALSE;
}

t_ObstacleGrid::~t_ObstacleGrid()
{
  u_long i;
  cout << "dim: " << rows << " " << cols << endl;

  for(i=0; i < rows; i++) {
    delete [] grid[i];
  } 

  delete [] grid;
  for(i = 0; i < all_obstacles.size(); i++)
    delete all_obstacles[i];
  for(i = 0; i < MAX_NUMBER_OF_THREADS; i++)
    delete ws[i];
}


void
t_ObstacleGrid::init(int border)
{
  lx = ly = hx = hy = 0;
  cols = 0;
  rows = 0;
  grid = NULL;
  has_border = border;
  for(int i = 0; i < MAX_NUMBER_OF_THREADS; i++)
    ws[i] = NULL;
}

t_ObstacleGrid::t_ObstacleGrid(int border) 
  : all_obstacles(0),
    active_obstacles(0),
    all_cameras(0)
{
  init(border);
}

t_ObstacleGrid::t_ObstacleGrid(float x1, float y1, float x2, float y2, int border)
  : all_obstacles(0),
    active_obstacles(0),
    all_cameras(0)
{
  init(border);
}

void
t_ObstacleGrid::setup_grid(float x1, float y1, float x2, float y2)
{
  u_long i;
  x1 -= BORDER_SPAREROOM;
  y1 -= BORDER_SPAREROOM;  
  x2 += BORDER_SPAREROOM;
  y2 += BORDER_SPAREROOM;  
  lx = x1; ly = y1;
  hx = x2; hy = y2;
  float w = x2 - x1+200;            // leave some room for a bounding box
  float d = y2 - y1+200;
  xoff = x1-100; yoff = y1-100;
  cols = (int)( (w) / OBST_GRID_SIZE + 2.0);
  rows = (int)( (d) / OBST_GRID_SIZE + 2.0);

  grid = new vector<t_Obstacle*>* [rows];
  for(i = 0; i < rows; i++) {
    grid[i] = new vector<t_Obstacle*> [cols];
  }

  t_Obstacle *new_obstacle;
  if(has_border) {
    new_obstacle = new t_Rectangle(x1-50,y1+d/2,DEF_Z,100,d,DEF_H);
    insert_grid(new_obstacle);
    new_obstacle = new t_Rectangle(x2+50,y1+d/2,DEF_Z,100,d,DEF_H);
    insert_grid(new_obstacle);
    new_obstacle = new t_Rectangle(x1+w/2,y1-50,DEF_Z,w,100,DEF_H);
    insert_grid(new_obstacle);
    new_obstacle = new t_Rectangle(x1+w/2,y2+50,DEF_Z,w,100,DEF_H);   
    insert_grid(new_obstacle);
  }

  for( i = 0; i < all_obstacles.size(); i++)
    insert_grid(all_obstacles[i]);

  for(i = 0; i < MAX_NUMBER_OF_THREADS; i++) {
    if(ws[i]) delete ws[i];
    ws[i] = new t_ObstacleWorkingSet(t_Obstacle::number_of_obstacles);
  }
}

float
t_ObstacleGrid::midCellX(int index) const
{
  return index*OBST_GRID_SIZE+xoff +0.5*OBST_GRID_SIZE;
}

float
t_ObstacleGrid::midCellY(int index) const
{
  return index*OBST_GRID_SIZE+yoff +0.5*OBST_GRID_SIZE;
}

int
t_ObstacleGrid::align2GridX(float fx) const
{
  return (int)( (fx-xoff) / OBST_GRID_SIZE);
}

int
t_ObstacleGrid::align2GridY(float fy) const
{
  return (int)( (fy-yoff) / OBST_GRID_SIZE);
}

void
t_ObstacleGrid::insert_grid(t_Obstacle *obst)
{

  SYNCHRONIZED();

  if(obst->active()) return;

  int minx,miny,maxx,maxy;
  float x1,y1,x2,y2;
  int i,j;

  for(i = 0; i < MAX_NUMBER_OF_THREADS; i++) {
    if(ws[i]) {
      if(!ws[i]->check_room(t_Obstacle::number_of_obstacles)) {
	delete ws[i];
	ws[i] = new t_ObstacleWorkingSet(t_Obstacle::number_of_obstacles);
      }
    }
    else {
      ws[i] = new t_ObstacleWorkingSet(t_Obstacle::number_of_obstacles);
    }
  }
  obst->clear_offsets();
  if(!obst->bounds(&x1,&y1,&x2,&y2)) {
    return;
  }
  minx = align2GridX(x1);
  miny = align2GridY(y1);
  if(minx < 0) minx = 0;
  if(miny < 0) miny = 0;
  maxx = align2GridX(x2) + 1;
  maxy = align2GridY(y2) + 1;
  if(maxx >= cols) maxx = cols - 1;
  if(maxy >= rows) maxy = rows - 1;
  for(i = miny; i < maxy; i++)
    for(j = minx; j < maxx; j++) {
      get(i,j).push_back(obst);
    }
}

void
t_ObstacleGrid::insert_lists(t_Obstacle* obst)
{
  SYNCHRONIZED();
   all_obstacles.push_back(obst);
  if(obst->active()) {
    active_obstacles.push_back(obst);
  }
  if(obst->label()) {
    if(!obstacle_index.insert(obst))
      fprintf(stderr, "Indexing of Obstacle %s failed\n", obst->label());
  }
}

t_Obstacle*
t_ObstacleGrid::insert(t_Obstacle* obst)
{
  SYNCHRONIZED();
  insert_lists(obst);
  insert_grid(obst);
  return obst;
}

void
t_ObstacleGrid::fill_ws_tri(float x1, float y1,
			  float x2, float y2,
			  float x3, float y3)
{
  SYNCHRONIZED();
  float xe, ye, t, m, inc;
  if( fabs(x3 - x2) < 0.01) {
    inc = OBST_GRID_SIZE / 2;
    ws[myThread()]->clear();
    if(y2 > y3) {
      t = y2;
      y2 = y3;
      y3 = t;
    }
    for(t = y2; t <= y3; t += inc)
      fill_ws_rayIntern(x1, y1, x2, t);
  }
  else {
    if( x3 < x2) {
      t = x2;
      x2 = x3;
      x3 = t;
      t = y2;
      y2 = y3;
      y3 = t;
    }
    m = (y3 - y2) / (x3 - x2);
    inc = OBST_GRID_SIZE / m;
    if( inc < 0) {
      inc = -inc;
    }
    if(inc > OBST_GRID_SIZE/2) inc = OBST_GRID_SIZE/2;
    ws[myThread()]->clear();
    for( t = 0; t <= (x3 - x2); t += inc) {
      ye = y2 + m * t;
      xe = x2 + t;
      fill_ws_rayIntern(x1, y1, xe, ye);
    }
  }
}

void
t_ObstacleGrid::fill_ws_ray(float x1, float y1,float x2, float y2)
{
  SYNCHRONIZED();
  ws[myThread()]->clear();
  fill_ws_rayIntern(x1,y1,x2,y2);
}

inline
void
swapFloats(float &f1, float &f2)
{
  float t = f1;
  f1 = f2;
  f2 = t;
}

void
t_ObstacleGrid::fill_ws_rayIntern(float x1, float y1,float x2, float y2)
{

  //special case line is vertical on grid scale
  if( align2GridX(x1) == align2GridX(x2) ) {
    int i, j, min_i, max_i;
    if(y2 < y1) {
      swapFloats(y1,y2);
    }
    j = align2GridX(x1);
    if(j < 0 || j >= cols) return;
    min_i = align2GridY(y1);
    max_i = align2GridY(y2);
    if(max_i >= rows) max_i = rows - 1;
    if(min_i < 0) min_i = 0;
    for(i = min_i; i <= max_i; i++) {
      vector<t_Obstacle*>& entry = get(i,j);
      for(u_long k = 0; k <entry.size(); k++)
	ws[myThread()]->add(entry[k]);
    }
  }
  else { 
    if(x1 > x2) {
      swapFloats(x1, x2);
      swapFloats(y1, y2);
    }
    if( align2GridY(y1) == align2GridY(y2) ) {
      int i, j, min_j, max_j;
      i = align2GridY(y1);
      if(i < 0 || i >= rows) return;
      
      min_j = align2GridX(x1);
      max_j = align2GridX(x2);
      if(min_j < 0) min_j = 0;
      if(max_j >= cols) max_j = cols - 1;
      for(j = 0; j <= max_j; j++) {
	vector<t_Obstacle*>& entry = get(i,j);
	for(u_long k = 0; k <entry.size(); k++)
	  ws[myThread()]->add(entry[k]);
      }
    }
    else {
      int i,j;
      float m;
      float tx,ty,inc;
      m = (y2-y1) / (x2-x1);
      inc = myABS((float)OBST_GRID_SIZE / m);
      if(inc > OBST_GRID_SIZE) inc = OBST_GRID_SIZE;
      for(tx = x1; tx <= x2; tx += inc/2) {
	ty = m*(tx-x1) + y1;
	j = align2GridX(tx);
	i = align2GridY(ty);
	if(j < 0) continue;
	if(i < 0) continue;
	if(j >= cols) continue;
	if(i >= rows) continue;  
	vector<t_Obstacle*>& entry = get(i,j);
	for(u_long k = 0; k <entry.size(); k++)
	  ws[myThread()]->add(entry[k]);
      }
    }
  }
}


void
t_ObstacleGrid::fill_ws(float x, float y,float dist)
{
  SYNCHRONIZED();
  int minx = (int)( (x-xoff-dist) / OBST_GRID_SIZE);
  int miny = (int)( (y-yoff-dist) / OBST_GRID_SIZE);
  int maxx = (int)( (x-xoff+dist) / OBST_GRID_SIZE + 1);
  int maxy = (int)( (y-yoff+dist) / OBST_GRID_SIZE + 1);
  int i,j;

#ifdef DEBUG
  color_ws("grey50");
#endif
  ws[myThread()]->clear();
  if(minx < 0) minx = 0;
  if(miny < 0) miny = 0;
  if(maxx >= cols) maxx = cols-1;
  if(maxy >= rows) maxy = rows-1;  
  for(i = miny; i < maxy; i++)
    for(j = minx; j < maxx; j++) {
      vector<t_Obstacle*>& entry = get(i,j);
      for(u_long k = 0; k <entry.size(); k++)
	ws[myThread()]->add(entry[k]);
    }
#ifdef DEBUG
  color_ws("red");
  printf("num_vis_obstacles: %d\n",ws[myThread()]->size());
#endif
}

t_Obstacle*
t_ObstacleGrid::install(t_ObstacleRecord *obst_rec)
{
    switch(obst_rec->type) {
    case CUBE:
	return new t_Rectangle(obst_rec->position.cx,
			       obst_rec->position.cy,
			       obst_rec->position.cz,
			       obst_rec->dimensions.w,
			       obst_rec->dimensions.d,
			       obst_rec->dimensions.h,
			       obst_rec->orientation,
			       obst_rec->color);
    case CYLINDER:
	return new t_Circle(obst_rec->position.cx,
			    obst_rec->position.cy,
			    obst_rec->position.cz,
			    obst_rec->dimensions.r,
			    obst_rec->dimensions.h,
			    obst_rec->color);
	
    case DOOR:
	return new t_Door(obst_rec->dimensions.w,
			  obst_rec->dimensions.d,
			  obst_rec->dimensions.h,
			  obst_rec->position.cx,
			  obst_rec->position.cy,
			  obst_rec->position.cz,
			  obst_rec->orientation,
			  obst_rec->color);
    case CAMERA: {
      t_Camera* new_cam = 
	new t_Camera(obst_rec->dimensions.w,
		     obst_rec->dimensions.d,
		     obst_rec->dimensions.h,
		     obst_rec->position.cx,
		     obst_rec->position.cy,
		     obst_rec->position.cz,
		     obst_rec->orientation,
		     obst_rec->yrotation,
		     this);
      all_cameras.push_back(new_cam);
      return new_cam;
    }
    case COMPOUND:
      return new t_ObstacleSet(obst_rec->position.cx,
			       obst_rec->position.cy,
			       obst_rec->position.cz,
			       obst_rec->orientation,
			       obst_rec->yrotation,
			       obst_rec->color_defined,
			       obst_rec->color);
    default:
      return NULL;
    }
}

void
t_ObstacleGrid::disableAll()
{
  for(u_long i = 0; i < all_obstacles.size(); i++) {
    all_obstacles[i]->disable();
  }
}

void
t_ObstacleGrid::enableAll()
{
  for(u_long i = 0; i < all_obstacles.size(); i++) {
    all_obstacles[i]->enable();
  }
}

void
t_ObstacleGrid::install_obstacle(t_ObstacleRecord* new_rec)
{
  t_Obstacle* new_obstacle;
  t_Position_4f offset;

  if(new_rec->type == COMPOUND_END) {
    t_ObstacleSet* newSet = obstacle_set_stack.top();
    newSet->closeSet();
    obstacle_set_stack.pop();
    if(obstacle_set_stack.empty())
      insert_lists(newSet);
    return;
  }

  // The rgb colors are statically dereferenced here
  // This should be replaced by an extra material stack
  // in the final implementation
  if( ! obstacle_set_stack.empty()) {
    if(!(new_rec->color_defined)) {
      new_rec->color = obstacle_set_stack.top()->getRGB();
      new_rec->color_defined = TRUE;
    }
  }

  new_obstacle = install(new_rec);

  if(!new_obstacle) return;

  if(new_rec->label != NULL) {
    new_obstacle->assign_label(new_rec->label);
  }

  if( !obstacle_set_stack.empty()) {
    obstacle_set_stack.top()->insert(new_obstacle);
    if(new_obstacle->label())
      if(!obstacle_index.insert(new_obstacle))
	fprintf(stderr, "Indexing of Obstacle %s failed\n", new_obstacle->label());
  }

  t_ObstacleSet* tmpSet = dynamic_cast<t_ObstacleSet*>(new_obstacle);
  if(tmpSet) {
    obstacle_set_stack.push(tmpSet);
  }
  else if(obstacle_set_stack.empty()) {
    insert_lists(new_obstacle);
  }
}

void
t_ObstacleGrid::check_read_ok()
{
  int i = 0;
  while(!obstacle_set_stack.empty()) {
    obstacle_set_stack.pop();
    i++;
  }
  if(i > 0)
    fprintf(stderr, "%d \"end\" bracket(s) where missing at end of file.\nClosed all open obstacle sets\n", i);
}

void
t_ObstacleGrid::map_bounds(float &min_x, float &min_y, float &max_x, float &max_y)
{
  float tx1, ty1, tx2, ty2;
  min_x = MAXFLOAT;
  min_y = MAXFLOAT;
  max_x = MINFLOAT;
  max_y = MINFLOAT;
  for(u_long i = 0; i < all_obstacles.size(); i++) {
    all_obstacles[i]->bounds(&tx1, &ty1, &tx2, &ty2);
    if(tx1 < min_x) min_x = tx1;
    if(ty1 < min_y) min_y = ty1;
    if(tx2 > max_x) max_x = tx2;
    if(ty2 > max_y) max_y = ty2;
  }
}

int
t_ObstacleGrid::read_obstacles(t_mapReader &sim_map)
{
  int i;
  t_Obstacle *new_obstacle;
  t_ObstacleRecord *obst_rec;
  while( (obst_rec = sim_map.nextObstacle()) ) {	
    install_obstacle(obst_rec);
  }
  check_read_ok();
  float x1, y1, x2, y2;
  map_bounds(x1, y1, x2, y2);
  setup_grid(x1, y1, x2, y2);
  return t_Obstacle::number_of_obstacles;
}

void
t_ObstacleGrid::SaveObstacles(FILE *fd)
{
  for(u_long i = 0; i < all_obstacles.size(); i++) {
    all_obstacles[i]->save(fd);
  }
}

t_Obstacle*
t_ObstacleGrid::InsideObstacle(const t_Vector3f& point)
{
  vector<t_Obstacle*>& cell =
    get(align2GridY(point.y()), align2GridX(point.x()));
  for(int i = 0 ; i < cell.size(); i++) {
    cell[i]->clear_offsets();
    if(cell[i]->inside(point)) {
      return cell[i];
    }
  }
  return NULL; 
}

t_Obstacle*
t_ObstacleGrid::InsideAnyObstacle(const t_Vector3f& point)
{
  for(u_long i = 0; i < all_obstacles.size(); i++) {
    all_obstacles[i]->clear_offsets();
    if(all_obstacles[i]->inside(point)) 
      return all_obstacles[i];
  }
  return NULL;
}

void
t_ObstacleGrid::remove_from_grid(t_Obstacle *remove)
{
  float x1,y1,x2,y2;
  int minx, maxx, miny, maxy;
  int i,j;

  if(remove->active()) return;

  remove->clear_offsets();
  if(!remove->bounds(&x1,&y1,&x2,&y2)) return;
  minx = align2GridX(x1);  
  miny = align2GridY(y1);
  if(minx < 0) minx = 0;
  if(miny < 0) miny = 0;
  maxx = align2GridX(x2) + 1;
  maxy = align2GridY(y2) + 1;
  if(maxx >= cols) maxx = cols - 1;
  if(maxy >= rows) maxy = rows - 1; 	
  for(i = miny; i < maxy; i++)
    for(j = minx; j < maxx; j++) {
      vector<t_Obstacle*>& list = get(i,j);
      vector<t_Obstacle*>::iterator 
	toremove = find(list.begin(), list.end(), remove);
      if(toremove != list.end())
	list.erase(toremove);
      else {
	cerr << "Warning: obstacle not on list within remove_from_grid" << endl;
      }
    }
}

void
t_ObstacleGrid::Update()
{
  SYNCHRONIZED();
  for(u_long i = 0; i < active_obstacles.size(); i++)
    Update(active_obstacles[i]);
}

void
t_ObstacleGrid::Update(t_Obstacle* obst)
{
  SYNCHRONIZED();
  if(obst->active())
    obst->update();
  else {
    remove_from_grid(obst);
    obst->clear_offsets();
    obst->update();
    insert_grid(obst);
  }
}

void
t_ObstacleGrid::Synchronise(t_Obstacle* obst)
{
  SYNCHRONIZED();
  remove_from_grid(obst);
  obst->clear_offsets();
  obst->synchronise();
  insert_grid(obst);
}

t_Obstacle*
t_ObstacleGrid::getObstacle(char *label)
{
  return obstacle_index.retrieve(label);
}

t_Obstacle*
t_ObstacleGrid::getObstacle(u_long id)
{
  for(vector<t_Obstacle*>::iterator it = all_obstacles.begin();
      it != all_obstacles.end(); ++it) {
    if((*it)->id() == id)
      return *it;
  }
}

void
t_ObstacleGrid::unlinkObstacle(t_Obstacle *remove)
{
  SYNCHRONIZED();
  if(!remove) return;
  vector<t_Obstacle*>::iterator it = find(all_obstacles.begin(), all_obstacles.end(), remove);
  if(it != all_obstacles.end())
    all_obstacles.erase(it);
  obstacle_index.remove(remove);
  if(remove->active()) {
    vector<t_Obstacle*>::iterator it = find(active_obstacles.begin(), 
					    active_obstacles.end(), remove);
    if(it != active_obstacles.end())
      active_obstacles.erase(it);
  }
  remove_from_grid(remove);
}

void
t_ObstacleGrid::RemoveObstacle(t_Obstacle *remove)
{
  unlinkObstacle(remove);
  delete remove;
}

void
t_ObstacleGrid::renderCamera(const t_Camera& cam)
{
  t_Obstacle *to_expose;
  float x2 = cam.pos.x() + 3000 * cos(cam.pan_tilt_rot.z() + cam.open_angle);
  float y2 = cam.pos.y() + 3000 * sin(cam.pan_tilt_rot.z() + cam.open_angle);
  float x3 = cam.pos.x() + 3000 * cos(cam.pan_tilt_rot.z() - cam.open_angle);
  float y3 = cam.pos.y() + 3000 * sin(cam.pan_tilt_rot.z() - cam.open_angle);    
  fill_ws_tri(cam.pos.x(),cam.pos.y(),x2,y2,x3,y3);
  while( (to_expose = ws[myThread()]->traverse()) ) {
    SYNCHRONIZED();    
    if(to_expose->polygonList() != NULL) {
      cam.renderPolygons(*(to_expose->polygonList()), to_expose);
    }
  }
  /*
  for(int i = 0; i < all_obstacles.size(); i++) {
    //    SYNCHRONIZED();    
    to_expose = all_obstacles[i];
    if(to_expose->polygonList() != NULL) {
      cam.renderPolygons(*(to_expose->polygonList()), to_expose);
    }
  }
  */
}

bool
t_ObstacleGrid::rayObstacleIntersection(const t_Vector3f &from,
					const t_Vector3f &direction,
					float maxLength,
					t_Vector3f &point_hit,
					t_Obstacle **obstacle_hit)
{

  SYNCHRONIZED();

  bool C_Hit, Hit = FALSE;
  float angle, distance, min_distance = maxLength;

  t_Vector3f to = from + maxLength*direction;

  t_Obstacle::new_stamp();
  t_WorkSet cellsOnRay(*this, t_Vector2f(from.x(), from.y()), 
			   t_Vector2f(to.x(), to.y()));

  vector<t_Obstacle*>& cell = get(0,0);
  while(! cellsOnRay.empty()) {
    cell = cellsOnRay++;

    for(u_long i = 0; i < cell.size(); i++) {
      if(cell[i]->already_checked()) continue;
      if(cell[i]->enabled()) {
	cell[i]->clear_offsets();
	C_Hit = cell[i]->distance(from, direction, 
				  min_distance, distance, angle);
      }
      else
	C_Hit = FALSE;
      cell[i]->mark_checked();
      if(C_Hit) {
	Hit = TRUE;
	min_distance = distance;
	*obstacle_hit = cell[i];
	point_hit = from + distance * direction;
      } 
    }
    if(Hit) break;
  }
  
  return Hit;
}

bool
t_ObstacleGrid::distance(const t_Vector3f &from,
			 const t_Vector3f &direction,
			 float maxLength,
			 float& min_distance,
			 float& hit_angle)
{
  SYNCHRONIZED();
  bool C_Hit, Hit = FALSE;
  float angle, distance;
  t_Vector3f to = from + maxLength*direction;
  min_distance = maxLength;

  t_Obstacle::new_stamp();
  t_WorkSet cellsOnRay(*this, t_Vector2f(from.x(), from.y()), 
				   t_Vector2f(to.x(), to.y()));

  while(! cellsOnRay.empty()) {
    vector<t_Obstacle*>& cell = cellsOnRay++;

    for(u_long i = 0; i < cell.size(); i++) {
      if(cell[i]->already_checked()) continue;
      if(cell[i]->enabled()) {
	cell[i]->clear_offsets();
	C_Hit = cell[i]->distance(from, direction, 
				  min_distance, distance, angle);
      }
      else
	C_Hit = FALSE;
      cell[i]->mark_checked();
      if(C_Hit) {
	Hit = TRUE;
	min_distance = distance;
	hit_angle = angle;
      } 
    }
    if(Hit) break;
  }
  return Hit;
}


bool
t_ObstacleGrid::wsDistance(const t_Vector3f &from,
			 const t_Vector3f &direction,
			 float maxLength,
			 float& min_distance,
			 float& hit_angle)
{
  SYNCHRONIZED();
  bool C_Hit, Hit = FALSE;
  float angle, distance;
  min_distance = maxLength;
  
  t_Obstacle* c_obstacle = NULL;
  t_Obstacle::new_stamp();
  while((c_obstacle = ws[myThread()]->traverse())) {
    if(c_obstacle->already_checked()) continue;
    c_obstacle->clear_offsets();
    C_Hit = c_obstacle->distance(from,direction, 
				 min_distance, distance, angle);
    if(C_Hit) {
      Hit = TRUE;
      min_distance = distance;
    }
    if(Hit) break;
  }
  return Hit;
}

bool
t_ObstacleGrid::fast_distance(const t_Vector3f &from,
			 const t_Vector3f &direction,
			 float maxLength,
			 float *min_distance)
{

  SYNCHRONIZED();
  bool C_Hit, Hit = FALSE;
  float angle, distance;
  t_Vector3f to = from + maxLength*direction;
  *min_distance = maxLength;

  t_Obstacle::new_stamp();
  t_WorkSet cellsOnRay(*this, t_Vector2f(from.x(), from.y()), 
				   t_Vector2f(to.x(), to.y()));

  t_Vector2f midCell;
  while(! cellsOnRay.empty()) {
    midCell = cellsOnRay.getCellPosition();
    vector<t_Obstacle*>& cell = cellsOnRay++;
    if(cell.size() > 0) {
      Hit = TRUE;
      break;
    }
  }
  if(Hit) {
    *min_distance = (midCell-t_Vector2f(from.x(), from.y())).length();
  }
  return Hit;
}

float t_ObstacleGrid::min_distance(float x, float y)
{
  float tmp, dist = MAXFLOAT;
  t_Obstacle *c_obstacle;

  fill_ws(x,y,500);

  while((c_obstacle = ws[myThread()]->traverse())) {
    c_obstacle->clear_offsets();
    tmp = c_obstacle->min_distance(x, y);
    if(tmp < dist) dist = tmp;
  }
  return dist;
}


bool
t_ObstacleGrid::get_distance(const t_Vector3f& from,
			     const t_Vector3f& direction,
			     float maxLength, 
			     float *dist)
{
  float dummy_angle;
  bool ret = distance(from, direction, maxLength, *dist, dummy_angle);
  return ret;
}


bool
t_ObstacleGrid::get_distance(const t_Vector3f& from,
			     const t_Vector3f& direction,
			     float maxLength,
			     float *dist, float *angle, 
			     const t_surface **surface)
{
  t_Obstacle *obst = NULL;
  if( (obst = InsideObstacle(from)) ) {
    *dist = 5.0;
    *angle = 90.0;
    *surface = default_surface;
    return TRUE;
  }
  t_Vector3f hitpoint;
  if( rayObstacleIntersection(from, direction, maxLength, hitpoint, &obst)) {
    *surface = obst->getSurface();
    obst->clear_offsets();
    obst->distance(from, direction, maxLength, *dist, *angle);
    *angle *= 180 / M_PI;
    *angle -= 90;
    return TRUE;
  }
  else return FALSE;
}


// the following one is used to connect old C stuff
// will be removed soon 

extern t_ObstacleGrid *obstore;

bool get_distance(int sensor_type,
		  float PosX, float PosY, float PosZ,
		  float DirX, float DirY, float DirZ, 
		  float maxLength,
		  float *dist)
{
  float angle;
  const t_surface *surface;
  float tdist;
  bool Hit = obstore->get_distance(t_Vector3f(PosX, PosY, PosZ),
				   t_Vector3f(DirX, DirY, DirZ),
				   maxLength,
				   &tdist, &angle, &surface);
  if(!Hit) return FALSE;
  if(!surface)
    cerr << "Warning: obstacle does not have a surface" << endl;
  else
    *dist = surface->add_error(sensor_type, tdist, angle);
  return TRUE;
}

float obstacles_min_distance(float x, float y)
{
    return obstore->min_distance(x,y);
}

void
update_obstacle(char *name, char *attribute, int ival, char *cval)
{
  obstore->update_obstacle(name, attribute, ival, cval);
}

void
update_obstacle_attrs(char* name, int nattrs, attr_spec* attrs)
{
  obstore->update_obstacle_attrs(name, nattrs, attrs);
}

void
update_obstacle_attrs_by_pointer(void *obst, int nattrs, attr_spec* attrs)
{
  obstore->update_obstacle_attrs_by_pointer((t_Obstacle*) obst, nattrs, attrs);
}


void
t_ObstacleGrid::update_obstacle_attrs(char* name, int nattrs, attr_spec* attrs)
{
  t_Obstacle* obst = getObstacle(name);
  if(obst == NULL) return;
  int i;
  removeFromGrid(obst);
  for(i = 0; i < nattrs; i++)
    obst->changeAttribute(attrs[i].id, attrs[i].ival, attrs[i].cval);
  insertToGrid(obst);
}


void
t_ObstacleGrid::update_obstacle_attrs_by_pointer(t_Obstacle *obst, 
						 int nattrs, attr_spec* attrs)
{
  if(obst == NULL) return;
  int i;
  removeFromGrid(obst);
  for(i = 0; i < nattrs; i++)
    obst->changeAttribute(attrs[i].id, attrs[i].ival, attrs[i].cval);
  insertToGrid(obst);
}

void
t_ObstacleGrid::update_obstacle(char *name, char *attribute, int ival, char *cval)
{
  t_Obstacle* obst = getObstacle(name);
  if(obst) {
    removeFromGrid(obst);
    obst->changeAttribute(attribute, ival, cval);
    insertToGrid(obst);
  }
  else 
    fprintf(stderr, "WARNING: Received update request for unknown obstacle %s.\n", name);
}

void
query_obstacle_attrs(char* name, int nattrs, char** attrs_in, attr_spec* attrs_out)
{
  t_Obstacle* obst = obstore->getObstacle(name);
  if(obst == NULL) return;
  int i;
  for(i = 0; i < nattrs; i++) {
    attrs_out[i].id = attrs_in[i]; 
    attrs_out[i].cval = "";
    obst->getAttribute(attrs_out[i].id, &(attrs_out[i].ival), &(attrs_out[i].cval));
  }
}

void
query_obstacle_attrs_by_pointer(void *vptr, int nattrs, char** attrs_in, attr_spec* attrs_out)
{
  t_Obstacle *obst = (t_Obstacle*) vptr;
  if(obst == NULL) return;
  int i;
  for(i = 0; i < nattrs; i++) {
    attrs_out[i].id = attrs_in[i]; 
    attrs_out[i].cval = "";
    obst->getAttribute(attrs_out[i].id, &(attrs_out[i].ival), &(attrs_out[i].cval));
  }
}



#define DATABASE_UPDATE 1
#define UNSUPPORTED_COMMAND 0

int
t_ObstacleGrid::netTransactionType()
{
  char cmd;
  char spec;
  u_short len;
  my_NetLink->readPacketHeader(&cmd, &spec, &len);
  switch(cmd) {
  case COMM_MODIFY:
    switch(spec) {
    case COMM_NAMEDOBJECT:
      return DATABASE_UPDATE;
    default:
      return UNSUPPORTED_COMMAND;
    }
  default:
    return UNSUPPORTED_COMMAND;
  }
}

char*
t_ObstacleGrid::netObstacleName()
{
  char* obstacle_name = NULL;
  t_TypedPtr type_info[2];
  type_info[0] =
    t_TypedPtr(t_Attrib::TI_STRING, "obstacle name", &obstacle_name);
  type_info[1] = TI_END_ATTR;
  my_NetLink->read(type_info);
  return obstacle_name;
}

void
t_ObstacleGrid::doUpdateFromNet()
{
  switch(netTransactionType()) {
  case DATABASE_UPDATE:
    Synchronise(obstacle_index.retrieve(netObstacleName()));
    break;
  default:
    fprintf(stderr, "Received unsupported transaction type\n");
    break;
  }
}

void
t_ObstacleGrid::assignNetLink(t_NetLink* link)
{
  my_NetLink = link;
}

#ifdef USE_PTHREADS
void
lock_store()
{
  store_lock.writelock();
}
void
unlock_store()
{
  store_lock.unlock();
}
#else
void
lock_store()
{
}
void
unlock_store()
{
}
#endif
