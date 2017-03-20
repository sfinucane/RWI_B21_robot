
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** File:                        store.cc
 *****
 ***** Part of:                     RHINO SOFTWARE
 *****
 ***** Creator:                     Dirk Schulz, University of Bonn
 *****
 ***** Date of creation:            July 1996
 *****
 *****
 *****
 *****
 ***** $Source: /usr/local/cvs/bee/src/simulator2/store.cc,v $
 *****
 ***** $Revision: 1.1 $
 *****
 ***** $Date: 2002/09/14 16:11:07 $
 *****
 ***** $Author: rstone $
 *****
 *****
 *****
 ***** Contact thrun@carbon.cs.bonn.edu or thrun@cs.cmu.edu.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** 
 ***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED BY
 ***** APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING THE
 ***** COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM "AS IS"
 ***** WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED,
 ***** INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 ***** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE ENTIRE
 ***** RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU.
 ***** SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL
 ***** NECESSARY SERVICING, REPAIR OR CORRECTION.
 ***** 
 ***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
 ***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
 ***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
 ***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
 ***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
 ***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO LOSS OF
 ***** DATA OR DATA BEING RENDERED INACCURATE OR LOSSES SUSTAINED BY YOU
 ***** OR THIRD PARTIES OR A FAILURE OF THE PROGRAM TO OPERATE WITH ANY
 ***** OTHER PROGRAMS), EVEN IF SUCH HOLDER OR OTHER PARTY HAS BEEN
 ***** ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: store.cc,v $
 * Revision 1.1  2002/09/14 16:11:07  rstone
 * *** empty log message ***
 *
 * Revision 1.45  2000/03/18 15:28:46  schulz
 * changed maximum step width to 0.5*OBST_GRID_SIZE in fill_ws_ray()
 *
 * Revision 1.44  2000/03/18 15:10:09  schulz
 * changed rounding in grid_range.
 * Now the bounding box is generally one grid cell larger then required,
 * but that should not matter
 *
 * Revision 1.43  2000/03/09 09:30:11  schulz
 * Added Saschas multi-robot extension
 *
 * Revision 1.40  1999/10/06 15:04:51  schulz
 *  The normal vector for vertical lines was wrong in
 *  linalg.c:line_min_dist_point(), fixed (hopefully)
 *
 * Revision 1.39  1999/09/28 12:03:41  schulz
 * Fixed a serious bug in the min_distance check!
 *
 * Revision 1.38  1999/09/27 10:46:59  schulz
 * Initial bounding box was not computed for rectangles; fixed
 *
 * Revision 1.37  1997/04/02 12:36:56  fox
 * Fixed a bug in tag handling.
 *
 * Revision 1.36  1997/04/01 11:51:14  fox
 * Removed a comment.
 *
 * Revision 1.35  1997/04/01 11:11:31  fox
 * Added comment handling for simulator maps. Comment lines must start with #.
 * Added the possibility to set a tag name for the different objects.
 *
 * Revision 1.34  1997/03/11 17:16:44  tyson
 * added IR simulation and other work
 *
 * Revision 1.33  1997/03/11 11:02:20  schulz
 * Removed the distinction between RECTANGLEs and CUBEs  when
 * saving the playground. We always save CUBEs now.
 * Dito for CIRCLE <-> CYLINDER
 *
 * Revision 1.32  1997/03/03 15:13:34  schulz
 * Fixed wrong counting of obstacles. As a result active obstacles are handled
 * correctly now (hopefully).
 *
 * Revision 1.31  1997/02/27 15:43:01  schulz
 * fixed handling of doors and some other bugs
 *
 * Revision 1.30  1997/01/29 17:47:17  schulz
 * fixed a bug fill_ws_ray()
 *
 * Revision 1.29  1997/01/27 15:13:30  schulz
 * added added multiple sensor planes to the sensor simulation stuff
 * I hope this does not introduce to many new bugs.
 *
 * Revision 1.28  1997/01/06 10:51:52  schulz
 * -- (0,0) is really in the lower left corner now!
 * -- sonar simulation works again.
 *
 * Revision 1.27  1996/12/19 15:54:15  schulz
 * 
 * 
 *
 * Revision 1.26  1996/12/13 12:19:53  schulz
 * added code to adjust the playground to the positive quadrant before
 * saving
 *
 * Revision 1.25  1996/12/10 14:13:29  schulz
 * changed char consts rhino ->bee BASE -> COLLI
 *
 * Revision 1.24  1996/12/10 13:44:00  schulz
 * -- added -tcx option
 * -- loading of maps now possible
 * -- file requester has bigger width now.
 *    The translation for the return key has still to be changed!
 *
 * Revision 1.23  1996/12/09 09:47:55  schulz
 * Updated the Makefile to use libXext and libICE on linux, as all newer
 * Linux distributions need these.
 *
 * Revision 1.22  1996/12/04 14:25:29  schulz
 * Human obstacles completed (hopefully)
 *
 * Revision 1.20  1996/12/04 09:25:45  schulz
 * Just an umask test!
 *
 * Revision 1.19  1996/12/03 15:18:38  schulz
 * -- some fixes
 * -- added humans as obstacles (still buggy)
 *
 * Revision 1.18  1996/12/02 17:02:02  schulz
 * new user interface for obstacle insertion
 * some fixes
 *
 * Revision 1.17  1996/11/15 12:21:24  ws
 * Changed InsideObstacle to use the obstacle grid.          DS
 *
 * Revision 1.16  1996/11/14 12:58:05  ws
 * Introduced a scheduler with priorities and aging, to
 * get better performance on heavily loaded systems.    DS
 *
 * Revision 1.15  1996/11/12 01:32:56  ws
 * Fixed some memory bugs/leaks.
 *
 * Revision 1.14  1996/11/05 22:43:27  ws
 * Some minor defines for SUN OS 4.1
 *
 * Revision 1.13  1996/10/29 16:05:12  ws
 * changed some class names
 *
 * Revision 1.12  1996/10/23 09:57:00  ws
 * removed a range overflow in store.cc
 *
 * Revision 1.11  1996/10/23 08:57:18  ws
 * eliminated cross file references to struct robot
 *
 * Revision 1.10  1996/10/18 14:00:00  ws
 * obstacles can be toggled on/off now using the right mouse button.
 *
 * Revision 1.9  1996/10/16 15:21:14  ws
 * fixed a bug in the obstacle deletion code
 *
 * Revision 1.8  1996/10/15 13:58:54  ws
 * - new features: deletion of obstacles (Shift+ right mouse button)
 *               usage information (-h or -help option)
 * - added -laser option
 * - Now the simulator waits for BASE to come up before it starts
 *   sending laserReports.
 *
 * Revision 1.7  1996/10/14 12:19:11  ws
 * added support for different surfaces (not yet complete) and sonar error
 *
 * Revision 1.6  1996/10/09 14:26:05  ws
 * removed some printfs fixed even more bugs
 *
 * Revision 1.5  1996/10/09 13:39:52  schulz
 * *** empty log message ***
 *
 * Revision 1.4  1996/10/09 13:27:44  schulz
 * reorganized obstacle store and fixed some more bugs.
 *
// Revision 1.3  1996/10/07  14:50:50  schulz
// - corrected an initialization error (on empty maps)
// - eliminated an endless loop in store.cc
//
 * Revision 1.2  1996/10/01 11:49:54  ws
 * corrected range checking in t_obstgrid methods
 *
 * Revision 1.4  1996/09/20 07:56:22  schulz
 * Added saving of maps. E.g. a filename requester and a save method
 * for each obstacle type.
 *
 * Revision 1.3  1996/09/11 15:49:21  schulz
 * patched a range overflow in store.cc which caused a bus error
 * on SUNS.
 *
 * Revision 1.2  1996/09/11 14:03:50  schulz
 * - Speed up sonar and laser simulation by a factor of 20.
 *   We precalculate the set of obstacles which can be hit by a
 *   laser/sonar beam and check only for these few obstacles, if they are hit.
 *   Switched to S.Thruns version of line and circle intersection routines,
 *   these are a little bit faster then the old ones.
 * - Included laser visualisation  (to slow an some machines)
 *
 * Revision 1.1  1996/08/27 15:23:09  schulz
 * Some files forgotten last time
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <X11/Intrinsic.h>
#include <stdio.h>
#include <math.h>

#include "sundefines.h"
#include "store.hh"
#include "obstacles.hh"
#include "rectangle.hh"
#include "circle.hh"
#include "doors.hh"
#include "human.hh"

extern Boolean multi_robots; /* AF:20000206*/

extern t_obst* install_obstacle(int,FILE*);


Boolean
t_obstws::check_room(int n)
{
    return n < maxsize;
}

t_obstws::~t_obstws()
{
    delete[] workingset;
    delete[] members;
}

t_obstws::t_obstws(int n)
{
    maxsize = n+WS_SPAREROOM;
    workingset = new (t_obst*)[n+WS_SPAREROOM];
    members = new Boolean[n+WS_SPAREROOM];
    top = 0;
    travptr = 0;
}

void
t_obstws::add(t_obst *obst)
{
    if(obst->number >= maxsize) {
	fprintf(stderr, "t_obstws::add() : set overflow (%d) !\n",obst->number);
	exit(-1);
    }
    if(!obst->enabled || members[obst->number]) return;
    workingset[top++] = obst;
    members[obst->number] = TRUE;
}

int
t_obstws::size()
{
    return top;
}

void
t_obstws::clear()
{
  int i;
  for(i = 0; i < (maxsize); i++) members[i] = FALSE;
  top = 0;
  travptr = 0;
}

t_obst*
t_obstws::traverse()
{
  if(travptr < top)
    return workingset[travptr++];
  else {
    travptr = 0;
    return NULL;
  }
}

/* ------------------------------------------------------------------------- */

t_obstlist*
t_obstgrid::get(int i, int j)
{
#ifdef DEBUG
    if( i < 0 ) printf("t_obstgrid::get row_underflow: %d %d\n", i, j);
    if( j < 0 ) printf("t_obstgrid::get col_underflow: %d %d\n", i, j);
    if( i >= rows ) printf("t_obstgrid::get row_overflow: %d %d\n", i, j);
    if( j >= cols ) printf("t_obstgrid::get col_overflow: %d %d\n", i, j);
#endif
    return grid[i*cols+j];
}

Boolean
t_obstgrid::off_ground(float x, float y)
{
    if(x < lx || y < ly || x > hx || y > hy)
	return TRUE;
    else return FALSE;
}

t_obstgrid::~t_obstgrid()
{
    int i,j;
    t_obstlist_entry *tmp, *del_entry = all_obstacles.first;
    while(del_entry) {
	tmp = del_entry;
	del_entry = del_entry->next;
	delete tmp->obstacle;
    }
    for(i = 0; i < rows*cols; i++) {
	delete grid[i];
    }
    delete ws;
}

t_obstgrid::t_obstgrid(float x1, float y1, float x2, float y2)
{
  int i;
  float cs = OBST_GRID_SIZE;
  float hcs = 0.5 * OBST_GRID_SIZE;
  lx = x1;
  ly = y1;
  hx = x2;
  hy = y2;
  float w = x2 - x1+2*cs;            // leave some room for a bounding box
  float d = y2 - y1+2*cs;
  t_obst *new_obstacle;
  xoff = x1-cs;
  yoff = y1-cs;
  cols = (int)( (w) / OBST_GRID_SIZE + 2.0);
  rows = (int)( (d) / OBST_GRID_SIZE + 2.0);
  grid = new (t_obstlist*)[cols*rows];
  for(i=0; i < cols*rows; i++) {
      grid[i] = new t_obstlist();
  }
  ws = new t_obstws(0);
  all_obstacles.first = NULL;
  all_obstacles.last = NULL;  
  active_obstacles.first = NULL;
  active_obstacles.last = NULL;
  n_obstacles = 0;

  new_obstacle = new t_rectangle(dummyTag,x1-hcs,y1+d/2,DEF_Z,cs,d,DEF_H);
  new_obstacle->number = n_obstacles++;
  insert_grid(new_obstacle);
  new_obstacle = new t_rectangle(dummyTag,x2+hcs,y1+d/2,DEF_Z,cs,d,DEF_H);
  new_obstacle->number = n_obstacles++;  
  insert_grid(new_obstacle);
  new_obstacle = new t_rectangle(dummyTag,x1+w/2,y1-hcs,DEF_Z,w,cs,DEF_H);
  new_obstacle->number = n_obstacles++;
  insert_grid(new_obstacle);
  new_obstacle = new t_rectangle(dummyTag,x1+w/2,y2+hcs,DEF_Z,w,cs,DEF_H);   
  n_obstacles = n_obstacles++;
  new_obstacle->number = n_obstacles++;  
  insert_grid(new_obstacle);

}


void
t_obstgrid::grid_range(t_obst *obst, 
		       int *minx, int *miny, 
		       int *maxx, int *maxy)
{
  float x1,y1,x2,y2;
  obst->bounds(&x1, &y1, &x2, &y2);
  *minx = (int)( (x1-xoff) / OBST_GRID_SIZE - 1);
  *miny = (int)( (y1-yoff) / OBST_GRID_SIZE - 1);
  *maxx = (int)( (x2-xoff) / OBST_GRID_SIZE + 1);
  *maxy = (int)( (y2-yoff) / OBST_GRID_SIZE + 1);
  if(*minx < 0 ) *minx = 0;
  if(*miny < 0 ) *miny = 0;
  if(*maxx > cols ) *maxx = cols;
  if(*maxy > rows ) *maxy = rows;
}

void
t_obstgrid::insert_grid(t_obst *obst)
{
    t_obstlist_entry *entry;
    t_obstlist *list;
    int minx,miny,maxx,maxy;
    int i,j;

    if(ws) {
	if(!ws->check_room(n_obstacles)) {
	    delete ws;
	    ws = new t_obstws(n_obstacles);
	}
    }

    grid_range(obst, &minx, &miny, &maxx, &maxy);

    for(i = miny; i < maxy; i++)
	for(j = minx; j < maxx; j++) {
	  entry = new t_obstlist_entry;
	  entry->obstacle = obst;
	    list = get(i,j);
	    entry->next = list->first;
	    list->first = entry;
	}
}

t_obstlist::~t_obstlist()
{
    t_obstlist_entry *tmp, *del_entry = first;
    while(del_entry) {
	tmp = del_entry;
	del_entry = del_entry->next;
	delete tmp;
    }
}

t_obstlist::t_obstlist()
{
    first = last = NULL;
}

t_obst*
t_obstlist::insert(t_obst *obst)
{
  t_obstlist_entry *new_obstacle = new t_obstlist_entry;
  new_obstacle->obstacle = obst;
  if(last) { 
    last->next = new_obstacle;	
    last = new_obstacle;
  }
  else {
    first = last = new_obstacle;
  }
  last->next = NULL;
}


t_obst*
t_obstgrid::insert(t_obst* obst)
{
  all_obstacles.insert(obst);
  obst->number = n_obstacles++;
  if(obst->active) {
      active_obstacles.insert(obst);
  }
  insert_grid(obst);
  return obst;
}

void
t_obstgrid::fill_ws_ray(float x1, float y1,float x2, float y2)
{
  float m;
  float tx,ty,inc;
  int i,j;
  t_obstlist_entry *entry;
  ws->clear();
  if( fabs(x1 - x2) < 0.01 ) {
    if(y2 < y1) {
      m = y1;
      y1 = y2;
      y2 = m;
    }
    j = (int)( (x1-xoff) / OBST_GRID_SIZE);
    if(j < 0) return;
    if(j >= cols) return;  
    for(ty = y1; ty <= (y2+OBST_GRID_SIZE); ty+=OBST_GRID_SIZE) {
      i = (int)( (ty-yoff) / OBST_GRID_SIZE);
      if(i < 0) continue;
      if(i >= rows) continue;  
      entry = get(i,j)->first;
      while(entry) {
	ws->add(entry->obstacle);
	entry = entry->next;
      }
    }
    return;
  }
  if(x1 > x2) {
    tx = x2;
    ty = y2;
    x2 = x1;
    y2 = y1;
    x1 = tx;
    y1 = ty;
  }
  if( fabs(y2 - y1) < 0.01) {
      m = 0;
      inc = OBST_GRID_SIZE;
  }
  else {
    m = (y2-y1)/(x2-x1);
    inc = fabs((float)OBST_GRID_SIZE / m);
    if(inc > 0.5*OBST_GRID_SIZE) inc = 0.5*OBST_GRID_SIZE;
  }
  for(tx = x1; tx <= (x2+OBST_GRID_SIZE); tx += inc) {
    ty = m*(tx-x1) + y1;
    j = (int)( (tx-xoff) / OBST_GRID_SIZE);
    i = (int)( (ty-yoff) / OBST_GRID_SIZE);
    if(j < 0) continue;
    if(i < 0) continue;
    if(j >= cols) continue;
    if(i >= rows) continue;  
    entry = get(i,j)->first;
    while(entry) {
      ws->add(entry->obstacle);
      entry = entry->next;
    }
  }
}


void t_obstgrid::fill_ws(float x, float y,float dist)
{
  int minx = (int)( (x-xoff-dist) / OBST_GRID_SIZE);
  int miny = (int)( (y-yoff-dist) / OBST_GRID_SIZE);
  int maxx = (int)( (x-xoff+dist) / OBST_GRID_SIZE + 1);
  int maxy = (int)( (y-yoff+dist) / OBST_GRID_SIZE + 1);
  int i,j;
  t_obstlist_entry *entry;
#ifdef DEBUG
  color_ws("grey50");
#endif
  ws->clear();
  if(minx < 0) minx = 0;
  if(miny < 0) miny = 0;
  if(maxx >= cols) maxx = cols-1;
  if(maxy >= rows) maxy = rows-1;  
  for(i = miny; i < maxy; i++)
    for(j = minx; j < maxx; j++) {
      entry = get(i,j)->first;
      while(entry) {
	ws->add(entry->obstacle);
	entry = entry->next;
      }
    }
#ifdef DEBUG
  color_ws("red");
  printf("num_vis_obstacles: %d\n",ws->size());
#endif
}

int
t_obstgrid::read_obstacles(FILE *mapfd)
{
    char otype[80];
    int i;
    Boolean found;
    t_obst *new_obstacle;
    while(!feof(mapfd)) {
	found = FALSE;    
	i = fscanf(mapfd,"%s",&otype);
	if( i < 1)
	  continue;
	else if ( otype[0] == commentSign) { // comment sign --> ignore end of line
	  fgets( otype, sizeof(otype),mapfd);
	  continue;
	}	
	for(i = 0; i < N_OBSTACLE_TYPES; i++) {
	    if(strcmp(o_types[i], otype) == 0) {
		found = TRUE;
		new_obstacle = install_obstacle(i,mapfd);
		insert(new_obstacle);
		break;
	    }
	}
    }
    if(ws) delete ws;
    ws = new t_obstws(n_obstacles);
    return n_obstacles;   
}

void t_obstgrid::ExposeObstacles()
{
    t_obstlist_entry *to_expose = all_obstacles.first;
    int i,j;
    while(to_expose) {
	to_expose->obstacle->expose();
	to_expose = to_expose->next;
    }
	
}

void t_obstgrid::SaveObstacles(FILE *fd)
{
    t_obstlist_entry *to_expose = all_obstacles.first;
    int i,j;
    while(to_expose) {
	to_expose->obstacle->save(fd);
	to_expose = to_expose->next;
    }
	
}

void
t_obstgrid::bounds(float *minX, float *minY, float *maxX, float *maxY)
{
    float tx1,ty1,tx2,ty2;
    t_obstlist_entry* trav_obst = all_obstacles.first;
    if(trav_obst) {
	trav_obst->obstacle->bounds(minX,minY,maxX,maxY);
	trav_obst = trav_obst->next;
    }
    else {
	*minX = 0;
	*maxX = 0;	
	*minY = 0;
	*maxY = 0;
	return;
    }
    while(trav_obst) {
	trav_obst->obstacle->bounds(&tx1,&ty1,&tx2,&ty2);
	if(tx1 < *minX) *minX = tx1;
	if(tx2 > *maxX) *maxX = tx2;	
	if(ty1 < *minY) *minY = ty1;
	if(ty2 > *maxY) *maxY = ty2;
	trav_obst = trav_obst->next;
    }
}

void
t_obstgrid::move(float diffX, float diffY)
{
    t_obstlist_entry *trav_obst = all_obstacles.first;
    while(trav_obst) {
	trav_obst->obstacle->move(diffX, diffY);
	trav_obst = trav_obst->next;
    }
}

t_obst*
t_obstgrid::InsideObstacle(float x, float y)
{
    t_obst *inside_test_obstacle;    
    fill_ws(x,y,OBST_GRID_SIZE);    
    while((inside_test_obstacle = ws->traverse())) {
	if(inside_test_obstacle->inside(x,y))
	    return inside_test_obstacle;
    }
    return NULL;
}

t_obst*
t_obstgrid::InsideAnyObstacle(float x, float y)
{
    t_obstlist_entry *inside_test_obstacle = all_obstacles.first;    
    while(inside_test_obstacle) {
	if(inside_test_obstacle->obstacle->inside(x,y))
	    return inside_test_obstacle->obstacle;
	inside_test_obstacle = inside_test_obstacle->next;
    }
    return NULL;
}

void
t_obstlist::remove(t_obst *remove)
{
    t_obstlist_entry *rem_test = first, *last_one = NULL;
	while(rem_test && rem_test->obstacle != remove) {
	last_one = rem_test;
	rem_test = rem_test->next;
    }
    if(!rem_test) {
	fprintf(stderr, "t_obstlist::remove() : obstacle not in list\n");
	exit(0);
    }
    if(last_one) {
	last_one->next = rem_test->next;
	delete rem_test;
	if(last_one->next == NULL) last = last_one;    
    }
    else {
	if( first->next == NULL ) first = last = NULL; 
	else first = first->next;
	delete rem_test;
    }

}

void
t_obstgrid::remove_from_grid(t_obst *to_remove)
{
  float x1,y1,x2,y2;
  int minx, maxx, miny, maxy;
  int i,j;
  t_obstlist *list, *dummy;
  grid_range(to_remove, &minx, &miny, &maxx, &maxy);
  for(i = miny; i < maxy; i++)
    for(j = minx; j < maxx; j++) {
      list = get(i,j);
      list->remove(to_remove);
    }
}

void
t_obstgrid::Update()
{
  t_obstlist_entry *traverse = active_obstacles.first;
  struct timeval now;
  gettimeofday(&now, NULL);
  while(traverse) {
    remove_from_grid(traverse->obstacle);
    traverse->obstacle->update(&now);
    insert_grid(traverse->obstacle);
    traverse = traverse->next;
  }
}

void
t_obstgrid::Redraw()
{
  t_obstlist_entry *traverse = active_obstacles.first;
  while(traverse) {
    remove_from_grid(traverse->obstacle);
    traverse->obstacle->redraw();
    insert_grid(traverse->obstacle);
    traverse = traverse->next;
  }
}

void
t_obstgrid::RemoveObstacle(t_obst *remove)
{
    if(!remove) return;
    if (!multi_robots) // AF
      printf("removing from all obstacles\n");
    all_obstacles.remove(remove);
    if(remove->active) {
	printf("removing from active obstacles\n");	
	active_obstacles.remove(remove);
    }
    if (!multi_robots) // AF
      printf("removing from obstacle grid\n");
    remove_from_grid(remove);
    delete remove;
}

void
t_obstgrid::new_rectangle(float x1, float y1, float x2, float y2)
{
  float w = x2-x1;
  float d = y2-y1;
  insert(new t_rectangle(dummyTag,x1+w/2,y1+d/2,DEF_Z,w,d,DEF_H))->expose();
}

static void
swapFloats(float *f1, float *f2)
{
    float t = *f1;
    *f1 = *f2;
    *f2 = t;
}

void
t_obstgrid::new_door(float apx, float apy, float w, float d)
{
  float tmp,angle;
  if(fabs(w) < fabs(d)) {        // doors orientation is vertical
      swapFloats(&w,&d);
      if(w < 0) {  // door drawn in negative y-direction
	  angle = -M_PI/2;
      }
      else {
	  angle = M_PI/2;
	  d *= -1;
      }
  }
  else {             // doors orientation is horizontal
      if(w < 0) {  // door drawn in negative x-direction
	  angle = M_PI;
	  d *= -1;
      }
      else angle = 0;      
  }
  insert(new t_door(dummyTag,w,d,DEF_H,apx,apy,DEF_Z,angle))->expose();    
}

void
t_obstgrid::new_circle(float x1, float y1, float r)
{
  insert(new t_circle(dummyTag,x1,y1,DEF_Z,r,DEF_H))->expose();
}

void
t_obstgrid::new_human(float x1, float y1, float s, float a)
{
  insert(new t_human(dummyTag,x1,y1,s,a))->expose();
}

Boolean t_obstgrid::distance(float xr, float yr, float zr,
			     float open_angle,
			     float xe, float ye,
			     float *dist, float *angle, t_surface **surface)
{
  float tangle,tmp;
  Boolean Hit = FALSE, C_Hit;
  t_obst *c_obstacle;
  t_surface *tsurface;
  fill_ws_ray(xr,yr,xe,ye); 
  while((c_obstacle = ws->traverse())) {
    C_Hit = FALSE;
    C_Hit = c_obstacle->distance(xr,yr,zr, open_angle,
				 xe,ye,&tmp,&tangle,&tsurface);
    if(C_Hit) {
	tangle = fabs(tangle);
	if(tangle > M_PI) tangle -= M_PI;    
	if(Hit) {
	    if(tmp < *dist) {
		*dist = tmp;
		*angle = tangle;
		*surface = tsurface;
	    }
	}
	else {
	    Hit = TRUE;
	    *dist = tmp;
	    *angle = tangle;
	    *surface = tsurface;
	}
    }
  }
  return Hit;
}

float t_obstgrid::min_distance(float x, float y)
{
  float tmp, dist = MAXFLOAT;
  t_obst *c_obstacle;
  fill_ws(x,y,500);

  while((c_obstacle = ws->traverse())) {
    tmp = c_obstacle->min_distance(x, y);
    if(tmp < dist) { 
      dist = tmp;
     }
  }

  return dist;
}

void t_obstgrid::color_ws(char *color)
{
  t_obst *c_obstacle;
  while((c_obstacle = ws->traverse())) {
    c_obstacle->new_color(color);
  }
}





