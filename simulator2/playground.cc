
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** File:                        playground.cc
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
 ***** $Source: /usr/local/cvs/bee/src/simulator2/playground.cc,v $
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
 * $Log: playground.cc,v $
 * Revision 1.1  2002/09/14 16:11:07  rstone
 * *** empty log message ***
 *
 * Revision 1.33  1999/08/26 16:53:54  schulz
 * -- The robot position can now be specified on the command line.
 * -- SIMULATOR send the robot's global position to modules, which
 *    have registered for it (Not tested yet!).
 *
 * Revision 1.32  1999/08/24 12:50:03  schulz
 * Added one old style cast NULL -> t_obst*
 *
 * Revision 1.31  1997/04/07 10:00:12  schulz
 * some changes in saving/loading playgrounds
 *
 * Revision 1.30  1997/03/26 13:04:28  schulz
 * fixed a bug, we have to invalidate absolute_points after obstacles have
 * been moved
 *
 * Revision 1.29  1997/03/11 17:16:43  tyson
 * added IR simulation and other work
 *
 * Revision 1.28  1997/03/11 11:02:19  schulz
 * Removed the distinction between RECTANGLEs and CUBEs  when
 * saving the playground. We always save CUBEs now.
 * Dito for CIRCLE <-> CYLINDER
 *
 * Revision 1.27  1997/03/05 14:37:36  thrun
 * Environment was enlarged, changed the definition of RAND_MAX.
 *
 * Revision 1.26  1997/02/06 12:39:08  schulz
 * Nothing special
 *
 * Revision 1.25  1997/02/06 09:52:55  schulz
 * - now we are leaving some room around the map after saving.
 *
 * Revision 1.24  1997/01/27 15:13:29  schulz
 * added added multiple sensor planes to the sensor simulation stuff
 * I hope this does not introduce to many new bugs.
 *
 * Revision 1.23  1997/01/10 15:19:33  schulz
 * minor bug fixes
 *
 * Revision 1.22  1997/01/07 10:43:47  schulz
 * cleaned up changes in laser.c
 * reoved out commented code from playground.cc
 *
 * Revision 1.21  1997/01/06 10:51:51  schulz
 * -- (0,0) is really in the lower left corner now!
 * -- sonar simulation works again.
 *
 * Revision 1.20  1996/12/23 10:47:15  fox
 * Changed the define for COUNTS_PER_CM. The base.c now uses the define
 * for BASE_COUNTS_PER_CM from rwibase_interface.h.
 * Communication with COLLI and baseServer should work consistently.
 *
 * Revision 1.19  1996/12/13 12:19:53  schulz
 * added code to adjust the playground to the positive quadrant before
 * saving
 *
 * Revision 1.18  1996/12/10 14:13:26  schulz
 * changed char consts rhino ->bee BASE -> COLLI
 *
 * Revision 1.17  1996/12/10 13:43:59  schulz
 * -- added -tcx option
 * -- loading of maps now possible
 * -- file requester has bigger width now.
 *    The translation for the return key has still to be changed!
 *
 * Revision 1.16  1996/12/09 13:30:32  schulz
 * -- added orintation to rectangles
 * -- additional minor fixes
 *
 * Revision 1.15  1996/12/03 15:18:33  schulz
 * -- some fixes
 * -- added humans as obstacles (still buggy)
 *
 * Revision 1.14  1996/12/02 17:01:55  schulz
 * new user interface for obstacle insertion
 * some fixes
 *
 * Revision 1.13  1996/11/15 12:21:23  ws
 * Changed InsideObstacle to use the obstacle grid.          DS
 *
 * Revision 1.12  1996/11/14 16:27:22  ws
 * *** empty log message ***
 *
 * Revision 1.11  1996/11/14 13:43:32  ws
 * Obstacles are solid now!             DS
 *
 * Revision 1.10  1996/10/29 16:05:11  ws
 * changed some class names
 *
 * Revision 1.9  1996/10/23 08:57:16  ws
 * eliminated cross file references to struct robot
 *
 * Revision 1.8  1996/10/18 13:59:59  ws
 * obstacles can be toggled on/off now using the right mouse button.
 *
 * Revision 1.7  1996/10/15 13:58:53  ws
 * - new features: deletion of obstacles (Shift+ right mouse button)
 *               usage information (-h or -help option)
 * - added -laser option
 * - Now the simulator waits for BASE to come up before it starts
 *   sending laserReports.
 *
 * Revision 1.6  1996/10/14 12:19:10  ws
 * added support for different surfaces (not yet complete) and sonar error
 *
 * Revision 1.5  1996/10/09 13:27:40  schulz
 * reorganized obstacle store and fixed some more bugs.
 *
// Revision 1.4  1996/10/07  14:50:49  schulz
// - corrected an initialization error (on empty maps)
// - eliminated an endless loop in store.cc
//
 * Revision 1.3  1996/10/07 14:08:09  schulz
 * robots initial position is read from map file now.
 *
 * Revision 1.2  1996/10/04 11:35:24  ws
 * some bug fixes in zooming and centering the robot
 *
 * Revision 1.1.1.1  1996/09/30 16:17:45  schulz
 * reimport on new source tree
 *
 * Revision 1.3  1996/09/20 07:56:21  schulz
 * Added saving of maps. E.g. a filename requester and a save method
 * for each obstacle type.
 *
 * Revision 1.2  1996/09/11 14:03:49  schulz
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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "playground.hh"
#include "obstacles.hh"
#include "surface.hh"
#include "store.hh"
#include "robot.h"
#include "simxw.h"

t_obstgrid* obstore;
t_playground* map;
extern t_surface *default_surface;

#define POSSIBLE_FILES 3 
char *p_filename[] = {"sim.map","test.map","default.map"};


/*
 * This defines the size of the environment. Increase, if you need larger
 * maps.
 */
#define BORDER_SPAREROOM 2500.0


t_playground::t_playground(char *fname, float robotX, float robotY, float robotOri)
{
  initialize(fname);
  setRobotPosition(robotX-x1, robotY-y1, robotOri);  
}

t_playground::t_playground(char *fname)
{
  initialize(fname);
}

void
t_playground::initialize(char *fname)
{
    int i;
    float xin,yin,ori_in;
    char def_mapfile[80];
  FILE* mapfd = NULL;
  enabled = FALSE;

  x1 = 0;                           // default playground bounds
  y1 = 0;
  x2 = 0;
  y2 = 0;

  xin = 0;                       // default robot position
  yin = 0;
  ori_in = 0;
                                    // mapfile specified on commandline
  if(strlen(fname) != 0) {        
      mapfd = fopen(fname, "r");
                                    // not in pwd look in data dir.
      if(!mapfd) {                
	  sprintf(def_mapfile, "%s/bee/data/simulator/%s",
		  getenv("HOME"),fname);
	  mapfd = fopen(def_mapfile, "r");
	  if(mapfd) strcpy(filename, def_mapfile);
      }
      else strcpy(filename,fname);
      if(!mapfd) fprintf(stderr, "Could not find mapfile: %s\n", fname);
  }
				// no filename given, try default maps
  else {
				// try pwd first
    for(i = 0; i < POSSIBLE_FILES; i++) {
      if((mapfd = fopen(p_filename[i], "r")))
	  {   strcpy(filename, p_filename[i]);
	      printf("Loaded default map %s\n",filename);
	      break;
	  }
    }
				// not in pwd try data directory
    if(!mapfd) {
	for(i = 0; i < POSSIBLE_FILES; i++) {
	    sprintf(def_mapfile, "%s/bee/data/simulator/%s",
		    getenv("HOME"), p_filename[i]);
	    if((mapfd = fopen(def_mapfile, "r")) != 0) {
		strcpy(filename, def_mapfile);
		printf("Loaded default map %s\n",filename);
		break;
	    }
	}
    }
				// still not found, print files tried
    if(!mapfd) {
	fprintf(stderr, "Simulator: Could not find any map!\n");
	fprintf(stderr, "I tried\n");
	for(i = 0; i < POSSIBLE_FILES; i++) {
	    fprintf(stderr, "\t%s\n", p_filename[i]);
	}      
	for(i = 0; i < POSSIBLE_FILES; i++) {
	    fprintf(stderr, "\t%s/bee/data/simulator/%s\n",
		    getenv("HOME"),p_filename[i]);
	}      
    }
  }
				// try to start with map
  if(mapfd) {
      i = fscanf(mapfd, "MAP %f %f %f %f\n", &x1, &y1, &x2, &y2);
      if( i < 4) {
	  fprintf(stderr, "Simulator: Mapfile corrupted!\n");
	  exit(0);
      }
      i = fscanf(mapfd, "ROBOT %f %f %f\n", &xin, &yin,
                 &ori_in);
      if( i < 3) {
	  fprintf(stderr, "Simulator: Mapfile corrupted!\n");
	  exit(0);
      }
      x1 -= BORDER_SPAREROOM;
      y1 -= BORDER_SPAREROOM;
      y2 += BORDER_SPAREROOM;
      x2 += BORDER_SPAREROOM;
      obstore = new t_obstgrid(x1, y1, x2, y2);  
      obstore->read_obstacles(mapfd);
      fclose(mapfd);
  }
				// start without map
  else {
      x1 -= BORDER_SPAREROOM;
      y1 -= BORDER_SPAREROOM;
      y2 += BORDER_SPAREROOM;
      x2 += BORDER_SPAREROOM;      
      obstore = new t_obstgrid(x1,y1,x2,y2);  
  }
  InitPlayground();
  setRobotPosition(xin-x1, yin-y1, ori_in);
  printf("Number of Obstacles: %d\n",obstore->n_obstacles);
  return;
}

void t_playground::InitPlayground()
{
    activated_obstacle = NULL;
  InitPlaygroundGraphics(filename,x1,y1,x2,y2);
  obstore->ExposeObstacles();
  return;
}

Boolean SavePlayground(char *fname)
{
    map->_SavePlayground(fname);
}


Boolean t_playground::_SavePlayground(char *fname)
{
    FILE *fd;
    float rx,ry,rori;
    float nx1,ny1,nx2,ny2;
    getRobotPosition(&rx,&ry,&rori);
    rx = playground_x(rx);
    ry = playground_y(ry);
    strcpy(filename, fname);
    obstore->move(0.0,0.0);              // force update of bounding box 
    obstore->bounds(&nx1, &ny1, &nx2, &ny2);
    if(nx1 > rx) nx1 = rx;
    if(ny1 > ry) ny1 = ry;
    fprintf(stderr, "bounds are: %g %g %g %g\n",nx1,ny1,nx2,ny2);
    obstore->move(-nx1,-ny1);
    rx = rx - nx1;
    ry = ry - ny1;    
    obstore->bounds(&nx1, &ny1, &nx2, &ny2);
    if((fd = fopen(filename, "w"))) {
	fprintf(fd, "MAP %f %f %f %f\n",
		nx1,
		ny1,
		nx2,
		ny2);
	fprintf(fd, "ROBOT %f %f %f\n",
		rx, ry, rori);
	obstore->SaveObstacles(fd);
	fclose(fd);
	_LoadPlayground(fname);
	return TRUE;
    }
    return FALSE;
}

void LoadPlayground(char *fname)
{
    map->_LoadPlayground(fname);
}

void
t_playground::_LoadPlayground(char *fname)
{
    int i;
    float xin = 0, yin = 0, ori_in = 0;
    t_obstgrid *obstore2;
    FILE *mapfd = fopen(fname, "r");
    x1 = 0;                           // default playground bounds
    y1 = 0;
    x2 = 2000;
    y2 = 2000;
    if(!mapfd) {
	printf("Could not open mapfile, keeping old map\n");
	return;
    }
    strcpy(filename, fname);
    fprintf(stderr,"opened mapfile %s\n",fname);
    i = fscanf(mapfd, "MAP %f %f %f %f\n", &x1, &y1, &x2, &y2);
    if( i < 4) {
	fprintf(stderr, "Simulator: Mapfile corrupted, keeping old map\n");
	return;
    }
    i = fscanf(mapfd, "ROBOT %f %f %f\n", &xin, &yin, &ori_in);
    if( i < 3) {
	fprintf(stderr, "Simulator: Mapfile corrupted, keeping old map\n");
	return;
    }
    x1 -= BORDER_SPAREROOM;
    y1 -= BORDER_SPAREROOM;
    y2 += BORDER_SPAREROOM;
    x2 += BORDER_SPAREROOM;
    obstore2 = new t_obstgrid(x1,y1,x2,y2);  
    if(!obstore2->read_obstacles(mapfd)) {
	delete obstore2;
	fprintf(stderr, "Simulator: Mapfile corrupted, keeping old map\n");
	return;
    };
    fclose(mapfd);
    delete obstore;
    obstore = obstore2;
    InitPlayground();
    setRobotPosition(xin-x1, yin-y1, ori_in);
    DrawRobot(xin, yin);
    printf("Number of Obstacles: %d\n",obstore->n_obstacles);
}    

Boolean t_playground::_get_distance(float PosX, float PosY, float PosZ,
				    float open_angle,
				    float EndX, float EndY, 
				    float *dist, float *angle, t_surface **surface)
{
    t_obst* obst;
    return obstore->distance(playground_x(PosX),
			     playground_y(PosY),
			     PosZ,
			     open_angle,
			     playground_x(EndX), 
			     playground_y(EndY),
			     dist, angle, surface);
}

void t_playground::set_active_obstacle(t_obst* obst)
{
    activated_obstacle = obst;
}
t_obst* t_playground::get_active_obstacle()
{
    return activated_obstacle;
}

Boolean InsideObstacle(float x, float y)
{
  if(obstore->InsideObstacle(x,y)) return TRUE;
  else return FALSE;
}

Boolean InsideAnyObstacle(float x, float y)
{
  if(obstore->InsideAnyObstacle(x,y)) return TRUE;
  else return FALSE;
}

void
RemoveObstacle(float x, float y)
{
    t_obst* obst = obstore->InsideAnyObstacle(x,y);
    if(obst) { 
	obstore->RemoveObstacle(obst);    
    }
}

void
ToggleObstacle(float x, float y)
{
    t_obst* obst = obstore->InsideAnyObstacle(x,y);
    if(obst) 
	obst->toggle();    
}

void
ObstaclePushAction(float x, float y)
{
    t_obst* obst;
    map->set_active_obstacle((t_obst*)NULL);
    if((obst = obstore->InsideAnyObstacle(x,y))) {
	map->set_active_obstacle(obst);
	obst->mouse_action(x, y);
	return;
    }
}
void
ObstacleDragAction(float x, float y)
{
    t_obst* obst;
    if((obst = map->get_active_obstacle())) {
	obst->mouse_action(x,y);
    }
}

void new_rectangle(float x1, float y1, float x2, float y2)
{
    obstore->new_rectangle(x1,y1,x2,y2);
}
void new_door(float x1, float y1, float x2, float y2)
{
    obstore->new_door(x1,y1,x2,y2);
}
void new_circle(float x1, float y1, float r)
{
    obstore->new_circle(x1,y1,r);
}
void new_human(float x1, float y1, float s, float a)
{
    obstore->new_human(x1,y1,s,a);
}

float obstacles_min_distance(float x, float y)
{
    return obstore->min_distance(x,y);
}

Boolean get_distance(int sensor_type,
		     float PosX, float PosY,
		     float PosZ,
		     float open_angle,
		     float EndX, float EndY, float *dist)
{
    float angle;
    t_surface *surface;
    float tdist;
    Boolean Hit = map->_get_distance(PosX, PosY, PosZ,
				     open_angle,
				     EndX, EndY, &tdist,
				     &angle, &surface);
    if(!Hit) return FALSE;
    *dist = surface->add_error(sensor_type, tdist, angle);
    return TRUE;
}

Boolean map_enabled()
{
    return map->enabled;
}

float playground_x(float xin)
{
    return xin+map->x1;
}
float playground_y(float yin)
{
    return yin+map->y1;
}
float base_x(float xin)
{
    return xin-map->x1;
}
float base_y(float yin)
{
    return yin-map->y1;
}
