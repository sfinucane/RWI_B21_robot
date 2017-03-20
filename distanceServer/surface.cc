// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/surface.cc,v $
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
//  $Log: surface.cc,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.2  1998/11/23 13:07:29  schulz
//  distanceServer is now also use by RTL. I made the
//  necessary changes, to bring the two modules into sync
//
//  Revision 1.1  1998/10/12 13:32:15  schulz
//  This is a complete new version
//
//  Revision 1.2  1998/09/30 15:07:20  schulz
//  Several changes to make things compile under gcc 2.7.2
//
//  Revision 1.1  1998/06/19 13:58:06  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.6  1997/10/20 08:29:19  schulz
//  latest version of  matchTest
//
//  Revision 1.5  1997/09/29 16:03:10  schulz
//  Made source files SUN C++ compatible
//
//  Revision 1.4  1997/03/05 16:29:52  schulz
//  removed obsolete files robot.cc robot.hh
//  added experimental support for MRTVR
//
//  Revision 1.3  1997/03/04 17:54:08  schulz
//  Added file header to *.h files
//  Removed some old files. This may lead to many further changes
//
//  Revision 1.2  1997/03/04 17:15:36  schulz
//  Added file header and revision history
//
//
// ----------------------------------------------------------------------------

#if defined(__GNUG__)
#include <typeinfo>
#endif

#include <stdlib.h>
#include <iostream.h>
#define _XOPEN_SOURCE
#include <math.h>
#undef _XOPEN_SOURCE
#include <string.h>

#include "sundefines.h"
#include "surface.hh"

#define SONAR_RANGE 500

t_surface *default_surface;
t_surflist *surfaces;

// returns a random number drawn to a normalized gaussian distribution
// that is zero mean and unit variance

static float
random_gauss()
{
    static int iset = 0;
    static float gset;
    float fac, rsq, v1, v2;
    if(iset == 0) {
	do {
	    v1 = 2.0 * rand()/(RAND_MAX+1.0) - 1.0;
	    v2 = 2.0 * rand()/(RAND_MAX+1.0) - 1.0;	    
	    rsq = v1*v1 + v2*v2;
	} while(rsq >= 1.0 || rsq == 0.0);
	fac = sqrt(-2.0*log(rsq)/rsq);
	gset = v1*fac;
	iset = 1;
	return v2*fac;
    }
    else {
	iset = 0;
	return gset;
    }
}

t_surface::t_surface(char *n, char *c)
{
  name = new char[strlen(n)+1];
  strcpy(name, n);
  color = new char[strlen(c)+1];
  strcpy(color, c);
  S_MIN_DIST = 5;
  S_MAX_DIST = SONAR_RANGE; 
  S_MINDIST_EANGLE = (60/180*M_PI);
  S_MAXDIST_EANGLE = (20/180*M_PI);
  S_ANGLERANGE = (30.0/180*M_PI);
}

t_surface::~t_surface()
{
  cout << "deleting surface" <<endl;
  delete [] name;
  delete [] color;  
}

void
t_surface::init_serror(float min_angle, float max_angle, float range)
{
    S_MINDIST_EANGLE = (min_angle/180*M_PI);
    S_MAXDIST_EANGLE = (max_angle/180*M_PI);
    S_ANGLERANGE = (range/180*M_PI);
}

float
t_surface::add_error(int sensor_type, float dist, float angle) const
{
    switch(sensor_type) {
    case SONAR_SENSOR: return sonar_error(dist, angle);
    case LASER_SENSOR: return  dist + random_gauss();
    default:    return dist;
    }
}

float
t_surface::ran() const
{
    float r;
    do {
	r = (float) rand() / RAND_MAX;
    } while(r == 0.0);
    return r;
}

float
t_surface::sonar_error(float dist, float angle) const
{
    float errorStartAngle;
    float errorProb;
    angle = 90 - angle;
    angle *= M_PI/180;
    if(dist < S_MIN_DIST)	// No response if distance < 5cm
	return SONAR_RANGE;
    errorStartAngle =
	S_MINDIST_EANGLE
	- (S_MINDIST_EANGLE-S_MAXDIST_EANGLE)
	/ (S_MAX_DIST-S_MIN_DIST)
	* (dist - S_MIN_DIST);
    if(angle < errorStartAngle) // correct measurement with certainty
	return dist;
				// error prob. increases to certainty
				// within 20 deg (linear approx.)
    errorProb =
	(angle - errorStartAngle)
	/ S_ANGLERANGE;
    if(errorProb > 1) errorProb = 1;
    if(errorProb > ran()) return SONAR_RANGE;
    else return dist;    
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

void
new_surf_entry(char *n, char *c,
	       float min_angle,
	       float max_angle,
	       float angle_range)
{
  surfaces->new_entry(n, c, min_angle, max_angle, angle_range);
}

void
t_surflist::new_entry(char *n, char *c,
	       float min_angle,
	       float max_angle,
	       float angle_range)
{
  t_surflist_entry *entry = new t_surflist_entry();
  if(!first) {
    first = last = entry;
  }
  else {
    last->next = entry;
    last = entry;
  }
  entry->surface = new t_surface(n, c);
  entry->surface->init_serror(min_angle, max_angle, angle_range);
  entry->next = NULL;
  if(first && (default_surface != first->surface) ) {
    delete default_surface;
    default_surface = first->surface;
  }
}

t_surflist::t_surflist()
{
  first = NULL;
  last = NULL;
  
  default_surface = new t_surface("default","grey80");
  default_surface->init_serror(60, 20, 30); 
}

t_surflist::~t_surflist()
{
  /*
  t_surflist_entry *tmp, *trav = first;
  while(trav) {
    tmp = trav;
    trav = trav->next;
    if(tmp->surface == default_surface)
      default_surface = NULL;
    delete tmp->surface;
    delete tmp;
  }
  */
  delete default_surface;
}

t_surflist_entry::t_surflist_entry()
{
  surface= NULL;
  next = NULL;
}
t_surflist_entry::~t_surflist_entry()
{}
