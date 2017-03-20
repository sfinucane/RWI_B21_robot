
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** Welcome!
 *****
 ***** This file is part of the robot control software provided
 ***** by Real World Interface Inc.
 *****
 ***** All copyrights are by Real World Interface Inc., Carnegie
 ***** Mellon University, and the University of Bonn, Germany.
 ***** Removing the copyright information is illegal. Please read
 ***** and make sure you understand the disclaimer below.
 *****
 ***** Contact tyson@rwii.com if you have questions or would
 ***** like to obtain further information.
 *****
 ***** We hope you enjoy working with this software package.
 *****
 *****                      Tyson D. Sawyer and Sebastian Thrun
 *****
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** 
 ***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED
 ***** BY APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING
 ***** THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM
 ***** "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR
 ***** IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 ***** OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE
 ***** ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM
 ***** IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME
 ***** THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
 ***** 
 ***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
 ***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
 ***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
 ***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
 ***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
 ***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO
 ***** LOSS OF DATA OR DATA BEING RENDERED INACCURATE OR LOSSES
 ***** SUSTAINED BY YOU OR THIRD PARTIES OR A FAILURE OF THE PROGRAM
 ***** TO OPERATE WITH ANY OTHER PROGRAMS OR FAILURE TO CONTROL A
 ***** PHYSICAL DEVICE OF ANY TYPE), EVEN IF SUCH HOLDER OR OTHER
 ***** PARTY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/makefloor.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 17:06:29 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: makefloor.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1996/12/02 10:32:08  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.1.1.1  1996/09/22 16:46:31  rhino
 * General reorganization of the directories/repository, fusion with the
 * RWI software.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


/* Generate part of university building. All units given in cm.
   Origin is lower left corner */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

const int resolution = 15;            /* for computation as well as map-file */
const int radius = 28;                 /* for specification in map-file only */

#define XSIZE 149
#define YSIZE 123

#define CLEAR   1
#define SET     0
#define UNKNOWN -1

float map[XSIZE][YSIZE];

FILE *mfile;


set_grid(int x, int y) {
/* WARNING: not in cm! */
  if ( (x>=0) && (x<XSIZE) && (y>=0) && (y<YSIZE) )
    map[x][y] = SET;
  else
    printf("WARNING: grid field (%i,%i) is not in (0,0)-(%i,%i)!\n",
	   x,y,XSIZE,YSIZE);
}


clear_grid(int x, int y) {
/* WARNING: not in cm! */
  if ( (x>=0) && (x<XSIZE) && (y>=0) && (y<YSIZE))
    map[x][y] = CLEAR;
  else
    printf("WARNING: grid field (%i,%i) is not in (0,0)-(%i,%i)!\n",
	   x,y,XSIZE,YSIZE);
}




set_hline(int x1, int x2, int y) {
int x;
  x1 /= resolution;
  x2 /= resolution;
  y /= resolution;
  for(x=x1; x<=x2; x++) set_grid(x,y);
}


clear_hline(int x1, int x2, int y) {
int x;
  x1 /= resolution;
  x2 /= resolution;
  y /= resolution;
  for(x=x1; x<=x2; x++) clear_grid(x,y);
}


set_vline(int x, int y1, int y2) {
int y;
  x /= resolution;
  y1 /= resolution;
  y2 /= resolution;
  for(y=y1; y<=y2; y++) set_grid(x,y);
}


clear_vline(int x, int y1, int y2) {
int y;
  x /= resolution;
  y1 /= resolution;
  y2 /= resolution;
  for(y=y1; y<=y2; y++) clear_grid(x,y);
}




set_block(int x1, int y1, int x2, int y2) {
int x,y;
  x1 /= resolution;
  x2 /= resolution;
  y1 /= resolution;
  y2 /= resolution;
  for(y=y1; y<=y2; y++) for(x=x1; x<=x2; x++) set_grid(x,y);
}


clear_block(int x1, int y1, int x2, int y2) {
int x,y;
  x1 /= resolution;
  x2 /= resolution;
  y1 /= resolution;
  y2 /= resolution;
  for(y=y1; y<=y2; y++) for(x=x1; x<=x2; x++) clear_grid(x,y);
}


set_rot_block(int x1, int y1, int dx, int dy, int alpha) {
/* this is different: second parameter pair is size, not coordinate! */
int x, y, tx, ty;
float rx,ry, step = resolution/2.0,
      alpha_x = alpha * M_PI / 180.0,
      alpha_y = (alpha+90.0) * M_PI / 180.0;
  for (rx=0.0; rx<=dx; rx += step) {
    x=cos(alpha_x)*rx + x1;
    y=sin(alpha_x)*rx + y1;
    for (ry=0.0; ry<=dy; ry += step) {
      tx=cos(alpha_y)*ry + x;
      ty=sin(alpha_y)*ry + y;
      set_grid(tx/resolution,ty/resolution);
    }
  }
}  



/*
 * composed elements
 */

put_room(int x1, int y1, int x2, int y2) {
/* A room is defined by upper left and lower right coordinate. It has a
   'wall' outline and containes 'clear' area. Openings like doors must be
   set later.
 */ 
  clear_block(x1,y1,x2,y2);
  set_hline(x1,x2,y1);
  set_hline(x1,x2,y2);
  set_vline(x1,y1,y2);
  set_vline(x2,y1,y2);
}


put_doorup(int x, int y, int state) {
/* state = 0: closed, state = 1: opened, else: unknown */
  clear_block(x+0,y+0,x+109,y+48);
  clear_block(x+7,y+48,x+102,y+65);
  set_vline(x+0,y+0,y+48);
  set_vline(x+109,y+0,y+48);
  set_hline(x+0,x+7,y+48);
  set_hline(x+102,x+109,y+48);
  set_vline(x+7,y+48,y+65);
  set_vline(x+102,y+48,y+65);
  if (state == 1)
    set_block(x+7,y-47,x+9,y+48);
  else if (state == 0)
    set_block(x+7,y+46,x+102,y+48);
}


put_doordown(int x, int y, int state) {
/* state = 0: closed, state = 1: opened, else: unknown */
  clear_block(x+0,y-48,x+109,y-0);
  clear_block(x+7,y-65,x+102,y-48);
  set_vline(x+0,y-48,y-0);
  set_vline(x+109,y-48,y-0);
  set_hline(x+0,x+7,y-48);
  set_hline(x+102,x+109,y-48);
  set_vline(x+7,y-65,y-48);
  set_vline(x+102,y-65,y-48);
  if (state == 1)
    set_block(x+7,y-48,x+9,y+47);
  else if (state == 0)
    set_block(x+7,y-48,x+102,y-46);
}



/*
 * single rooms
 */


do_dieter(int x, int y) {
  put_room(x+0,y+0,x+395,y+695);
  /* left side */
  set_block(x+0,y+0,x+124,y+83);
  set_block(x+0,y+236,x+124,y+316);
  set_block(x+0,y+366,x+38,y+466);
  /* right side */
  set_block(x+285,y+32,x+385,y+67);
  set_block(x+310,y+111,x+395,y+271);
  set_block(x+250,y+271,x+395,y+361);
  set_block(x+265,y+351,x+395,y+392);
  /* tables at window */
  set_rot_block(x+360,y+655,80,160,150);
  set_rot_block(x+150,y+430,80,240,20);
}


do_sebastian(int x, int y) {
  put_room(x+0,y+0,x+395,y+695);
  /* right side (seen from door) */
  set_block(x+0,y+215,x+42,y+315);
  set_block(x+0,y+355,x+85,y+475);
  set_block(x+0,y+535,x+42,y+655);
  /* left side */
  set_block(x+350,y+0,x+395,y+180);
  set_block(x+315,y+215,x+395,y+335);
  /* tilted tables */
  set_rot_block(x+390,y+555,80,80,150);
  set_rot_block(x+200,y+10,160,80,70);
}


do_corridor(int x, int y) {
  put_room(x+0,y+0,x+2227,y+320);
  put_doorup(x+229,y+320,0);
  put_doordown(x+229,y+0,0);
  put_doorup(x+645,y+320,0);
  put_doordown(x+645,y+0,0);
  put_doorup(x+1067,y+320,0);
  put_doordown(x+1067,y+0,0);
  put_doorup(x+1483,y+320,1);
  put_doordown(x+1483,y+0,1);
  put_doorup(x+1904,y+320,0);
  put_doordown(x+1904,y+0,0);
}




main() {
int x,y;
  /*
   * generate map image in memory
   */
  for (y=0; y<YSIZE; y++) for (x=0; x<XSIZE; x++) map[x][y] = UNKNOWN;

  /*
   * fill map
   */
  do_dieter(1340,1145);
  do_sebastian(1340,0);
  do_corridor(0,760);         /* after the rooms because of connecting doors */


  /*
   * write map to file
   */
  if ((mfile=fopen("corridor.cad","w"))==NULL) {
    fprintf(stderr,"Cannot open corridorcad!\n");
    exit;
  }

  fprintf(mfile,"robot_specifications->global_mapsize_x                 %i\n",
          XSIZE*resolution);
  fprintf(mfile,"robot_specifications->global_mapsize_y                 %i\n",
          YSIZE*resolution);
  fprintf(mfile,"robot_specifications->resolution                       %i\n",
          resolution);
  fprintf(mfile,"robot_specifications->robot_size                       %i\n",
	  radius);
  fprintf(mfile,"maptype is hand-drawn ('CAD') map\n");
  fprintf(mfile,"global_map[0]: %i %i\n",XSIZE,YSIZE);
          
  for (y=0; y<YSIZE; y++) {
    for (x=0; x<XSIZE; x++) 
      fprintf(mfile,"%f ", map[x][YSIZE-y-1]);
    fprintf(mfile,"\n");
  }
  fclose(mfile);
  printf("FILE corridor.cad WRITTEN\n");
}
