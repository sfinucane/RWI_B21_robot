

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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/detection-old2/graphics.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:04 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: graphics.c,v $
 * Revision 1.1  2002/09/14 20:45:04  rstone
 * *** empty log message ***
 *
 * Revision 1.17  1998/09/05 00:25:27  fox
 * Works fine to detect people in the museum.
 *
 * Revision 1.16  1998/08/29 21:44:43  fox
 * Changed the detection report and improved stuck detection.
 *
 * Revision 1.15  1998/08/23 22:57:40  fox
 * First version of building maps of humans.
 *
 * Revision 1.14  1997/06/03 11:49:15  fox
 * Museum version.
 *
 * Revision 1.13  1997/05/28 14:40:01  thrun
 * .
 *
 * Revision 1.12  1997/05/28 14:04:10  fox
 * Fixed a bug.
 *
 * Revision 1.11  1997/05/28 09:15:55  thrun
 * Delay in graphical update.
 *
 * Revision 1.10  1997/05/28 09:12:16  thrun
 * .
 *
 * Revision 1.9  1997/05/28 09:09:36  thrun
 * .
 *
 * Revision 1.8  1997/05/28 09:01:29  wolfram
 * added motion-only mode
 *
 * Revision 1.7  1997/05/28 07:17:56  fox
 * is egal.
 *
 * Revision 1.6  1997/05/25 10:43:10  thrun
 * nothing.
 *
 * Revision 1.5  1997/05/25 10:40:52  fox
 * Nothing special.
 *
 * Revision 1.4  1997/05/25 10:39:12  thrun
 * test.
 *
 * Revision 1.3  1997/05/09 16:28:39  fox
 * Works quiet fine.
 *
 * Revision 1.2  1997/05/06 14:22:58  fox
 * Nothing special.
 *
 * Revision 1.1  1997/05/05 16:54:06  fox
 * Incorporated graphics from people.c.
 *
 * Revision 1.5  1997/04/28 00:12:46  thrun
 * Latest version, some smarter decision making (it's getting longer, though)
 *
 * Revision 1.4  1997/04/27 18:14:11  thrun
 * pantilt tracking (not yet tested)
 *
 * Revision 1.3  1997/04/27 16:33:31  thrun
 * new version. Decide has now its own file.
 *
 * Revision 1.2  1997/04/27 14:48:45  thrun
 * minor changes.
 *
 * Revision 1.1.1.1  1997/04/27 11:43:47  thrun
 * New program for the detection of people. Connects to colliServer and
 * detection, and analyzes data from both. The current version is not
 * yet functional (but nicely displays some data).
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/





#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include <signal.h>
#include "tcx.h"
#include "tcxP.h"
#include "Application.h"
#include "Net.h"
#include "MAP-messages.h"
#include "SONAR-messages.h"
#include "BASE-messages.h"
#include "LASER-messages.h"
#include "SOUND-messages.h"
#include "MOUTH-messages.h"
#include "DETECTION-messages.h"
#include "PANTILT-messages.h"
#include "EZX11.h"
#include "o-graphics.h"
#include "general.h"
#include "detection.h"
#include "graphics.h"
#include "human.h"
#include "function.h"
#include "allocate.h"

#ifdef RHINO_PLUS
#include "rst.h" /* rhino stuff */
#endif

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* BUTTONS and other graphic objects */

#define COL_UNKNOWN     C_LAWNGREEN           /* color of unknown area      */
#define COL_RED         C_RED
#define COL_BLACK       54
#define COL_WHITE       154

int LOCAL_ROBOT;
int LOCAL_DETECTION;
int QUIT_BUTTON;
int MODE_BUTTON;
int PERSON_BUTTON;
int BASE_CONNECTED_BUTTON;
int LOCALIZE_CONNECTED_BUTTON;
int SOUND_CONNECTED_BUTTON;
int MOUTH_CONNECTED_BUTTON;
int PANTILT_CONNECTED_BUTTON;
int PERSON;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

static int
fieldColor( float f);

void usleep(unsigned long usec);


/************************************************************************
 *
 *   NAME:         init_graphics
 *                 
 *   FUNCTION:     Initiaizes graphics window
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR  program_state  Pointer  to general
 *                                                   program state description
 *                                                   structure 
 *                 ROBOT_SPECIFICATIONS_PTR robot_specifications
 *                                                   Pointer to robot spe-
 *                                                   cifications (see above)
 *                                                   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


#define LOCAL_WINDOW_SIZE 2.0
#define RIGHT_BAR  6.1, (6.1+LOCAL_WINDOW_SIZE)


void 
initGraphics( ROBOT_STATE_PTR    robot_state,
	      PROGRAM_STATE_PTR  program_state,
	      ROBOT_SPECIFICATIONS_PTR robot_specifications)
{

  static char *myfonts[] = {"5x8", "6x10", "7x13bold", 
			      "9x15", "10x20", "12x24", "lucidasans-bold-24"};

  if (!program_state->use_graphics)
    return;

  /* =============================================================
     ====================  1) INITIALIZATIONS  ===================
     ============================================================= */
  
  if (!program_state->graphics_initialized){
    
    G_initialize_fonts(7, myfonts);
    G_initialize_graphics("BeeSoft People Detector", 80.0, 10.0, C_SLATEGREY);
    
    
    
    /* =============================================================
       ====================  2) SPECIFICATIONS  ====================
       ============================================================= */

    {
      
      /******** QUIT_BUTTON ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_BAR, 5.7, 6.0};
      static char *switch_texts[]         = {"quit", "quit"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_WHITE, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_WHITE};
      QUIT_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    
    {
      
      /******** PERSON_BUTTON ****************************/
      int switch_num                      = 4;
      static float switch_pos[]           = {RIGHT_BAR, 5.3, 5.6};
      static char *switch_texts[]         = 
	{"(nobody there)", "? ? ?", "(waiting ...)", "P E R S O N"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]=
	{C_SLATEGREY, C_WHITE, C_YELLOW, C_LAWNGREEN};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_GREY40, C_BLACK, C_BLACK, C_BLACK};
      PERSON_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    
    {
      
      /******** MODE_BUTTON ****************************/
      int switch_num                      = 3;
      static float switch_pos[]           = {RIGHT_BAR, 4.9, 5.2};
      static char *switch_texts[]         = {"MOTION", "UNEXPECTED", "STOP"};
      static int switch_fonts[]           = {2,2,2};
      static int switch_background_color[]=
	{C_YELLOW, C_LAWNGREEN, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_GREY40, C_BLACK, C_BLACK};
      MODE_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    
    {      
      /******** BASE_CONNECTED_BUTTON ****************************/
      int switch_num                      = 3;
      static float switch_pos[]           = {RIGHT_BAR, 4.7, 4.9};
      static char *switch_texts[]         = {"(COLLI not connected)",
					       " (COLLI connected) ",
					       "...trying to connect"};
      static int switch_fonts[]           = {1,1,1};
      static int switch_background_color[]= 
	{C_SLATEGREY, C_SLATEGREY, C_SLATEGREY};
      static int switch_frame_color[]     = {NO_COLOR, NO_COLOR, NO_COLOR};
      static int switch_text_color[]      = {C_WHITE, C_LAWNGREEN, C_RED};
      BASE_CONNECTED_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    
    {
      /******** LOCALIZE_CONNECTED_BUTTON ****************************/
      int switch_num                      = 3;
      static float switch_pos[]           = {RIGHT_BAR, 4.5, 4.7};
      static char *switch_texts[]         = {"(LOCALIZE not connected)",
					       " (LOCALIZE connected) ",
					       "...trying to connect"};
      static int switch_fonts[]           = {1,1,1};
      static int switch_background_color[]=
	{C_SLATEGREY, C_SLATEGREY, C_SLATEGREY};
      static int switch_frame_color[]     = {NO_COLOR, NO_COLOR, NO_COLOR};
      static int switch_text_color[]      = {C_WHITE, C_LAWNGREEN, C_RED};
      LOCALIZE_CONNECTED_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    
    {
      /******** SOUND_CONNECTED_BUTTON ****************************/
      int switch_num                      = 3;
      static float switch_pos[]           = {RIGHT_BAR, 4.3, 4.5};
      static char *switch_texts[]         = {"(SOUND not connected)",
					       " (SOUND connected) ",
					       "...trying to connect"};
      static int switch_fonts[]           = {1,1,1};
      static int switch_background_color[]=
	{C_SLATEGREY, C_SLATEGREY, C_SLATEGREY};
      static int switch_frame_color[]     = {NO_COLOR, NO_COLOR, NO_COLOR};
      static int switch_text_color[]      = {C_WHITE, C_LAWNGREEN, C_RED};
      SOUND_CONNECTED_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    
    {
      /******** PANTILT_CONNECTED_BUTTON ****************************/
      int switch_num                      = 3;
      static float switch_pos[]           = {RIGHT_BAR, 4.1, 4.3};
      static char *switch_texts[]         = {"(PANTILT not connected)",
					       " (PANTILT connected) ",
					       "...trying to connect"};
      static int switch_fonts[]           = {1,1,1};
      static int switch_background_color[]=
	{C_SLATEGREY, C_SLATEGREY, C_SLATEGREY};
      static int switch_frame_color[]     = {NO_COLOR, NO_COLOR, NO_COLOR};
      static int switch_text_color[]      = {C_WHITE, C_LAWNGREEN, C_RED};
      PANTILT_CONNECTED_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    
    
    {

      /******** MOUTH_CONNECTED_BUTTON ****************************/
      int switch_num                      = 3;
      static float switch_pos[]           = {RIGHT_BAR, 4.3, 4.5};
      static char *switch_texts[]         = {"(MOUTH not connected)",
					       " (MOUTH connected) ",
					       "...trying to connect"};
      static int switch_fonts[]           = {1,1,1};
      static int switch_background_color[]=
	{C_SLATEGREY, C_SLATEGREY, C_SLATEGREY};
      static int switch_frame_color[]     = {NO_COLOR, NO_COLOR, NO_COLOR};
      static int switch_text_color[]      = {C_WHITE, C_LAWNGREEN, C_RED};
      MOUTH_CONNECTED_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    
    {
      /******** DETECTION IN LOCAL MAP *********************************/ 
      static float pos_r[]                 = {4.0, 5.9, 4.1, 6.0};
      static char *text_r                  = "local";
      static int robot_font                = 3;
      static int colors_r[]                = {NO_COLOR, C_GREY40, C_RED,
						C_GREY50, C_VIOLET, C_BLACK, 
						C_GREY40, NO_COLOR};

      
      
      LOCAL_DETECTION =
	G_create_robot_object(pos_r, text_r, 
			      -robot_specifications->max_detection_range*1.01,
			      robot_specifications->max_detection_range*1.01,
			      -robot_specifications->max_detection_range*1.01,
			      robot_specifications->max_detection_range*1.01,
			      0.0, 0.0, 90.0,
			      robot_specifications->robot_size, 
			      robot_specifications->num_detections, 
			      robot_specifications->max_detection_range, 
			      robot_state->detection_values, 
			      robot_specifications->detection_angles,
			      colors_r, robot_font);
      
    }


    /******** PERSON *********************************************/
    {
      static float pos_mar[]                   = {4.0, 5.9, 4.1, 6.0};
      int num_mar                            = 1;
      static char *text_mar[]                = {"PEOPLE"};
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = {C_BLACK};
      static int mar_text_color[]            = {NO_COLOR};
      static int mar_fonts[]                 = {2};
      
      PERSON
	= G_create_markers_object(pos_mar, 1,
				  robot_specifications->robot_size * 0.4,
				  -robot_specifications->max_detection_range*1.01,
				  robot_specifications->max_detection_range*1.01,
				  -robot_specifications->max_detection_range*1.01,
				  robot_specifications->max_detection_range*1.01,
				  num_mar, text_mar, mar_background_color, 
				  mar_frame_color, mar_foreground_color, 
				  mar_text_color, mar_fonts);
      
    }
  
    
    /* insert new graphics object here XXXXX */

  }
    
  
  
  /* =============================================================
     ====================  4) DISPLAY  ===========================
     ============================================================= */


  G_display_all();
  G_display_switch(LOCALIZE_CONNECTED_BUTTON, program_state->localize_connected);
  G_display_switch(BASE_CONNECTED_BUTTON, program_state->base_connected);
  G_display_switch(PANTILT_CONNECTED_BUTTON, program_state->pantilt_connected);
  G_display_switch(SOUND_CONNECTED_BUTTON, program_state->sound_connected);
  G_display_switch(MOUTH_CONNECTED_BUTTON, program_state->mouth_connected);
  G_display_switch(MODE_BUTTON, detectionMode);

  EZX_Flush();

  program_state->graphics_initialized = 1;
}










/************************************************************************
 *
 *   NAME:         mouse_test_loop
 *                 
 *   FUNCTION:     Checks mouse events and changes the variables
 *                 "action" and "Program_state" correspondingly
 *                 
 *   PARAMETERS:   ROBOT_STATE_PTR    robot_state    Pointer to robot state
 *                                                   description structure
 *                 PROGRAM_STATE_PTR  program_state  Pointer  to general
 *                                                   program state description
 *                                                   structure (see above)
 *                 ROBOT_SPECIFICATIONS_PTR robot_specifications
 *                                                   Pointer to robot spe-
 *                                                   cifications (see above)
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/




int 
mouse_test_loop(ROBOT_STATE_PTR          robot_state,
		PROGRAM_STATE_PTR        program_state,
		ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  G_mouse_ptr mouse_events;
  int num_mouse_events, button;
  float mouse_x, mouse_y;
  int number;
  int test;
  char text[80];
  char *text_ptr;

  strcpy(text, "Hello");
  text_ptr = (char *) &text;

  if (!program_state->graphics_initialized)
    return 0;



  /****************** CHECK FOR MOUSE EVENT *******************/
  
  test = G_test_mouse(0);

  if (test == 1){
    mouse_events = G_evaluate_mouse(&mouse_x, &mouse_y, 
				    &button, &num_mouse_events);
    
    /****************** EVALUATE MOUSE EVENT *******************/
    


    if (G_mouse_event_at(QUIT_BUTTON, mouse_events, &number)){
      G_display_switch(QUIT_BUTTON, 1);
      usleep(200000);
      program_state->quit = 1;
    }


    /* insert new graphics object here XXXXX */
    
    
    else if (button == RIGHT_BUTTON){
      G_display_all();
    }
  }
  
  
  return test;
}



void
displayDetection( detectionStruct* obstacles)
{
  int i;
  int diff_found = 0;
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference;
  
  for (i = 0; i < obstacles->numberOfAngles && !diff_found; i++)
    if (fabs(obstacles->distances[i] - obstacles->prev_distances[i]) > 0.001)
      diff_found = 1;
  

  if (diff_found){

    gettimeofday(&this_time, NULL);
    time_difference = 
      ((float) (this_time.tv_sec - last_time.tv_sec))
	+ (((float) (this_time.tv_usec - last_time.tv_usec))
	   /  1000000.0);
    if (time_difference > 0.5){
      
      if ( useGraphics) {
	G_display_all();
      
	G_display_robot(LOCAL_DETECTION, 0.0, 0.0, 90.0,
			obstacles->numberOfAngles,
			obstacles->distances);
	
	G_display_markers( PERSON);
      }
      for (i = 0; i < obstacles->numberOfAngles; i++)
	obstacles->prev_distances[i] = obstacles->distances[i];

      last_time.tv_sec = this_time.tv_sec;
      last_time.tv_usec = this_time.tv_usec;
    }
  }
}


/* creates a window under EZX depending on map_size_x, map_size_y and
   scale (size of a pixel) */
gridWindow *
createMapWindow( char* text, 
		 int x, int y, int scale) {

  char corner[80];
  gridWindow *mapwin;
  
  mapwin = (gridWindow *) malloc(sizeof(gridWindow));
  mapwin->scale = scale;
  
  mapwin->winSizeX = x * scale;
  mapwin->winSizeY = y * scale;
  
  mapwin->sizeX = x;
  mapwin->sizeY = y;
  
  fprintf( stderr, "Create window for %d %d map (win size %d %d).\n", 
	   x, y, mapwin->winSizeX, mapwin->winSizeY);
  
  sprintf(corner,"+%d+%d",x,y);
  
  EZX_NoMotionEvents();
  mapwin->window = EZX_MakeWindow(text,mapwin->winSizeX,mapwin->winSizeY,corner);
  EZX_SetWindowBackground (mapwin->window, C_BLACK);
  EZX_Flush();

  return(mapwin);
}


void
showRobot( gridWindow *mapWin, realPosition* rpos, float resolution) 
{
  int x,y, radius;
  float rot;

#define ROB_RAD 30.0

  if ( rpos == NULL) {
    x = mapWin->sizeX * mapWin->scale / 2;
    y = mapWin->sizeY * mapWin->scale / 2;
    rot = 0.0;
    radius = ROB_RAD / resolution * mapWin->scale;
  }
  else {
    x = rpos->x / resolution * mapWin->scale;
    y = mapWin->winSizeY - rpos->y / resolution * mapWin->scale;
    rot = rpos->rot;
    radius = ROB_RAD / resolution * mapWin->scale;
    fprintf( stderr, "%f %f  -> %d %d %d \n", rpos->x, rpos->y, x, y, radius);
  }

  EZX_SetColor(C_YELLOW);
  EZX_FillCircle( mapWin->window, x, y, radius);

  EZX_SetColor(C_BLACK);

  EZX_DrawLine(mapWin->window, 
	       x, y, 
	       x + radius * cos(rot), 
	       y - radius * sin(rot));


  EZX_Flush();
}

void
displayMapWindow( probabilityGrid *m, gridWindow *mapWin, 
		  realPosition* rpos) 
{

  int x,y;
  int winX, winY;

  if ( m->sizeX != mapWin->sizeX || m->sizeY != mapWin->sizeY) {
    fprintf(stderr, "Error: wrong dimensions in display map window: %d %d - %d %d.\n",
	    m->sizeX, mapWin->sizeX, m->sizeY, mapWin->sizeY);
    return;
  }

/*     EZX_SetColor(COL_UNKNOWN); */
  /*     EZX_FillRectangle(mapWin->window,mapWin->startX,mapWin->startY,endX,endY); */
  
  x = 0;
  for (winX=0; winX < mapWin->winSizeX; winX += mapWin->scale){
    y = m->sizeY - 1;

    for (winY=0; winY <= mapWin->winSizeY; winY += mapWin->scale){
      EZX_SetColor( fieldColor( m->probs[x][y]));
      EZX_FillRectangle(mapWin->window,winX,winY,
			mapWin->scale,mapWin->scale);
      y--;
    }
    x++;
  }

  showRobot(mapWin, rpos, m->resolution);
  
  EZX_Flush();
}

static int
fieldColor( float f)
{
#define MIN_VAL 0.001
#define MAX_VAL 0.999

  /* #define TEST */
#ifdef TEST
  if (f < 0.0)
    return COL_RED;
  else if (f > 1.0)
    return COL_UNKNOWN;
  else {
    return (int) fNorm( f, MIN_VAL, MAX_VAL, COL_WHITE, COL_BLACK);
  }
#endif

  if (f <= MIN_VAL)
    if ( f < 0.0) {
      return C_BLUE;
    }
    else
      return COL_WHITE;
  else if (f >= MAX_VAL)
    return COL_BLACK;
  else {
    return (int) fNorm( f, MIN_VAL, MAX_VAL, COL_WHITE, COL_BLACK);
  }
}


void
displayHumanProbFunction( humanProbTable humanProbs)
{
  probabilityGrid map;
  gridWindow *win;
  int obstacle, human, distance, num = humanProbs.numberOfDistances;

  map.sizeX = humanProbs.numberOfDistances;
  map.sizeY = humanProbs.numberOfDistances;
  map.resolution = humanProbs.deltaDist;
  
  map.probs = (float**) allocate2D( map.sizeX, map.sizeY, FLOAT);
  
  win = createMapWindow( "hallo", map.sizeX, map.sizeY, 4);
    
  for ( obstacle = 0; obstacle < num; obstacle+=10) {

    fprintf(stderr, "Obstacle distance: %f\n", obstacle * humanProbs.deltaDist);
    for ( distance = 0; distance < num; distance++)
      for ( human = 0; human < num; human++) {
	if ( human == obstacle)
	  map.probs[human][distance] = 0.5;
	else
	  map.probs[human][distance] = humanProbs.probs[human][distance][obstacle];
      }
    displayMapWindow( &map, win, NULL);
    getchar();
  }
}



