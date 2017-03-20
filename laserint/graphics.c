
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/laserint/graphics.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:34:06 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: graphics.c,v $
 * Revision 1.1  2002/09/14 15:34:06  rstone
 * *** empty log message ***
 *
 * Revision 1.24  2000/09/06 05:42:19  thrun
 * Map editing buttons.
 *
 * Revision 1.23  2000/03/14 05:23:57  thrun
 * fixed a few problems with the 3D mapping. This version
 * pruced the results first shown to the DAPRA folks.
 * In particular, it reads the texture entirely from file and
 * constructs a new texture file (laserint.ppm, which has
 * to be converted manyally into laserint.jpg)
 *
 * Revision 1.22  2000/01/02 16:58:02  thrun
 * intermediate version with 2 objects
 *
 * Revision 1.21  1999/12/27 04:12:12  thrun
 * Intermediate version, with dww and dlw (might not work).
 *
 * Revision 1.20  1999/12/27 03:43:16  thrun
 * Magic: Gradient descent for adapting
 * wall parameters xw yw and aw seems to work!!!
 *
 * Revision 1.19  1999/12/19 03:25:17  thrun
 * graphics for 3D object matching.
 *
 * Revision 1.18  1999/12/12 23:13:38  thrun
 * Suprios change in graphics library - might improve grey-scale display
 *
 * Revision 1.17  1999/11/09 02:07:57  thrun
 * fixed minor display bug.
 *
 * Revision 1.16  1999/10/15 18:07:19  thrun
 * more 3D formats, more compact files.
 *
 * Revision 1.15  1999/10/15 01:30:46  thrun
 * improved interface for robot positioning
 *
 * Revision 1.14  1999/10/15 00:29:30  thrun
 * improved multi-robot handling
 *
 * Revision 1.13  1999/09/28 03:31:00  thrun
 * Improved version in the server mode.
 *
 * Revision 1.12  1999/09/06 03:19:44  thrun
 * first working version of the multi-robot server architecture with
 * positon control.
 *
 * Revision 1.11  1999/09/05 21:57:22  thrun
 * initial client server robot for multi-robot mapping
 *
 * Revision 1.10  1999/09/05 17:18:51  thrun
 * 3D ceiling mapping
 *
 * Revision 1.8  1999/05/08 19:47:28  thrun
 * drastically improved version with recursive matching and samples.
 * Not quite ready yet: samples aren't used in the right way, queue
 * seems not to work, and some constants are hardcoded (phase!) But
 * impressive results!
 *
 * Revision 1.7  1999/05/04 01:14:21  thrun
 * queue instead of stack.
 *
 * Revision 1.6  1998/11/18 04:21:10  thrun
 * incorporated wall angle into estimation.
 *
 * Revision 1.5  1998/11/16 01:50:41  thrun
 * positon control - works nice, includes global mapping and learning
 * the drift parameters.
 *
 * Revision 1.4  1998/11/15 16:19:18  thrun
 * initial position control method - works reasonably well. But
 * e careful with the timing, isn't an anytime algorithm yet.
 *
 * Revision 1.3  1997/05/28 13:05:13  thrun
 * nicer colors
 *
 * Revision 1.2  1997/02/02 22:32:35  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
 *
 * Revision 1.1.1.1  1996/09/22 16:46:28  rhino
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



#ifdef VMS
#include "vms.h"
#include "vmstime.h"
#endif



#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include "tcx.h"
#include "tcxP.h"
#include "Application.h"
#include "Net.h"
#include "MAP-messages.h"
#include "LASER-messages.h"
#include "BASE-messages.h"
#include "LASERINT.h"
#include "EZX11.h"
#include "o-graphics.h"
#include "corr.h"
/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

float *obj_dist = NULL;
#define OBJ_DIST_RES 200

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* BUTTONS and other graphic objects */


int DISPLAY_LOCAL_MAPVALUES_BUTTON;
int DISPLAY_LOCAL_ERROR_BUTTON;
int LOCAL_ROBOT;
int LOCAL_BACKGROUND;
int LOCAL_MAPVALUES;
int LOCAL_ERROR;
int QUIT_BUTTON;
int SCRIPT_BUTTON;
int BASE_CONNECTED_BUTTON;
int MAP_CONNECTED_BUTTON;
int SERVER_CONNECTED_BUTTON;
int REGRESSION;
int OBSTACLES;
int MATCHES;
int PREV_OBSTACLES;
int PATH[MAX_NUMBER_ROBOTS];
int SAMPLES;
int ERROR_DIAL;
int PLOT_BUTTON;
int EDIT_BUTTON_PLUS_ORIENT;
int EDIT_BUTTON_MINUS_ORIENT;
int EDIT_BUTTON_PLUS_X;
int EDIT_BUTTON_MINUS_X;
int EDIT_BUTTON_PLUS_Y;
int EDIT_BUTTON_MINUS_Y;
int SERVER_BUTTON;
int CLIENT_BUTTON;
int ROBOT_BUTTON[MAX_NUMBER_ROBOTS];
int OBJECTS[MAX_NUMBER_OBJECTS];
int OBJ_DIST;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



/************************************************************************
 *
 *   NAME:         init_graphics
 *                 
 *   FUNCTION:     Initiaizes graphics window
 *                 
 *   PARAMETERS:   ROBOT_STATE_PTR    robot_state    Pointer to robot state
 *                                                   description structure
 *                 PROGRAM_STATE_PTR  program_state  Pointer  to general
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


#define LOCAL_WINDOW_SIZE 0.5
#define RIGHT_BAR  6.03, (6.1+LOCAL_WINDOW_SIZE)
#define RIGHT_CENTER  (6.03+(0.5 *  (0.07+LOCAL_WINDOW_SIZE)))




void init_graphics(ROBOT_STATE_PTR    robot_state,
		   PROGRAM_STATE_PTR  program_state,
		   ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  int i,j;
  

  static char *myfonts[] = {"5x8", "6x10", "6x10", "7x13bold", 
			      "9x15", "10x20", "lucidasans-bold-24"};

  G_set_display(program_state->use_graphics);


  /*********************************************************************/

  obj_dist = (float *) malloc(sizeof(float) * OBJ_DIST_RES * OBJ_DIST_RES);

  /* =============================================================
     ====================  1) INITIALIZATIONS  ===================
     ============================================================= */
  
  if (!program_state->graphics_initialized){
    
    G_initialize_fonts(7, myfonts);
    G_initialize_graphics("BeeSoft Laser Interpreter", 220.0 /*80*/, 
			  10.0, C_SLATEGREY);
    
    
    
    /* =============================================================
       ====================  2) SPECIFICATIONS  ====================
       ============================================================= */

    
    /******** ERROR_DIAL *****************************/
    {
      static float value_pos[]            = {5.95, 6.0, 4.1, 6.0};
      static char *value_text             = "error";
      static int value_font               = 3;
      int direction_value                 = 2;
      static float value                  = 1.0;
      float min_value                     = 0.0;
      float max_value                     = 1.0;
      static int value_colors[]           = {C_GREY70, C_BLUE, C_GREY40,
					       NO_COLOR}; 
      
      
      ERROR_DIAL = G_create_value_object(value_pos, value_text, 
					  direction_value, value, 
					  min_value,
					  max_value, value_colors, 
					  value_font);
      
    }
   



    {
      /******** CLIENT_BUTTON ****************************/
      int switch_num                      = 1;
      static float switch_pos[]           = {RIGHT_BAR, 5.83, 6.0};
      static char *switch_texts[]         = 
	{"CLIENT"};
      static int switch_fonts[]           = {6};
      static int switch_background_color[]= {NO_COLOR};
      static int switch_frame_color[]     = {NO_COLOR};
      static int switch_text_color[]      = {C_WHITE};

      CLIENT_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }



    {
      /******** SERVER_BUTTON ****************************/
      int switch_num                      = 1;
      static float switch_pos[]           = {RIGHT_BAR, 5.83, 6.0};
      static char *switch_texts[]         = 
	{"SERVER"};
      static int switch_fonts[]           = {6};
      static int switch_background_color[]= {NO_COLOR};
      static int switch_frame_color[]     = {NO_COLOR};
      static int switch_text_color[]      = {C_WHITE};

      SERVER_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    {
      
      /******** QUIT_BUTTON ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_BAR, 5.73, 5.8};
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
      
      /******** ROBOT_BUTTON ****************************/
      int switch_num                      = 4;
      static float switch_pos[]           = {RIGHT_BAR, 5.63, 5.7};
      static char *switch_texts[]         = 
      {"robot ?", "robot ?", "robot ?", "robot ?"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]= 
      {C_WHITE, C_RED, C_LAWNGREEN, C_WHITE};
      static int switch_frame_color[]     = 
      {C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
      {C_BLACK, C_WHITE, C_BLACK, C_BLACK};
      int i;

      for (i = 0; i < MAX_NUMBER_ROBOTS; i++){
	ROBOT_BUTTON[i]
	  = G_create_switch_object(switch_pos, switch_num, switch_texts,
				   switch_background_color,switch_frame_color, 
				   switch_text_color, switch_fonts);
	switch_pos[2] -= 0.1;
	switch_pos[3] -= 0.1;
      }
    }

    {
      /******** DISPLAY_LOCAL_MAPVALUES_BUTTON ****************************/
      int switch_num                      = 3;
      static float switch_pos[]           = {RIGHT_BAR, 5.63, 5.7};
      static char *switch_texts[]         = 
	{"display map (OFF)", "display map (ON)", "..busy"};
      static int switch_fonts[]           = {2,2,2};
      static int switch_background_color[]= {C_WHITE, C_YELLOW, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK, C_WHITE};

      DISPLAY_LOCAL_MAPVALUES_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    {
      /******** DISPLAY_LOCAL_ERROR_BUTTON ****************************/
      int switch_num                      = 5;
      static float switch_pos[]           = {RIGHT_BAR, 5.53, 5.6};
      static char *switch_texts[]         = 
	{"display error (OFF)", "display error", "display dE/dx",
	 "display dE/dy", "display dE/do"};
      static int switch_fonts[]           = {2,2,2,2,2};
      static int switch_background_color[]= 
      {C_WHITE, C_YELLOW, C_YELLOW, C_YELLOW, C_YELLOW};
      static int switch_frame_color[]     = 
      {C_GREY40, C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      =
      {C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK};

      DISPLAY_LOCAL_ERROR_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    {      
      /******** SCRIPT_BUTTON ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_BAR, 5.43, 5.5};
      static char *switch_texts[]         = {"read script", "..processing script"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_WHITE, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_WHITE};
      SCRIPT_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    {      
      /******** PLOT_BUTTON ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_BAR, 5.33, 5.4};
      static char *switch_texts[]         = {"plot OFF", "plot ON"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_WHITE, C_YELLOW};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      PLOT_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }




    {      
      /******** EDIT_BUTTON_MINUS_ORIENT ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_CENTER + 0.16, 
					     RIGHT_CENTER + 0.26, 5.13, 5.2};
      static char *switch_texts[]         = {"R+", "R+"};
      static int switch_fonts[]           = {2, 2};
      static int switch_background_color[]= {C_WHITE, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_WHITE};
      EDIT_BUTTON_MINUS_ORIENT
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    {      
      /******** EDIT_BUTTON_PLUS_ORIENT ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_CENTER - 0.26, 
					     RIGHT_CENTER - 0.16, 5.13, 5.2};
      static char *switch_texts[]         = {"L+", "L+"};
      static int switch_fonts[]           = {2, 2};
      static int switch_background_color[]= {C_WHITE, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_WHITE};
      EDIT_BUTTON_PLUS_ORIENT
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    {      
      /******** EDIT_BUTTON_MINUS_X ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_CENTER - 0.05, 
					     RIGHT_CENTER + 0.05, 5.03, 5.1};
      static char *switch_texts[]         = {"X-", "X-"};
      static int switch_fonts[]           = {2, 2};
      static int switch_background_color[]= {C_WHITE, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_WHITE};
      EDIT_BUTTON_MINUS_X
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    {      
      /******** EDIT_BUTTON_PLUS_X ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_CENTER - 0.05, 
					     RIGHT_CENTER + 0.05, 5.23, 5.3};
      static char *switch_texts[]         = {"X+", "X+"};
      static int switch_fonts[]           = {2, 2};
      static int switch_background_color[]= {C_WHITE, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_WHITE};
      EDIT_BUTTON_PLUS_X
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    {      
      /******** EDIT_BUTTON_MINUS_Y ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_CENTER + 0.03, 
					     RIGHT_CENTER + 0.13, 5.13, 5.2};
      static char *switch_texts[]         = {"Y-", "Y-"};
      static int switch_fonts[]           = {2, 2};
      static int switch_background_color[]= {C_WHITE, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_WHITE};
      EDIT_BUTTON_MINUS_Y
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    {      
      /******** EDIT_BUTTON_PLUS_Y ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_CENTER - 0.13, 
					     RIGHT_CENTER - 0.03, 5.13, 5.2};
      static char *switch_texts[]         = {"Y+", "Y+"};
      static int switch_fonts[]           = {2, 2};
      static int switch_background_color[]= {C_WHITE, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_WHITE};
      EDIT_BUTTON_PLUS_Y
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    {      
      /******** BASE_CONNECTED_BUTTON ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_BAR, 4.4, 4.45};
      static char *switch_texts[]         = {"(COLLI not connected)",
					       "(COLLI connected)"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_SLATEGREY, C_SLATEGREY};
      static int switch_frame_color[]     = {NO_COLOR, NO_COLOR};
      static int switch_text_color[]      = {C_WHITE, C_LAWNGREEN};
      BASE_CONNECTED_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    
    {
      /******** MAP_CONNECTED_BUTTON ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_BAR, 4.3, 4.35};
      static char *switch_texts[]         = {"(MAP not connected)",
					       "(MAP connected)"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_SLATEGREY, C_SLATEGREY};
      static int switch_frame_color[]     = {NO_COLOR, NO_COLOR};
      static int switch_text_color[]      = {C_WHITE, C_LAWNGREEN};
      MAP_CONNECTED_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    
    {
      /******** SERVER_CONNECTED_BUTTON ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_BAR, 4.2, 4.25};
      static char *switch_texts[]         = {"(SERVER not connected)",
					       "(SERVER connected)"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_SLATEGREY, C_SLATEGREY};
      static int switch_frame_color[]     = {NO_COLOR, NO_COLOR};
      static int switch_text_color[]      = {C_WHITE, C_LAWNGREEN};
      SERVER_CONNECTED_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    
    {
      /******** LOCAL_BACKGROUND **************************************/
      int switch_num                      = 1;
      static float switch_pos[]           = {4.0, 5.9, 4.1, 6.0}; 
      static char *switch_texts[]         = {"oops"};
      static int switch_fonts[]           = {5};
      static int switch_background_color[]= {C_MEDIUMVIOLETRED};
      static int switch_frame_color[]     = {C_GREY40};
      static int switch_text_color[]      = {NO_COLOR};

      if (program_state->is_server)
	switch_background_color[0] = C_ORANGERED4;

      LOCAL_BACKGROUND 
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    
    


    {
      /******** LOCAL_ERROR *******************************************/
      static float matrix_pos[]            = {4.0, 5.9, 4.1, 6.0};
      char *matrix_text                    = "Oops - should not be here";
      static int matrix_font               = 5;
      float min_value_matrix               = 0.0;
      float max_value_matrix               = 1.0;
      static int matrix_colors[]           = 
	{C_MEDIUMVIOLETRED,NO_COLOR,NO_COLOR}; 


      matrix_pos[0] = (0.5*(5.9+4.0))
	- (1.0* robot_specifications->pos_corr_max_range
	   / robot_specifications->world_size *(5.9-4.0));
      matrix_pos[1] = (0.5*(5.9+4.0))
	+ (1.0* robot_specifications->pos_corr_max_range
	   / robot_specifications->world_size *(5.9-4.0));
      matrix_pos[2] = matrix_pos[0] + 0.1;
      matrix_pos[3] = matrix_pos[1] + 0.1;


      LOCAL_ERROR = 
	G_create_matrix_object(matrix_pos, matrix_text, 
			       local_error, NULL,
			       robot_specifications->pos_map_dim,
			       robot_specifications->pos_map_dim,
			       min_value_matrix, max_value_matrix, 
			       matrix_colors, matrix_font);
    }

    {
      /******** LOCAL_MAPVALUES *******************************************/
      static float matrix_pos[]            = {4.0, 5.9, 4.1, 6.0};
      char *matrix_text                    = "Oops - should not be here";
      static int matrix_font               = 5;
      float min_value_matrix               = 0.0;
      float max_value_matrix               = 1.0;
      static int matrix_colors[]           = 
	{C_MEDIUMVIOLETRED,NO_COLOR,NO_COLOR}; 

      matrix_pos[0] = (0.5*(5.9+4.0))
	- (0.5* robot_specifications->pos_corr_max_range
	   / robot_specifications->world_size *(5.9-4.0));
      matrix_pos[1] = (0.5*(5.9+4.0))
	+ (0.5* robot_specifications->pos_corr_max_range
	   / robot_specifications->world_size *(5.9-4.0));
      matrix_pos[2] = matrix_pos[0] + 0.1;
      matrix_pos[3] = matrix_pos[1] + 0.1;



      fprintf(stderr, "%g %g %g    %g %g %g %g\n",
	      robot_specifications->max_sensors_range
	   / robot_specifications->world_size,
	      robot_specifications->max_sensors_range,
	    robot_specifications->world_size,
	      matrix_pos[0], matrix_pos[1], matrix_pos[2], matrix_pos[3]);
      LOCAL_MAPVALUES = 
	G_create_matrix_object(matrix_pos, matrix_text, 
			       local_map, local_active,
			       robot_specifications->local_map_dim_x,
			       robot_specifications->local_map_dim_y,
			       min_value_matrix, max_value_matrix, 
			       matrix_colors, matrix_font);
    }


    {
      /******** OBJ_DIST *******************************************/
      static float matrix_pos[]            = {4.0, 5.9, 4.1, 6.0};
      char *matrix_text                    = "Oops - should not be here";
      static int matrix_font               = 5;
      float min_value_matrix               = 0.0;
      float max_value_matrix               = 1.0;
      static int matrix_colors[]           = 
	{C_MEDIUMVIOLETRED,NO_COLOR,NO_COLOR}; 



      OBJ_DIST =
	G_create_matrix_object(matrix_pos, matrix_text, 
			       obj_dist, NULL,
			       OBJ_DIST_RES, OBJ_DIST_RES,
			       min_value_matrix, max_value_matrix, 
			       matrix_colors, matrix_font);
    }


    {
      /******** ROBOT (SMALL) IN LOCAL MAP *********************************/ 
      static float pos_r[]                 = {4.0, 5.9, 4.1, 6.0};
      static char *text_r                  = "local";
      static int robot_font                = 3;
      static int colors_r[]                = {NO_COLOR, C_GREY40, C_CYAN,
						C_GREY50, C_VIOLET, NO_COLOR, 
						NO_COLOR, NO_COLOR};

      
      
      LOCAL_ROBOT =
	G_create_robot_object(pos_r, text_r, 
			      -0.5 * robot_specifications->world_size,
			      0.5 * robot_specifications->world_size,
			      -0.5 * robot_specifications->world_size,
			      0.5 * robot_specifications->world_size,
			      0.0, 0.0, 90.0,
			      robot_specifications->robot_size, 
			      0, /*!*/
			      robot_specifications->max_sensors_range, 
			      robot_state->sensor_values, 
			      robot_specifications->sensor_angles,
			      colors_r, robot_font);
      
    }


    /******** REGRESSION *********************************************/
    {
      static float pos_mar[]                   = {4.0, 5.9, 4.1, 6.0};
      int num_mar                            = 1;
      static char *text_mar[]                = {"REGRESSION"};
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = {C_BLACK, C_BLACK};
      static int mar_text_color[]            = {NO_COLOR};
      static int mar_fonts[]                 = {2};
      
      REGRESSION
	= G_create_markers_object(pos_mar, 1,
				  robot_specifications->robot_size * 0.3,
				  -0.5 * robot_specifications->world_size,
				  0.5 * robot_specifications->world_size,
				  -0.5 * robot_specifications->world_size,
				  0.5 * robot_specifications->world_size,
				  num_mar, text_mar, mar_background_color, 
				  mar_frame_color, mar_foreground_color, 
				  mar_text_color, mar_fonts);
      
    }
  
    /******** OBJECTS *********************************************/
    {
      static float pos_mar[]                   = {4.0, 5.9, 4.1, 6.0};
      int num_mar                            = 1;
      static char *text_mar[]                = {"OBJECTS"};
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = {C_OLDLACE, C_BLACK};
      static int mar_text_color[]            = {NO_COLOR};
      static int mar_fonts[]                 = {2};
      int i;

      for (i = 0; i < MAX_NUMBER_OBJECTS; i++)
	OBJECTS[i]
	  = G_create_markers_object(pos_mar, 1, 0.0,
				    -0.5 * robot_specifications->world_size,
				    0.5 * robot_specifications->world_size,
				    -0.5 * robot_specifications->world_size,
				    0.5 * robot_specifications->world_size,
				    num_mar, text_mar, mar_background_color, 
				    mar_frame_color, mar_foreground_color, 
				    mar_text_color, mar_fonts);
    }
  

  

    /******** PREV_OBSTACLES *********************************************/
    {
      static float pos_mar[]                   = {4.0, 5.9, 4.1, 6.0};
      int num_mar                            = 3;
      static char *text_mar[]                = {"PREV_OBSTACLES",
						"PREV_OBSTACLES",
						"PREV_OBSTACLES"};
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = {C_WHITE, C_YELLOW, C_BLACK};
      static int mar_text_color[]            = {NO_COLOR, NO_COLOR, NO_COLOR};
      static int mar_fonts[]                 = {2, 2,2};
      
      PREV_OBSTACLES
	= G_create_markers_object(pos_mar, 0,
				  robot_specifications->robot_size * 0.1,
				  -0.5 * robot_specifications->world_size,
				  0.5 * robot_specifications->world_size,
				  -0.5 * robot_specifications->world_size,
				  0.5 * robot_specifications->world_size,
				  num_mar, text_mar, mar_background_color, 
				  mar_frame_color, mar_foreground_color, 
				  mar_text_color, mar_fonts);
      
    }

    /******** PATH *********************************************/
    {
      static float pos_mar[]                   = {4.0, 5.9, 4.1, 6.0};
      int num_mar                            = 1;
      static char *text_mar[]                = {"PATH"};
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = 
      {C_RED, C_MEDIUMVIOLETRED, C_MEDIUMPURPLE3, C_PALEGREEN4,
       C_CYAN, C_STEELBLUE4, C_ORANGERED4, C_KHAKI4, C_DARKTURQUOISE,
       C_FORESTGREEN, C_OLDLACE, C_LIGHTSLATEGREY, C_SLATEGREY,
       C_DARKSLATEGREY, C_PALETURQUOISE4, C_LIMEGREEN, C_CADETBLUE,
       C_DEEPPINK, C_MAGENTA2, C_SIENNA4, C_PINK1, C_TURQUOISE4, C_ROYALBLUE};

      static int mar_text_color[]            = {NO_COLOR};
      static int mar_fonts[]                 = {2};
      int i;

      for (i = 0; i < MAX_NUMBER_ROBOTS; i++)
	
	PATH[i]
	  = G_create_markers_object(pos_mar, 1,
				    robot_specifications->robot_size * 0.001,
				    -0.5 * robot_specifications->world_size,
				    0.5 * robot_specifications->world_size,
				    -0.5 * robot_specifications->world_size,
				    0.5 * robot_specifications->world_size,
				    num_mar, text_mar, mar_background_color, 
				    mar_frame_color, 
				    &(mar_foreground_color[i]), 
				    mar_text_color, mar_fonts);

    }

    /******** OBSTACLES *********************************************/
    {
      static float pos_mar[]                   = {4.0, 5.9, 4.1, 6.0};
      int num_mar                            = 2;
      static char *text_mar[]                = {"OBSTACLES", "OBSTACLES"};
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = {C_CYAN,C_BLUE};
      static int mar_text_color[]            = {NO_COLOR,NO_COLOR};
      static int mar_fonts[]                 = {2,2};
      
      OBSTACLES
	= G_create_markers_object(pos_mar, 0,
				  robot_specifications->robot_size * 0.1,
				  -0.5 * robot_specifications->world_size,
				  0.5 * robot_specifications->world_size,
				  -0.5 * robot_specifications->world_size,
				  0.5 * robot_specifications->world_size,
				  num_mar, text_mar, mar_background_color, 
				  mar_frame_color, mar_foreground_color, 
				  mar_text_color, mar_fonts);
      
    }
  
    /******** SAMPLES *********************************************/
    {
      static float pos_mar[]                   = {4.0, 5.9, 4.1, 6.0};
      int num_mar                            = 1;
      static char *text_mar[]                = {"SAMPLES"};
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = {C_GREY50};
      static int mar_text_color[]            = {NO_COLOR,NO_COLOR};
      static int mar_fonts[]                 = {2};
      
      SAMPLES
	= G_create_markers_object(pos_mar, 0,
				  robot_specifications->robot_size * 0.1,
				  -0.5 * robot_specifications->world_size,
				  0.5 * robot_specifications->world_size,
				  -0.5 * robot_specifications->world_size,
				  0.5 * robot_specifications->world_size,
				  num_mar, text_mar, mar_background_color, 
				  mar_frame_color, mar_foreground_color, 
				  mar_text_color, mar_fonts);
      
    }
  

    /******** MATCHES*********************************************/
    {
      static float pos_mar[]                   = {4.0, 5.9, 4.1, 6.0};
      int num_mar                            = 2;
      static char *text_mar[]                = {"OBSTACLES", "OBSTACLES"};
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = {C_RED,C_LAWNGREEN};
      static int mar_text_color[]            = {NO_COLOR,NO_COLOR};
      static int mar_fonts[]                 = {2,2};
      
      MATCHES
	= G_create_markers_object(pos_mar, 1,
				  robot_specifications->robot_size * 0.1,
				  -0.5 * robot_specifications->world_size,
				  0.5 * robot_specifications->world_size,
				  -0.5 * robot_specifications->world_size,
				  0.5 * robot_specifications->world_size,
				  num_mar, text_mar, mar_background_color, 
				  mar_frame_color, mar_foreground_color, 
				  mar_text_color, mar_fonts);
      
    }
  

    
    /* insert new graphics object here XXXXX */

  }
    
  
  
  /* =============================================================
     ====================  4) DISPLAY  ===========================
     ============================================================= */


  G_deactivate(OBJ_DIST);
  G_deactivate(LOCAL_MAPVALUES);
  G_deactivate(LOCAL_ERROR);
  G_set_matrix_display_style(1);
  if (program_state->is_server){
    G_deactivate(DISPLAY_LOCAL_MAPVALUES_BUTTON);
    G_deactivate(DISPLAY_LOCAL_ERROR_BUTTON);
    G_deactivate(SCRIPT_BUTTON);
    G_deactivate(PLOT_BUTTON);
    G_deactivate(EDIT_BUTTON_PLUS_ORIENT);
    G_deactivate(EDIT_BUTTON_MINUS_ORIENT);
    G_deactivate(EDIT_BUTTON_PLUS_X);
    G_deactivate(EDIT_BUTTON_MINUS_X);
    G_deactivate(EDIT_BUTTON_PLUS_Y);
    G_deactivate(EDIT_BUTTON_MINUS_Y);
    G_deactivate(CLIENT_BUTTON);
    G_deactivate(BASE_CONNECTED_BUTTON);
    /* G_deactivate(MAP_CONNECTED_BUTTON); */
    G_deactivate(SERVER_CONNECTED_BUTTON);
  }
  else{
    if (robotname)
      G_set_new_text(CLIENT_BUTTON, robotname, 0);
    G_deactivate(SERVER_BUTTON);
  }

  for (i = 0; i < MAX_NUMBER_ROBOTS; i++)
    G_deactivate(ROBOT_BUTTON[i]);

  G_display_all();
  G_display_switch(MAP_CONNECTED_BUTTON, program_state->map_connected);
  G_display_switch(BASE_CONNECTED_BUTTON, program_state->base_connected);


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




int mouse_test_loop(ROBOT_STATE_PTR          robot_state,
		    PROGRAM_STATE_PTR        program_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  
  G_mouse_ptr mouse_events;
  int num_mouse_events, button;
  float mouse_x, mouse_y;
  int i, number;
  float min_value, max_value;
  int test;
  float change_x, change_y, change_orientation;
  float raw_x, raw_y, raw_orientation;
  float raw_x2, raw_y2, raw_orientation2;
  float raw_x3, raw_y3, raw_orientation3;


  if (!program_state->graphics_initialized)
    return 0;



  /****************** CHECK FOR MOUSE EVENT *******************/
  
  test = G_test_mouse(0);

  if (test == 1){
    mouse_events = G_evaluate_mouse(&mouse_x, &mouse_y, 
				    &button, &num_mouse_events);
    
    /****************** EVALUATE MOUSE EVENT *******************/
    
    

    if (G_mouse_event_at(DISPLAY_LOCAL_MAPVALUES_BUTTON, 
			      mouse_events, &number)){
      if (!robot_specifications->display_fixed_rotation){
	program_state->regular_local_map_display ^= 1;
	G_display_switch(DISPLAY_LOCAL_MAPVALUES_BUTTON, 2);
	if (program_state->regular_local_map_display){
	  G_deactivate(LOCAL_BACKGROUND);
	  G_activate(LOCAL_MAPVALUES);
	}
	else{
	  G_activate(LOCAL_BACKGROUND);
	  G_deactivate(LOCAL_MAPVALUES);
	}
	G_display_switch(LOCAL_BACKGROUND, 0);
	G_display_matrix(LOCAL_ERROR);
	G_display_matrix(LOCAL_MAPVALUES);
	for (i = 0; i < program_state->number_objects; i++)
	  if (program_state->objects[i])
	    G_display_markers(OBJECTS[i]);
	G_display_robot(LOCAL_ROBOT, 0.0, 0.0, 90.0, 0, NULL);
	G_display_switch(DISPLAY_LOCAL_MAPVALUES_BUTTON, 
			 program_state->regular_local_map_display);
      }
    }


    if (G_mouse_event_at(DISPLAY_LOCAL_ERROR_BUTTON, 
			      mouse_events, &number)){
      if (!robot_specifications->display_fixed_rotation){
	
	if (button == RIGHT_BUTTON)
	  program_state->regular_local_map_display = 0;
	else
	  program_state->regular_local_map_display =
	    (program_state->regular_local_map_display + 1) % 2;
	
	G_display_switch(DISPLAY_LOCAL_ERROR_BUTTON, 
			 program_state->regular_local_map_display);
	
	if (program_state->regular_local_map_display != 0)
	  G_activate(LOCAL_ERROR);
	else
	  G_deactivate(LOCAL_ERROR);
	
	G_display_switch(LOCAL_BACKGROUND, 0);
	G_display_matrix(LOCAL_ERROR);
	for (i = 0; i < program_state->number_objects; i++)
	  if (program_state->objects[i])
	    G_display_markers(OBJECTS[i]);
	G_display_robot(LOCAL_ROBOT, 0.0, 0.0, 90.0, 0, NULL);
      }
    }


    else if (G_mouse_event_at(LOCAL_BACKGROUND,
			      mouse_events, &number)){
      fprintf(stderr, "?");
    }





    else if (program_state->calibration == -1 &&
	     program_state->object_matching_mode &&
	     program_state->number_objects < MAX_NUMBER_OBJECTS &&
	     G_mouse_event_at(LOCAL_ROBOT, mouse_events, &number)){
      stack_item_ptr s =
	&(program_state->stack[program_state->first_stack_item]);  
      object_item_ptr obj = program_state->objects[program_state->number_objects-1];

#define NUM_X 2

      float dist[NUM_X];
      float newdx, newdy, newdo, mindist, lrate, mom, factor, cumul_change;
      int i, j, k, n, count[NUM_X], o, mino;
      object_item_ptr object;
      float dxw[NUM_X], dyw[NUM_X], dlw[NUM_X], dww[NUM_X], daw[NUM_X];
      float p_dxw[NUM_X], p_dyw[NUM_X], p_dlw[NUM_X];
      float p_dww[NUM_X], p_daw[NUM_X];
      float aux_dxw[NUM_X], aux_dyw[NUM_X], aux_dlw[NUM_X];
      float aux_dww[NUM_X], aux_daw[NUM_X];
      object_item_ptr obj_ptr[NUM_X];
      float max, *d, x, y, aux_dist;
      float angle = s->laser_orientation * M_PI / 180.0;

      program_state->number_objects = 0;

      fprintf(stderr, "-> %g %g\n",
	      mouse_events[number].value_x, mouse_events[number].value_y);


      if (!robot_specifications->display_fixed_rotation){
	change_x = (cos(angle) * mouse_events[number].value_y +
		    sin(angle) * mouse_events[number].value_x);
	change_y = (sin(angle) * mouse_events[number].value_y -
		    cos(angle) * mouse_events[number].value_x);
      }
      else{
	change_y = -mouse_events[number].value_x + program_state->offs_y;
	change_x = mouse_events[number].value_y + program_state->offs_x;
      }

      for (o = 0; o < NUM_X; o++){
	obj_ptr[o] = program_state->objects[program_state->number_objects] =
	  (object_item_ptr) malloc(sizeof(object_item_type));
	
	obj_ptr[o]->x = - change_x;
	obj_ptr[o]->y = - change_y;
	obj_ptr[o]->z = 0.0;
	obj_ptr[o]->angle  = 225.0;
	obj_ptr[o]->length = 400.0;
	obj_ptr[o]->width  = 300.0;
	obj_ptr[o]->height = 200.0;
	p_dxw[o] = p_dyw[o] = p_dlw[o] = p_dww[o] = p_daw[o] = 0.0;      
	
	program_state->number_objects++;
      }

      for (cumul_change = 100.0; cumul_change > 1.0; ){
	cumul_change = 0.0;
	for (o = 0; o < NUM_X; o++){
	  dist[o] = 0.0;
	  dxw[o] = dyw[o] = dlw[o] = dww[o] = daw[o] = 0.0;
	  count[o] = 0;
	}
	for (n = 0; n < 5; n++){
	  if (s->admissble_sensor_value[n]){
	    mindist = 999999.9;
	    mino = -1;
	    for (o = 0; o < NUM_X; o++){
	      aux_dist = 
		distPointToObj(s->sensor_x[n], s->sensor_y[n],
			       obj_ptr[o]->x, obj_ptr[o]->y,
			       obj_ptr[o]->length, obj_ptr[o]->width,
			       obj_ptr[o]->angle,
			       s->laser_x, s->laser_y, s->laser_orientation,
			       s->sensor_values[n],
			       s->sensor_angle_to_base[n],
			       &(aux_dxw[o]), &(aux_dyw[o]), &(aux_dlw[o]),
			       &(aux_dww[o]), &(aux_daw[o]));
	      if (aux_dist < mindist){
		mindist = aux_dist;
		mino = o;
	      }
	    }
	    
	    dxw[mino] += aux_dxw[mino];
	    dyw[mino] += aux_dyw[mino];
	    dlw[mino] += aux_dlw[mino];
	    dww[mino] += aux_dww[mino];
	    daw[mino] += aux_daw[mino];
	    dist[mino] += mindist;
	    count[mino]++;
	  }
	}

	for (o = 0; o < NUM_X; o++){
	  if (count[o] > 0){
	    lrate = 0.05;
	    mom = 0.2;
	    factor = 1.0 / ((float) count[o]);
	    
	    dxw[o] = (dxw[o] * dist[o] * factor);
	    dyw[o] = (dyw[o] * dist[o] * factor);
	    dlw[o] = (dlw[o] * dist[o] * factor) + (0.0 * obj_ptr[o]->length);
	    dww[o] = (dww[o] * dist[o] * factor) + (0.0 * obj_ptr[o]->width);
	    daw[o] = (daw[o] * dist[o] * factor);
	    
	    p_dxw[o] = (mom * p_dxw[o]) + dxw[o];
	    p_dyw[o] = (mom * p_dyw[o]) + dyw[o];
	    p_dlw[o] = (mom * p_dlw[o]) + dlw[o];
	    p_dww[o] = (mom * p_dww[o]) + dww[o];
	    p_daw[o] = (mom * p_daw[o]) + daw[o];
	    
	    obj_ptr[o]->x      -= (lrate * p_dxw[o]);
	    obj_ptr[o]->y      -= (lrate * p_dyw[o]);
	    obj_ptr[o]->length -= (lrate * p_dlw[o]);
	    obj_ptr[o]->width  -= (lrate * p_dww[o]);
	    obj_ptr[o]->angle  -= (lrate * 5.0 * p_daw[o]);
	    
	    if (obj_ptr[o]->length < 100.0)
	      obj_ptr[o]->length = 100.0;
	    if (obj_ptr[o]->width < 100.0)
	      obj_ptr[o]->width = 100.0;
	    
	    cumul_change +=  
	      ((fabs(dxw[o]) + fabs(dyw[o]) + fabs(dlw[o]) +
		fabs(dww[o]) + fabs(daw[o])));
	  }
	}

	display_scans_and_map(robot_specifications, program_state, 
			      robot_state, 0);


      }

      /* ------ */
      G_activate(OBJ_DIST);
      max = 0.0;
      n = 0;
      for (i = 0; i < OBJ_DIST_RES; i++)
	for (j = 0; j < OBJ_DIST_RES; j++){
	  d = &(obj_dist[i * OBJ_DIST_RES + j]);
	  x = ((((float) i) / ((float) (OBJ_DIST_RES))) - 0.5)
	    * robot_specifications->world_size;
	  y = ((((float) j) / ((float) (OBJ_DIST_RES))) - 0.5)
	    * robot_specifications->world_size;

	  if (!robot_specifications->display_fixed_rotation){
	    change_x = (cos(angle) * y + sin(angle) * x);
	    change_y = (sin(angle) * y - cos(angle) * x);
	  }
	  else{
	    change_y = -x + program_state->offs_y;
	    change_x = y + program_state->offs_x;
	  }

	  for (o = 0; o < NUM_X; o++){
	    aux_dist = 
	      distPointToObj(-change_x, -change_y, 
			     obj_ptr[o]->x, obj_ptr[o]->y,
			     obj_ptr[o]->length, obj_ptr[o]->width,
			     obj_ptr[o]->angle,
			     s->laser_x, s->laser_y, s->laser_orientation,
			     s->sensor_values[n],
			     s->sensor_angle_to_base[n],
			     &(aux_dxw[o]), &(aux_dyw[o]), &(aux_dlw[o]),
			     &(aux_dww[o]), &(aux_daw[o]));
	    if (o == 0 || aux_dist < *d)
	      *d = aux_dist;
	  }
	  if (*d > max)
	    max = *d;
	}
      G_matrix_set_display_range(OBJ_DIST, 0.0, max);
      display_scans_and_map(robot_specifications, program_state, 
			    robot_state, 0);
      G_deactivate(OBJ_DIST);
      /* ------ */


    }


    else if (program_state->calibration != -1 &&
	     G_mouse_event_at(LOCAL_ROBOT, mouse_events, &number)){
      stack_item_ptr s =
	&(program_state->stack[program_state->last_stack_item]);  
      change_x = change_y = change_orientation = 0.0;

      compute_backward_correction(s->laser_x,
				  s->laser_y,
				  s->laser_orientation,
				  auto_update_modules
				  [program_state->calibration].
				  correction_parameter_x,
				  auto_update_modules
				  [program_state->calibration].
				  correction_parameter_y,
				  auto_update_modules
				  [program_state->calibration].
				  correction_parameter_angle,
				  auto_update_modules
				  [program_state->calibration].correction_type,
				  &raw_x, &raw_y, &raw_orientation);
      compute_backward_correction(robot_state->laser_x,
				  robot_state->laser_y,
				  robot_state->laser_orientation,
				  auto_update_modules
				  [program_state->calibration].
				  correction_parameter_x,
				  auto_update_modules
				  [program_state->calibration].
				  correction_parameter_y,
				  auto_update_modules
				  [program_state->calibration].
				  correction_parameter_angle,
				  auto_update_modules
				  [program_state->calibration].correction_type,
				  &raw_x2, &raw_y2, &raw_orientation2);
      compute_backward_correction(robot_state->x,
				  robot_state->y,
				  robot_state->orientation,
				  auto_update_modules
				  [program_state->calibration].
				  correction_parameter_x,
				  auto_update_modules
				  [program_state->calibration].
				  correction_parameter_y,
				  auto_update_modules
				  [program_state->calibration].
				  correction_parameter_angle,
				  auto_update_modules
				  [program_state->calibration].correction_type,
				  &raw_x3, &raw_y3, &raw_orientation3);


      if (button == LEFT_BUTTON){
	if (!robot_specifications->display_fixed_rotation){
	  float angle = s->laser_orientation * M_PI / 180.0;
	  change_x = (cos(angle) * mouse_events[number].value_y +
		      sin(angle) * mouse_events[number].value_x);
	  change_y = (sin(angle) * mouse_events[number].value_y -
		      cos(angle) * mouse_events[number].value_x);
	}
	else{
	  fprintf(stderr, "\n%g %g   %g %g   %g %g\n",
		  mouse_events[number].value_x,
		  mouse_events[number].value_y,
		  program_state->offs_y,
		  - program_state->offs_x,
		  s->laser_x,
		  s->laser_y);
	  change_y = -mouse_events[number].value_x + program_state->offs_y;
	  change_x = mouse_events[number].value_y + program_state->offs_x;
	}
      }
      else if (button == MIDDLE_BUTTON){
	if (!robot_specifications->display_fixed_rotation)
	  change_orientation += (atan2(mouse_events[number].value_y,
					 mouse_events[number].value_x)
				   - (0.5 * M_PI)) / M_PI * 180.0;
	else{
	  change_orientation += - atan2(-mouse_events[number].value_y 
					- program_state->offs_x, 
					mouse_events[number].value_x 
					- program_state->offs_y) 
	    * 180.0/ M_PI - 90.0 - s->laser_orientation;
		  
	}
      }
      s->laser_x += change_x;
      s->laser_y += change_y;
      s->laser_orientation += change_orientation;
      for (;s->laser_orientation < -180.0;) s->laser_orientation += 360.0;
      for (;s->laser_orientation > 180.0;)  s->laser_orientation -= 360.0;
      compute_correction_parameters(raw_x, raw_y, raw_orientation,
				    s->laser_x, s->laser_y,
				    s->laser_orientation,
				    &(auto_update_modules
				      [program_state->calibration].
				      correction_parameter_x),
				    &(auto_update_modules
				      [program_state->calibration].
				      correction_parameter_y),
				    &(auto_update_modules
				      [program_state->calibration].
				      correction_parameter_angle),
				    &(auto_update_modules
				      [program_state->calibration].
				      correction_type));
      compute_forward_correction(raw_x2, raw_y2, raw_orientation2,
				 auto_update_modules
				 [program_state->calibration].
				 correction_parameter_x,
				 auto_update_modules
				 [program_state->calibration].
				 correction_parameter_y,
				 auto_update_modules
				 [program_state->calibration].
				 correction_parameter_angle,
				 auto_update_modules
				 [program_state->calibration].correction_type,
				 &(robot_state->laser_x),
				 &(robot_state->laser_y),
				 &(robot_state->laser_orientation));
      compute_forward_correction(raw_x3, raw_y3, raw_orientation3,
				 auto_update_modules
				 [program_state->calibration].
				 correction_parameter_x,
				 auto_update_modules
				 [program_state->calibration].
				 correction_parameter_y,
				 auto_update_modules
				 [program_state->calibration].
				 correction_parameter_angle,
				 auto_update_modules
				 [program_state->calibration].correction_type,
				 &(robot_state->x),
				 &(robot_state->y),
				 &(robot_state->orientation));


      if (button == RIGHT_BUTTON){
	G_display_switch(ROBOT_BUTTON[program_state->calibration], 2);
	auto_update_modules[program_state->calibration].calibrated = 2;
	program_state->calibration = -1;
      }
      num_path_items = 0;
      display_scans_and_map(robot_specifications, program_state, 
			    robot_state, 1);
    }


    else if (button == RIGHT_BUTTON &&
	     G_mouse_event_at(LOCAL_ROBOT, mouse_events, &number)){
      G_display_switch(DISPLAY_LOCAL_MAPVALUES_BUTTON, 2);
      G_activate(LOCAL_MAPVALUES);
      G_display_matrix(LOCAL_MAPVALUES);
      for (i = 0; i < program_state->number_objects; i++)
	if (program_state->objects[i])
	  G_display_markers(OBJECTS[i]);
      G_display_robot(LOCAL_ROBOT, 0.0, 0.0, 90.0, 0, NULL);
      if (!program_state->regular_local_map_display)
	G_deactivate(LOCAL_MAPVALUES);
      G_display_switch(DISPLAY_LOCAL_MAPVALUES_BUTTON, 
		       program_state->regular_local_map_display);
    }


    else if (G_mouse_event_at(QUIT_BUTTON, mouse_events, &number)){
      G_display_switch(QUIT_BUTTON, 1);
      usleep(200000);
      program_state->quit = 1;
    }


    else if (G_mouse_event_at(SCRIPT_BUTTON, mouse_events, &number)){
      
      if (!program_state->processing_script)
	initiate_read_script(robot_state, program_state, robot_specifications,
			     SCRIPT_NAME);      
      else
	close_script(robot_state, program_state, robot_specifications);
    }


    else if (G_mouse_event_at(PLOT_BUTTON, mouse_events, &number)){
      if (plot_iop == NULL)
	plot_on(robot_specifications, program_state, robot_state, 
		PLOT_NAME, VR_NAME, SMF_NAME);
      else
	plot_off(robot_specifications, program_state, robot_state);
    }

    else if (G_mouse_event_at(EDIT_BUTTON_MINUS_ORIENT, mouse_events, &number)){
      G_display_switch(EDIT_BUTTON_MINUS_ORIENT, 1);
      edit(robot_specifications, program_state, robot_state, button, 
	   0.0, 0.0, -1.0);
      usleep(100000);
      G_display_switch(EDIT_BUTTON_MINUS_ORIENT, 0);
    }
    else if (G_mouse_event_at(EDIT_BUTTON_PLUS_ORIENT, mouse_events, &number)){
      G_display_switch(EDIT_BUTTON_PLUS_ORIENT, 1);
      edit(robot_specifications, program_state, robot_state, button, 
	   0.0, 0.0, 1.0);
      usleep(100000);
      G_display_switch(EDIT_BUTTON_PLUS_ORIENT, 0);
    }

    else if (G_mouse_event_at(EDIT_BUTTON_MINUS_X, mouse_events, &number)){
      G_display_switch(EDIT_BUTTON_MINUS_X, 1);
      edit(robot_specifications, program_state, robot_state, button, 
	   -1.0, 0.0, 0.0);
      usleep(100000);
      G_display_switch(EDIT_BUTTON_MINUS_X, 0);
    }
    else if (G_mouse_event_at(EDIT_BUTTON_PLUS_X, mouse_events, &number)){
      G_display_switch(EDIT_BUTTON_PLUS_X, 1);
      edit(robot_specifications, program_state, robot_state, button, 
	   1.0, 0.0, 0.0);
      usleep(100000);
      G_display_switch(EDIT_BUTTON_PLUS_X, 0);
    }

    else if (G_mouse_event_at(EDIT_BUTTON_MINUS_Y, mouse_events, &number)){
      G_display_switch(EDIT_BUTTON_MINUS_Y, 1);
      edit(robot_specifications, program_state, robot_state, button, 
	   0.0, -1.0, 0.0);
      usleep(100000);
      G_display_switch(EDIT_BUTTON_MINUS_Y, 0);
    }
    else if (G_mouse_event_at(EDIT_BUTTON_PLUS_Y, mouse_events, &number)){
      G_display_switch(EDIT_BUTTON_PLUS_Y, 1);
      edit(robot_specifications, program_state, robot_state, button, 
	   0.0, 1.0, 0.0);
      usleep(100000);
      G_display_switch(EDIT_BUTTON_PLUS_Y, 0);
    }



    /* insert new graphics object here XXXXX */
    
    
    else if (button == RIGHT_BUTTON)
      G_display_all();


  }
  
  check_redraw_screen(theDisplay);

  return test;
}


