
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/controller/graphics.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 16:53:36 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: graphics.h,v $
 * Revision 1.1  2002/09/14 16:53:36  rstone
 * *** empty log message ***
 *
 * Revision 1.10  1999/09/28 01:39:52  thrun
 * connects to LOCALIZE and receives correction parameeters from there.
 *
 * Revision 1.9  1997/08/22 02:39:20  swa
 * Support for another CD (#ifdef UNIBONN is the old one).
 *
 * Revision 1.8  1997/08/16 23:10:43  thrun
 * Tourguide with CD
 *
 * Revision 1.7  1997/08/16 18:38:02  thrun
 * tourguide.
 *
 * Revision 1.6  1997/06/18 15:58:42  thrun
 * new camera handling...
 *
 * Revision 1.5  1997/02/02 22:32:33  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
 *
 * Revision 1.4  1997/01/26 20:59:03  thrun
 * ARM is not part of the BeeSoft release any longer (compiles now
 * without ARM with the -DUNIBONN flag is not set).
 *
 * Revision 1.3  1996/11/27 23:20:24  thrun
 * (a) Modifications of Tyson's Makefile: they now work under Solaris again
 * (b) Major modifications of the CONTROLLER module.
 *
 * Revision 1.2  1996/09/23 09:51:25  thrun
 * Changes necessary to make the "-DUNIBONN" flag working again.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:27  rhino
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





#ifdef CD_VERSION

#ifdef UNIBONN
#define NUM_CD_ROWS 28
#else
#define NUM_CD_ROWS 7
#endif

#define NUM_CD_COLUMNS 1
#define NUM_CD_TEXTS ((NUM_CD_ROWS)*(NUM_CD_COLUMNS))


typedef struct {
  int msg;
  int initial_gong;
} cd_texts_type;


extern CD_msg_type MSGs[NUM_MSGs];
extern cd_texts_type CD_TEXTS[NUM_CD_ROWS];

#endif /* CD_VERSION */


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

extern int SONAR_ROBOT;
extern int LASER_ROBOT;
extern int SONAR_ROBOT_BACKGROUND;
extern int TITLE_BUTTON;
extern int QUIT_BUTTON;
extern int REFRESH_BUTTON;
extern int BUSY_BUTTON;
extern int IN_MOTION_BUTTON;
extern int GLOBAL_ROBOT;
extern int GLOBAL_ROBOT_BACKGROUND;
extern int PATH;
extern int SCRIPT_PATH;
extern int CAMERA_TILT;
extern int CAMERA_PAN;
extern int SCRIPT_CAMERA_TILT;
extern int SCRIPT_CAMERA_PAN;
/*extern int LOGGING_BUTTON;*/
/*extern int SCRIPT_BUTTON;*/
/*extern int EPISODE_DIAL;*/
/*extern int EVENT_DIAL;*/
/*extern int DECREASE_EPISODE_BUTTON;*/
/*extern int INCREASE_EPISODE_BUTTON;*/
/*extern int DECREASE_EVENT_BUTTON;*/
/*extern int INCREASE_EVENT_BUTTON;*/
extern int SCRIPT_EVENT_DATE_BUTTON;
extern int RESET_ROBOT_BUTTON;
extern int PAN_TILT_BACKGROUND;
extern int PSEUDO_PICTURE;

extern int CONNECT_BASE_BUTTON;
extern int CONNECT_PANTILT_BUTTON;
extern int CONNECT_SPEECH_BUTTON;
extern int CONNECT_BUTTONS_BUTTON;
extern int CONNECT_BASESERVER_BUTTON;
extern int CONNECT_SIMULATOR_BUTTON;
extern int CONNECT_SONARINT_BUTTON;
extern int CONNECT_LASERINT_BUTTON;
extern int CONNECT_LOCALIZE_BUTTON;

extern int CONNECT_CAMERA_BUTTON;
extern int ACQUIRE_CAMERA_IMAGE_BUTTON;
extern int CONTINUOUS_CAMERA_IMAGE_BUTTON;
#ifdef CD_VERSION
extern int CONNECT_CD_BUTTON;
extern int CD_BUTTONS[NUM_CD_TEXTS];
#endif /* CD_VERSION */
#ifdef UNIBONN
extern int LEFT_CAMERA_BUTTON;
extern int RIGHT_CAMERA_BUTTON;
extern int CONNECT_ARM_BUTTON;
extern int CONNECT_FLOW_BUTTON;
extern int CONNECT_SUNVIS_BUTTON;
extern int CONNECT_TRACKER_BUTTON;
extern int CAMERA_X_DISPLAY_BUTTON;
extern int CAMERA_OBJECTS_BUTTON;
extern int CAMERA_MAPS_BUTTON;
extern int CAMERA_COLLI_LINES_BUTTON;
extern int HUNTING_BUTTON;
extern int SONAR_BUTTON;
extern int GOALS;
extern int TARGET_OBJECTS;
extern int NUM_OBJECTS_BUTTON;
extern int TRACKER_BUTTON;

extern int ARM_MOVE_OUT_IN_BUTTON;
extern int ARM_PICK_BUTTON;
extern int ARM_CLOSE_OPEN_GRIPPER_BUTTON;
extern int ARM_MAST_POSITION_BUTTON;
extern int ARM_GRIPPER_ORIENTATION_BUTTON;
extern int ARM_GRIPPER_PAD_POSITION_BUTTON;
extern int ARM_IN_OUT_BUTTON;
#endif /* UNIBONN */


#ifdef TOURGUIDE_VERSION
extern int TOUR_GOALS[MAX_NUM_TOURS];
extern int TOUR_ACTIVE_GOALS[MAX_NUM_TOURS];
extern int TOUR_OBJECTS[MAX_NUM_TOURS];
extern int TOUR_LEARN_BUTTON;
extern int TOUR_GIVE_BUTTON;
extern int TOUR_SAVE_BUTTON;
extern int TOUR_LOAD_BUTTON;
#endif /* TOURGUIDE_VERSION */


extern int TARGET_POINT_LOCAL;
extern int TARGET_POINT_GLOBAL;
extern int CONNECT_MAP_BUTTON;
extern int CONNECT_PLAN_BUTTON;
extern int SONAR_ROBOT_SUBTITLE;
extern int SONAR_ROBOT_SUBTITLE2;
extern int PANTILT_SUBTITLE;
extern int GLOBAL_ROBOT_SUBTITLE;
extern int ROBOT_TRANS_SET_VELOCITY;
extern int ROBOT_ROT_SET_VELOCITY;
extern int ROBOT_TRANS_VELOCITY;
extern int ROBOT_ROT_VELOCITY;
extern int ROBOT_TRANS_SET_ACCELERATION;
extern int ROBOT_ROT_SET_ACCELERATION;
extern int MARKERS;
extern int POSITION_TEXT_BUTTON;
extern int OCCUPANCY_MAP;
extern int DUMP_BUTTON;
extern int AUTONOMOUS_BUTTON;
extern int STATUS_NEW_AREA_BUTTON;
extern int STATUS_TOTAL_AREA_BUTTON;
extern int STATUS_ADVANCEMENT_BUTTON;
extern int BASE_MODE_BUTTON;
extern int EXPL_RESET_BUTTON;
extern int RED_LIGHT_BUTTON;
extern int GREEN_LIGHT_BUTTON;
extern int YELLOW_LIGHT_BUTTON;
extern int BLUE_LIGHT_BUTTON;
extern int RED_PUSHED_BUTTON;
extern int GREEN_PUSHED_BUTTON;
extern int YELLOW_PUSHED_BUTTON;
extern int BLUE_PUSHED_BUTTON;
extern int MAP_UPDATE_BUTTON;
extern int BATTERY_STATUS;
extern int STOP_ROBOT;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


extern float pseudo_picture[PSEUDO_PICTURE_SIZE_HORIZONTAL
			    *PSEUDO_PICTURE_SIZE_VERTICAL];
extern int pseudo_picture_active[PSEUDO_PICTURE_SIZE_HORIZONTAL
				 *PSEUDO_PICTURE_SIZE_VERTICAL];

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/*** coordinates of buttons ****/

#ifdef UNIBONN
#define NUM_ROWS             5	/* you can change this and recompile */
#define NUM_COLUMNS          5  /* you can change this and recompile */
#define BOTTOM_BUTTON_BORDER 4.3 /* you can change this and recompile */
#define NUM_CONNECTS         16
#else
#ifdef TOURGUIDE_VERSION
#define NUM_ROWS             4 	/* you can change this and recompile */
#define BOTTOM_BUTTON_BORDER 3.5 /* you can change this and recompile */
#else
#define NUM_ROWS             3 	/* you can change this and recompile */
#define BOTTOM_BUTTON_BORDER 4.5 /* you can change this and recompile */
#endif /* TOURGUIDE_VERSION */
#define NUM_COLUMNS          4  /* you can change this and recompile */
#define NUM_CONNECTS         12
#endif



#define LEFT_BUTTON_BORDER   -1.0 /* matches control window dimension */
#define RIGHT_BUTTON_BORDER  8.8 /* matches control window dimension */
#define RIGHT_WINDOW_BORDER  12.8 /* matches control window dimension */
#define TOP_BUTTON_BORDER    5.6 /* should not exceed 5.6 */
#define TOP_BUTTON_BORDER2   9.9
#define BUTTON_SEPARATOR     0.08 /* space between buttons,
				   * you can change this and recompile */
#define SLIDER_WIDTH         0.25
#define SLIDER_SEP_SPACE     0.08

#define RIGHT_WINDOW_BORDER2  (((RIGHT_WINDOW_BORDER) + 2 * (SLIDER_WIDTH) + 3 * (SLIDER_SEP_SPACE)) + 0.2)

/****** don't change the following ********/

#define BAR_COLUMN(n) \
  (LEFT_BUTTON_BORDER+\
   ((n)*((RIGHT_BUTTON_BORDER-LEFT_BUTTON_BORDER+BUTTON_SEPARATOR)\
       /NUM_COLUMNS))),\
  (LEFT_BUTTON_BORDER+\
   (((n)+1)*((RIGHT_BUTTON_BORDER-LEFT_BUTTON_BORDER+BUTTON_SEPARATOR)\
	   /NUM_COLUMNS))-BUTTON_SEPARATOR)

#define BAR_ROW(n) \
  (BOTTOM_BUTTON_BORDER+\
   ((NUM_ROWS-(n)-1)*((TOP_BUTTON_BORDER-BOTTOM_BUTTON_BORDER+\
		       BUTTON_SEPARATOR)/NUM_ROWS))),\
  (BOTTOM_BUTTON_BORDER+\
   ((NUM_ROWS-(n))*((TOP_BUTTON_BORDER-BOTTOM_BUTTON_BORDER+BUTTON_SEPARATOR)\
		  /NUM_ROWS))-BUTTON_SEPARATOR)


#define BAR_ROW_3(n) \
  (BOTTOM_BUTTON_BORDER+\
   ((NUM_ROWS-(n+2)-1)*((TOP_BUTTON_BORDER-BOTTOM_BUTTON_BORDER+\
		       BUTTON_SEPARATOR)/NUM_ROWS))),\
  (BOTTOM_BUTTON_BORDER+\
   ((NUM_ROWS-(n))*((TOP_BUTTON_BORDER-BOTTOM_BUTTON_BORDER+BUTTON_SEPARATOR)\
		  /NUM_ROWS))-BUTTON_SEPARATOR)

#define BAR_ROW_4(n) \
  (BOTTOM_BUTTON_BORDER+\
   ((NUM_ROWS-(n+3)-1)*((TOP_BUTTON_BORDER-BOTTOM_BUTTON_BORDER+\
		       BUTTON_SEPARATOR)/NUM_ROWS))),\
  (BOTTOM_BUTTON_BORDER+\
   ((NUM_ROWS-(n))*((TOP_BUTTON_BORDER-BOTTOM_BUTTON_BORDER+BUTTON_SEPARATOR)\
		  /NUM_ROWS))-BUTTON_SEPARATOR)


#define RIGHT_SLIDER(n) \
  RIGHT_BUTTON_BORDER+SLIDER_SEP_SPACE+SLIDER_SEP_SPACE,\
  RIGHT_WINDOW_BORDER-SLIDER_SEP_SPACE,\
  TOP_BUTTON_BORDER-(((n)+1) * (SLIDER_WIDTH+SLIDER_SEP_SPACE)),\
  TOP_BUTTON_BORDER-SLIDER_SEP_SPACE-((n) * (SLIDER_WIDTH+SLIDER_SEP_SPACE))

#define CONNECT_COORDNIATES(n) \
  (LEFT_BUTTON_BORDER+\
   ((n)*((RIGHT_WINDOW_BORDER-LEFT_BUTTON_BORDER+BUTTON_SEPARATOR)\
       /NUM_CONNECTS))),\
  (LEFT_BUTTON_BORDER+\
   (((n)+1)*((RIGHT_WINDOW_BORDER-LEFT_BUTTON_BORDER+BUTTON_SEPARATOR)\
	   /NUM_CONNECTS))-BUTTON_SEPARATOR),\
   BOTTOM_BUTTON_BORDER - 0.3 - BUTTON_SEPARATOR,\
   BOTTOM_BUTTON_BORDER - BUTTON_SEPARATOR


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
 *                 ACTION_PTR         action         Pointer to action 
 *                                                   structure
 *                 SENSARTION_PTR     sensation      Pointer to sensation
 *                                                   description structure
 *                 PROGRAM_STATE_PTR  program_state  Pointer  to general
 *                                                   program state description
 *                                                   structure 
 *                 ROBOT_SPECIFICATIONS_PTR robot_specifications
 *                                                   Pointer to robot spe-
 *                                                   cifications (see above)
 *                                                   
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void init_graphics(ALL_PARAMS);




/************************************************************************
 *
 *   NAME:         select_active_objects()
 *                 
 *   FUNCTION:     depending on the status of the progam, graphic
 *                 objects are activated and de-activated
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void select_active_objects(ALL_PARAMS);



/************************************************************************
 *
 *   NAME:         display_robot_window
 *                 
 *   FUNCTION:     displays the robot's windows, can't you read?
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void display_robot_window(ALL_PARAMS);





/************************************************************************
 *
 *   NAME:         autoshift_display
 *                 
 *   FUNCTION:     Shifts robot display to always keep robot centered
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void autoshift_display(float target_x, float target_y, 
		       float safety_margin, float autoshift_distance,
		       ALL_PARAMS);
