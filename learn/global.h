
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
 ***** Please read and make sure you understand the disclaimer below.
 *****
 ***** Contact thrun@cs.cmu.edu if you have questions or would
 ***** like to obtain further information.
 *****
 ***** We hope you enjoy working with this software package.
 *****
 *****                                     (c) Sebastian Thrun, 1997
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/learn/global.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 16:36:05 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: global.h,v $
 * Revision 1.1  2002/09/14 16:36:05  rstone
 * *** empty log message ***
 *
 * Revision 1.1  1999/07/11 18:48:32  thrun
 * slight reorganization
 *
 * Revision 1.36  1998/08/22 03:08:37  thrun
 * .
 *
 * Revision 1.35  1998/07/04 15:21:38  thrun
 * variable logging.
 *
 * Revision 1.34  1998/07/03 23:59:59  thrun
 * .
 *
 * Revision 1.33  1998/05/05 04:00:34  thrun
 * Direct control mode (and button). Other stuff I don't fully remember.
 * Intermediate version.
 *
 * Revision 1.32  1998/04/28 23:30:46  thrun
 * autosave of weights, we can now switch off learning in selected
 * networks and variables, which is important for incremental
 * programming/learning
 *
 * Revision 1.31  1998/04/26 15:03:08  thrun
 * subroutines appear to work fine. also changed the initialiation.
 *
 * Revision 1.30  1998/04/24 04:49:04  thrun
 * intermediate and buggy version - subroutines don't have their gradient
 * computation quite right yet.
 *
 * Revision 1.29  1998/04/18 20:42:25  thrun
 * 1. neuro net can now be called with the same variable as input
 * and output
 * 2. direct control modus for the robot. controls are
 * now generated exclusively through the display (left upper corner)
 *
 * Revision 1.28  1998/04/14 02:35:56  thrun
 * .
 *
 * Revision 1.27  1998/04/13 01:45:19  thrun
 * Thermomter encoding for sigmodial neural networks. Delay for
 * pausing (when recording data).
 *
 * Revision 1.26  1997/10/23 02:28:23  thrun
 * .
 *
 * Revision 1.25  1997/10/05 18:11:18  thrun
 * new data library "libdat.a"
 *
 * Revision 1.24  1997/08/06 13:56:09  thrun
 * intermediate version (fixed a bug with the name field in variables/networks)
 *
 * Revision 1.23  1997/08/05 04:15:23  thrun
 * (1) avg_value are saved for each stoch variable, (2) enhanced replay
 * functions.
 *
 * Revision 1.22  1997/07/30 23:29:34  thrun
 * recording of base motion commands (velocity).
 *
 * Revision 1.21  1997/07/30 21:02:02  thrun
 * baseServer now sends confirmations of all motion commands. Is used
 * for logging in the module "learn"
 *
 * Revision 1.20  1997/07/30 03:46:53  thrun
 * New events: pantilt and buttons
 *
 * Revision 1.19  1997/07/29 22:44:48  thrun
 * .
 *
 * Revision 1.18  1997/07/26 13:38:49  thrun
 * Reacted to some of Tyson's changes
 *
 * Revision 1.17  1997/07/16 17:09:43  thrun
 * Neat, working version. Simpliefied the application, too.
 *
 * Revision 1.16  1997/07/14 22:17:14  thrun
 * Fixed the bug (I believe). Now comes testing.
 *
 * Revision 1.15  1997/07/07 04:45:23  thrun
 * Now with training and testing set support
 *
 * Revision 1.14  1997/07/06 22:51:49  thrun
 * Autosaving of the parameters (every 2 minutes)
 *
 * Revision 1.13  1997/07/06 18:42:04  thrun
 * Radial basis networks implemented.
 *
 * Revision 1.12  1997/07/04 00:28:55  thrun
 * Intermediate verson of the learning stuff. What works is:
 * - training of deterministic variables (incl. chaining)
 * - training of multiple determinstic with a stochastic variables (incl chaining)
 * What doesn't yet work includes
 * - temporal chaining of stochastic variables, using Bayes rule
 * - convolving: gradients
 * - stochatic input
 *
 * Revision 1.11  1997/06/30 05:53:58  thrun
 * intermediate version - new neural network (which seems to work!).
 *
 * Revision 1.10  1997/06/29 04:04:56  thrun
 * intrmediate version - not really good for anything
 *
 * Revision 1.9  1997/06/28 13:41:42  thrun
 * This is the fully functional recorder/display/analysis tool.
 * Check out this version if you'd like to have it (without any
 * of the learning stuff). What's missing is the ability to feed back
 * the data intp the baseServer, colli and camera. Also missing is a link
 * to the pantilt unit.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/





#define DATA_FILENAME  "learn.dat"
#define DATA_FILENAME2  DATA_FILENAME
#define DATA_FILENAME3  "learn.dat.autosave-%.6d"
#define LOG_FILENAME   "learn.log"
#define NET_LOAD_FILENAME "learn.params"
#define NET_SAVE_FILENAME NET_LOAD_FILENAME
#define NET_BEST_SAVE_FILENAME "learn_best.params"
#define NET_AUTOSAVE_FILENAME "learn_autosave.params"
#define NET_BEST_AUTOSAVE_FILENAME "learn_best_autosave.params"
#define NET_BACKUP_FILENAME   "learn_backup.params"
#define NET_BEST_BACKUP_FILENAME   "learn_best_backup.params"
#define NET_AUTOSAVE_FREQUENCY 120.0 /* in seconds */

#define MIN_TIME_DIFF_SONAR   0.15
#define MIN_TIME_DIFF_LASER   0.15
#define MIN_TIME_DIFF_IR      0.15
#define MIN_TIME_DIFF_CAMERA  0.15
#define MIN_TIME_DIFF_STATUS  0.0
#define MIN_TIME_DIFF_TACTILE 0.0
#define MIN_TIME_DIFF_BUTTONS 0.0
#define MIN_TIME_DIFF_PANTILT 0.0
#define MIN_TIME_DIFF_CONTROL 0.0 /* leave this at 0.0 */
#define MIN_ERROR_REPORT_FREQUENCEY 0.1

#define MIN_STALLED_TIME_FOR_PAUSING 1.0

#define MAX_TRANS_VEL  800.0
#define MAX_ROT_VEL    150.0

#define AUTOSAVE_PERIOD_DURING_RECORDING 300.0 /* in seconds, set to negative
					       * value to deactivate */
#define DELETE_PATTERNS_AFTER_AUTOSAVING 1


#define DEFAULT_AUTOMATIC_BASE_CONNECT      1
#define DEFAULT_AUTOMATIC_CAMERA_CONNECT    1
#define DEFAULT_AUTOMATIC_COLLI_CONNECT     1
#define DEFAULT_AUTOMATIC_BUTTONS_CONNECT   1
#define DEFAULT_AUTOMATIC_PANTILT_CONNECT   1

/*------------------------------------------------------------*/


#define NUM_ROWS_COLUMNS 10
#define HORIZONTAL_SPACE 0.05
#define VERTICAL_SPACE   0.05



/*------------------------------------------------------------*/





#define NUM_BARS 2
#define BARS_HEIGTH (0.5*(NUM_BARS))
#define LEFT_MARGIN -2.8


#define BUTTON_NUM_X 4
#define BUTTON_NUM_Y 5
#define BUTTON_MIN_X (LEFT_MARGIN)
#define BUTTON_MAX_X /*((float) max_x)*/ 11.5
#define BUTTON_MIN_Y (BAR_Y_POS-2.4-(BARS_HEIGTH))
#define BUTTON_MAX_Y (BAR_Y_POS-(BARS_HEIGTH))
#define BUTTON_SEP   0.1
#define BAR_Y_POS 0.4

#define BUTTON_X1(x,y) ((BUTTON_MIN_X)+(((BUTTON_MAX_X)-(BUTTON_MIN_X)+(BUTTON_SEP))/(BUTTON_NUM_X)*(x)))
#define BUTTON_X2(x,y) ((BUTTON_MIN_X)+((((BUTTON_MAX_X)-(BUTTON_MIN_X)+(BUTTON_SEP))/(BUTTON_NUM_X)*((x+1)))-(BUTTON_SEP)))
#define BUTTON_Y1(x,y) ((BUTTON_MAX_Y)-((((BUTTON_MAX_Y)-(BUTTON_MIN_Y)+(BUTTON_SEP))/(BUTTON_NUM_Y)*((y+1)))-(BUTTON_SEP)))
#define BUTTON_Y2(x,y) ((BUTTON_MAX_Y)-(((BUTTON_MAX_Y)-(BUTTON_MIN_Y)+(BUTTON_SEP))/(BUTTON_NUM_Y)*(y)))

#define SET_BUTTON(x,y) {switch_pos[0]=BUTTON_X1(x,y); switch_pos[1]=BUTTON_X2(x,y); switch_pos[2]=BUTTON_Y1(x,y); switch_pos[3]=BUTTON_Y2(x,y); }


#define SET_BAR_BUTTON1(y)  {switch_pos[0]=(BUTTON_MIN_X); switch_pos[1]=(BUTTON_MIN_X)+0.4; switch_pos[2]=BAR_Y_POS-0.4-(0.5*(y)); switch_pos[3]=BAR_Y_POS-(0.5*(y)); }
#define SET_BAR_BUTTON2(y)  {switch_pos[0]=(BUTTON_MIN_X)+0.5; switch_pos[1]=(BUTTON_MIN_X)+0.9; switch_pos[2]=BAR_Y_POS-0.4-(0.5*(y)); switch_pos[3]=BAR_Y_POS-(0.5*(y)); }
#define SET_BAR_BUTTON3(y)  {switch_pos[0]=(BUTTON_MAX_X)-1.4; switch_pos[1]=(BUTTON_MAX_X)-1.0; switch_pos[2]=BAR_Y_POS-0.4-(0.5*(y)); switch_pos[3]=BAR_Y_POS-(0.5*(y)); }
#define SET_BAR_BUTTON4(y)  {switch_pos[0]=(BUTTON_MAX_X)-0.9; switch_pos[1]=(BUTTON_MAX_X); switch_pos[2]=BAR_Y_POS-0.4-(0.5*(y)); switch_pos[3]=BAR_Y_POS-(0.5*(y)); }
#define SET_BAR(y) {switch_pos[0]=(BUTTON_MIN_X)+1.0; switch_pos[1]=(BUTTON_MAX_X)-1.5; switch_pos[2]=BAR_Y_POS-0.35-(0.5*(y)); switch_pos[3]=BAR_Y_POS-0.05-(0.5*(y)); }





/************************************************************************\
 ************************************************************************
\************************************************************************/



extern float *sonar_angles;
extern float *sonar_values;		/* purely for display */
extern float *ir_values;
extern float *ir_angles;
extern float *tactile_values;
extern float *tactile_angles;
extern float *laser_values;
extern float *laser_angles;
extern float *d_image;		/* purely for display */




extern int global_modus_recording;
extern int global_modus_suspend_recording_when_not_moving;
extern int global_modus_execution;
extern int global_modus_logging;
extern int global_modus_live;
extern int global_modus_movie;
extern int global_modus_real_time_display;

extern int global_use_tcx;
extern int global_use_X;
extern int global_prob_c;
extern int global_small_X;

extern int automatic_base_connect;
extern int automatic_camera_connect;
extern int automatic_colli_connect;
extern int automatic_buttons_connect;
extern int automatic_pantilt_connect;


extern int global_graphics_initialized;
extern int global_modus_training;

extern FILE *log_iop;

extern int global_modus_control;
extern int global_modus_marking;
extern struct timeval global_movie_start_time;
extern struct timeval global_movie_start_time_tag;
extern struct timeval last_autosave_time;


/*extern int cameraRequestPending;*/
extern int baseConnected;
extern int colliConnected;
extern int pantiltConnected;

extern int first_irs;
extern int first_sonars;
extern int first_tactiles;
extern int first_lasers;
extern int first_buttons;
extern int first_pantilt;

/*------------------------------------------------------------*/

extern control_type desired_control;

/************************************************************************\
 ************************************************************************
\************************************************************************/

#define IMAGE_WINDOW   0
#define SONAR_WINDOW   1
#define LASER_WINDOW   2
#define IR_WINDOW      3
#define TACTILE_WINDOW 4
#define CONTROL_WINDOW 5


/************************************************************************\
 ************************************************************************
\************************************************************************/


extern int CONTROL_ROBOT1;
extern int CONTROL_ROBOT2;
extern int CONTROL_DOTS;
extern int CONTROL_FRAME;
extern int SONAR_ROBOT;
extern int LASER_ROBOT;
extern int IR_ROBOT1;
extern int IR_ROBOT2;
extern int IR_ROBOT3;
extern int TACTILE_ROBOT1;
extern int TACTILE_ROBOT2;
extern int TACTILE_ROBOT3;
extern int TACTILE_ROBOT4;
extern int TACTILE_ROBOT;
extern int POSITION_DATA;
extern int IMAGE;
extern int PAN_DIAL;
extern int TILT_DIAL;
extern int TIME_DATA;
extern int TRANS_VEL_POS;
extern int TRANS_VEL_NEG;
extern int ROT_VEL_POS;
extern int ROT_VEL_NEG;
extern int QUIT_BUTTON;

extern int CONTROL_MARKERS;
extern int SONAR_MARKERS;
extern int LASER_MARKERS;
extern int IR_MARKERS;
extern int TACTILE_MARKERS;
extern int IMAGE_MARKERS;

extern int SONAR_TITLE;
extern int LASER_TITLE;
extern int IR_TITLE;
extern int TACTILE_TITLE;

extern int DATABASE_SIZE_BUTTON;
extern int DISPLAY_MODUS_BUTTON;
extern int RECORD_BUTTON;
extern int EXECUTION_BUTTON;
extern int LOGGING_BUTTON;

extern int FIRST_BUTTON;
extern int PREVIOUS_BUTTON;
extern int CURRENT_BUTTON;
extern int NEXT_BUTTON;
extern int SAVE_BUTTON;
extern int LOAD_BUTTON;
extern int APPEND_BUTTON;
extern int MOVIE_BUTTON;
extern int DIAL_BUTTON;

extern int SET_FIRST_BUTTON;
extern int SET_PREVIOUS_BUTTON;
extern int SET_NEXT_BUTTON;
extern int SET_DIAL_BUTTON;
extern int SET_MOVIE_BUTTON;

extern int BASE_CONNECT_BUTTON;
extern int CAMERA_CONNECT_BUTTON;
extern int COLLI_CONNECT_BUTTON;
extern int BUTTONS_CONNECT_BUTTON;
extern int PANTILT_CONNECT_BUTTON;

extern int MARK_BUTTON;
extern int TRAIN_BUTTON;
extern int TEST_BUTTON;
extern int SAVE_NET_BUTTON;
extern int LOAD_NET_BUTTON;

extern int BUTTON_LIT[6];
extern int BUTTON_PUSHED[4];

extern float graphics_max_x;	/* for QUIT_BUTTON */





/************************************************************************\
 ************************************************************************
\************************************************************************/

void stdin_inputHnd(int fd, long chars_available);

/************************************************************************\
 ************************************************************************
\************************************************************************/


typedef struct {
  int   row;
  int   column;
  float angle;
} sensor_location_type;


#ifdef DEFINE_SENSOR_LOCATIONS


sensor_location_type sonar_locations[NUM_SONAR_SENSORS] = 
{
  {0, 0, 172.5},
  {0, 1, 157.5},
  {0, 2, 142.5},
  {0, 3, 127.5},
  {0, 4, 112.5},
  {0, 5, 97.5},
  {0, 6, 82.5},
  {0, 7, 67.5},
  {0, 8, 52.5},
  {0, 9, 37.5},
  {0, 10, 22.5},
  {0, 11, 7.5},
  {0, 12, 352.5},
  {0, 13, 337.5},
  {0, 14, 322.5},
  {0, 15, 307.5},
  {0, 16, 292.5},
  {0, 17, 277.5},
  {0, 18, 262.5},
  {0, 19, 247.5},
  {0, 20, 232.5},
  {0, 21, 217.5},
  {0, 22, 202.5},
  {0, 23, 187.5}
};


sensor_location_type ir_locations[NUM_IR_SENSORS] = 
{
  {0, 0, 352.5},
  {0, 1, 337.5},
  {0, 2, 322.5},
  {0, 3, 307.5},
  {0, 4, 292.5},
  {0, 5, 277.5},
  {0, 6, 262.5},
  {0, 7, 247.5},
  {0, 8, 232.5},
  {0, 9, 217.5},
  {0, 10, 202.5},
  {0, 11, 187.5},
  {0, 12, 172.5},
  {0, 13, 157.5},
  {0, 14, 142.5},
  {0, 15, 127.5},
  {0, 16, 112.5},
  {0, 17, 97.5},
  {0, 18, 82.5},
  {0, 19, 67.5},
  {0, 20, 52.5},
  {0, 21, 37.5},
  {0, 22, 22.5},
  {0, 23, 7.5},
  {1, 0, 172.5},
  {1, 1, 157.5},
  {1, 2, 142.5},
  {1, 3, 127.5},
  {1, 4, 112.5},
  {1, 5, 97.5},
  {1, 6, 82.5},
  {1, 7, 67.5},
  {1, 8, 52.5},
  {1, 9, 37.5},
  {1, 10, 22.5},
  {1, 11, 7.5},
  {1, 12, 352.5},
  {1, 13, 337.5},
  {1, 14, 322.5},
  {1, 15, 307.5},
  {1, 16, 292.5},
  {1, 17, 277.5},
  {1, 18, 262.5},
  {1, 19, 247.5},
  {1, 20, 232.5},
  {1, 21, 217.5},
  {1, 22, 202.5},
  {1, 23, 187.5},
  {2, 0, 135.0},
  {2, 1, 90.0},
  {2, 2, 45.0},
  {2, 3, 0.0},
  {2, 4, 315.0},
  {2, 5, 270.0},
  {2, 6, 235.0},
  {2, 7, 180.0}
};

sensor_location_type tactile_locations[NUM_TACTILE_SENSORS] = 
{
  {0, 0, 11.25},
  {0, 1, 33.75},
  {0, 2, 56.25},
  {0, 3, 78.75},
  {0, 4, 101.25},
  {0, 5, 123.75},
  {0, 6, 146.25},
  {0, 7, 168.75},
  {0, 8, 191.25},
  {0, 9, 213.75},
  {0, 10, 236.25},
  {0, 11, 258.75},
  {0, 12, 281.25},
  {0, 13, 303.75},
  {0, 14, 326.25},
  {0, 15, 348.75},
  {1, 0, 11.25},
  {1, 1, 33.75},
  {1, 2, 56.25},
  {1, 3, 78.75},
  {1, 4, 101.25},
  {1, 5, 123.75},
  {1, 6, 146.25},
  {1, 7, 168.75},
  {1, 8, 191.25},
  {1, 9, 213.75},
  {1, 10, 236.25},
  {1, 11, 258.75},
  {1, 12, 281.25},
  {1, 13, 303.75},
  {1, 14, 326.25},
  {1, 15, 348.75},
  {2, 0, 11.25},
  {2, 1, 33.75},
  {2, 2, 56.25},
  {2, 3, 78.75},
  {2, 4, 101.25},
  {2, 5, 123.75},
  {2, 6, 146.25},
  {2, 7, 168.75},
  {2, 8, 191.25},
  {2, 9, 213.75},
  {2, 10, 236.25},
  {2, 11, 258.75},
  {2, 12, 281.25},
  {2, 13, 303.75},
  {2, 14, 326.25},
  {2, 15, 348.75},
  {3, 0, 11.25},
  {3, 1, 33.75},
  {3, 2, 56.25},
  {3, 3, 78.75},
  {3, 4, 101.25},
  {3, 5, 123.75},
  {3, 6, 146.25},
  {3, 7, 168.75},
  {3, 8, 191.25},
  {3, 9, 213.75},
  {3, 10, 236.25},
  {3, 11, 258.75},
  {3, 12, 281.25},
  {3, 13, 303.75},
  {3, 14, 326.25},
  {3, 15, 348.75}
};

sensor_location_type laser_locations[NUM_LASER_SENSORS] = 
{
  {0, 0, 271.0},
  {0, 1, 272.0},
  {0, 2, 273.0},
  {0, 3, 274.0},
  {0, 4, 275.0},
  {0, 5, 276.0},
  {0, 6, 277.0},
  {0, 7, 278.0},
  {0, 8, 279.0},
  {0, 9, 280.0},
  {0, 10, 281.0},
  {0, 11, 282.0},
  {0, 12, 283.0},
  {0, 13, 284.0},
  {0, 14, 285.0},
  {0, 15, 286.0},
  {0, 16, 287.0},
  {0, 17, 288.0},
  {0, 18, 289.0},
  {0, 19, 290.0},
  {0, 20, 291.0},
  {0, 21, 292.0},
  {0, 22, 293.0},
  {0, 23, 294.0},
  {0, 24, 295.0},
  {0, 25, 296.0},
  {0, 26, 297.0},
  {0, 27, 298.0},
  {0, 28, 299.0},
  {0, 29, 300.0},
  {0, 30, 301.0},
  {0, 31, 302.0},
  {0, 32, 303.0},
  {0, 33, 304.0},
  {0, 34, 305.0},
  {0, 35, 306.0},
  {0, 36, 307.0},
  {0, 37, 308.0},
  {0, 38, 309.0},
  {0, 39, 310.0},
  {0, 40, 311.0},
  {0, 41, 312.0},
  {0, 42, 313.0},
  {0, 43, 314.0},
  {0, 44, 315.0},
  {0, 45, 316.0},
  {0, 46, 317.0},
  {0, 47, 318.0},
  {0, 48, 319.0},
  {0, 49, 320.0},
  {0, 50, 321.0},
  {0, 51, 322.0},
  {0, 52, 323.0},
  {0, 53, 324.0},
  {0, 54, 325.0},
  {0, 55, 326.0},
  {0, 56, 327.0},
  {0, 57, 328.0},
  {0, 58, 329.0},
  {0, 59, 330.0},
  {0, 60, 331.0},
  {0, 61, 332.0},
  {0, 62, 333.0},
  {0, 63, 334.0},
  {0, 64, 335.0},
  {0, 65, 336.0},
  {0, 66, 337.0},
  {0, 67, 338.0},
  {0, 68, 339.0},
  {0, 69, 340.0},
  {0, 70, 341.0},
  {0, 71, 342.0},
  {0, 72, 343.0},
  {0, 73, 344.0},
  {0, 74, 345.0},
  {0, 75, 346.0},
  {0, 76, 347.0},
  {0, 77, 348.0},
  {0, 78, 349.0},
  {0, 79, 350.0},
  {0, 80, 351.0},
  {0, 81, 352.0},
  {0, 82, 353.0},
  {0, 83, 354.0},
  {0, 84, 355.0},
  {0, 85, 356.0},
  {0, 86, 357.0},
  {0, 87, 358.0},
  {0, 88, 359.0},
  {0, 89, 0.0},
  {0, 90, 1.0},
  {0, 91, 2.0},
  {0, 92, 3.0},
  {0, 93, 4.0},
  {0, 94, 5.0},
  {0, 95, 6.0},
  {0, 96, 7.0},
  {0, 97, 8.0},
  {0, 98, 9.0},
  {0, 99, 10.0},
  {0, 100, 11.0},
  {0, 101, 12.0},
  {0, 102, 13.0},
  {0, 103, 14.0},
  {0, 104, 15.0},
  {0, 105, 16.0},
  {0, 106, 17.0},
  {0, 107, 18.0},
  {0, 108, 19.0},
  {0, 109, 20.0},
  {0, 110, 21.0},
  {0, 111, 22.0},
  {0, 112, 23.0},
  {0, 113, 24.0},
  {0, 114, 25.0},
  {0, 115, 26.0},
  {0, 116, 27.0},
  {0, 117, 28.0},
  {0, 118, 29.0},
  {0, 119, 30.0},
  {0, 120, 31.0},
  {0, 121, 32.0},
  {0, 122, 33.0},
  {0, 123, 34.0},
  {0, 124, 35.0},
  {0, 125, 36.0},
  {0, 126, 37.0},
  {0, 127, 38.0},
  {0, 128, 39.0},
  {0, 129, 40.0},
  {0, 130, 41.0},
  {0, 131, 42.0},
  {0, 132, 43.0},
  {0, 133, 44.0},
  {0, 134, 45.0},
  {0, 135, 46.0},
  {0, 136, 47.0},
  {0, 137, 48.0},
  {0, 138, 49.0},
  {0, 139, 50.0},
  {0, 140, 51.0},
  {0, 141, 52.0},
  {0, 142, 53.0},
  {0, 143, 54.0},
  {0, 144, 55.0},
  {0, 145, 56.0},
  {0, 146, 57.0},
  {0, 147, 58.0},
  {0, 148, 59.0},
  {0, 149, 60.0},
  {0, 150, 61.0},
  {0, 151, 62.0},
  {0, 152, 63.0},
  {0, 153, 64.0},
  {0, 154, 65.0},
  {0, 155, 66.0},
  {0, 156, 67.0},
  {0, 157, 68.0},
  {0, 158, 69.0},
  {0, 159, 70.0},
  {0, 160, 71.0},
  {0, 161, 72.0},
  {0, 162, 73.0},
  {0, 163, 74.0},
  {0, 164, 75.0},
  {0, 165, 76.0},
  {0, 166, 77.0},
  {0, 167, 78.0},
  {0, 168, 79.0},
  {0, 169, 80.0},
  {0, 170, 81.0},
  {0, 171, 82.0},
  {0, 172, 83.0},
  {0, 173, 84.0},
  {0, 174, 85.0},
  {0, 175, 86.0},
  {0, 176, 87.0},
  {0, 177, 88.0},
  {0, 178, 89.0},
  {0, 179, 90.0},
  {1, 180, 91.0},
  {1, 181, 92.0},
  {1, 182, 93.0},
  {1, 183, 94.0},
  {1, 184, 95.0},
  {1, 185, 96.0},
  {1, 186, 97.0},
  {1, 187, 98.0},
  {1, 188, 99.0},
  {1, 189, 100.0},
  {1, 190, 101.0},
  {1, 191, 102.0},
  {1, 192, 103.0},
  {1, 193, 104.0},
  {1, 194, 105.0},
  {1, 195, 106.0},
  {1, 196, 107.0},
  {1, 197, 108.0},
  {1, 198, 109.0},
  {1, 199, 110.0},
  {1, 200, 111.0},
  {1, 201, 112.0},
  {1, 202, 113.0},
  {1, 203, 114.0},
  {1, 204, 115.0},
  {1, 205, 116.0},
  {1, 206, 117.0},
  {1, 207, 118.0},
  {1, 208, 119.0},
  {1, 209, 120.0},
  {1, 210, 121.0},
  {1, 211, 122.0},
  {1, 212, 123.0},
  {1, 213, 124.0},
  {1, 214, 125.0},
  {1, 215, 126.0},
  {1, 216, 127.0},
  {1, 217, 128.0},
  {1, 218, 129.0},
  {1, 219, 130.0},
  {1, 220, 131.0},
  {1, 221, 132.0},
  {1, 222, 133.0},
  {1, 223, 134.0},
  {1, 224, 135.0},
  {1, 225, 136.0},
  {1, 226, 137.0},
  {1, 227, 138.0},
  {1, 228, 139.0},
  {1, 229, 140.0},
  {1, 230, 141.0},
  {1, 231, 142.0},
  {1, 232, 143.0},
  {1, 233, 144.0},
  {1, 234, 145.0},
  {1, 235, 146.0},
  {1, 236, 147.0},
  {1, 237, 148.0},
  {1, 238, 149.0},
  {1, 239, 150.0},
  {1, 240, 151.0},
  {1, 241, 152.0},
  {1, 242, 153.0},
  {1, 243, 154.0},
  {1, 244, 155.0},
  {1, 245, 156.0},
  {1, 246, 157.0},
  {1, 247, 158.0},
  {1, 248, 159.0},
  {1, 249, 160.0},
  {1, 250, 161.0},
  {1, 251, 162.0},
  {1, 252, 163.0},
  {1, 253, 164.0},
  {1, 254, 165.0},
  {1, 255, 166.0},
  {1, 256, 167.0},
  {1, 257, 168.0},
  {1, 258, 169.0},
  {1, 259, 170.0},
  {1, 260, 171.0},
  {1, 261, 172.0},
  {1, 262, 173.0},
  {1, 263, 174.0},
  {1, 264, 175.0},
  {1, 265, 176.0},
  {1, 266, 177.0},
  {1, 267, 178.0},
  {1, 268, 179.0},
  {1, 269, 180.0},
  {1, 270, 181.0},
  {1, 271, 182.0},
  {1, 272, 183.0},
  {1, 273, 184.0},
  {1, 274, 185.0},
  {1, 275, 186.0},
  {1, 276, 187.0},
  {1, 277, 188.0},
  {1, 278, 189.0},
  {1, 279, 190.0},
  {1, 280, 191.0},
  {1, 281, 192.0},
  {1, 282, 193.0},
  {1, 283, 194.0},
  {1, 284, 195.0},
  {1, 285, 196.0},
  {1, 286, 197.0},
  {1, 287, 198.0},
  {1, 288, 199.0},
  {1, 289, 200.0},
  {1, 290, 201.0},
  {1, 291, 202.0},
  {1, 292, 203.0},
  {1, 293, 204.0},
  {1, 294, 205.0},
  {1, 295, 206.0},
  {1, 296, 207.0},
  {1, 297, 208.0},
  {1, 298, 209.0},
  {1, 299, 210.0},
  {1, 300, 211.0},
  {1, 301, 212.0},
  {1, 302, 213.0},
  {1, 303, 214.0},
  {1, 304, 215.0},
  {1, 305, 216.0},
  {1, 306, 217.0},
  {1, 307, 218.0},
  {1, 308, 219.0},
  {1, 309, 220.0},
  {1, 310, 221.0},
  {1, 311, 222.0},
  {1, 312, 223.0},
  {1, 313, 224.0},
  {1, 314, 225.0},
  {1, 315, 226.0},
  {1, 316, 227.0},
  {1, 317, 228.0},
  {1, 318, 229.0},
  {1, 319, 230.0},
  {1, 320, 231.0},
  {1, 321, 232.0},
  {1, 322, 233.0},
  {1, 323, 234.0},
  {1, 324, 235.0},
  {1, 325, 236.0},
  {1, 326, 237.0},
  {1, 327, 238.0},
  {1, 328, 239.0},
  {1, 329, 240.0},
  {1, 330, 241.0},
  {1, 331, 242.0},
  {1, 332, 243.0},
  {1, 333, 244.0},
  {1, 334, 245.0},
  {1, 335, 246.0},
  {1, 336, 247.0},
  {1, 337, 248.0},
  {1, 338, 249.0},
  {1, 339, 250.0},
  {1, 340, 251.0},
  {1, 341, 252.0},
  {1, 342, 253.0},
  {1, 343, 254.0},
  {1, 344, 255.0},
  {1, 345, 256.0},
  {1, 346, 257.0},
  {1, 347, 258.0},
  {1, 348, 259.0},
  {1, 349, 260.0},
  {1, 350, 261.0},
  {1, 351, 262.0},
  {1, 352, 263.0},
  {1, 353, 264.0},
  {1, 354, 265.0},
  {1, 355, 266.0},
  {1, 356, 267.0},
  {1, 357, 268.0},
  {1, 358, 269.0},
  {1, 359, 270.0}
};

#else

sensor_location_type sonar_locations[NUM_SONAR_SENSORS];

extern sensor_location_type ir_locations[NUM_IR_SENSORS];

extern sensor_location_type tactile_locations[NUM_TACTILE_SENSORS];

extern sensor_location_type laser_locations[NUM_LASER_SENSORS];

#endif

/************************************************************************\
 ************************************************************************
\************************************************************************/



void
init_graphics(int global_use_X);

void
init_graphics_2(int global_use_X);

int 
mouse_test_loop();

void
exit_proc(int garbage);

int
main(int argc, char *argv[]);

void
main_loop();

void
init_bee();

void
connect_to_camera_server();

void
connect_to_base_server();

void
connect_to_colli_server();

void
connect_to_buttons_server();

void
connect_to_pantilt_server();

void
set_velocity(float desired_trans_velocity,
	     float desired_rot_velocity,
	     int immediate);


void
init_all(int argc, char *argv[]);
