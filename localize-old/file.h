
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** Welcome!
 *****
 ***** This file is part of the robotcontrol software provided
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/file.h,v $
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
 * $Log: file.h,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.70  1999/11/02 18:12:33  fox
 * Version of multi localize journal paper.
 *
 * Revision 1.69  1999/10/21 17:30:43  fox
 * Fixed some bugs in real time scripts.
 *
 * Revision 1.68  1999/10/02 09:06:43  thrun
 * New robot type: XR4000.
 *
 * Revision 1.67  1999/09/29 16:06:09  fox
 * Should work.
 *
 * Revision 1.66  1999/09/26 18:55:21  fox
 * Added scout robot.
 *
 * Revision 1.65  1999/09/01 00:02:56  fox
 * Getting closer. But still not close enough.
 *
 * Revision 1.64  1999/07/13 23:08:01  fox
 * Some changes.
 *
 * Revision 1.63  1999/06/25 19:48:11  fox
 * Minor changs for the urbie.
 *
 * Revision 1.62  1999/06/24 00:21:49  fox
 * Some changes for the urbies.
 *
 * Revision 1.61  1999/06/23 16:22:01  fox
 * Added robot type urban.
 *
 * Revision 1.60  1999/05/21 14:48:46  fox
 * Added SEND_REPORT keyword.
 *
 * Revision 1.59  1999/05/18 15:15:19  fox
 * Added keywords to determine whether correction parameters should be sent to map and plan or not.
 *
 * Revision 1.58  1999/04/29 13:35:20  fox
 * Further adaptation to make multi localize run.
 *
 * Revision 1.57  1999/04/18 19:00:09  fox
 * This is just to get the most recent version into the cmu bee tree. It shouldn't do any harm (hopefully!).
 *
 * Revision 1.56  1999/03/01 17:44:28  wolfram
 * Added support for Pioneer II.  Pioneer I will be added next
 *
 * Revision 1.55  1999/01/14 23:39:30  wolfram
 * Added sensorMaxRange and standardDeviaton to the sensor model for lasers
 *
 * Revision 1.54  1999/01/11 19:47:48  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.52  1999/01/08 22:28:45  wolfram
 * Better integration of scanAlignment
 *
 * Revision 1.51  1998/11/23 19:45:06  fox
 * Latest version.
 *
 * Revision 1.50  1998/11/17 23:26:18  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.49  1998/11/03 21:02:17  fox
 * Lasers will not be integrated if the robot rotates faster than
 * MAX_ROT_VEL_FOR_LASERS deg/sec.
 *
 * Revision 1.48  1998/10/29 03:44:59  fox
 * Nothing special.
 *
 * Revision 1.47  1998/10/02 15:16:37  fox
 * Several improvements of condensation. Updated the function calls to the
 * new version of the distancServer.
 *
 * Revision 1.46  1998/09/25 04:02:53  fox
 * First version of CONDENSATION!!!!
 *
 * Revision 1.45  1998/09/18 17:24:42  fox
 * Added skeleton files for condensation.
 *
 * Revision 1.44  1998/08/26 15:34:03  wolfram
 * Finished integration of vision
 *
 * Revision 1.43  1998/08/23 00:00:58  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.42  1998/08/20 00:22:57  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.41  1998/08/19 16:33:54  fox
 * Minor changes. Position is dumped upon exit of tcxServer.
 *
 * Revision 1.40  1998/06/12 10:16:28  fox
 * Implemented virutal sensor.
 *
 * Revision 1.39  1998/04/06 19:44:12  wolfram
 * Added -robot option for multi-robot support
 *
 * Revision 1.38  1998/01/27 15:25:24  fox
 * Minor changes.
 *
 * Revision 1.37  1998/01/05 10:37:11  fox
 * Several changes especially in selection parts.
 *
 * Revision 1.36  1997/12/17 16:48:29  wolfram
 * Added NEIGHBOR_PLANES_TO_ADD keyword for normalize
 *
 * Revision 1.35  1997/12/11 17:06:29  fox
 * Added some parameters.
 *
 * Revision 1.34  1997/12/03 09:09:07  fox
 * Renamde USE_MOVEMENT to USE_POSITION.
 *
 * Revision 1.33  1997/12/02 15:20:37  fox
 * Nothing remarkable.
 *
 * Revision 1.32  1997/11/25 17:12:47  fox
 * Should work.
 *
 * Revision 1.31  1997/11/07 12:39:38  fox
 * Added some graphic features.
 *
 * Revision 1.30  1997/09/26 17:02:08  wolfram
 * Intermediate version, do not use
 *
 * Revision 1.29  1997/09/11 20:39:09  fox
 * Added possibility to read script from stdin.
 *
 * Revision 1.28  1997/09/09 19:45:11  fox
 * Fixed a bug in finding local maxima and did several changes to active
 * localization.
 *
 * Revision 1.27  1997/08/02 16:51:02  wolfram
 * 1. Changed the order of indexes of the grid (and only of the grid):
 * The order now is grid->prob[rot][x][y].  This results in a significant
 * speed-up for different operations such as normalization and
 * integration of new sensory data.  Reimplemented the ConvolveThirdDim
 * procedure for convolving over rot.
 *
 * 2. Changed the algorithm to detect linear alignments of readings.  Now
 * we use the approach of Lu.
 *
 * 3. Linear alignments of readings is also checked for laser readings.
 *
 * 4. Expected distances are now computed given the simulator map if
 * available.  For that purpose the library libGetDistance is included.
 *
 * 5. Graphic output now displays the simulator map (if available). This
 * concernes the map overlay as well as the robot window.
 *
 * 6. Fixed some minor bugs.
 *
 * 7. Added different parameters to the ini-file (see example.ini).
 *
 * Revision 1.26  1997/07/04 17:29:13  fox
 * Final version before holiday!!!
 *
 * Revision 1.25  1997/06/26 11:53:43  fox
 * Minor changes.
 *
 * Revision 1.24  1997/06/03 11:49:21  fox
 * Museum version.
 *
 * Revision 1.23  1997/05/26 08:47:46  fox
 * Last version before major changes.
 *
 * Revision 1.22  1997/04/24 21:25:42  wolfram
 * Ooops, fourth parameter of CUBE's is radius and not diameter!
 *
 * Revision 1.21  1997/03/24 06:55:28  wolfram
 * Cleaned up graphic.c and added a graphic window as a global variable
 * in graphic.c
 *
 * Revision 1.20  1997/03/19 17:52:41  fox
 * New laser parameters.
 *
 * Revision 1.19  1997/03/14 11:20:53  wolfram
 * Added windows for different maps
 *
 * Revision 1.18  1997/03/13 17:36:35  fox
 * Temporary version. Don't use!
 *
 * Revision 1.17  1997/01/30 13:34:12  fox
 * Minor changes.
 *
 * Revision 1.16  1997/01/29 12:23:06  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.15  1997/01/16 19:43:23  fox
 * And another bug ...
 *
 * Revision 1.14  1997/01/16 12:42:48  fox
 * Fixed a bug in scan masks.
 *
 * Revision 1.13  1997/01/08 15:52:55  fox
 * Rotations are not any longer treated like movements.
 * Send parameters to PLAN.
 *
 * Revision 1.12  1997/01/07 09:50:37  fox
 * Added a new parameter.
 *
 * Revision 1.11  1997/01/06 17:38:40  fox
 * Improved version.
 *
 * Revision 1.10  1997/01/03 18:07:46  fox
 * Successfully localized the robot and moved it into several rooms without
 * loosing the position again.
 *
 * Revision 1.9  1997/01/03 10:09:45  fox
 * First version with exploration.
 *
 * Revision 1.8  1996/12/31 11:43:55  fox
 * First version using RANDOM_MODE of COLLI.
 *
 * Revision 1.7  1996/12/31 09:19:23  fox
 * Last version before I switched exploration mode in PLAN to random
 * mode in COLLI.
 *
 * Revision 1.6  1996/12/20 15:29:38  fox
 * Added four parameters.
 *
 * Revision 1.5  1996/12/02 10:32:04  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.4  1996/11/15 17:44:05  ws
 * *** empty log message ***
 *
 * Revision 1.3  1996/10/25 13:05:22  ws
 * Added MIN_WINDOW_SCALE keyword for map and grid windows
 *
 * Revision 1.2  1996/10/24 12:07:09  fox
 * Fixed a bug.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:32  rhino
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


#ifndef FILE_INCLUDE
#define FILE_INCLUDE

/*---------------------------------------------------
 *---------------------------------------------------
 * LogFile.
 *---------------------------------------------------
 *---------------------------------------------------*/

#include<stdarg.h>

void
openLogFile( char* logFileName);

void
closeLogAndExit( int sig);

int
writeLog(char* fmt,...);

typedef struct {
    char* keyWord;
    char* format;
    void* variable;
    bool  initialized;
} token;

void
readTokens( char* file,
	    token* tokens, int numberOfTokens,
	    bool allTokenHaveToBeGiven);

void
setTokensInitialized(token *tok, int numberOfValues);

void
getInitValuesFloat( char *string, float *value, int size);

void
getDirectory(char *name);

void
dumpDistanceToMarker( realPosition estimatedPosition, int markerNumber);


realPosition
markerPosition( int markerNumber);


#define INT_FORMAT "%d"
#define FLOAT_FORMAT "%f"
#define STRING_FORMAT "%s"
#define MULTI_VALUE_FORMAT ""

/*---------------------------------------------------
 *---------------------------------------------------
 * All the following keywords have to be given in
 * the parameter file.
 *---------------------------------------------------
 *---------------------------------------------------*/


/*---------------------------------------------------
 * map parameters
 *---------------------------------------------------*/
#define NUMBER_OF_MAP_PARAMETERS 6

#define MAP_KEYWORD                             "MAP_FILE"
#define MAP_UNKNOWN_KEYWORD                     "MAP_UNKNOWN"
#define CAD_MAP_KEYWORD                         "CAD_MAP"
#define DESIRED_RESOLUTION_KEYWORD              "DESIRED_RESOLUTION"
#define SECURITY_DIST_KEYWORD                   "PLAN_SECURITY_DISTANCE"
#define ONLINE_LOCALIZATION_KEYWORD             "ONLINE_LOCALIZATION"

/* This name has to be used for the map if it is provided by the map builder. */
#define ONLINE_MAP                              "ONLINE"

/*---------------------------------------------------
 * probGrid parameters
 *---------------------------------------------------*/
#define NUMBER_OF_PROBGRID_PARAMETERS 16

#define USE_PROBGRID_KEYWORD                    "USE_PROBGRID"
#define NUMBER_OF_ANGLES_KEYWORD                "NUMBER_OF_ANGLES"
#define MAX_QUOTA_OF_PLANES_FOR_NORMALIZATION_KEYWORD "MAX_QUOTA_OF_PLANES_FOR_NORMALIZATION"
#define MIN_SHIFT_FOR_RESET_KEYWORD             "MIN_SHIFT_FOR_RESET"
#define NUMBER_OF_SHIFTS_FOR_NORMALIZE_KEYWORD  "NUMBER_OF_SHIFTS_FOR_NORMALIZE"
#define NUMBER_OF_SONARS_FOR_NORMALIZE_KEYWORD  "NUMBER_OF_SONARS_FOR_NORMALIZE"
#define NUMBER_OF_ANGLES_FOR_NORMALIZE_KEYWORD  "NUMBER_OF_ANGLES_FOR_NORMALIZE"
#define NUMBER_OF_LASERS_FOR_NORMALIZE_KEYWORD  "NUMBER_OF_LASERS_FOR_NORMALIZE"
#define COMPUTE_STATISTICS_KEYWORD              "COMPUTE_STATISTICS"
#define SET_START_POS_KEYWORD                   "SET_START_POS"
#define START_X_KEYWORD                         "START_X"
#define START_Y_KEYWORD                         "START_Y"
#define START_ROT_KEYWORD                       "START_ROT"
#define NEIGHBOR_PLANES_TO_ADD_KEYWORD          "NEIGHBOR_PLANES_TO_ADD"
#define GENERATE_VIRTUAL_SENSOR_KEYWORD         "GENERATE_VIRTUAL_SENSOR"
#define NUMBER_OF_VISIONS_FOR_NORMALIZE_KEYWORD  "NUMBER_OF_VISIONS_FOR_NORMALIZE"

/*---------------------------------------------------
 * probGrid parameters
 *---------------------------------------------------*/
#define NUMBER_OF_CONDENSATION_PARAMETERS 19

/* Five token are taken from probGrid. */
  
#define NUMBER_OF_SAMPLES_KEYWORD                "NUMBER_OF_SAMPLES"
#define SAMPLING_DISTANCE_NOISE_KEYWORD          "SAMPLING_DISTANCE_NOISE"
#define SAMPLING_ANGLE_NOISE_KEYWORD             "SAMPLING_ANGLE_NOISE"
#define SAMPLING_SIDE_DRIFT_KEYWORD              "SAMPLING_SIDE_DRIFT"
#define SAMPLING_ROT_DRIFT_KEYWORD               "SAMPLING_ROT_DRIFT"
#define DUMP_SAMPLES_KEYWORD                     "DUMP_SAMPLES"
#define VARIABLE_SAMPLE_SIZE_KEYWORD             "VARIABLE_SAMPLE_SIZE"
#define MIN_NUMBER_OF_SAMPLES_KEYWORD            "MIN_NUMBER_OF_SAMPLES"
#define SAMPLE_INTEGRATION_THRESHOLD_KEYWORD     "SAMPLE_INTEGRATION_THRESHOLD"
#define FRACTION_OF_UNIFORM_SAMPLES_KEYWORD      "FRACTION_OF_UNIFORM_SAMPLES"
#define LOAD_SAMPLES_FILE_KEYWORD                 "LOAD_SAMPLES_FILE"
#define SAMPLES_FILE_KEYWORD                      "SAMPLES_FILE"
#define MULTI_LOCALIZE_KEYWORD                    "MULTI_LOCALIZE"
#define RESET_SAMPLE_PROBABILITY_KEYWORD          "RESET_SAMPLE_PROBABILITY"

/*---------------------------------------------------
 * movement parameters
 *---------------------------------------------------*/
#define NUMBER_OF_MOVEMENT_PARAMETERS 8

#define USE_POSITION_KEYWORD                       "USE_POSITION"
#define CONVOLVE_MOVEMENT_THRESHOLD_KEYWORD        "CONVOLVE_MOVEMENT_THRESHOLD"
#define INTEGRATE_MOVEMENT_THRESHOLD_KEYWORD       "INTEGRATE_MOVEMENT_THRESHOLD"
#define MAX_QUOTA_OF_PLANES_FOR_MOVEMENT_KEYWORD   "MAX_QUOTA_OF_PLANES_FOR_MOVEMENT"
#define XY_KERNEL_SIZE_KEYWORD                     "XY_KERNEL_SIZE"
#define Z_KERNEL_SIZE_KEYWORD                      "Z_KERNEL_SIZE"
#define XY_KERNEL_VALUES_KEYWORD                   "XY_KERNEL_VALUES"
#define Z_KERNEL_VALUES_KEYWORD                    "Z_KERNEL_VALUES"

/*---------------------------------------------------
 * Robot typeinitialized by communication (will be used by sonar and laser)
 *---------------------------------------------------*/

#define ROBOT_TYPE_KEYWORD                         "ROBOT_TYPE"

/* One of the following INTEGERS has to be used. */
#define B21_ONE_LASER_ROBOT  0
#define B21_TWO_LASERS_ROBOT 1
#define B18_ROBOT            2
#define PIONEER_ATRV         3
#define PIONEER_II           4
#define URBAN_ROBOT          5
#define SCOUT                6
#define XR4000               7

/*---------------------------------------------------
 * sonar parameters
 *---------------------------------------------------*/
#define NUMBER_OF_SONAR_PARAMETERS 12

#define USE_SONAR_KEYWORD                             "USE_SONAR"
#define SONAR_COMPUTE_EXPECTED_DIST_KEYWORD           "SONAR_COMPUTE_EXPECTED_DIST"
#define SONAR_EXPECTED_DIST_KEYWORD                   "SONAR_EXPECTED_DIST_FILE"
#define SONAR_NUMBER_OF_EXPECTED_DIST_ANGLES_KEYWORD  "SONAR_NUMBER_OF_EXPECTED_DIST_ANGLES"
#define NUMBER_OF_SONARS_TO_BE_USED_KEYWORD           "NUMBER_OF_SONARS_TO_BE_USED"
#define NUMBER_OF_FIRST_SONAR_KEYWORD                 "NUMBER_OF_FIRST_SONAR"
#define AAAI_SONARS_KEYWORD                           "AAAI_SONARS"
#define SONAR_INTEGRATE_THRESHOLD_KEYWORD             "SONAR_INTEGRATE_THRESHOLD"
#define SONAR_MAX_QUOTA_OF_PLANES_KEYWORD             "SONAR_MAX_QUOTA_OF_PLANES"
#define SONAR_CHOOSE_OPTIMAL_SENSOR_KEYWORD           "SONAR_CHOOSE_OPTIMAL_SENSOR"
#define SONAR_MAX_FACTOR_KEYWORD                      "SONAR_MAX_FACTOR"
#define SONAR_COMBINE_SCANS_KEYWORD                   "SONAR_COMBINE_SCANS"


/*---------------------------------------------------
 * laser parameters
 *---------------------------------------------------*/
#define NUMBER_OF_LASER_PARAMETERS 16

#define USE_LASER_KEYWORD                             "USE_LASER"
#define LASER_COMPUTE_EXPECTED_DIST_KEYWORD           "LASER_COMPUTE_EXPECTED_DIST"
#define LASER_EXPECTED_DIST_KEYWORD                   "LASER_EXPECTED_DIST_FILE"
#define LASER_NUMBER_OF_EXPECTED_DIST_ANGLES_KEYWORD  "LASER_NUMBER_OF_EXPECTED_DIST_ANGLES"
#define NUMBER_OF_LASERS_TO_BE_USED_KEYWORD           "NUMBER_OF_LASERS_TO_BE_USED"
#define NUMBER_OF_FIRST_LASER_KEYWORD                 "NUMBER_OF_FIRST_LASER"
#define LASER_INTEGRATE_THRESHOLD_KEYWORD             "LASER_INTEGRATE_THRESHOLD"
#define LASER_MAX_QUOTA_OF_PLANES_KEYWORD             "LASER_MAX_QUOTA_OF_PLANES"
#define LASER_CHOOSE_OPTIMAL_SENSOR_KEYWORD           "LASER_CHOOSE_OPTIMAL_SENSOR"
#define LASER_MAX_FACTOR_KEYWORD                      "LASER_MAX_FACTOR"
#define NUMBER_OF_LASERS_FOR_CONVOLVE_KEYWORD         "NUMBER_OF_LASERS_FOR_CONVOLVE"
#define MAX_NUMBER_OF_INTEGRATIONS_PER_STOP_KEYWORD   "MAX_NUMBER_OF_INTEGRATIONS_PER_STOP"
#define LASER_CUT_DISTANCE_KEYWORD                    "LASER_CUT_DISTANCE"
#define MAX_ROT_VEL_FOR_LASERS_KEYWORD                "MAX_ROT_VEL_FOR_LASERS"
#define LASER_MAX_EXPECTED_DISTANCE_KEYWORD           "LASER_MAX_EXPECTED_DISTANCE"
#define LASER_NUMBER_OF_STDDEVS_KEYWORD               "LASER_NUMBER_OF_STDDEVS"


/*---------------------------------------------------
 * angle parameters
 *---------------------------------------------------*/
#define NUMBER_OF_ANGLE_PARAMETERS 14

#define USE_ANGLE_KEYWORD                       "USE_ANGLE"
#define ANGLE_KEYWORD                           "ANGLE_FILE"
#define UPDATE_ANGLE_GRID_THRESHOLD_KEYWORD     "UPDATE_ANGLE_GRID_THRESHOLD"
#define CONVOLVE_ANGLE_GRID_THRESHOLD_KEYWORD   "CONVOLVE_ANGLE_GRID_THRESHOLD"
#define ANGLE_USE_SONAR_KEYWORD                 "ANGLE_USE_SONAR"
#define ALIGNED_SONAR_READINGS_KEYWORD          "ALIGNED_SONAR_READINGS"
#define MIN_ALIGNED_SONAR_READINGS_KEYWORD      "MIN_ALIGNED_SONAR_READINGS"
#define ANGLE_USE_LASER_KEYWORD                 "ANGLE_USE_LASER"
#define ALIGNED_LASER_READINGS_KEYWORD          "ALIGNED_LASER_READINGS"
#define MIN_ALIGNED_LASER_READINGS_KEYWORD      "MIN_ALIGNED_LASER_READINGS"
#define MIN_QUOTA_OF_PLANES_FOR_ANGLE_KEYWORD   "MIN_QUOTA_OF_PLANES_FOR_ANGLE"
#define ANGLE_KERNEL_SIZE_KEYWORD               "ANGLE_KERNEL_SIZE"
#define ANGLE_KERNEL_VALUES_KEYWORD             "ANGLE_KERNEL_VALUES"
#define INTEGRATE_ANGLE_GRID_THRESHOLD_KEYWORD  "INTEGRATE_ANGLE_GRID_SKIP"


/*---------------------------------------------------
 * vision parameters
 *---------------------------------------------------*/
#define NUMBER_OF_VISION_PARAMETERS 5

#define USE_VISION_KEYWORD                     "USE_VISION"
#define VISION_INTEGRATE_THRESHOLD_KEYWORD "VISION_INTEGRATE_THRESHOLD"
#define VISION_MAX_QUOTA_OF_PLANES_KEYWORD "VISION_MAX_QUOTA_OF_PLANES"
#define VISION_CRUNCH_FACTOR_KEYWORD "VISION_CRUNCH_FACTOR"
#define VISION_INTEGRATIONS_PER_STOP_KEYWORD "VISION_INTEGRATIONS_PER_STOP"

/*---------------------------------------------------
 * communication parameters
 *---------------------------------------------------*/
#define NUMBER_OF_COMMUNICATION_PARAMETERS 17

#define USE_TCX_KEYWORD                         "USE_TCX"
#define CONNECT_TO_BASE_KEYWORD                 "CONNECT_TO_BASE"
#define SUBSCRIBE_BASE_REPORT_KEYWORD           "SUBSCRIBE_BASE_REPORT"
#define SUBSCRIBE_PROXIMITY_REPORT_KEYWORD      "SUBSCRIBE_PROXIMITY_REPORT"
#define SCRIPT_KEYWORD                          "SCRIPT_FILE"
#define TIME_FACTOR_KEYWORD                     "TIME_FACTOR"
#define LOG_FILE_KEYWORD                        "LOG_FILE"
#define ODOMETRY_CORRECTION_KEYWORD             "ODOMETRY_CORRECTION"
#define ROBOT_NAME_KEYWORD                      "ROBOT_NAME"
#define SUBSCRIBE_CAMERA_KEYWORD                "SUBSCRIBE_CAMERA_REPORT"
#define REAL_TIME_SCRIPT_KEYWORD                "REAL_TIME_SCRIPT"
#define SEND_REPORTS_KEYWORD                    "SEND_REPORTS"
#define SEND_CORRECTION_TO_PLAN_KEYWORD         "SEND_CORRECTION_TO_PLAN"
#define SEND_CORRECTION_TO_MAP_KEYWORD          "SEND_CORRECTION_TO_MAP"
#define TIME_TO_BE_SKIPPED_KEYWORD              "TIME_TO_BE_SKIPPED"
#define RUN_TIME_KEYWORD                        "RUN_TIME"

#define STDIN_NAME                              "STDIN"
#define STDOUT_NAME                             "STDOUT"

/*---------------------------------------------------
 * graphic parameters
 *---------------------------------------------------*/
#define NUMBER_OF_GRAPHIC_PARAMETERS 19

#define USE_GRAPHIC_KEYWORD                     "USE_GRAPHIC"
#define START_POS_KEYWORD                       "START_POS"
#define START_POS_MAP_KEYWORD                   "START_POS_MAP"
#define START_POS_SCRIPT_KEYWORD                "START_POS_SCRIPT"
#define SHOW_MAP_KEYWORD                        "SHOW_MAP"
#define SHOW_SONAR_MAP_KEYWORD                  "SHOW_SONAR_MAP"
#define SHOW_LASER_MAP_KEYWORD                  "SHOW_LASER_MAP"
#define SHOW_INITIAL_POSITION_PROBS_KEYWORD     "SHOW_INITIAL_POSITION_PROBS"
#define SHOW_ANGLES_KEYWORD                     "SHOW_ANGLES"
#define CREATE_MAP_OVERLAY_KEYWORD              "CREATE_MAP_OVERLAY"
#define SET_ROBOT_POSITION_KEYWORD              "SET_ROBOT_POSITION"
#define SHOW_ROBOT_ZOOM_KEYWORD                 "SHOW_ROBOT_ZOOM"
#define MIN_WINDOW_SCALE_KEYWORD                "MIN_WINDOW_SCALE"
#define ROBOT_ZOOM_SCALE_KEYWORD                "ROBOT_ZOOM_SCALE"
#define SHOW_SELECTED_SENSINGS_KEYWORD          "SHOW_SELECTED_SENSINGS"
#define SHOW_EXPECTED_DISTANCES_KEYWORD         "SHOW_EXPECTED_DISTANCES"
#define DISPLAY_SKIP_KEYWORD                    "DISPLAY_SKIP"
#define DUMP_XY_GRAPHIC_KEYWORD                 "DUMP_XY_GRAPHIC"
#define DUMP_ROBOT_WINDOW_KEYWORD               "DUMP_ROBOT_WINDOW"


/*---------------------------------------------------
 * graphic parameters
 *---------------------------------------------------*/
#define NUMBER_OF_ACTION_PARAMETERS   17

#define MAX_NUMBER_OF_MAXIMA_FOR_ACTION_KEYWORD    "MAX_NUMBER_OF_MAXIMA_FOR_ACTION"
#define MIN_DIST_FROM_CURRENT_POS_KEYWORD          "MIN_DIST_FROM_CURRENT_POS"
#define MIN_SUM_OF_PROBS_KEYWORD                   "MIN_SUM_OF_PROBS"
#define SHRINK_MAXIMA_SIZE_KEYWORD                 "SHRINK_MAXIMA_SIZE"
#define MAX_COST_OF_POSITION_KEYWORD               "MAX_COST_OF_POSITION"
#define QUOTA_OF_AREA_KEYWORD                      "QUOTA_OF_AREA"
#define MIN_PROB_OF_LOCAL_MAX_KEYWORD              "MIN_PROB_OF_LOCAL_MAX"
#define GO_TO_GOAL_KEYWORD                         "GO_TO_GOAL"
#define GOAL_X_KEYWORD                             "GOAL_X"
#define GOAL_Y_KEYWORD                             "GOAL_Y"
#define SHOW_FIELDS_KEYWORD                        "SHOW_FIELDS"
#define MIN_NUMBER_OF_SELECTED_SENSORS_KEYWORD     "MIN_NUMBER_OF_SELECTED_SENSORS"
#define MAX_NUMBER_OF_SELECTED_SENSORS_KEYWORD     "MAX_NUMBER_OF_SELECTED_SENSORS"
#define USE_MEASURED_FEATURES_KEYWORD              "USE_MEASURED_FEATURES"
#define CONSIDER_INACTIVE_CELLS_KEYWORD            "CONSIDER_INACTIVE_CELLS"
#define SELECTION_MODE_KEYWORD                     "SELECTION_MODE"
#define SELECTION_THRESHOLD_KEYWORD                "SELECTION_THRESHOLD"

/* different selection modes. */
#define ENTROPY_SELECTION 0
#define UNEXPECTED_SELECTION 1
#define HUMAN_SELECTION 2
#endif



