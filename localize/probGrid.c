
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/probGrid.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:40 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: probGrid.c,v $
 * Revision 1.1  2002/09/14 20:45:40  rstone
 * *** empty log message ***
 *
 * Revision 1.133  1999/12/15 16:16:40  fox
 * First attempt to extract p(l | o).
 *
 * Revision 1.132  1999/11/02 18:12:36  fox
 * Version of multi localize journal paper.
 *
 * Revision 1.131  1999/08/27 22:22:33  fox
 * Better communication for multi robot localization.
 *
 * Revision 1.130  1999/03/08 16:47:44  wolfram
 * Fixed problems with map, caused by the fact that it no longer accpets
 * correction parameters from LOCALIZE.  Improved the handling of sonars
 * with Pioneers.  Currently sonars do not work properly on a B21.
 *
 * Revision 1.129  1999/01/22 17:48:08  fox
 * Final version of sampling paper and denver demo.
 *
 * Revision 1.128  1999/01/14 00:33:02  wolfram
 * Changes for vision
 *
 * Revision 1.127  1999/01/11 19:47:53  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.125  1999/01/08 22:28:47  wolfram
 * Better integration of scanAlignment
 *
 * Revision 1.124  1999/01/07 01:07:09  wolfram
 * Changes to integrate scan matching
 *
 * Revision 1.123  1998/11/19 03:14:28  fox
 * Newest version with integrated mapping and localization. If MAP is connected,
 * then the correction parameters are NOT sent to PLAN.
 * Furthermore, the size of the onlinemap adapts to the partial maps sent by MAP.
 *
 * Revision 1.122  1998/11/17 23:26:24  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.121  1998/11/03 21:02:21  fox
 * Lasers will not be integrated if the robot rotates faster than
 * MAX_ROT_VEL_FOR_LASERS deg/sec.
 *
 * Revision 1.120  1998/10/02 15:16:41  fox
 * Several improvements of condensation. Updated the function calls to the
 * new version of the distancServer.
 *
 * Revision 1.119  1998/09/25 04:02:57  fox
 * First version of CONDENSATION!!!!
 *
 * Revision 1.118  1998/09/18 17:24:43  fox
 * Added skeleton files for condensation.
 *
 * Revision 1.117  1998/08/31 22:29:23  wolfram
 * Several changes
 *
 * Revision 1.116  1998/08/23 00:01:02  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.115  1998/08/20 00:23:01  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.114  1998/08/19 16:33:57  fox
 * Minor changes. Position is dumped upon exit of tcxServer.
 *
 * Revision 1.113  1998/08/11 23:05:39  wolfram
 * Changes for the Smithsonian museum.
 *
 * WARNING: LOCALIZE now uses only one stddev for Lasers. Laser beams have 25 m
 * Maxrange now.
 * Please use ~wolfram/bee/data/localize/params/laser-0.1stdev with this version
 * of LOCALIZE
 *
 * Revision 1.112  1998/07/01 10:46:00  fox
 * Final update of question sensor.
 *
 * Revision 1.111  1998/06/30 13:55:06  fox
 * Updated question sensor.
 *
 * Revision 1.110  1998/06/12 10:16:35  fox
 * Implemented virutal sensor.
 *
 * Revision 1.109  1998/03/09 10:07:46  wolfram
 * slight changes
 *
 * Revision 1.108  1998/03/09 09:36:28  wolfram
 * LOCALIZE now checks the consistency of the various maps.
 *
 * Revision 1.107  1998/02/13 14:12:24  fox
 * Minor changes.
 *
 * Revision 1.106  1998/02/12 15:47:21  derr
 * compiler warnings (unused variable ... ) removed
 *
 * Revision 1.105  1998/01/29 16:46:51  fox
 * Removed some hacks.
 *
 * Revision 1.104  1998/01/22 13:06:18  fox
 * First version after selection-submission.
 *
 * Revision 1.103  1998/01/06 15:11:22  fox
 * Added evaluation tools.
 *
 * Revision 1.102  1998/01/06 09:26:44  wolfram
 * Minor change
 *
 * Revision 1.101  1998/01/05 12:44:36  wolfram
 * Fixed a bug in shiftAndMultiplyPlane
 *
 * Revision 1.100  1998/01/05 10:37:13  fox
 * Several changes especially in selection parts.
 *
 * Revision 1.99  1997/12/19 11:30:12  fox
 * FIXED a bug I added.
 *
 * Revision 1.98  1997/12/19 10:30:32  wolfram
 * Changed MINIMUM_UPDATE_PROBABILITY_QUOTA and resetNewPlanes
 *
 * Revision 1.97  1997/12/18 12:57:26  wolfram
 * Fixed exceptions in shiftAndMultiplyPlane
 *
 * Revision 1.96  1997/12/17 16:48:30  wolfram
 * Added NEIGHBOR_PLANES_TO_ADD keyword for normalize
 *
 * Revision 1.95  1997/12/11 14:17:40  wolfram
 * Now writing the marker file format.
 *
 * Revision 1.94  1997/12/09 22:53:41  wolfram
 * Distance to markers is written
 *
 * Revision 1.93  1997/12/03 09:09:08  fox
 * Renamde USE_MOVEMENT to USE_POSITION.
 *
 * Revision 1.92  1997/12/02 15:20:40  fox
 * Nothing remarkable.
 *
 * Revision 1.91  1997/11/28 14:11:51  fox
 * Minor changes.
 *
 * Revision 1.90  1997/11/28 13:34:36  fox
 * Added questions.
 *
 * Revision 1.89  1997/11/27 18:11:20  fox
 * Several changes to make angles work better.
 *
 * Revision 1.88  1997/11/26 15:47:43  fox
 * Added some structures for questions.
 *
 * Revision 1.87  1997/11/21 15:36:05  fox
 * Modifications in graphic
 *
 * Revision 1.86  1997/11/20 12:58:13  fox
 * Version with good sensor selection.
 *
 * Revision 1.85  1997/11/07 12:39:42  fox
 * Added some graphic features.
 *
 * Revision 1.84  1997/11/07 06:58:07  wolfram
 * Now logging correction parameters
 *
 * Revision 1.83  1997/10/31 15:27:20  wolfram
 * Offset of grid maps is set to zero
 *
 * Revision 1.82  1997/10/31 13:11:43  fox
 * Version for active sensing.
 *
 * Revision 1.81  1997/10/02 09:20:28  wolfram
 * Better initialization for angles
 *
 * Revision 1.80  1997/10/01 11:29:59  fox
 * Minor changes.
 *
 * Revision 1.79  1997/09/30 13:38:52  wolfram
 * Integration of angles is stopped after integration of any proximity sensor
 *
 * Revision 1.78  1997/09/29 10:45:25  wolfram
 * Added default values for all values in .ini file
 *
 * Revision 1.77  1997/09/26 17:02:10  wolfram
 * Intermediate version, do not use
 *
 * Revision 1.76  1997/09/11 20:01:05  fox
 * Final cmu version.
 *
 * Revision 1.75  1997/09/09 19:45:13  fox
 * Fixed a bug in finding local maxima and did several changes to active
 * localization.
 *
 * Revision 1.74  1997/08/22 04:16:39  fox
 * Final version before IJCAI.
 *
 * Revision 1.73  1997/08/19 22:19:09  wolfram
 * Fixed a bug in display of robot window and in planeOfRotation
 *
 * Revision 1.72  1997/08/19 18:24:06  wolfram
 * Fixed a bug in computeWeightedPositionAndSum
 *
 * Revision 1.71  1997/08/16 22:59:51  fox
 * Last version before I change selsection.
 *
 * Revision 1.70  1997/08/16 21:52:03  wolfram
 * Grid maps can now be cad maps.  Obstacles are enlarged accordingly
 *
 * Revision 1.69  1997/08/16 18:16:54  wolfram
 * Planmap and initprobs are now saved in every case
 *
 * Revision 1.68  1997/08/02 16:51:05  wolfram
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
 * Revision 1.67  1997/07/04 17:29:15  fox
 * Final version before holiday!!!
 *
 * Revision 1.66  1997/06/27 16:26:28  fox
 * New model of the proximity sensors.
 *
 * Revision 1.65  1997/06/26 11:53:43  fox
 * Minor changes.
 *
 * Revision 1.64  1997/06/26 11:23:17  fox
 * Fixed a bug in normalize.
 *
 * Revision 1.63  1997/06/25 17:13:20  wolfram
 * Added starting Positions of museum scripts
 *
 * Revision 1.62  1997/06/25 14:16:40  fox
 * Changed laser incorporation.
 *
 * Revision 1.61  1997/06/20 07:36:12  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.60  1997/06/03 11:49:23  fox
 * Museum version.
 *
 * Revision 1.59  1997/05/27 07:42:35  fox
 * Nothing special.
 *
 * Revision 1.58  1997/05/26 10:32:01  fox
 * Added dump of rear laser and sonar data.
 *
 * Revision 1.57  1997/05/26 08:47:53  fox
 * Last version before major changes.
 *
 * Revision 1.56  1997/05/11 19:08:07  wolfram
 * Faster normalization, slight changes in graphic.c
 *
 * Revision 1.55  1997/04/30 12:25:41  fox
 * Some minor changes.
 *
 * Revision 1.54  1997/04/24 21:25:43  wolfram
 * Ooops, fourth parameter of CUBE's is radius and not diameter!
 *
 * Revision 1.53  1997/04/10 13:01:03  fox
 * Fixed a bug.
 *
 * Revision 1.52  1997/04/08 14:56:24  fox
 * Changed the reset function for new planes and the prob function.
 *
 * Revision 1.51  1997/04/07 11:03:22  fox
 * Should be ok.
 *
 * Revision 1.50  1997/04/06 22:38:33  wolfram
 * Dumping expected and measured features in log file now
 *
 * Revision 1.49  1997/04/03 13:17:51  fox
 * Some minor changes.
 *
 * Revision 1.48  1997/04/02 08:57:34  fox
 * Display the robot right after sensor selection.
 *
 * Revision 1.47  1997/03/26 09:42:58  fox
 * Updated correction parameter message.
 *
 * Revision 1.46  1997/03/19 17:52:43  fox
 * New laser parameters.
 *
 * Revision 1.45  1997/03/17 20:47:09  wolfram
 * The initial position probs are now dumped to a file
 *
 * Revision 1.44  1997/03/17 18:41:14  fox
 * First version with sensor selection in cluttered environments.
 *
 * Revision 1.43  1997/03/14 17:58:21  fox
 * This version should run quite stable now.
 *
 * Revision 1.42  1997/03/13 17:36:22  fox
 * Temporary version. Don't use!
 *
 * Revision 1.41  1997/03/03 12:59:22  wolfram
 * Initial position probabilities are computed out of the simulator map if
 * the simulator map is available
 *
 * Revision 1.40  1997/02/11 13:21:48  fox
 * Fixed a bug in setPosition.
 *
 * Revision 1.39  1997/02/11 10:09:32  fox
 * No comment.
 *
 * Revision 1.38  1997/01/30 13:34:13  fox
 * Minor changes.
 *
 * Revision 1.37  1997/01/29 12:23:12  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.36  1997/01/23 14:07:23  fox
 * Version of ijcai-submission.
 *
 * Revision 1.35  1997/01/19 19:31:17  fox
 * yeah
 *
 * Revision 1.34  1997/01/19 15:42:26  wolfram
 * Script time difference is compted correctly even if the time in the script
 * does not increase
 *
 * Revision 1.33  1997/01/19 14:05:27  wolfram
 * Added counter for number of integrated readings
 *
 * Revision 1.32  1997/01/18 19:41:03  fox
 * Improved action selection.
 *
 * Revision 1.31  1997/01/18 17:24:44  wolfram
 * Fixed two bugs
 *
 * Revision 1.30  1997/01/18 16:04:16  wolfram
 * Fixed a bug
 *
 * Revision 1.29  1997/01/18 14:07:55  fox
 * Test version.
 *
 * Revision 1.28  1997/01/17 13:21:07  fox
 * Added timer for non relevant functions.
 *
 * Revision 1.27  1997/01/16 19:43:24  fox
 * And another bug ...
 *
 * Revision 1.26  1997/01/16 12:42:51  fox
 * Fixed a bug in scan masks.
 *
 * Revision 1.25  1997/01/14 16:53:24  fox
 * Removed a very very nasty bug for sensor selection.
 *
 * Revision 1.24  1997/01/14 10:38:20  wolfram
 * Test Version
 *
 * Revision 1.23  1997/01/13 16:54:13  fox
 * Nothing special.
 *
 * Revision 1.22  1997/01/10 15:19:23  fox
 * Improved several methods.
 *
 * Revision 1.21  1997/01/08 18:28:20  fox
 * Fixed a bug in setCellList.
 *
 * Revision 1.20  1997/01/08 15:52:58  fox
 * Rotations are not any longer treated like movements.
 * Send parameters to PLAN.
 *
 * Revision 1.19  1997/01/07 12:31:27  wolfram
 * FindLocalMaxima additionally computes a list of localMaxima
 *
 * Revision 1.18  1997/01/07 09:27:38  wolfram
 * Added procedure to compute weighted position and sum of probs
 *
 * Revision 1.17  1997/01/03 10:09:48  fox
 * First version with exploration.
 *
 * Revision 1.16  1996/12/31 09:19:24  fox
 * Last version before I switched exploration mode in PLAN to random
 * mode in COLLI.
 *
 * Revision 1.15  1996/12/20 15:29:40  fox
 * Added four parameters.
 *
 * Revision 1.14  1996/12/19 14:33:29  wolfram
 * More sensitive computation of expected distance tables
 *
 * Revision 1.13  1996/12/13 13:55:38  fox
 * Implemented connection to PLAN.
 *
 * Revision 1.12  1996/12/04 14:30:00  fox
 * ok
 *
 * Revision 1.11  1996/12/03 15:40:25  fox
 * ok
 *
 * Revision 1.10  1996/12/03 12:27:41  fox
 * Seems to work with the thin corridor.
 *
 * Revision 1.9  1996/12/02 10:32:11  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.8  1996/11/28 09:05:49  fox
 * nicks spezielles.
 *
 * Revision 1.7  1996/11/26 16:08:26  fox
 * Nothing special.
 *
 * Revision 1.6  1996/11/25 19:35:41  fox
 * Test version for decisions of movements.
 *
 * Revision 1.5  1996/11/18 09:58:31  ws
 * Added support for selecting the optimal sensor
 * Changed some variables to improve compatibility with C++
 *
 * Revision 1.4  1996/11/15 17:44:07  ws
 * *** empty log message ***
 *
 * Revision 1.3  1996/10/24 12:07:12  fox
 * Fixed a bug.
 *
 * Revision 1.2  1996/10/24 09:56:55  fox
 * LOCALIZE also works in a write protected directory.
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



#include <math.h>

#include "general.h"
#include "probGrid.h"
#include "function.h"
#include "file.h"
#include "sonar.h"
#include "laser.h"
#include "vision.h"
#include "movement.h"
#include "angle.h"
#include "graphic.h"
#include "allocate.h"
#include "script.h"
#include "localTcx.h"
#include "probGridTools.h"
#include "proximityTools.h"
#include "communication.h"
#include "scanAlignment.h"

#define NUMBER_OF_ANGLES_TOKEN 0
#define MAX_QUOTA_OF_PLANES_TOKEN 1
#define MIN_SHIFT_FOR_RESET_TOKEN 2
#define NUMBER_OF_SHIFTS_FOR_NORMALIZE_TOKEN 3
#define NUMBER_OF_SONARS_FOR_NORMALIZE_TOKEN 4
#define NUMBER_OF_ANGLES_FOR_NORMALIZE_TOKEN 5
#define NUMBER_OF_LASERS_FOR_NORMALIZE_TOKEN 6
#define COMPUTE_STATISTICS_TOKEN             7
#define START_X_TOKEN                        8
#define START_Y_TOKEN                        9
#define START_ROT_TOKEN                      10
#define SET_START_POS_TOKEN                  11
#define NEIGHBOR_PLANES_TO_ADD_TOKEN         12
#define GENERATE_VIRTUAL_SENSOR_TOKEN        13
#define USE_PROBGRID_TOKEN                   14
#define NUMBER_OF_VISIONS_FOR_NORMALIZE_TOKEN 15

#define MAX_QUOTA_OF_PLANES_SECOND_MAX 0.0

#define INITIAL_POSITION_PROBS_EXTENSION ".initprobs"

/* #define DONT_USE_POSTIION  */

probGridParameters globalProbGridParameters;
extern informationsFor_COMMUNICATION communicationInfo;

#define POSITION_FILE "loc.pos"
#define REMOVE_POSITION_FILE "rm loc.pos"

/********************************************************************
 ********************************************************************
 * Forward declarations of internal functions.
 ********************************************************************
 ********************************************************************/

static positionProbabilityGrid
initializedPositionProbabilityGrid( probabilityGrid* positionProbabilities,
				    int numberOfAngles);

static int
markNeighboringPlanes( positionProbabilityGrid* posGrid,
		       int numberOfNeighbors);

static probability
sumOverAllProbabilities( positionProbabilityGrid* posGrid);

static void
computeGridStatistics( actionInformation* info,
		       sensingActionMask* mask,
		       realPosition correction);

static void
setTestPositions( actionInformation* info);

/**************************************************************************
 **************************************************************************
 * Mandatory functions.
 **************************************************************************
 **************************************************************************/

void
initialize_PROBGRID( char* fileName,
		     actionInformation* actionInfo)
{
  extern mapParameters globalMapParameters;
  realPosition startPos;
  int setStartPos;
  
  token tok[NUMBER_OF_PROBGRID_PARAMETERS];
  
  actionInfo->proximityIntegrated = FALSE;
  
  /**********************************************************
   * Initialize parameters
   **********************************************************/
  globalProbGridParameters.numberOfAngles = 180;
  globalProbGridParameters.maxQuotaOfPlanesForNormalization = 0.4;
  globalProbGridParameters.minShiftForReset = 500.0;
  globalProbGridParameters.numberOfShiftsForNormalize = 4;
  globalProbGridParameters.numberOfSonarsForNormalize = 2;
  globalProbGridParameters.numberOfLasersForNormalize = 1;
  globalProbGridParameters.numberOfAnglesForNormalize = 2;
  globalProbGridParameters.numberOfVisionsForNormalize = 4;
  globalProbGridParameters.computeStatistics = 0;
  globalProbGridParameters.neighborPlanesToAdd = 1;
  globalProbGridParameters.generateVirtualSensor = 0;
  setStartPos = 0;
  startPos.x = startPos.y = startPos.rot = 0.0;

  actionInfo->useProbGrid = TRUE;
  
  setTokensInitialized(tok, NUMBER_OF_PROBGRID_PARAMETERS);
  
  /**********************************************************
   * Get the parameters from the file.
   **********************************************************/

  tok[USE_PROBGRID_TOKEN].format   = INT_FORMAT;
  tok[USE_PROBGRID_TOKEN].variable = &(actionInfo->useProbGrid);
  tok[USE_PROBGRID_TOKEN].keyWord  = USE_PROBGRID_KEYWORD;
  
  tok[NUMBER_OF_ANGLES_TOKEN].format   = INT_FORMAT;
  tok[NUMBER_OF_ANGLES_TOKEN].variable = &(globalProbGridParameters.numberOfAngles);
  tok[NUMBER_OF_ANGLES_TOKEN].keyWord  = NUMBER_OF_ANGLES_KEYWORD;
  
  tok[MAX_QUOTA_OF_PLANES_TOKEN].format   = FLOAT_FORMAT;
  tok[MAX_QUOTA_OF_PLANES_TOKEN].variable =
    &(globalProbGridParameters.maxQuotaOfPlanesForNormalization);
  tok[MAX_QUOTA_OF_PLANES_TOKEN].keyWord  =
    MAX_QUOTA_OF_PLANES_FOR_NORMALIZATION_KEYWORD;
  
  tok[MIN_SHIFT_FOR_RESET_TOKEN].format   = FLOAT_FORMAT;
  tok[MIN_SHIFT_FOR_RESET_TOKEN].variable = &(globalProbGridParameters.minShiftForReset);
  tok[MIN_SHIFT_FOR_RESET_TOKEN].keyWord  = MIN_SHIFT_FOR_RESET_KEYWORD;
  
  tok[NUMBER_OF_SHIFTS_FOR_NORMALIZE_TOKEN].format   = INT_FORMAT;
  tok[NUMBER_OF_SHIFTS_FOR_NORMALIZE_TOKEN].variable =
    &(globalProbGridParameters.numberOfShiftsForNormalize);
  tok[NUMBER_OF_SHIFTS_FOR_NORMALIZE_TOKEN].keyWord  = NUMBER_OF_SHIFTS_FOR_NORMALIZE_KEYWORD;
  
  tok[NUMBER_OF_SONARS_FOR_NORMALIZE_TOKEN].format   = INT_FORMAT;
  tok[NUMBER_OF_SONARS_FOR_NORMALIZE_TOKEN].variable =
    &(globalProbGridParameters.numberOfSonarsForNormalize);
  tok[NUMBER_OF_SONARS_FOR_NORMALIZE_TOKEN].keyWord  = NUMBER_OF_SONARS_FOR_NORMALIZE_KEYWORD;

  tok[NUMBER_OF_ANGLES_FOR_NORMALIZE_TOKEN].format   = INT_FORMAT;
  tok[NUMBER_OF_ANGLES_FOR_NORMALIZE_TOKEN].variable =
    &(globalProbGridParameters.numberOfAnglesForNormalize);
  tok[NUMBER_OF_ANGLES_FOR_NORMALIZE_TOKEN].keyWord  = NUMBER_OF_ANGLES_FOR_NORMALIZE_KEYWORD;

  tok[NUMBER_OF_LASERS_FOR_NORMALIZE_TOKEN].format   = INT_FORMAT;
  tok[NUMBER_OF_LASERS_FOR_NORMALIZE_TOKEN].variable =
    &(globalProbGridParameters.numberOfLasersForNormalize);
  tok[NUMBER_OF_LASERS_FOR_NORMALIZE_TOKEN].keyWord  = NUMBER_OF_LASERS_FOR_NORMALIZE_KEYWORD;

  tok[NUMBER_OF_VISIONS_FOR_NORMALIZE_TOKEN].format   = INT_FORMAT;
  tok[NUMBER_OF_VISIONS_FOR_NORMALIZE_TOKEN].variable =
    &(globalProbGridParameters.numberOfVisionsForNormalize);
  tok[NUMBER_OF_VISIONS_FOR_NORMALIZE_TOKEN].keyWord  = NUMBER_OF_VISIONS_FOR_NORMALIZE_KEYWORD;

  tok[COMPUTE_STATISTICS_TOKEN].format   = INT_FORMAT;
  tok[COMPUTE_STATISTICS_TOKEN].variable =
    &(globalProbGridParameters.computeStatistics);
  tok[COMPUTE_STATISTICS_TOKEN].keyWord  = COMPUTE_STATISTICS_KEYWORD;

  tok[SET_START_POS_TOKEN].format   = INT_FORMAT;
  tok[SET_START_POS_TOKEN].variable = &(setStartPos);
  tok[SET_START_POS_TOKEN].keyWord  = SET_START_POS_KEYWORD;

  tok[START_X_TOKEN].format   = FLOAT_FORMAT;
  tok[START_X_TOKEN].variable = &(startPos.x);
  tok[START_X_TOKEN].keyWord  = START_X_KEYWORD;

  tok[START_Y_TOKEN].format   = FLOAT_FORMAT;
  tok[START_Y_TOKEN].variable = &(startPos.y);
  tok[START_Y_TOKEN].keyWord  = START_Y_KEYWORD;

  tok[START_ROT_TOKEN].format   = FLOAT_FORMAT;
  tok[START_ROT_TOKEN].variable = &(startPos.rot);
  tok[START_ROT_TOKEN].keyWord  = START_ROT_KEYWORD;

  tok[NEIGHBOR_PLANES_TO_ADD_TOKEN].format   = INT_FORMAT;
  tok[NEIGHBOR_PLANES_TO_ADD_TOKEN].variable =
    &(globalProbGridParameters.neighborPlanesToAdd);
  tok[NEIGHBOR_PLANES_TO_ADD_TOKEN].keyWord  = NEIGHBOR_PLANES_TO_ADD_KEYWORD;

  tok[GENERATE_VIRTUAL_SENSOR_TOKEN].format   = INT_FORMAT;
  tok[GENERATE_VIRTUAL_SENSOR_TOKEN].variable =
    &(globalProbGridParameters.generateVirtualSensor);
  tok[GENERATE_VIRTUAL_SENSOR_TOKEN].keyWord  = GENERATE_VIRTUAL_SENSOR_KEYWORD;

  readTokens( fileName, tok, NUMBER_OF_PROBGRID_PARAMETERS, FALSE);

  if (globalProbGridParameters.numberOfShiftsForNormalize <= 0)
    globalProbGridParameters.numberOfShiftsForNormalize = 1;
  if (globalProbGridParameters.numberOfSonarsForNormalize <= 0)
    globalProbGridParameters.numberOfSonarsForNormalize = 1;
  if (globalProbGridParameters.numberOfAnglesForNormalize <= 0)
    globalProbGridParameters.numberOfAnglesForNormalize = 1;
  if (globalProbGridParameters.numberOfLasersForNormalize <= 0)
    globalProbGridParameters.numberOfLasersForNormalize = 1;
  if (globalProbGridParameters.numberOfVisionsForNormalize <= 0)
    globalProbGridParameters.numberOfVisionsForNormalize = 1;
  
  /**********************************************************
   * done.
   **********************************************************/

  /* Initialize the estimated robot and the local maxima. */
  actionInfo->estimatedRobot.radius  = ROB_RADIUS;
  actionInfo->estimatedRobot.pos.x   = 0.0;
  actionInfo->estimatedRobot.pos.y   = 0.0;
  actionInfo->estimatedRobot.pos.rot = 0.0;

  actionInfo->localMaxima.numberOfCells = 0;

  /* The a priori position probabilities given the map. The map is necessary
   * for the extensions of the grid and the initialization. */
  if ( ! actionInfo->onlineMapping && ! actionInfo->map.initialized) {
    fprintf( stderr, "Error: map not given to initialize the position probabilities.\n");
    closeLogAndExit(1);
  }

  if ( ! actionInfo->onlineMapping) {
    
    if (! readGridMap( globalMapParameters.mapFileName,
		       INITIAL_POSITION_PROBS_EXTENSION,
		       &actionInfo->initialPositionProbs)){
      
      if (actionInfo->simMap.initialized){
	computeGridMap( &actionInfo->simMap,
			globalMapParameters.desiredResolution,
			ROB_RADIUS,
			0.0, 120.0,
			&(actionInfo->initialPositionProbs));
	
	invertMap(&(actionInfo->initialPositionProbs));
	
	(void) writeGridMap( globalMapParameters.mapFileName,
			     INITIAL_POSITION_PROBS_EXTENSION,
			     &(actionInfo->initialPositionProbs));
      }
      else {
	actionInfo->initialPositionProbs =
	  preprocessedPositionProbabilities( &(actionInfo->map), 0.0);
	(void) writeGridMap( globalMapParameters.mapFileName,
			     INITIAL_POSITION_PROBS_EXTENSION,
			     &(actionInfo->initialPositionProbs));
      }
    }
    actionInfo->initialPositionProbs.initialized = TRUE;
    setFreeSpace( &(actionInfo->initialPositionProbs));
    actionInfo->initialPositionProbs.offsetX = actionInfo->map.offsetX;
    actionInfo->initialPositionProbs.offsetY = actionInfo->map.offsetY;
    
    checkMapConsistency(actionInfo->map.sizeX,
			actionInfo->map.sizeY,
			actionInfo->map.resolution,
			actionInfo->initialPositionProbs.sizeX,
			actionInfo->initialPositionProbs.sizeY,
			actionInfo->initialPositionProbs.resolution,
			globalMapParameters.mapFileName,
			GRID_MAP_EXTENSION,
			INITIAL_POSITION_PROBS_EXTENSION);
  }
  else {
    if ( actionInfo->useProbGrid) {
      fprintf(stderr, "Sorry. Can't perform online mapping with probgrid.\n");
      closeLogAndExit(1);
    }

    actionInfo->initialPositionProbs.initialized = FALSE;
  }
  
  if ( actionInfo->useProbGrid) {

    fprintf(stderr, "# Use prob grid.\n");

    /* The position probability grid is initialized with the a priori position
     * probabilities. It is the same for each angle. */
    /* Now place the struct in the global information struct. */
    actionInfo->positionProbs =
      initializedPositionProbabilityGrid( &(actionInfo->initialPositionProbs),
					  globalProbGridParameters.numberOfAngles);
    
    if ( setStartPos) {
      fprintf(stderr, "# Start position: %f %f %f\n",
	      startPos.x, startPos.y, startPos.rot);
      writeLog("# Start position: %f %f %f\n",
	       startPos.x, startPos.y, startPos.rot);
      startPos.rot = deg2Rad( startPos.rot);
      if (setStartPos == 1)
	setPosition( startPos, &(actionInfo->positionProbs), 1.0);
      else
	setPositionUniform( startPos, &(actionInfo->positionProbs), 1.0);
	
    }
      
    globalProbGridParameters.deltaPosForLocalMax =
      iMax( 2, cmToCells( XY_CUBE, &(actionInfo->positionProbs)));
    globalProbGridParameters.deltaRotForLocalMax =
      iMax( 2, radToPlanes( Z_CUBE, &(actionInfo->positionProbs)));
      
#ifdef SET_POSITIONS
    setTestPositions( actionInfo);
#endif
  }
}



/*****************************************************************************
 * Checks what has to be done with the grid independently from what kind of
 * sensings have been integrated.
 *****************************************************************************/
void
checkWhichActionsToPerform_PROBGRID( actionInformation* info, sensingActionMask* mask)
{
  if ( info->useProbGrid) {
    
    static int sonarIntegrateCnt = 0;
    static int laserIntegrateCnt = 0;
    static int movementIntegrateCnt = 0;
    static int angleIntegrateCnt = 0;
    static int visionIntegrateCnt = 0;
    static bool firstTimeBelowThreshold = TRUE;
    static bool firstCall = TRUE;
    
    /* We must perform actions at the first call. */
    if ( firstCall) {
      firstCall = FALSE;
      mask->normalizeGrid = TRUE;
      return;
    }
    
    /*-----------------------------------------------------------------
     * Check wether normalization is necessary.
     *-----------------------------------------------------------------*/
    mask->normalizeGrid = FALSE;
    
    if ( ! mask->consider[ANGLE] || info->positionProbs.quotaOfPlanesToBeUpdated
	 < globalProbGridParameters.maxQuotaOfPlanesForNormalization) {
      
      /* The first time below the threshold. We have to normalize the grid. */
      if ( firstTimeBelowThreshold) {
	mask->normalizeGrid = TRUE;
	firstTimeBelowThreshold = FALSE;
      }
      else {
	
	/* Check for sonar integration. */
	if ( mask->consider[SONAR] && mask->perform[SONAR][INTEGRATE_SONAR]) {
	  sonarIntegrateCnt++;
	  mask->normalizeGrid = mask->normalizeGrid ||
	    (sonarIntegrateCnt % globalProbGridParameters.numberOfSonarsForNormalize == 0);
	}

	/* Check for laser integration. */
	if ( mask->consider[LASER] && mask->perform[LASER][INTEGRATE_LASER]) {
	  laserIntegrateCnt++;
	  mask->normalizeGrid = mask->normalizeGrid ||
	    (laserIntegrateCnt % globalProbGridParameters.numberOfLasersForNormalize == 0);
	}
	
	/* Check for vision integration. */
	if ( mask->consider[VISION] && mask->perform[VISION][INTEGRATE_VISION]) {
	  visionIntegrateCnt++;
	  mask->normalizeGrid = mask->normalizeGrid ||
	    (visionIntegrateCnt %
	     globalProbGridParameters.numberOfVisionsForNormalize == 0);
	}
	
	/* Check for movement integration. */
	if ( mask->consider[MOVEMENT] && mask->perform[MOVEMENT][INTEGRATE_MOVEMENT]) {
	  movementIntegrateCnt++;
	  mask->normalizeGrid = mask->normalizeGrid ||
	    (movementIntegrateCnt % globalProbGridParameters.numberOfShiftsForNormalize == 0);
	}
	
	/* Check for angle integration. */
	if ( mask->consider[ANGLE] && mask->perform[ANGLE][INTEGRATE_ANGLE_GRID]) {
	  angleIntegrateCnt++;
	  mask->normalizeGrid = mask->normalizeGrid ||
	    (angleIntegrateCnt % globalProbGridParameters.numberOfAnglesForNormalize == 0);
	}
      }
    }
    /* Above threshold. */
    else {
      firstTimeBelowThreshold = TRUE;
      mask->normalizeGrid = FALSE;
      sonarIntegrateCnt = movementIntegrateCnt = angleIntegrateCnt = laserIntegrateCnt = 0;
    }
    
    
    /* If the robot does not move we update the grid. */
    if ( info->actualSensings.noMoveCnt == 10
	 && ( sonarIntegrateCnt > 0 || movementIntegrateCnt > 0
	      || laserIntegrateCnt > 0 || angleIntegrateCnt > 0)) {
      writeLog( "# No motion. Normalize.\n");
      mask->normalizeGrid = TRUE;
    }
    
    mask->normalizeGrid = mask->normalizeGrid || communicationInfo.scr->newMarker;
    
    if ( mask->normalizeGrid)
      sonarIntegrateCnt = movementIntegrateCnt = angleIntegrateCnt = laserIntegrateCnt = 0;
  }
}


/*****************************************************************************
 * This is mostly normalizing.
 *****************************************************************************/
void
performActions_PROBGRID( actionInformation* info, sensingActionMask* mask)
{
  if ( info->useProbGrid) {

    gridPosition pos;
    realPosition correction;
    static bool firstTime = TRUE;

  /* These things should also be done if the grid is initialized with some start
   * positions. */
    if ( firstTime || mask->normalizeGrid) {

      pos = normalizePositionProbabilityGrid( &(info->positionProbs),
					      &(info->initialPositionProbs));

      findLocalMaxima( &(info->positionProbs), &(info->localMaxima),
		       globalProbGridParameters.deltaPosForLocalMax,
		       globalProbGridParameters.deltaRotForLocalMax);

      if (info->localMaxima.numberOfCells > 0) {
      
	correction.x = info->estimatedRobot.pos.x - info->localMaxima.cell[0].pos.x;
	correction.y = info->estimatedRobot.pos.y - info->localMaxima.cell[0].pos.y;
	correction.rot = info->estimatedRobot.pos.rot - info->localMaxima.cell[0].pos.rot;
	if (correction.rot > 180.0) correction.rot -= 360.0;
      
	/* Set the estimated position to the global maximum. */
	info->estimatedRobot.pos = info->localMaxima.cell[0].pos;
      
	computeCorrectionParam( info->actualSensings.basePosition,
				info->estimatedRobot.pos,
				&(info->correctionParam));
      
	writeLog( "%.2f %f %f %f %d #corr\n",
		  elapsedScriptTime,
		  info->correctionParam.x,
		  info->correctionParam.y,
		  info->correctionParam.rot,
		  info->correctionParam.type);
      }
      else {
	correction.x = correction.y = correction.rot = 0.0;
	info->estimatedRobot.pos =
	  weightedPosition( pos, &(info->positionProbs));
      }
    
      /* Generate virtual measurements. */
      if ( globalProbGridParameters.generateVirtualSensor) {
	generateVirtualSensor( info);
      }

      if ( globalProbGridParameters.computeStatistics)
	computeGridStatistics( info, mask, correction);
      else {

	static int prevNumberOfSensors = 0;
	int numberOfSensors = 0, integrateAny = FALSE;
	int numberOfNewSensors = 0;
	informationsFor_PROXIMITY *proximityInfo;

	/* Number of integrated readings */
	if ( mask->use[SONAR]) {
	  proximityInfo = (informationsFor_PROXIMITY *) info->info[SONAR];
	  numberOfSensors = proximityInfo->numberOfIntegratedReadings;
	  if ( mask->perform[SONAR][INTEGRATE_SONAR])
	    integrateAny = TRUE;
	}

	if ( mask->use[LASER]) {
	  proximityInfo = (informationsFor_PROXIMITY *) info->info[LASER];
	  numberOfSensors += proximityInfo->numberOfIntegratedReadings;
	  if ( mask->perform[LASER][INTEGRATE_LASER])
	    integrateAny = TRUE;
	}

	if ( ! integrateAny)
	  numberOfNewSensors = -1;
	else
	  numberOfNewSensors = numberOfSensors - prevNumberOfSensors;

	writeLog( "%.2f %1f %1f %1f %f %d %f %d #max\n",
		  elapsedScriptTime,
		  info->localMaxima.cell[0].pos.x,
		  info->localMaxima.cell[0].pos.y,
		  rad2Deg(info->localMaxima.cell[0].pos.rot),
		  info->localMaxima.cell[0].prob * info->localMaxima.originalSumOfProbs,
		  info->localMaxima.numberOfCells,
		  info->actualSensings.distanceTraveled,
		  numberOfNewSensors);

	prevNumberOfSensors = numberOfSensors;
      }

      if (communicationInfo.scr->newMarker){
	informationsFor_PROXIMITY *laserInfo =
	  (informationsFor_PROXIMITY*) info->info[LASER] ;
	dumpDistanceToMarker( info->estimatedRobot.pos,
			      communicationInfo.scr->markerCount);
	if (info->simMap.initialized && 0){
	  (info->estimatedRobot.pos) =
	    scanMatchingPosition(info->estimatedRobot.pos,
				 &(info->simMap), info);
	  
	  dumpDistanceToMarker( info->estimatedRobot.pos,
				communicationInfo.scr->markerCount);
	  
	  updateGlobalRobotWindow( info, mask);
	  getchar();
	  info->estimatedRobot.pos =
	    markerPosition(communicationInfo.scr->markerCount);
	  updateGlobalRobotWindow( info, mask);
	  getchar();
	}
      }
      /* Perform a broadcast at the first call. */
      if ( firstTime) {
	firstTime = FALSE;
	broadcastStatusReport( NULL, CONSIDER_PLAN);
      }
    }
  }
}

/********************************************************************
 ********************************************************************
 * Global functions.
 ********************************************************************
 ********************************************************************/

bool
readStartPosition( realPosition* startPos)
{
  FILE* fp = fopen( POSITION_FILE, "r");
  bool success = FALSE;

  if ( fp != NULL) {
    if ( fscanf( fp, "%f %f %f", &(startPos->x), &(startPos->y), &(startPos->rot)) == 3) {
      writeLog("Read position %f %f %f from file.\n",
	       startPos->x, startPos->y, startPos->rot);
      system( REMOVE_POSITION_FILE);
      success = TRUE;
    }
    fclose(fp);
  }
  return success;  
}

void
writeCurrentPosition( realCellList* localMaxima)
{
  if ( localMaxima->numberOfCells > 0) {
    
    FILE* fp = fopen( POSITION_FILE, "w");
    
    if ( fp != NULL) {
      fprintf( fp, "%f %f %f\n", 
	       localMaxima->cell[0].pos.x,
	       localMaxima->cell[0].pos.y,
	       localMaxima->cell[0].pos.rot);
      fclose(fp);
    }
  }
}

/*****************************************************************************
 *  Returns the initial rotation of the given plane in the grid.
 *****************************************************************************/
static float
initialRotationOfPlane( int plane, positionProbabilityGrid* grid)
{
  return normalizedAngle( plane * grid->angleResolution);
}


/*****************************************************************************
 *  Returns the rotation of the given plane in the grid.
 *****************************************************************************/

float
rotationOfPlane( int plane, positionProbabilityGrid* grid)
{
  return normalizedAngle( grid->summedMovementOfPlane[plane].rot);
}


/*****************************************************************************
 *  Returns the plane closest to the rotation.
 *****************************************************************************/
int
planeOfRotation( float rot, positionProbabilityGrid* grid)
{
  int plane = round( ( rot - grid->summedMovementOfPlane[0].rot)
		     / grid->angleResolution);
  while (plane < 0) plane += grid->sizeZ;
  while (plane >= grid->sizeZ) plane -= grid->sizeZ;
  return plane;
}


/*****************************************************************************
 *  Returns the real position of the robot in the given grid.
 *****************************************************************************/
realPosition
realPositionOfGridPosition( gridPosition pos, positionProbabilityGrid* grid)
{
  /* Initialize the position with the remainder below resolution in the grid. */
  realPosition realPos = grid->summedMovementOfPlane[pos.rot];

  realPos.x += ( pos.x + 0.5) * grid->positionResolution + grid->offsetX;
  realPos.y += ( pos.y + 0.5) * grid->positionResolution + grid->offsetY;

  return realPos;
}


/*****************************************************************************
 *  Returns the grid position closest to the real position.
 *****************************************************************************/
gridPosition
gridPositionOfRealPosition( realPosition pos, positionProbabilityGrid* grid)
{
  gridPosition gridPos;
  int plane = planeOfRotation( pos.rot, grid);
  realPosition additionalMovement = grid->summedMovementOfPlane[plane];

  gridPos.x = ( pos.x - grid->offsetX - additionalMovement.x) / grid->positionResolution;
  gridPos.y = ( pos.y - grid->offsetY - additionalMovement.y) / grid->positionResolution;
  gridPos.rot = plane;

  return gridPos;
}


bool
coordinateInGrid( int x, int y, positionProbabilityGrid *grid) {
    return (x >= 0 && x < grid->sizeX && y >= 0 && y < grid->sizeY);
}

bool
realPositionInGrid( float x, float y, positionProbabilityGrid *grid) {
  return ( x >= grid->offsetX && x < grid->maxRealX &&
	   y >= grid->offsetY && y < grid->maxRealY);
}


/*****************************************************************************
 *  Returns the probability of the grid cell closest to the position.
 *****************************************************************************/
probability
probabilityOfRealPosition( realPosition pos, positionProbabilityGrid* grid)
{
  gridPosition gPos = gridPositionOfRealPosition( pos, grid);

  return grid->prob[gPos.rot][gPos.x][gPos.y];
}


/*****************************************************************************
 * Checks wether the normalize factor fo one of the planes has changed such
 * that this plane has to be updated again (or not any longer).
 * This only works if the single values of the planes have not been changed.
 *****************************************************************************/
int
normalizeFactorHasChangedUpdatePlanes( positionProbabilityGrid* grid)
{
  int plane, numberOfUpdates = 0;
  probability sum = 0.0, normalizeFactor;
  int changes = 0;

  /* Extract the normalize factor of the grid. */
  for ( plane = 0; plane < grid->sizeZ; plane++)
    sum += grid->sumOfPlane[plane] * grid->normalizeFactorOfPlane[plane];

  normalizeFactor = 1.0 / sum;
  fprintf(stderr, "%g %g %g %g %g\n", grid->minimumProbability, grid->maxProbabilityOfPlane[plane],
	  MIN_NORM_FACTOR, normalizeFactor, grid->maxProbabilityOfPlane[plane]
	  * MIN_NORM_FACTOR * normalizeFactor);
	  
  for ( plane = 0; plane < grid->sizeZ; plane++) {

    if ( grid->maxProbabilityOfPlane[plane]
	 * grid->normalizeFactorOfPlane[plane] * normalizeFactor
	 >= grid->minimumProbability) {

      numberOfUpdates++;
      if ( ! grid->updatePlane[plane]) {
	grid->updatePlane[plane] = TRUE;
	changes++;
      }
    }
    else if ( grid->updatePlane[plane]) {
      grid->updatePlane[plane] = FALSE;
      changes++;
    }
  }

  grid->quotaOfPlanesToBeUpdated = (float) numberOfUpdates / (float) grid->sizeZ;

  return changes;
}


/*****************************************************************************
 * Initializes the given planes with the a priori position probabilities.
 *****************************************************************************/
void
resetNewPlanes( positionProbabilityGrid* grid,
		bool* updatedLastCycle,
		probabilityGrid* aPrioriProbs)
{
  int x, y, z, index;
  probability tmp, minProb;
  float absMove;

  static int* resetIndex;
  static probability** normalizedAPrioriProbs;
  static bool firstTime = TRUE;
  int numberOfPlanesToBeReset = 0;


  if ( grid->sizeX != aPrioriProbs->sizeX || grid->sizeY != aPrioriProbs->sizeY) {
    fprintf( stderr, "Cannot set a priori probabilities with different sizes");
    fprintf( stderr, " ((%d, %d), (%d, %d)).\n",
	     grid->sizeX, grid->sizeY,
	     aPrioriProbs->sizeX, aPrioriProbs->sizeY);
    return;
  }

  /* Allocate memory and initialize the probabilities. */
  if ( firstTime) {

    float sum = 0.0, factor;

    firstTime = FALSE;
    resetIndex = allocate1D( grid->sizeZ, INT);
    normalizedAPrioriProbs = (probability**) allocate2D( aPrioriProbs->sizeX,
							 aPrioriProbs->sizeY,
							 PROBABILITY);

    for ( x = 0; x < grid->sizeX; x++)
      for ( y = 0; y < grid->sizeY; y++) {
	normalizedAPrioriProbs[x][y] = aPrioriProbs->prob[x][y];
	sum += aPrioriProbs->prob[x][y];
      }

    /* Normalize such that the sum of all planes becomes 1.0. */
    if ( sum == 0.0 || grid->sizeZ == 0.0) {
      fprintf( stderr, "Error in resetNewPlanes ( %g, %d).\n", sum, grid->sizeZ);
      closeLogAndExit( -1);
    }
    factor = 1.0 / (sum * grid->sizeZ);

    for ( x = 0; x < grid->sizeX; x++)
      for ( y = 0; y < grid->sizeY; y++) {
	normalizedAPrioriProbs[x][y] *= factor;
      }
  }

  /* Check which planes are updated the next time. If the movement since
   * the last update is too big we reset the probabilities. */
  for ( z=0; z < grid->sizeZ; z++) {

    if (1) {
      if ( grid->updatePlane[z] && ! updatedLastCycle[z])
	writeLog( "++++++++++++ %d\n", z);
      if ( ! grid->updatePlane[z] && updatedLastCycle[z])
	writeLog( "------------ %d\n", z);
    }

    if ( grid->updatePlane[z] && ! updatedLastCycle[z]) {

      absMove = sqrt( fSqr( grid->summedMovementOfPlane[z].x)
		      + fSqr( grid->summedMovementOfPlane[z].y));

      if ( absMove > globalProbGridParameters.minShiftForReset) {

	writeLog( "# Too much movement in plane %d (%f). Reset probabilities.\n",
		  z, absMove);

	resetIndex[numberOfPlanesToBeReset] = z;
	numberOfPlanesToBeReset++;

	/* Reset the movement. */
	grid->summedMovementOfPlane[z].x = 0.0;
	grid->summedMovementOfPlane[z].y = 0.0;
	grid->normalizeFactorOfPlane[z]  = 1.0;
      }
    }
  }

  minProb = grid->minimumProbability * 100;
  /* Reset the probabilities. */
  for ( x = 0; x < grid->sizeX; x++)
    for ( y = 0; y < grid->sizeY; y++) {
      tmp = fMin( normalizedAPrioriProbs[x][y], minProb);
      for ( index = 0; index < numberOfPlanesToBeReset; index++)
	grid->prob[resetIndex[index]][x][y] = tmp;
    }
}

/*****************************************************************************
 * Initializes the given planes with the a priori position probabilities.
 *****************************************************************************/
void
multiplyPlane( positionProbabilityGrid* grid,
	       int plane,
	       probability factor,
	       probability* globalMaxProb,
	       gridPosition* globalMaxPosition)
{
  int x, y;
  register probability *probPtr;

  /* Let's chech which is the smallest value not causing an underflow. */
  probability minVal = MINIMUM_PROBABILITY / factor;

  for ( x = 0; x < grid->sizeX; x++){
    probPtr = grid->prob[plane][x];
    for ( y = 0; y < grid->sizeY; y++, probPtr++) {

      if ( *probPtr > minVal) {
	*probPtr *= factor;

	/* Check for very extreme probabilities. */
	if (*probPtr > MAXIMUM_PROBABILITY) {
	  *probPtr = MAXIMUM_PROBABILITY;
	}
	else if (*probPtr < MINIMUM_PROBABILITY) {
	  *probPtr = MINIMUM_PROBABILITY;
	}

	/* Check for local and global maximum. */
	if ( *probPtr > grid->maxProbabilityOfPlane[plane]) {
	  grid->maxProbabilityOfPlane[plane] = *probPtr;
	  if ( *probPtr > *globalMaxProb) {
	    *globalMaxProb = *probPtr;
	    globalMaxPosition->x = x;
	    globalMaxPosition->y = y;
	    globalMaxPosition->rot = plane;
	  }
	}
      }
    }
  }
}


int
torusPosition(int position, int size)
{
  while (position < 0)
    position += size;

  while (position >= size)
    position -= size;

  return position;
}


int
cmToCells( float cm, positionProbabilityGrid *posGrid)
{
  return round( cm / posGrid->positionResolution);
}

int
radToPlanes( float rad, positionProbabilityGrid *posGrid)
{
  return round( rad / posGrid->angleResolution);
}


void
computeWeightedPositionAndSum( gridPosition max,
			       positionProbabilityGrid *posGrid,
			       realPosition *realPos,
			       probability *probSum,
			       probability *stdDeviation,
			       int deltaPos,
			       int deltaRot)
{
  int zCount;
  int deltaPosX = deltaPos, deltaPosY = deltaPos;
  int startX, endX, startY, endY;
  bool first = TRUE;
  probability sum, prob, prevRot=0.0, dist, stdDev=0.0;
  realPosition weightedPos,tmpPos;
  gridPosition gridPos;

  if (max.x < deltaPosX)
    deltaPosX = max.x;
  if (posGrid->sizeX - max.x - 1 < deltaPosX)
    deltaPosX = posGrid->sizeX - max.x -1;
  startX = max.x - deltaPosX;
  endX = max.x + deltaPosX + 1;

  if (max.y < deltaPosY)
    deltaPosY = max.y;
  if (posGrid->sizeY - max.y - 1 < deltaPosY)
    deltaPosY = posGrid->sizeY - max.y -1;
  startY = max.y - deltaPosY;
  endY = max.y + deltaPosY + 1;

  sum = weightedPos.x = weightedPos.y = weightedPos.rot = 0.0;

  for ( zCount = max.rot - deltaRot;
	zCount <= max.rot + deltaRot; zCount++) {
    gridPos.rot = torusPosition(zCount, posGrid->sizeZ);
    if (posGrid->updatePlane[gridPos.rot]) {
      for ( gridPos.x = startX; gridPos.x < endX; gridPos.x++)
	for ( gridPos.y = startY; gridPos.y < endY; gridPos.y++){
	  sum += (prob = posGrid->prob[gridPos.rot][gridPos.x][gridPos.y]);
	  tmpPos = realPositionOfGridPosition(gridPos, posGrid);
	  tmpPos.rot = normalizedAngle(tmpPos.rot);

	  /* Check wether the angle has skipped from 360 to 0 degrees.
	   * If so, we must add 360 degrees to get the average. */
	  if ( ! first) {
	    if ( tmpPos.rot < prevRot)
	      tmpPos.rot += DEG_360;
	  }
	  else
	    first = FALSE;
	  prevRot = tmpPos.rot;

	  weightedPos.x += tmpPos.x * prob;
	  weightedPos.y += tmpPos.y * prob;
 	  weightedPos.rot += tmpPos.rot * prob;
	}
    }
  }

  if (sum > 0){
    probability factor = 1/sum;
    weightedPos.x *=  factor;
    weightedPos.y *=  factor;
    weightedPos.rot = normalizedAngle(weightedPos.rot*factor);

    /* Now compute the standard deviation of the positions. */
    for ( zCount = max.rot - deltaRot;
	  zCount <= max.rot + deltaRot; zCount++) {
      gridPos.rot = torusPosition(zCount, posGrid->sizeZ);
      if (posGrid->updatePlane[gridPos.rot]) {
	for ( gridPos.x = startX; gridPos.x < endX; gridPos.x++)
	  for ( gridPos.y = startY; gridPos.y < endY; gridPos.y++){

	    prob = factor * posGrid->prob[gridPos.rot][gridPos.x][gridPos.y];

	    tmpPos = realPositionOfGridPosition(gridPos, posGrid);
	    tmpPos.rot = normalizedAngle(tmpPos.rot);
	    dist = distanceBetweenPoints( tmpPos, weightedPos);

	    stdDev += prob * fSqr( dist);
	  }
      }
    }
    stdDev = sqrt( stdDev);
  }
  *realPos = weightedPos;
  *probSum = sum;
  *stdDeviation = stdDev;
}


realPosition
weightedPosition( gridPosition max,
		  positionProbabilityGrid *posGrid)
{
  realPosition weightedPos;
  probability probSum, stdDev;

  computeWeightedPositionAndSum( max, posGrid, &weightedPos,
				 &probSum, &stdDev,
				 globalProbGridParameters.deltaPosForLocalMax,
				 globalProbGridParameters.deltaRotForLocalMax);

  return weightedPos;
}



/*****************************************************************************
 * Normalizes the values in a probability grid to sum up to 1.
 * returns the position of the cell containing the maximum;
 *****************************************************************************/
gridPosition
normalizePositionProbabilityGrid( positionProbabilityGrid* posGrid,
				  probabilityGrid* aPrioriProbs)
{
  register int z;
  double normalizeFactor;
  probability maxProb = 0.0;
  gridPosition maxProbPos;
  int markedFields;

  /* These variables are used to check which planes are updated for
   * the first time. */
  static bool* updatedLastCycle = NULL;
  if ( updatedLastCycle == NULL) {
    updatedLastCycle = allocate1D( posGrid->sizeZ, BOOL);
    for ( z=0; z < posGrid->sizeZ; z++)
      updatedLastCycle[z] = TRUE;
  }


  setTimer(0);
  writeLog( "# Normalizing grid ");
  fprintf(stderr, "# Normalizing grid ... ");

  /* Get the normalize factor. */
  normalizeFactor = 1.0 / sumOverAllProbabilities( posGrid);

  posGrid->normalizeFactor = normalizeFactor;

  writeLog( "\n%g #factor\n", normalizeFactor);

  /* Normalize all the values. For the planes not updated we only have
   * to adjust the maximal value and the sum of the values of the plane.
   */

  for ( z=0; z < posGrid->sizeZ; z++) {

    /*-----------------------------------------------------------------------
     * No update in the last cycle.
     *-----------------------------------------------------------------------*/

    if ( ! posGrid->updatePlane[z]) {

      /* If the normalizeFactor sets the maximal value above the update level
       * this plane has to be updated again.
       */
      posGrid->updatePlane[z] =
	( posGrid->maxProbabilityOfPlane[z]
	  * normalizeFactor * posGrid->normalizeFactorOfPlane[z])
	> posGrid->minimumProbability;

      /* We only have to adjust the glboal values of the plane. */
      posGrid->normalizeFactorOfPlane[z] *= normalizeFactor;
    }

    /*-----------------------------------------------------------------------
     * These planes have to be updated.
     *-----------------------------------------------------------------------*/
    else {

      /* Get the normalize factor for the plane. */
      probability planeFactor =
	posGrid->normalizeFactorOfPlane[z] * normalizeFactor;

      /* Reset all accumulated values. */
      posGrid->maxProbabilityOfPlane[z]  = 0.0;
      posGrid->sumOfPlane[z]            *= planeFactor;
      posGrid->normalizeFactorOfPlane[z] = 1.0;

      swallowStatusReports(DONT_WAIT);

      /* Multiply each value of the plane with the normalize factor. */
      multiplyPlane( posGrid,
		     z,
		     planeFactor,
		     &maxProb,
		     &maxProbPos);

      posGrid->updatePlane[z] =
	posGrid->maxProbabilityOfPlane[z] > posGrid->minimumProbability;
    }
  }


  /* Planes neighboring updated planes are also updated. */
  markedFields =
    markNeighboringPlanes( posGrid,
			   globalProbGridParameters.neighborPlanesToAdd);
  posGrid->quotaOfPlanesToBeUpdated = (float) markedFields
    / (float) posGrid->sizeZ;

  /* Check which planes are updated the next time. If the movement since
   * the last update is too big we reset the probabilities. */
  resetNewPlanes( posGrid, updatedLastCycle, aPrioriProbs);

  for ( z=0; z < posGrid->sizeZ; z++) {
    if (posGrid->normalizeFactorOfPlane[z] < MIN_NORM_FACTOR)
      posGrid->normalizeFactorOfPlane[z] = MIN_NORM_FACTOR;
    updatedLastCycle[z] = posGrid->updatePlane[z];
  }

  {
    writeLog( "active planes: ");

    for ( z=0; z < posGrid->sizeZ; z++)

      if ( posGrid->updatePlane[z])
	  writeLog( "%d ", z);
      writeLog( "\n");
  }

  writeLog( "done in %.2f secs (%d planes).\n", timeExpired(0), markedFields);
  fprintf(stderr, "done in %.2f secs (%d planes).\n", timeExpired(0), markedFields);

  return maxProbPos;
}





/*****************************************************************************
 * Shifts the values by <delta>. At the same time multiplies them with the
 * probabilities of the probabilityGrid.
 *****************************************************************************/
void
shiftAndMultiplyProbabilities( positionProbabilityGrid* posGrid,
			       probabilityGrid* mapPositionProbs,
			       int plane,
			       int sizeX, int sizeY,
			       int deltaX, int deltaY,
			       int usePosition)
{
  int x, y;
  int stepX, stepY;
  int firstX, firstY;
  int originalX, originalY;
  int firstOutOfRangeX, firstOutOfRangeY;
  int firstUndefinedX, firstUndefinedY;
  probability** gridPlane = posGrid->prob[plane];
  probability minProb = posGrid->minimumProbability;

  /* The motion is bigger than the whole environment. Set all positions to
   * impossible. */
  if ( iAbs( deltaX) > sizeX || iAbs( deltaY) > sizeY) {
    if ( plane == 0) {
      fprintf( stderr, "Motion too big for the environment. Reset probabilities.\n");
	writeLog( "Motion too big for the environment. Reset probabilities.\n");
	resetGrid( posGrid, mapPositionProbs);
    }
    return;
  }

  /* Set the directions to move through the grid. */
  if ( deltaX >= 0) {
    stepX = -1;
    firstX = sizeX - 1;
    firstOutOfRangeX = -1;
    firstUndefinedX = deltaX - 1;
  }
  else {
    stepX = 1;
    firstX = 0;
    firstOutOfRangeX = sizeX;
    firstUndefinedX = firstOutOfRangeX + deltaX;
  }

  if ( deltaY >= 0) {
    stepY = -1;
    firstY = sizeY - 1;
    firstOutOfRangeY = -1;
    firstUndefinedY = deltaY - 1;
  }
  else {
    stepY = 1;
    firstY = 0;
    firstOutOfRangeY = sizeY;
    firstUndefinedY = firstOutOfRangeY + deltaY;
  }

  /* These indices should be guaranteed to exist. No range check has
   * to be made even for the shifted values. */


  if ( usePosition) {
    for ( x = firstX, originalX = firstX - deltaX;
	  x != firstUndefinedX;
	  x += stepX, originalX += stepX)
      for ( y = firstY, originalY = firstY - deltaY;
	    y != firstUndefinedY;
	    y += stepY, originalY += stepY)
	if (gridPlane[originalX][originalY] > minProb)
	  gridPlane[x][y]  =
	    gridPlane[originalX][originalY] * mapPositionProbs->prob[x][y];
	else
	  gridPlane[x][y] = gridPlane[originalX][originalY];
  }
  else
    for ( x = firstX, originalX = firstX - deltaX;
	  x != firstUndefinedX;
	  x += stepX, originalX += stepX)
      for ( y = firstY, originalY = firstY - deltaY;
	    y != firstUndefinedY;
	    y += stepY, originalY += stepY)
	gridPlane[x][y] =
	  gridPlane[originalX][originalY];

  /* Fill the undefined fields with probability representing impossibility. */
  if ( firstUndefinedX >= 0 && firstUndefinedX < sizeX)
    for ( x = firstUndefinedX; x != firstOutOfRangeX; x += stepX)
      for ( y = 0; y < sizeY; y++)
	gridPlane[x][y] = IMPOSSIBLE;

  if ( firstUndefinedY >= 0 && firstUndefinedY < sizeY)
    for ( y = firstUndefinedY; y != firstOutOfRangeY; y += stepY)
      for ( x = 0; x < sizeX; x++)
	  gridPlane[x][y] = IMPOSSIBLE;
}


/********************************************************************
 ********************************************************************
 ********************************************************************
 * The local functions.
 ********************************************************************
 ********************************************************************
 ********************************************************************/

/*****************************************************************************
 * Initializes the grid with the a priori position probabilities.
 *****************************************************************************/
void
resetGrid( positionProbabilityGrid* grid,
	    probabilityGrid* aPrioriProbs)
{
  int x, y, z;
  probability tmp;

  writeLog( "# Reset position probabilities.\n");

  if ( grid->sizeX != aPrioriProbs->sizeX || grid->sizeY != aPrioriProbs->sizeY) {
    fprintf( stderr, "Cannot set a priori probabilities with different sizes");
    fprintf( stderr, " ((%d, %d), (%d, %d)).\n",
	     grid->sizeX, grid->sizeY,
	     aPrioriProbs->sizeX, aPrioriProbs->sizeY);
    return;
  }

  for ( x = 0; x < grid->sizeX; x++)
    for ( y = 0; y < grid->sizeY; y++) {
      tmp = aPrioriProbs->prob[x][y];

      for ( z = 0; z < grid->sizeZ; z++) {
	grid->prob[z][x][y] = tmp;
	grid->prob[z][x][y] = 1.0;
      }
    }

  grid->quotaOfValuesToBeUpdated                     = 1.0;
  grid->quotaOfPlanesToBeUpdated                     = 1.0;

  /* The next elements are necessary to be able update only planes
   * which contain high enough probabilities.
   */
  for ( z = 0; z < grid->sizeZ; z++) {
    grid->updatePlane[z]                               = TRUE;
    grid->sumOfPlane[z]                                = 0.0;
    grid->maxProbabilityOfPlane[z]                     = 0.0;
    grid->normalizeFactorOfPlane[z]                    = 1.0;

    grid->summedMovementOfPlane[z].x                   = 0.0;
    grid->summedMovementOfPlane[z].y                   = 0.0;
    grid->summedMovementOfPlane[z].rot                 = initialRotationOfPlane( z, grid);
  }
  normalizePositionProbabilityGrid( grid, aPrioriProbs);
}



probabilityGrid
preprocessedPositionProbabilities( probabilityGrid* map, float additionalSize)
{
  int x,y;
  probability sumOfProbs = 0.0, normalizeFactor;
  probabilityGrid posProb;

  posProb = *map;
  posProb.prob = (mapProbability**) allocate2D( map->sizeX, map->sizeY, MAP_PROBABILITY);
  
  for (x = 0; x < posProb.sizeX; x++)
    for (y = 0; y < posProb.sizeY; y++)
      posProb.prob[x][y] = positionProbabilityMap( x, y, *map, additionalSize);

  /* Normalize the values such that the average probability will be 1.0. */
  for (x = 0; x < posProb.sizeX; x++)
    for (y = 0; y < posProb.sizeY; y++)
      sumOfProbs += posProb.prob[x][y];

  normalizeFactor = (probability) posProb.sizeX
    * (probability) posProb.sizeY / sumOfProbs;
  
  setStatistics( &posProb);
  
  posProb.initialized = TRUE;

  return posProb;
}


/*****************************************************************************
 * Allocates memory and initializes the probabilities with the values of the
 * probabilityGrid.
 *****************************************************************************/
static positionProbabilityGrid
initializedPositionProbabilityGrid( probabilityGrid* positionProbabilities,
				    int numberOfAngles)
{
  int sizeX, sizeY, sizeZ;

  positionProbabilityGrid grid;

  /* Copy relevant information from the map. */
  grid.sizeX              = positionProbabilities->sizeX;
  grid.sizeY              = positionProbabilities->sizeY;
  grid.sizeZ              = numberOfAngles;
  grid.positionResolution = positionProbabilities->resolution;
  grid.offsetX            = positionProbabilities->offsetX;
  grid.offsetY            = positionProbabilities->offsetY;
  grid.maxRealX           = positionProbabilities->maxRealX;
  grid.maxRealY           = positionProbabilities->maxRealY;
  grid.origsizeX          = positionProbabilities->origsizeX;
  grid.origsizeY          = positionProbabilities->origsizeY;

  sizeX = grid.sizeX;
  sizeY = grid.sizeY;
  sizeZ = grid.sizeZ;

  grid.angleResolution = DEG_360 / numberOfAngles;
  grid.minimumProbability = MINIMUM_UPDATE_PROBABILITY_QUOTA / (sizeX * sizeY * sizeZ);
  grid.quotaOfValuesToBeUpdated = 1.0;
  grid.quotaOfPlanesToBeUpdated = 1.0;

  /*********************************************
   * Allocate memory
   *********************************************/
  grid.prob = (probability***) allocate3D(  sizeZ, sizeX, sizeY,PROBABILITY);

  /* The next elements are necessary to be able to update only planes
   * which contain high enough probabilities.
   */
  grid.updatePlane            = (bool*)         allocate1D( sizeZ, BOOL);
  grid.summedMovementOfPlane  = (realPosition*) allocate1D( sizeZ, REAL_POSITION);
  grid.sumOfPlane             = (probability*)  allocate1D( sizeZ, PROBABILITY);
  grid.maxProbabilityOfPlane  = (probability*)  allocate1D( sizeZ, PROBABILITY);
  grid.normalizeFactorOfPlane = (probability*)  allocate1D( sizeZ, PROBABILITY);

  /* Initializes the grid with the a priori position probabilities. */
  resetGrid( &grid, positionProbabilities);

  /* Normalize the probabilities such that they sum up to 1. */
  (void) normalizePositionProbabilityGrid(&grid, positionProbabilities);

  grid.initialized = TRUE;

  return grid;
}



/*****************************************************************************
 * Computes the sum over all planes and uses the sumOfPlane-values.
 *****************************************************************************/
static probability
sumOverAllProbabilities( positionProbabilityGrid* posGrid)
{
  register int z,y,x;
  float ***grid;
  double sum = 0.0;
  probability planeSum;

  /* compute the sum over all elements. For planes not updated
   * we can use the precomputed sums.
   */

  grid = posGrid->prob;
  for ( z=0; z < posGrid->sizeZ; z++) {
    if ( posGrid->updatePlane[z]) {
      planeSum = 0.0;
      for ( x = 0; x < posGrid->sizeX; x++)
	for ( y = 0; y < posGrid->sizeY; y++)
	  planeSum += grid[z][x][y];
      posGrid->sumOfPlane[z] = planeSum;
    }
    sum += posGrid->sumOfPlane[z] * posGrid->normalizeFactorOfPlane[z];
  }
  return (probability) sum;
}



/***********************************************************************
 * Computes the normalized distance of two grid positions
 * The distance is normalized to [0;1]
 ***********************************************************************/
float
gridPositionAndAngleDistance( gridPosition pos1, gridPosition pos2,
			      positionProbabilityGrid *grid)
{
  float dist, angleDiff, mapDiagonalSquare;
  int rotDiff;

  rotDiff = abs(pos1.rot - pos2.rot);
  if (rotDiff > (grid->sizeZ / 2))
    rotDiff = grid->sizeZ - rotDiff;

  angleDiff = (2.0 * rotDiff) / grid->sizeZ;

  /* large rotational errors are much more important than small */
  angleDiff *= angleDiff;

  /* Compute the length of the diagonal of the map */

  mapDiagonalSquare = (fSqr(grid->sizeX) + fSqr(grid->sizeY));

  dist = sqrt( (float) 0.5 * ((fSqr(pos1.x - pos2.x) +
			       fSqr(pos1.y - pos2.y))
			      / mapDiagonalSquare +
			      fSqr(angleDiff)));

  /*   fprintf(stderr, "Distance : %f\n", dist); */

  return dist;
}


/*****************************************************************************
 * Marks planes of planes marked as update planes.
 *****************************************************************************/
static int
markNeighboringPlanes( positionProbabilityGrid* posGrid,
		       int numberOfNeighbors)
{
  int z;
  int markedFields = 0;
  int loopCnt;

  for (loopCnt = 0; loopCnt < numberOfNeighbors; loopCnt++) {

    bool updatePreviousPlane = posGrid->updatePlane[posGrid->sizeZ-1];
    bool updateNextPlane    = posGrid->updatePlane[1];

    markedFields = 0;

    /* Mark the neighboring planes of update planes. */
    for ( z=0; z < posGrid->sizeZ; z++) {

      if ( ! posGrid->updatePlane[z]) {
	posGrid->updatePlane[z] =
	  updatePreviousPlane || updateNextPlane;
	if ( posGrid->updatePlane[z])
	  markedFields++;
	updatePreviousPlane = FALSE;
      }
      else {
	markedFields++;
	updatePreviousPlane = TRUE;
      }

      updateNextPlane = posGrid->updatePlane[(z+2) % posGrid->sizeZ];
    }
  }
  return markedFields;
}



void
setPosition( realPosition centerReal,
	     positionProbabilityGrid* grid,
	     float deviation)
{
  int x, y, z, zCount;
  float sumOfPlane = grid->sizeX * grid->sizeY * grid->minimumProbability;
  int cubeSizeXY = 2 * deviation;
  int cubeSizeZ  = 2 * deviation;
  float angleDist;

  gridPosition center = gridPositionOfRealPosition( centerReal, grid);

  /* Set all values to min probability. */
  for ( z = 0; z < grid->sizeZ; z++)
    for ( x = 0; x < grid->sizeX; x++)
      for ( y = 0; y < grid->sizeY; y++)
	grid->prob[z][x][y] = grid->minimumProbability;

  /* Initialize the values of the planes. */
  for ( z = 0; z < grid->sizeZ; z++) {
    grid->updatePlane[z]               = FALSE;
    grid->normalizeFactorOfPlane[z]    = 1.0;
    grid->maxProbabilityOfPlane[z]     = grid->minimumProbability;
    grid->sumOfPlane[z]                = sumOfPlane;
    grid->summedMovementOfPlane[z].x   = 0.0;
    grid->summedMovementOfPlane[z].y   = 0.0;
    grid->summedMovementOfPlane[z].rot = initialRotationOfPlane( z, grid);
  }


  /* Set the position. */
  for ( zCount = center.rot - cubeSizeZ;
	zCount <= center.rot + cubeSizeZ;
	zCount++) {

    z = torusPosition( zCount, grid->sizeZ);
    angleDist = rad2Deg( angleDistance( centerReal.rot, rotationOfPlane( z, grid)));

    /* The maximum probability is given by the cell vertically above the center.  */
    grid->maxProbabilityOfPlane[z] = fMax( grid->minimumProbability,
					   gauss( angleDist,
						  deviation,
						  0.0));

    if ( grid->maxProbabilityOfPlane[z] > grid->minimumProbability)
      grid->updatePlane[z] = TRUE;

    for ( x = iMax( 0, center.x - cubeSizeXY);
	  x < iMin( grid->sizeX, center.x + cubeSizeXY);
	  x++)
      for ( y = iMax( 0, center.y - cubeSizeXY);
	    y < iMin( grid->sizeY, center.y + cubeSizeXY);
	    y++) {

	float dist = sqrt( iSqr( x - center.x)
			   + iSqr( y - center.y)
			   + fSqr( angleDist));

	float gaussValue = gauss( dist, deviation, 0.0);

	if ( gaussValue > grid->minimumProbability) {

	  grid->prob[z][x][y] = gaussValue;

	  grid->sumOfPlane[z] += grid->prob[z][x][y]
	    - grid->minimumProbability;
	}
      }
  }

  grid->initialized = TRUE;
}



void
setPositionUniform( realPosition centerReal,
		    positionProbabilityGrid* grid,
		    float deviation)
{
  int x, y, z, zCount;
  float sumOfPlane = grid->sizeX * grid->sizeY * grid->minimumProbability;
  int cubeSizeXY = 6 * deviation;
  int cubeSizeZ  = 3 * deviation;
  float p;
  
  gridPosition center = gridPositionOfRealPosition( centerReal, grid);

  /* Set all values to min probability. */
  for ( z = 0; z < grid->sizeZ; z++)
    for ( x = 0; x < grid->sizeX; x++)
      for ( y = 0; y < grid->sizeY; y++)
	grid->prob[z][x][y] = grid->minimumProbability;

  /* Initialize the values of the planes. */
  for ( z = 0; z < grid->sizeZ; z++) {
    grid->updatePlane[z]               = FALSE;
    grid->normalizeFactorOfPlane[z]    = 1.0;
    grid->maxProbabilityOfPlane[z]     = grid->minimumProbability;
    grid->sumOfPlane[z]                = sumOfPlane;
    grid->summedMovementOfPlane[z].x   = 0.0;
    grid->summedMovementOfPlane[z].y   = 0.0;
    grid->summedMovementOfPlane[z].rot = initialRotationOfPlane( z, grid);
  }

  p = 1.0;
  
  p = p / (iMin(grid->sizeX, center.x + cubeSizeXY)
	   - iMax(0, center.x - cubeSizeXY)); 
  p = p / (iMin(grid->sizeY, center.y + cubeSizeXY)
	   - iMax(0, center.y - cubeSizeXY)); 
  p = p / (2 * cubeSizeZ - 1);
  
  /* Set the position. */
  for ( zCount = center.rot - cubeSizeZ;
	zCount <= center.rot + cubeSizeZ;
	zCount++) {

    z = torusPosition( zCount, grid->sizeZ);

    grid->maxProbabilityOfPlane[z] = fMax( grid->minimumProbability,
					   p);
    if ( grid->maxProbabilityOfPlane[z] > grid->minimumProbability)
      grid->updatePlane[z] = TRUE;

    for ( x = iMax( 0, center.x - cubeSizeXY);
	  x < iMin( grid->sizeX, center.x + cubeSizeXY);
	  x++)
      for ( y = iMax( 0, center.y - cubeSizeXY);
	    y < iMin( grid->sizeY, center.y + cubeSizeXY);
	    y++) {


	if ( p  > grid->minimumProbability) {

	  grid->prob[z][x][y] = p;

	  grid->sumOfPlane[z] += grid->prob[z][x][y]
	    - grid->minimumProbability;
	}
      }
  }

  grid->initialized = TRUE;
}


/***********************************************************************
 * Sets peaks for each cell in the list
 ***********************************************************************/
void
setCellList( positionProbabilityGrid *grid, gridCellList *cells)
{
  int i, x, y, z;
  gridPosition pos;

  /* set every field to zero */

  if (cells->numberOfCells > 0){
    for (z = 0; z < grid->sizeZ; z++){
      grid->updatePlane[z] = FALSE;
      grid->normalizeFactorOfPlane[z]=1.0;
      grid->sumOfPlane[z] = MINIMUM_PROBABILITY * grid->sizeX * grid->sizeY;
      grid->maxProbabilityOfPlane[z] = MINIMUM_PROBABILITY;

      for (x = 0; x < grid->sizeX; x++)
	for (y = 0; y < grid->sizeY; y++)
	  grid->prob[z][x][y] = MINIMUM_PROBABILITY;
    }

    /* set peaks */
    for (i = 0; i < cells->numberOfCells; i++){
      pos = cells->cell[i].pos;
      grid->prob[pos.rot][pos.x][pos.y] =  cells->cell[i].prob;
      grid->sumOfPlane[pos.rot] += grid->prob[pos.rot][pos.x][pos.y]
	- MINIMUM_PROBABILITY;
      grid->updatePlane[pos.rot] = TRUE;
      grid->maxProbabilityOfPlane[z] = cells->cell[i].prob;
    }

    grid->quotaOfValuesToBeUpdated = (float) cells->numberOfCells
      / (float) (grid->sizeX * grid->sizeY * grid->sizeZ);
    grid->quotaOfPlanesToBeUpdated = (float) cells->numberOfCells / (float) grid->sizeZ;
  }
}


/* Remove the cells no longer in the map */
void
removeCellsNotInGrid( realCellList* cellList,
		      positionProbabilityGrid* grid)
{
  int cell = 0;

  while ( cell < cellList->numberOfCells) {

    int i;
    gridPosition gridPos = gridPositionOfRealPosition( cellList->cell[cell].pos,
						       grid);

    if ( coordinateInGrid( gridPos.x, gridPos.y, grid)) {
      cellList->inMap[cell] = TRUE;
      cell++;
    }
    else {
      cellList->originalSumOfProbs -= cellList->cell[cell].prob;
      cellList->numberOfCellsInMap--;
      for (i = cell; i < cellList->numberOfCells - 1; i++)
	cellList->cell[i] = cellList->cell[i+1];
      cellList->numberOfCells--;
    }
  }
  normalizeRealCellList( cellList);
}

static void
computeGridStatistics( actionInformation* info,
		       sensingActionMask* mask,
		       realPosition correction)
{
  float entropy = 0.0;
  float entropyOfLocalMax = 0.0;
  float distance = 0.0;
  float angleDistance = 0.0;
  float usedTime;
  bool success;
  realPosition estimatedPos, measuredPos;
  realPosition dummyPos;
  probability dummy;
  probability bayesError = 0.0, probOfMeasuredPos = 0.0,
    probOfEstimatedPos = 0.0, stdDev;
  informationsFor_PROXIMITY *proximityInfo;
  static int prevNumberOfSensors = 0;
  int numberOfSensors = 0;
  float correctionDist = sqrt(fSqr(correction.x)+fSqr(correction.y));

  fprintf( stderr, "# =================================================\n");
  fprintf( stderr, "# Computing statistics ...");
  setTimer( 0);

  /*-------------------------------------------------------
   * Get some values.
   *-------------------------------------------------------*/

  /* Number of integrated readings */
  if ( mask->use[SONAR]) {
    proximityInfo = (informationsFor_PROXIMITY *) info->info[SONAR];
    numberOfSensors = proximityInfo->numberOfIntegratedReadings;
  }
  if ( mask->use[LASER]) {
    proximityInfo = (informationsFor_PROXIMITY *) info->info[LASER];
    numberOfSensors += proximityInfo->numberOfIntegratedReadings;
  }

  /* Reference position from position.log. */
  measuredPos = measuredRobotPosition( elapsedScriptTime, &success);

  /* Position of the global maximum. */
  estimatedPos = info->estimatedRobot.pos;

#ifndef PROXIMITY_OUTPUT
  /*-------------------------------------------------------
   * Compute statistics.
   *-------------------------------------------------------*/
  /* Entropy of the probability distribution. */
  entropy = posGridEntropy( &(info->positionProbs));
  entropyOfLocalMax = realCellListEntropy( &(info->localMaxima));

  /* Weighted error of position estimation. */
  bayesError = bayesEstimationError( &(info->positionProbs), measuredPos);
#endif

  /* Summed probability of cube around estimated position. */
  computeWeightedPositionAndSum( gridPositionOfRealPosition( estimatedPos,
							     &(info->positionProbs)),
				 &(info->positionProbs),
				 &dummyPos,
				 &probOfEstimatedPos,
				 &stdDev,
				 globalProbGridParameters.deltaPosForLocalMax,
				 globalProbGridParameters.deltaRotForLocalMax);

  /* Summed probability of cube around measured position. */
  computeWeightedPositionAndSum( gridPositionOfRealPosition( measuredPos,
							     &(info->positionProbs)),
				 &(info->positionProbs),
				 &dummyPos,
				 &probOfMeasuredPos,
				 &dummy,
				 globalProbGridParameters.deltaPosForLocalMax,
				 globalProbGridParameters.deltaRotForLocalMax);

  /* Distance between measured and estimated position. */
  distance = realPositionDistance(measuredPos, estimatedPos);
  angleDistance = rad2Deg(normalizedAngle(measuredPos.rot - estimatedPos.rot));
  if ( angleDistance > 180.0) angleDistance = 360.0 - angleDistance;

  /* Info line. */
  {
    static bool firstTime = TRUE;
    if ( firstTime) {
      firstTime = FALSE;
      writeLog( "#readings time bayesError entropy  probOfEstimated ");
      writeLog( "probOfMeasured  dist angleDist correction numberOfMaxima stdDev");
      writeLog( " newReadings #statistics\n");
    }
  }

  /* Special position line. */
  writeLog( "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f #position\n",
	    elapsedScriptTime, estimatedPos.x, estimatedPos.y,
	    rad2Deg(estimatedPos.rot),
	    correction.x, correction.y, rad2Deg(correction.rot),
	    correctionDist);

  /* All values. */
  writeLog( "%d %.2f %f %f %f %f %f %f %f %d %f %d #statistics\n",
	    numberOfSensors, elapsedScriptTime, bayesError, entropy,
	    probOfEstimatedPos,
	    probOfMeasuredPos,
	    distance, angleDistance,
	    correctionDist,
	    info->localMaxima.numberOfCells,
	    stdDev, numberOfSensors - prevNumberOfSensors);

  prevNumberOfSensors = numberOfSensors;

/* #define PROXIMITY_OUTPUT   */
#ifdef PROXIMITY_OUTPUT
  if (1) generateVirtualSensor( info);
  fprintf(stderr, "\n");
/*   dumpProximityValues( info, mask, estimatedPos);  */
/*   fprintf(stderr, "\n"); */
#endif
  
  usedTime = timeExpired( 0);
  nonRelevantTime += usedTime;
  fprintf( stderr, " done in %f secs.\n", usedTime);
}


static void
setTestPositions( actionInformation* info)
{

  /* test */
  if (0) {

    gridCellList list;
    int x = info->positionProbs.sizeX / 2;
    int y = 16;
    float rot = info->positionProbs.sizeZ / 4.0;

    list.numberOfCells = 2;
    list.cell[0].prob =0.499;
    list.cell[1].prob = 0.49;

    list.cell[0].pos.x = x - 0;
    list.cell[0].pos.y = y + 0;
    list.cell[0].pos.rot = rot;

    list.cell[1].pos.x = x - 3;
    list.cell[1].pos.y = info->positionProbs.sizeY - 1 - y;
    list.cell[1].pos.rot = rot * 3.0;

    setCellList( &(info->positionProbs), &list);
  }

  /* corridor-disturb */
  if (0) {

    gridCellList list;
    int x = info->positionProbs.sizeX / 2;
    int y = info->positionProbs.sizeY / 2;
    float rot = info->positionProbs.sizeZ / 4.0;

    list.numberOfCells = 2;
    list.cell[0].prob =0.5;
    list.cell[1].prob = 0.5;

    list.cell[0].pos.x = x - 0;
    list.cell[0].pos.y = y + 1;
    list.cell[0].pos.rot = 0.0 * rot;

    list.cell[1].pos.x = x - 3;
    list.cell[1].pos.y = y - 1;
    list.cell[1].pos.rot = rot * 2.0;

    setCellList( &(info->positionProbs), &list);
  }
  /* aaai */
  if (0) {

    gridCellList list;

    float rot = info->positionProbs.sizeZ / 4.0;

    list.numberOfCells = 2;
    list.cell[0].prob = 0.498;
    list.cell[1].prob = 0.499;

    list.cell[0].pos.x = 155;
    list.cell[0].pos.y = 67;
    list.cell[0].pos.rot = 0.0 * rot;

    list.cell[1].pos.x = 64;
    list.cell[1].pos.y = 90;
    list.cell[1].pos.rot = rot * 1.0;

    setCellList( &(info->positionProbs), &list);
  }

  /* floor2 */
  if (0) {

    gridCellList list;

    float rot = info->positionProbs.sizeZ / 4.0;

    list.numberOfCells = 2;
    list.cell[0].prob = 0.25;
    list.cell[1].prob = 0.25;

    list.cell[0].pos.x = 132;
    list.cell[0].pos.y = info->positionProbs.sizeY / 2.0 - 4;
    list.cell[0].pos.rot = 0.0 * rot;

    list.cell[1].pos.x = 64;
    list.cell[1].pos.y = info->positionProbs.sizeY / 2.0 + 5;
    list.cell[1].pos.rot = rot * 2.0;

    setCellList( &(info->positionProbs), &list);
  }

  /* floor4 */
  if (0) {

    gridCellList list;

    float rot = info->positionProbs.sizeZ / 4.0;

    list.numberOfCells = 4;
    list.cell[0].prob = 0.25;
    list.cell[1].prob = 0.25;
    list.cell[2].prob = 0.25;
    list.cell[3].prob = 0.25;

    list.cell[0].pos.x = 132;
    list.cell[0].pos.y = info->positionProbs.sizeY / 2.0 - 6;
    list.cell[0].pos.rot = 0.0 * rot;

    list.cell[1].pos.x = 104;
    list.cell[1].pos.y = info->positionProbs.sizeY / 2.0 - 6;
    list.cell[1].pos.rot = 0.0 * rot;

    list.cell[2].pos.x = 64;
    list.cell[2].pos.y = info->positionProbs.sizeY / 2.0 + 6;
    list.cell[2].pos.rot = rot * 2.0;

    list.cell[3].pos.x = 92;
    list.cell[3].pos.y = info->positionProbs.sizeY / 2.0 + 6;
    list.cell[3].pos.rot = rot * 2.0;

    setCellList( &(info->positionProbs), &list);
  }

  /* envA */
  if (0) {

    gridCellList list;

    float rot = info->positionProbs.sizeZ / 4.0;

    list.numberOfCells = 2;
    list.cell[0].prob = 0.499;
    list.cell[1].prob = 0.4991;

    if (0) {
      list.cell[0].pos.x = 5;
      list.cell[0].pos.y = 85;
      list.cell[0].pos.rot = 1.0 * rot;

      list.cell[1].pos.x = 105;
      list.cell[1].pos.y = 15;
      list.cell[1].pos.rot = 0.0 * rot;
    }
    else {
      list.cell[0].pos.x = 5;
      list.cell[0].pos.y = 55;
      list.cell[0].pos.rot = 1.0 * rot;

      list.cell[1].pos.x = 105;
      list.cell[1].pos.y = 114;
      list.cell[1].pos.rot = 0.0 * rot;
    }
    setCellList( &(info->positionProbs), &list);
  }

  /* envB */
  if (0) {

    gridCellList list;

    float rot = info->positionProbs.sizeZ / 4.0;

    list.numberOfCells = 2;
    list.cell[0].prob = 0.499;
    list.cell[1].prob = 0.499;

    list.cell[0].pos.x = 83;
    list.cell[0].pos.y = 23;
    list.cell[0].pos.rot = 1.0 * rot;

    list.cell[1].pos.x = 95;
    list.cell[1].pos.y = 24;
    list.cell[1].pos.rot = 3.0 * rot;

    setCellList( &(info->positionProbs), &list);
  }

  /* envC */
  if (0) {

    gridCellList list;

    int middle = info->positionProbs.sizeX / 2;
    list.numberOfCells = 2;

    list.cell[0].prob = 0.4991;
    list.cell[1].prob = 0.499;

    list.cell[0].pos.x = middle;
    list.cell[0].pos.y = 14;
    list.cell[0].pos.rot = 0;

    list.cell[1].pos.x = middle - 1;
    list.cell[1].pos.y = 15;
    list.cell[1].pos.rot = 180;

    setCellList( &(info->positionProbs), &list);
  }
}
