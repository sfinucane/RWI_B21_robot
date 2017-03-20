
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/localTcx.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:39 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: localTcx.c,v $
 * Revision 1.1  2002/09/14 20:45:39  rstone
 * *** empty log message ***
 *
 * Revision 1.121  2000/02/07 18:36:07  wolfram
 * Changed localTcx.c to maintain the robot position even if colliServer stops.
 * Fixed a memory leak.
 *
 * Revision 1.120  1999/11/02 18:12:34  fox
 * Version of multi localize journal paper.
 *
 * Revision 1.119  1999/10/21 17:30:44  fox
 * Fixed some bugs in real time scripts.
 *
 * Revision 1.118  1999/09/29 16:06:10  fox
 * Should work.
 *
 * Revision 1.117  1999/09/28 21:49:56  thrun
 * Multi-robot support in various modules.
 * In particular, I had to introduce a new message for MAP that
 * communicates back robot positions and correction parameters.
 * This might make it necessary to modify existing modules that use
 * MAP - I tried to do this for all I am aware of, but I might have missed some.
 *
 * Revision 1.116  1999/09/26 21:20:57  fox
 * Nothing special.
 *
 * Revision 1.115  1999/09/09 02:48:37  fox
 * Final version before germany.
 *
 * Revision 1.114  1999/09/06 16:36:03  fox
 * Many changes.
 *
 * Revision 1.113  1999/09/03 22:22:39  fox
 * Changed hadnling of real time script. This version contains both.
 *
 * Revision 1.112  1999/09/03 14:10:38  fox
 * Added timestamp to sample set.
 *
 * Revision 1.111  1999/09/03 13:43:36  fox
 * Nothing special.
 *
 * Revision 1.110  1999/09/01 00:02:57  fox
 * Getting closer. But still not close enough.
 *
 * Revision 1.109  1999/08/30 05:48:42  fox
 * Doesn't work!!
 *
 * Revision 1.108  1999/08/27 22:22:33  fox
 * Better communication for multi robot localization.
 *
 * Revision 1.107  1999/08/27 04:58:34  fox
 * Implemented simulation for multi robot localization.
 *
 * Revision 1.106  1999/06/29 21:36:22  fox
 * Changes for new script reader.
 *
 * Revision 1.105  1999/06/24 00:21:52  fox
 * Some changes for the urbies.
 *
 * Revision 1.104  1999/06/23 16:22:02  fox
 * Added robot type urban.
 *
 * Revision 1.103  1999/05/21 14:48:47  fox
 * Added SEND_REPORT keyword.
 *
 * Revision 1.102  1999/05/18 15:15:20  fox
 * Added keywords to determine whether correction parameters should be sent to map and plan or not.
 *
 * Revision 1.101  1999/04/29 13:35:21  fox
 * Further adaptation to make multi localize run.
 *
 * Revision 1.100  1999/04/28 22:15:59  fox
 * The script can now contain robot detection information. If such a detection
 * is found, then it is sent to the module MULTI_LOCALIZE (not perfect though).
 *
 * Revision 1.99  1999/04/26 18:55:39  fox
 * Communication with sampling seems to work (no more stuck situations).
 *
 * Revision 1.98  1999/04/21 22:58:01  fox
 * First attempt to get samples from multi back.
 *
 * Revision 1.97  1999/04/21 14:06:00  fox
 * Just an intermediate version.
 *
 * Revision 1.96  1999/04/19 22:40:40  fox
 * Minor changes.
 *
 * Revision 1.95  1999/04/18 19:00:10  fox
 * This is just to get the most recent version into the cmu bee tree. It shouldn't do any harm (hopefully!).
 *
 * Revision 1.94  1999/03/16 16:11:15  fox
 * Added more information to sample set message and created a new subscription
 * message for status, maps, robot position, and samples (maps and robot
 * positions not implemented yet).
 *
 * Revision 1.93  1999/03/13 17:50:21  fox
 * Updated localTcx.c
 *
 * Revision 1.92  1999/03/12 23:11:38  fox
 * Changed message of sample sets.
 *
 * Revision 1.91  1999/03/10 15:30:35  schulz
 * Added message for querying samples
 *
 * Revision 1.90  1999/03/08 16:47:43  wolfram
 * Fixed problems with map, caused by the fact that it no longer accpets
 * correction parameters from LOCALIZE.  Improved the handling of sonars
 * with Pioneers.  Currently sonars do not work properly on a B21.
 *
 * Revision 1.89  1999/03/01 17:44:31  wolfram
 * Added support for Pioneer II.  Pioneer I will be added next
 *
 * Revision 1.88  1999/02/17 19:42:24  fox
 * Enhanced gif utilities.
 *
 * Revision 1.87  1999/01/22 17:48:05  fox
 * Final version of sampling paper and denver demo.
 *
 * Revision 1.86  1999/01/11 19:47:51  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.85  1999/01/07 01:07:08  wolfram
 * Changes to integrate scan matching
 *
 * Revision 1.83  1998/12/10 17:56:07  fox
 * Fixed a bug in displayPositions.
 *
 * Revision 1.82  1998/11/23 21:19:25  fox
 * Fixed some minor bugs.
 *
 * Revision 1.81  1998/11/23 19:45:08  fox
 * Latest version.
 *
 * Revision 1.80  1998/11/19 03:14:27  fox
 * Newest version with integrated mapping and localization. If MAP is connected,
 * then the correction parameters are NOT sent to PLAN.
 * Furthermore, the size of the onlinemap adapts to the partial maps sent by MAP.
 *
 * Revision 1.79  1998/11/17 23:26:22  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.78  1998/11/03 21:02:19  fox
 * Lasers will not be integrated if the robot rotates faster than
 * MAX_ROT_VEL_FOR_LASERS deg/sec.
 *
 * Revision 1.77  1998/10/29 03:45:02  fox
 * Nothing special.
 *
 * Revision 1.76  1998/08/31 22:29:20  wolfram
 * Several changes
 *
 * Revision 1.75  1998/08/26 15:34:04  wolfram
 * Finished integration of vision
 *
 * Revision 1.74  1998/08/24 07:39:49  wolfram
 * final version for Washington
 *
 * Revision 1.73  1998/08/23 00:01:01  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.72  1998/08/20 00:22:59  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.71  1998/08/19 16:33:55  fox
 * Minor changes. Position is dumped upon exit of tcxServer.
 *
 * Revision 1.70  1998/08/11 23:05:38  wolfram
 * Changes for the Smithsonian museum.
 *
 * WARNING: LOCALIZE now uses only one stddev for Lasers. Laser beams have 25 m
 * Maxrange now.
 * Please use ~wolfram/bee/data/localize/params/laser-0.1stdev with this version
 * of LOCALIZE
 *
 * Revision 1.69  1998/07/01 10:45:58  fox
 * Final update of question sensor.
 *
 * Revision 1.68  1998/06/30 13:55:05  fox
 * Updated question sensor.
 *
 * Revision 1.67  1998/06/12 10:16:32  fox
 * Implemented virutal sensor.
 *
 * Revision 1.66  1998/04/19 10:40:36  wolfram
 * Added graphical display to multi localization
 *
 * Revision 1.65  1998/04/08 21:59:10  wolfram
 * Completed multi-robot support. Added neat stuff for nick.
 *
 * Revision 1.64  1998/04/08 16:36:01  wolfram
 * Changed the concept of multi-robot support. TCX now contains a procedure to
 * generate module names.
 *
 * Revision 1.63  1998/04/07 18:31:50  wolfram
 * Removed Multi calls from tcx. Version runs. Major rework has to be
 * done in order to support multiple robots.
 *
 * Revision 1.62  1998/04/06 19:44:12  wolfram
 * Added -robot option for multi-robot support
 *
 * Revision 1.61  1998/03/04 14:46:44  fox
 * This version should run.
 *
 * Revision 1.60  1998/02/12 15:47:20  derr
 * compiler warnings (unused variable ... ) removed
 *
 * Revision 1.59  1998/01/25 00:58:41  thrun
 * minor cange.
 *
 * Revision 1.58  1998/01/15 00:47:18  thrun
 * New: correction parameters are now sent to MAP, if MAP exists.
 * MAP broadcasts those then on to other modules.
 *
 * Revision 1.57  1997/11/07 12:39:40  fox
 * Added some graphic features.
 *
 * Revision 1.56  1997/10/31 13:11:42  fox
 * Version for active sensing.
 *
 * Revision 1.55  1997/08/16 22:59:50  fox
 * Last version before I change selsection.
 *
 * Revision 1.54  1997/08/16 18:42:09  thrun
 * changed a map message format
 *
 * Revision 1.53  1997/08/02 16:51:04  wolfram
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
 * Revision 1.52  1997/07/04 17:29:14  fox
 * Final version before holiday!!!
 *
 * Revision 1.51  1997/06/27 16:26:27  fox
 * New model of the proximity sensors.
 *
 * Revision 1.50  1997/06/25 14:43:31  fox
 * Minor change.
 *
 * Revision 1.49  1997/06/25 14:16:39  fox
 * Changed laser incorporation.
 *
 * Revision 1.48  1997/06/20 07:36:10  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.47  1997/06/03 11:49:22  fox
 * Museum version.
 *
 * Revision 1.46  1997/05/27 07:42:33  fox
 * Nothing special.
 *
 * Revision 1.45  1997/05/26 10:31:58  fox
 * Added dump of rear laser and sonar data.
 *
 * Revision 1.44  1997/05/26 09:34:14  fox
 * Replaced entropy by information.
 *
 * Revision 1.43  1997/05/09 16:28:17  fox
 * Nothing special.
 *
 * Revision 1.42  1997/04/30 12:25:41  fox
 * Some minor changes.
 *
 * Revision 1.41  1997/04/27 15:48:21  wolfram
 * Changes in script.c
 *
 * Revision 1.40  1997/04/27 10:40:36  fox
 * Changed status report.
 *
 * Revision 1.39  1997/04/03 13:17:50  fox
 * Some minor changes.
 *
 * Revision 1.38  1997/04/02 08:57:33  fox
 * Display the robot right after sensor selection.
 *
 * Revision 1.37  1997/03/26 09:42:57  fox
 * Updated correction parameter message.
 *
 * Revision 1.36  1997/03/19 17:52:42  fox
 * New laser parameters.
 *
 * Revision 1.35  1997/03/14 17:58:20  fox
 * This version should run quite stable now.
 *
 * Revision 1.34  1997/03/13 17:36:21  fox
 * Temporary version. Don't use!
 *
 * Revision 1.33  1997/02/28 12:57:30  fox
 * Minor changes.
 *
 * Revision 1.32  1997/02/27 16:02:35  fox
 * Added command line argument to SetRobot.
 *
 * Revision 1.31  1997/02/22 05:16:41  thrun
 * Version numbers are now also checked for libraries.
 *
 * Revision 1.30  1997/02/22 00:59:05  thrun
 * Introduced version number support
 *
 * Revision 1.29  1997/02/12 15:08:37  fox
 * Integrated laser support.
 *
 * Revision 1.28  1997/02/11 10:11:17  fox
 * Nothing special.
 *
 * Revision 1.27  1997/02/11 10:09:32  fox
 * No comment.
 *
 * Revision 1.26  1997/02/06 09:05:30  fox
 * Added parameter to send map.
 *
 * Revision 1.25  1997/02/05 14:43:02  wolfram
 * Completed laser update
 *
 * Revision 1.24  1997/01/31 17:11:02  fox
 * Integrated laser reply.
 *
 * Revision 1.23  1997/01/31 16:19:17  wolfram
 * Added start position to each reading of a scan
 *
 * Revision 1.22  1997/01/30 17:17:25  fox
 * New version with integrated laser.
 *
 * Revision 1.21  1997/01/30 13:34:12  fox
 * Minor changes.
 *
 * Revision 1.20  1997/01/29 12:23:09  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.19  1997/01/20 13:09:38  fox
 * Unbelievable, that this positino estimation ever worked ..........
 *
 * Revision 1.18  1997/01/19 19:31:17  fox
 * yeah
 *
 * Revision 1.17  1997/01/18 18:19:22  wolfram
 * *** empty log message ***
 *
 * Revision 1.16  1997/01/14 16:53:23  fox
 * Removed a very very nasty bug for sensor selection.
 *
 * Revision 1.15  1997/01/08 15:52:56  fox
 * Rotations are not any longer treated like movements.
 * Send parameters to PLAN.
 *
 * Revision 1.14  1997/01/07 09:38:06  fox
 * Changed the loading of the ini file such that all parameters have to be
 * given only if the corresponding feature has to be used.
 *
 * Revision 1.13  1997/01/06 17:38:40  fox
 * Improved version.
 *
 * Revision 1.12  1997/01/03 18:07:47  fox
 * Successfully localized the robot and moved it into several rooms without
 * loosing the position again.
 *
 * Revision 1.11  1997/01/03 10:09:46  fox
 * First version with exploration.
 *
 * Revision 1.10  1996/12/31 09:19:23  fox
 * Last version before I switched exploration mode in PLAN to random
 * mode in COLLI.
 *
 * Revision 1.9  1996/12/20 15:29:39  fox
 * Added four parameters.
 *
 * Revision 1.8  1996/12/13 13:55:37  fox
 * Implemented connection to PLAN.
 *
 * Revision 1.7  1996/12/09 10:12:00  wolfram
 * Fixed a bug in angle computation
 *
 * Revision 1.6  1996/12/04 10:25:22  wolfram
 * Removed two warnings
 *
 * Revision 1.5  1996/12/03 05:35:27  thrun
 * If TCXHOST is not se, the software will now assume "localhost" and
 * won't terminate.
 *
 * Revision 1.4  1996/12/02 10:32:08  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.3  1996/11/18 09:58:30  ws
 * Added support for selecting the optimal sensor
 * Changed some variables to improve compatibility with C++
 *
 * Revision 1.2  1996/10/24 12:07:11  fox
 * Fixed a bug.
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



#include <sys/time.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

#include "tcx.h"
#include "tcxP.h"

#define TCX_define_variables /* this makes sure variables are installed */

#include "LOCALIZE-messages.h"
#include "ROBOT_DETECTION-messages.h"

#define DEFINE_REPLY_HANDLERS /* this makes sure we'll get the handler array */

#include "BASE-messages.h"
#include "SONAR-messages.h"
#include "LASER-messages.h"
#include "MAP-messages.h"
#include "PLAN-messages.h"
#include "BUTTONS-messages.h"
#include "SOUND-messages.h"
#include "CAMERA-messages.h"

#include "general.h"
#include "script.h"
#include "function.h"
#include "movement.h"
#include "correction.h"
#include "localTcx.h"
#include "file.h"
#include "planTcx.h"
#include "colliTcx.h"
#include "proximityTools.h"
#include "sonar.h"
#include "laser.h"
#include "vision.h"
#include "condensation.h"
#include "allocate.h"
#include "question.h"
#include "beeSoftVersion.h"
#include "communication.h"
#include "graphic.h"


#define TCX_USER_MODULE_NAME "LOCALIZE"

extern informationsFor_COMMUNICATION communicationInfo;
extern sonarParameters globalSonarParameters;
char moduleName[NUMBER_OF_TCX_MODULES][LENGTH_OF_MODULE_NAMES];

static TCX_MODULE_PTR MULTI_LOCALIZE = NULL;
 
static void
sendSamplesReplyType( int numSamples,
		      struct timeval t,
		      TCX_MODULE_PTR module,
		      char * name);

/* initialize module names */
void
initializeModuleNames(char *robotName){
  tcxSetModuleName(TCX_USER_MODULE_NAME, robotName,
		   moduleName[USER_MODULE]);
  tcxSetModuleName(TCX_BASE_MODULE_NAME, robotName, moduleName[BASE_MODULE]);
  tcxSetModuleName(TCX_PLAN_MODULE_NAME, robotName, moduleName[PLAN_MODULE]);
  tcxSetModuleName(TCX_MAP_MODULE_NAME, robotName, moduleName[MAP_MODULE]);
  tcxSetModuleName(TCX_BUTTONS_MODULE_NAME, robotName, moduleName[BUTTONS_MODULE]);
  tcxSetModuleName(TCX_SOUND_MODULE_NAME, robotName, moduleName[SOUND_MODULE]);
  tcxSetModuleName("METEOR1", robotName,
		   moduleName[CAMERA_MODULE]);
  if (0) {
    int i;
    fprintf(stderr, "# Connecting to modules:");
    for (i = 0; i < NUMBER_OF_TCX_MODULES; i++)
      fprintf(stderr, " %s",  moduleName[i]);
    fprintf(stderr, "\n");
  }
}


/* #define LOCALIZE_debug  */

extern void
tcxRegisterCloseHnd(void (*closeHnd)());

struct {
  bool newSensing;
  int positionCount;
  realPosition currentPos, newPos;
  bool newPosition;
  float sonar[MAX_NUMBER_OF_SONARS];
  bool newSonar;
  float frontLaser[MAX_SIZE_OF_LASER_SCAN];
  int sizeOfFrontLaserScan;
  bool newFrontLaser;
  float rearLaser[MAX_SIZE_OF_LASER_SCAN];
  int sizeOfRearLaserScan;
  bool newRearLaser;
  realPosition laserPos;
  bool newButton;
  int colorOfButton;
  bool newCamera;
  pixel pix[IMAGE_SIZE_X][IMAGE_SIZE_Y];
} baseStatus;

bool tcx_initialized = FALSE;

cameraImage localImage;

extern int multiLocalize;

int block_wait(struct timeval *timeout, int tcx_initialized,
	       int X_initialized);


/**********************************************************************
 **********************************************************************
 *               Functions for data transfer via TCX                  *
 **********************************************************************
 **********************************************************************/

typedef struct {
  probabilityGrid*         originalMap;
  probabilityGrid*         initialPositionProbs;
  positionProbabilityGrid* grid;
  sampleSet*               samples;
  sampleSet*               tcxSamples;
  sampleSetHistory*        samplesHistory;
  correctionParameter*     correctionParam;
  realCellList*            localMaxima;
  realPosition*            basePosition;
  realPosition*            mapPosition;
  int*                     useProbGrid;
  int                      subscribeReport;
  int                      subscribeProximityReport;
  int                      subscribeBaseReport;
  int                      subscribeCameraReport;
  float*                   transVelocity;
  float*                   rotVelocity;

  /* These maps to be updated whenever a new map arrives. */
  int                      onlineMapping;
  probabilityGrid*         onlineMap;
  probabilityGrid*         planMap;
  probabilityGrid*         laserMap;
  probabilityGrid*         sonarMap;
  probabilityGrid*         positionProbabilityMap;
} tcxInfoStruct;

tcxInfoStruct tcxInfo;

void
initializeTcxInfoStructure( actionInformation* info)
{
  tcxInfo.originalMap            = &(info->map);
  tcxInfo.initialPositionProbs   = &(info->initialPositionProbs);
  tcxInfo.grid                   = &(info->positionProbs);
  tcxInfo.samples                = &(info->samples);
  tcxInfo.tcxSamples             = &(info->tcxSamples);
  tcxInfo.samplesHistory         = &(info->samplesHistory);
  tcxInfo.correctionParam        = &(info->correctionParam);
  tcxInfo.localMaxima            = &(info->localMaxima);
  tcxInfo.basePosition           = &(info->actualSensings.basePosition);
  tcxInfo.transVelocity          = &(info->actualSensings.transVelocity);
  tcxInfo.rotVelocity            = &(info->actualSensings.rotVelocity);
  tcxInfo.useProbGrid            = &(info->useProbGrid);
  tcxInfo.mapPosition            = &(info->estimatedRobot.pos);
  tcxInfo.onlineMap              = &(info->onlineMap);
  tcxInfo.planMap                = &(info->planMap);
  tcxInfo.laserMap               = &(info->laserMap);
  tcxInfo.sonarMap               = &(info->sonarMap);
  tcxInfo.positionProbabilityMap = &(info->initialPositionProbs);
  tcxInfo.samplesHistory         = &(info->samplesHistory);
  tcxInfo.onlineMapping          = info->onlineMapping;
}


/*****************************************************************************
 * Checks wether the connection to the module is established. If not the function
 * tries to establish the connection.
 *****************************************************************************/
bool
connectionEstablished( TCX_MODULE_PTR* module, char* name)
{
  if (!tcx_initialized)
    return FALSE;
  if ( *module == NULL) {

    *module = tcxConnectOptional( name);
    if (*module != NULL) {
      writeLog( "Connected to %s.\n", name);
      fprintf(stderr, "Connected to %s.\n", name);
      return TRUE;
    }
    else
      return FALSE;
  }
  else
    return TRUE;
}

/* If the seconds are set, then the function swallows reports until the
 * desired time is expired. */
void
swallowStatusReports( float waitTime)
{
  if ( tcx_initialized) {
    struct timeval TCX_waiting_time = {0, 0};

    if ( waitTime == DONT_WAIT) 
      tcxRecvLoop((void *) &TCX_waiting_time);
    else {
      float expiredTime = 0.0;
      setTimer( 0);
      writeLog( "%f #SLEEP ...", waitTime);
      while ( expiredTime < waitTime) {

	struct timeval t;
	float blockWaitTime = 0.5 * expiredTime;
	t.tv_sec = floor(blockWaitTime);
	t.tv_usec = (blockWaitTime - t.tv_sec) * 1000000;
	block_wait( &t, 1, 0);

	tcxRecvLoop((void *) &TCX_waiting_time);

	expiredTime = timeExpired(0);
      }
      writeLog( "awake.\n");
    }
  }
}




/**********************************************************************
 **********************************************************************
 ******* auto-reply stuff, data definitions ***************
 **********************************************************************
 **********************************************************************/



#define MAX_N_AUTO_UPDATE_MODULES 100


int n_auto_update_modules = 0;
int n_auto_status_modules = 0;
int n_auto_samples_modules = 0;
int n_auto_map_modules = 0;
int n_auto_robotPosition_modules = 0;

#define NOT_ADDED       0
#define RE_SUBSCRIPTION 1
#define NEW_MODULE_SUBSCRIPTION 2

typedef struct{
  TCX_MODULE_PTR module;	/* pointer to TCX module */
  int        subscribeStatus;
  int        subscribeRobotPosition;
  int        subscribeMap;
  int        subscribeSamples;
  int        numberOfSamples;
} auto_update_type;


auto_update_type auto_update_modules[MAX_N_AUTO_UPDATE_MODULES];
                                /* collection of TCX-module ptrs */



/************************************************************************
 *
 *   NAME:         count_auto_update_modules()
 *
 *   FUNCTION:     Counts, how many modules of the different types require
 *                 auto-updates
 *
 *   RETURN-VALUE: 1, if successful, 0 if not
 *
 ************************************************************************/

void
count_auto_update_modules()
{
  int i;
  n_auto_status_modules = 0;
  n_auto_samples_modules = 0;
  n_auto_map_modules = 0;
  n_auto_robotPosition_modules = 0;

  for (i = 0; i < n_auto_update_modules; i++){
    if (auto_update_modules[i].subscribeStatus) n_auto_status_modules++;
    if (auto_update_modules[i].subscribeSamples) n_auto_samples_modules++;
    if (auto_update_modules[i].subscribeMap) n_auto_map_modules++;
    if (auto_update_modules[i].subscribeRobotPosition) n_auto_robotPosition_modules++;
  }
}


/************************************************************************
 *
 *   NAME:         add_auto_update_module()
 *
 *   FUNCTION:     Adds a module to the list of modules that
 *                 get automatical map updates
 *
 *   PARAMETERS:   TCX_MODULE_PTR module       module pointer
 *                 int map                     1, if subscribe to map
 *                 int correction              1, if subscribe to correction
 *
 *
 *   RETURN-VALUE: 1, if successful, 0 if not
 *
 ************************************************************************/

static int
add_auto_update_module( TCX_MODULE_PTR module,
			int subscribeStatus,
			int subscribeRobotPosition,
			int subscribeMap,
			int subscribeSamples,
			int numberOfSamples)
{
  int i;

  /* Check for not yet implemented functionalities. */
  if ( subscribeMap > 0) {
    fprintf( stderr, "Sorry. Map update not yet implemented.\n");
    writeLog( "Sorry. Map update not yet implemented.\n");
  }
  if (numberOfSamples > 0) {
    fprintf( stderr, "Sorry. Variable sample set not yet implemented.\n");
    writeLog( "Sorry. Variable sample set not yet implemented.\n");
  }
  if (subscribeRobotPosition > 0) {
    fprintf( stderr, "Sorry. Robot position update not yet implemented.\n");
    writeLog( "Sorry. Robot position update not yet implemented.\n");
  }
  
  if (n_auto_update_modules >= MAX_N_AUTO_UPDATE_MODULES){
    fprintf(stderr, "ERROR: auto-module memory overflow. Request rejected.\n");
    return NOT_ADDED;
  }
  else
    for (i = 0; i < n_auto_update_modules; i++)
      if (auto_update_modules[i].module == module){
	writeLog( "Module %s already known. Subscription modified: %d %d %d %d\n",
		  tcxModuleName(module), subscribeStatus, subscribeRobotPosition,
		 subscribeMap, subscribeSamples);
	if ( subscribeStatus >= 0)
	  auto_update_modules[i].subscribeStatus        = subscribeStatus; /* subsrc? */
	if ( subscribeRobotPosition >= 0)
	  auto_update_modules[i].subscribeRobotPosition = subscribeRobotPosition;
	if ( subscribeMap >= 0)
	  auto_update_modules[i].subscribeMap           = subscribeMap;      /* subsrc? */
	if ( subscribeSamples >= 0)
	  auto_update_modules[i].subscribeSamples       = subscribeSamples;  /* subsrc? */
	if ( numberOfSamples >= 0)
	  auto_update_modules[i].numberOfSamples        = numberOfSamples;   /* subsrc? */
	return RE_SUBSCRIPTION;
      }

  fprintf( stderr, "Add %s to auto-reply list: %d %d %d %d\n",
	   tcxModuleName(module), subscribeStatus, subscribeRobotPosition,
	   subscribeMap, subscribeSamples);

  writeLog( "Add %s to auto-reply list: %d %d %d %d\n",
	    tcxModuleName(module), subscribeStatus, subscribeRobotPosition,
	    subscribeMap, subscribeSamples);
  
  auto_update_modules[n_auto_update_modules].module           = module;         /* pointer*/
  auto_update_modules[n_auto_update_modules].subscribeStatus  = subscribeStatus; /* subsrc? */
  auto_update_modules[n_auto_update_modules].subscribeRobotPosition = subscribeRobotPosition;
  auto_update_modules[n_auto_update_modules].subscribeMap     = subscribeMap;
  auto_update_modules[n_auto_update_modules].subscribeSamples = subscribeSamples;
  auto_update_modules[n_auto_update_modules].numberOfSamples  = numberOfSamples;

  n_auto_update_modules++;
  count_auto_update_modules();

  return NEW_MODULE_SUBSCRIPTION;
}



/************************************************************************
 *
 *   NAME:         remove_auto_update_module()
 *
 *   FUNCTION:     Attempts to remove a module from the list of
 *                 modules that get automatical map updates
 *
 *   PARAMETERS:   TCX_MODULE_PTR module       module pointer
 *
 *
 *   RETURN-VALUE: 1, if successful, 0 if not
 *
 ************************************************************************/

static int
remove_auto_update_module(TCX_MODULE_PTR module)
{
  int i, j, found = 0;;
  for (i = 0; i < n_auto_update_modules; i++) /* search for module */
    if (auto_update_modules[i].module == module){ /* if module found */

      fprintf(stderr, "Remove %s from auto-reply list.\n", tcxModuleName(module));
      writeLog( "Remove %s from auto-reply list.\n", tcxModuleName(module));
      found++;

      n_auto_update_modules--;	/* remove that entry, one less now */

      for (j = i; j < n_auto_update_modules; j++){
	auto_update_modules[j].module          =  auto_update_modules[j+1].module;
	auto_update_modules[j].subscribeStatus =  auto_update_modules[j+1].subscribeStatus;
	auto_update_modules[j].subscribeRobotPosition =  auto_update_modules[j+1].subscribeRobotPosition;
	auto_update_modules[j].subscribeMap     =  auto_update_modules[j+1].subscribeMap;
	auto_update_modules[j].subscribeSamples =  auto_update_modules[j+1].subscribeSamples;
	auto_update_modules[j].numberOfSamples  =  auto_update_modules[j+1].numberOfSamples;
      }
    }
  if ( ! found) {
    fprintf(stderr, "(%s: no auto-replies removed)\n",
	    tcxModuleName(module));
    writeLog( "(%s: no auto-replies removed)\n",
	      tcxModuleName(module));
  }

  count_auto_update_modules();

  return found;
}


/************************************************************************
 *
 *   NAME: LOCALIZE_register_auto_update_handler()
 *
 *   FUNCTION:     handles a message of that type
 *
 *   PARAMETERS:   standard TCX handler parameters
 *
 *   RETURN-VALUE:
 *
 ************************************************************************/
void
LOCALIZE_register_auto_update_handler( TCX_REF_PTR  ref,
				       LOCALIZE_register_auto_update_ptr data)
{
  LOCALIZE_update_status_reply_type status;

  fprintf( stderr, "Received a  LOCALIZE_register_auto_update_message from %s.\n",
	    tcxModuleName(ref->module));
  writeLog( "Received a  LOCALIZE_register_auto_update_message from %s.\n",
	    tcxModuleName(ref->module));

  if (add_auto_update_module( ref->module,
			      data->subscribe,
			      0, 0, 0, 0)
      != NEW_MODULE_SUBSCRIPTION)
    return;

  /* Send initial status update to the new module. */
  status.corrX    = tcxInfo.correctionParam->x;
  status.corrY    = tcxInfo.correctionParam->y;
  status.corrRot  = tcxInfo.correctionParam->rot;
  status.corrType = tcxInfo.correctionParam->type;

  status.robotX   = tcxInfo.mapPosition->x;
  status.robotY   = tcxInfo.mapPosition->y;
  status.robotRot = tcxInfo.mapPosition->rot;
  
  status.numberOfLocalMaxima = tcxInfo.localMaxima->numberOfCells;
  if ( status.numberOfLocalMaxima > 0 )
    status.probOfGlobalMaximum = tcxInfo.localMaxima->cell[0].prob;
  else
    status.probOfGlobalMaximum = 0.0;

  tcxSendMsg( ref->module, "LOCALIZE_update_status_reply", &status);

  if (data != NULL){
    tcxFree("LOCALIZE_register_auto_update", data);
    data = NULL;
  }
}

void
LOCALIZE_register_update_handler( TCX_REF_PTR  ref,
				  LOCALIZE_register_update_ptr data)
{
  LOCALIZE_update_status_reply_type status;

  fprintf( stderr, "Received a  LOCALIZE_register_update_message from %s.\n",
	    tcxModuleName(ref->module));
  writeLog( "Received a  LOCALIZE_register_update_message from %s.\n",
	    tcxModuleName(ref->module));

  if (  add_auto_update_module( ref->module,
			  data->subscribeStatus,
			  data->subscribeRobotPosition,
			  data->subscribeMap,
			  data->subscribeSamples,
			  data->numberOfSamples)
	!= NEW_MODULE_SUBSCRIPTION)
    return;

  
  /* Send initial status update to the new module. */
  if ( data->subscribeStatus) {
    fprintf(stderr, "Send initial status.\n");
    
    status.corrX    = tcxInfo.correctionParam->x;
    status.corrY    = tcxInfo.correctionParam->y;
    status.corrRot  = tcxInfo.correctionParam->rot;
    status.corrType = tcxInfo.correctionParam->type;
    
    status.robotX   = tcxInfo.mapPosition->x;
    status.robotY   = tcxInfo.mapPosition->y;
    status.robotRot = tcxInfo.mapPosition->rot;
    
    status.numberOfLocalMaxima = tcxInfo.localMaxima->numberOfCells;
    if ( status.numberOfLocalMaxima > 0 )
      status.probOfGlobalMaximum = tcxInfo.localMaxima->cell[0].prob;
    else
      status.probOfGlobalMaximum = 0.0;
    
    tcxSendMsg( ref->module, "LOCALIZE_update_status_reply", &status);
  }

  if ( data->subscribeSamples) {
    struct timeval t = {0,0};
    fprintf(stderr, "Send initial samples.\n");
    sendSamplesReplyType( iMin( tcxInfo.tcxSamples->numberOfSamples,
				MAX_NUMBER_OF_TCX_SAMPLES),
			  t,
			  ref->module,
			  tcxModuleName(ref->module));
  }
  
  if (data != NULL){
    tcxFree("LOCALIZE_register_auto_update", data);
    data = NULL;
  }
}


/************************************************************************
 *
 *   Name:         LOCALIZE_close_handler
 *
 *   FUNCTION:     handles a close message (special case)
 *
 *   PARAMETERS:   standard TCX handler parameters
 *
 *   RETURN-VALUE:
 *
 ************************************************************************/

void
LOCALIZE_close_handler(char *name, TCX_MODULE_PTR module)
{

  writeLog( "LOCALIZE: closed connection detected: %s\n", name);

  remove_auto_update_module(module);

  if (!strcmp(name, SERVER_NAME)) { /* TCX shut down */
    writeCurrentPosition( tcxInfo.localMaxima);
    exit(0);
  }
  else if ( module == PLAN) {
    fprintf( stderr, "PLAN disconnected.\n");
    writeLog( "PLAN disconnected.\n");
    PLAN = NULL;
  }
  else if ( module == MAP) {
    fprintf( stderr, "MAP disconnected.\n");
    writeLog( "MAP disconnected.\n");
    MAP = NULL;
  }
  else if ( module == BASE) {
    fprintf( stderr, "BASE disconnected.\n");
    writeLog( "BASE disconnected.\n");
    BASE = NULL;
  }
  else if ( module == MULTI_LOCALIZE) {
    fprintf( stderr, "MULTI_LOCALIZE disconnected.\n");
    writeLog( "MULTI_LOCALIZE disconnected.\n");
    MULTI_LOCALIZE = NULL;
  }
}


/**********************************************************************
 **********************************************************************
 *
 *  LOCALIZE handlers
 *
 **********************************************************************
 **********************************************************************/




/************************************************************************
 *
 *   NAME:         send_automatic_localize_update
 *
 *   FUNCTION:     sends to all modules on the list an automatic
 *                 update.
 *
 *   PARAMETERS:   none
 *
 *   RETURN-VALUE: none
 *
 ************************************************************************/


static int auto_update_counter = 0;

void
broadcastStatusReport( sensingActionMask* mask,
		       bool considerPlan)
{
  static realPosition lastPositionSent = {0,0,0};

  if ( tcx_initialized) {

    /* If no mask is given the status is broadcasted in any case.
     * Otherwise only send it if the mask is set accordingly. */
    bool performBroadcast = (mask == NULL);
    
    if ( ! performBroadcast) {
      performBroadcast = ( mask->consider[MOVEMENT] && mask->perform[MOVEMENT][INTEGRATE_MOVEMENT]);

      /* Check, whether the position of the robot has changed. If so we need to send a new
       * update for robot position. */
      if ( ! performBroadcast && n_auto_update_modules > 0) {

#define MAX_DISTANCE_WITHOUT_BROADCAST 30
#define MAX_ROTATION_WITHOUT_BROADCAST (deg2Rad(5.0))	
	
	if  ( realPositionDistance( lastPositionSent, *tcxInfo.mapPosition)
	      > MAX_DISTANCE_WITHOUT_BROADCAST ||
	      angleDistance( lastPositionSent.rot, tcxInfo.mapPosition->rot)
	      > MAX_ROTATION_WITHOUT_BROADCAST) {

	  performBroadcast = TRUE;
	
	  fprintf(stderr, "Position changed too much. Broadcast %f %f %f\n",
		  tcxInfo.mapPosition->x,
		  tcxInfo.mapPosition->y,
		  tcxInfo.mapPosition->rot);
	  
	  lastPositionSent = *tcxInfo.mapPosition;
	}
      }
      
      /* If we use probability grids then only broadcast if the grid has been normalized. */
      if ( *tcxInfo.useProbGrid)
	performBroadcast = performBroadcast && mask->normalizeGrid;
    }
    
    if ( PLAN != NULL || (performBroadcast && (n_auto_update_modules > 0))) {

      /* There is at least one module that subscribed for the status report. */
      if ( n_auto_status_modules > 0) {
	
	int i;
	LOCALIZE_update_status_reply_type status;
	
	status.corrX    = tcxInfo.correctionParam->x;
	status.corrY    = tcxInfo.correctionParam->y;
	status.corrRot  = tcxInfo.correctionParam->rot;
	status.corrType = tcxInfo.correctionParam->type;
	
	status.robotX   = tcxInfo.mapPosition->x;
	status.robotY   = tcxInfo.mapPosition->y;
	status.robotRot = tcxInfo.mapPosition->rot;

	lastPositionSent = *tcxInfo.mapPosition;
	
	if ( *tcxInfo.useProbGrid) {
	  
	  status.numberOfLocalMaxima = tcxInfo.localMaxima->numberOfCells;
	  if ( status.numberOfLocalMaxima > 0)
	    status.probOfGlobalMaximum = tcxInfo.localMaxima->cell[0].prob *
	      tcxInfo.localMaxima->originalSumOfProbs;
	  else
	    status.probOfGlobalMaximum = 0.0;
	  
	  if (tcxInfo.grid->quotaOfPlanesToBeUpdated > 0.7){
	    status.probOfGlobalMaximum = 0.0;
	  }
	}
	else {
	  status.probOfGlobalMaximum = 1.0;
	  status.numberOfLocalMaxima = 1;
	}
      
	for (i = 0; i < n_auto_update_modules; i++){
	  
	  /* Status update. */
	  if ( auto_update_modules[i].subscribeStatus > 0 &&
	       ( auto_update_counter % auto_update_modules[i].subscribeStatus == 0)) {
	    
	    writeLog( "# Send status update (%f %f %f) [%f %f %f] to %s.\n",
		      status.corrX, status.corrY, status.corrRot,
		      status.robotX, status.robotY, status.robotRot,
		      tcxModuleName(auto_update_modules[i].module));
	    
	    tcxSendMsg( auto_update_modules[i].module, "LOCALIZE_update_status_reply",
			&status);
	  }
	}
      }
      
      /* At least one module subscribed for samples. */
      if ( n_auto_samples_modules > 0) {

	/* Copy the samples into the tcx struct. */
	int i, s;
	LOCALIZE_samples_reply_type samplesReply;
	sampleType* replySamples;
	
	/* Send all samples. */
	int numberOfSamples = iMin( tcxInfo.tcxSamples->numberOfSamples, MAX_NUMBER_OF_TCX_SAMPLES);

	samplesReply.numberOfSet = tcxInfo.tcxSamples->numberOfSet;
      
	/* Allocate memory. */
	replySamples = (sampleType*) malloc ( numberOfSamples * sizeof(sampleType));	

	for( s = 0; s < numberOfSamples; s++) {
	  replySamples[s].pos    = tcxInfo.tcxSamples->sample[s].pos;
	  replySamples[s].weight = tcxInfo.tcxSamples->sample[s].weight;
	}

	samplesReply.boundingBoxMin.x   = tcxInfo.tcxSamples->minX;
	samplesReply.boundingBoxMin.y   = tcxInfo.tcxSamples->minY;
	samplesReply.boundingBoxMin.rot = -DEG_360;
	samplesReply.boundingBoxMax.x   = tcxInfo.tcxSamples->maxX;
	samplesReply.boundingBoxMax.y   = tcxInfo.tcxSamples->maxY;
	samplesReply.boundingBoxMax.rot = +DEG_360;
	
	samplesReply.numberOfSamples = numberOfSamples;
	samplesReply.replySamples = replySamples; 

	for (i = 0; i < n_auto_update_modules; i++){
	  
	  if ( auto_update_modules[i].subscribeSamples > 0 &&
	       ( auto_update_counter % auto_update_modules[i].subscribeSamples == 0)) {
	    
	    writeLog( "# %s: Sending %d samples [%f:%f] [%f:%f] to %s.\n"
		      , moduleName[USER_MODULE], numberOfSamples,
		      samplesReply.boundingBoxMin.x, samplesReply.boundingBoxMax.x,
		      samplesReply.boundingBoxMin.y, samplesReply.boundingBoxMax.y,
		      tcxModuleName(auto_update_modules[i].module));
	    fprintf(stderr, "#Sending %d samples of set no. %d to %s ...",
		    numberOfSamples, samplesReply.numberOfSet,
		      tcxModuleName(auto_update_modules[i].module));

	    swallowStatusReports(DONT_WAIT);
	    
	    if ( auto_update_modules[i].subscribeSamples > 0)
	      tcxSendMsg( auto_update_modules[i].module,
			  "LOCALIZE_samples_reply", &samplesReply);
	    else
	      writeLog( "# %s: Skip sample set.\n", moduleName[USER_MODULE]);
	    
	    fprintf( stderr, "done.\n");
	  }
	  else {
	    fprintf( stderr, "Skip sample set.\n");
	    writeLog( "# %s: Skip sample set.\n", moduleName[USER_MODULE]);
	  }
	}
	free(replySamples);
      }
      
      /* Send the correction parameters either to PLAN or MAP. */
      if ( considerPlan) {
	if ( tcxInfo.localMaxima->numberOfCells > 0 || ! *tcxInfo.useProbGrid) {
	  if ( MAP == NULL || communicationInfo.sendCorrectionParametersToPlan) {
	    sendCorrectionParams( *(tcxInfo.correctionParam),
				  PLAN, moduleName[PLAN_MODULE]);
	  }
	  else
	    if (communicationInfo.sendCorrectionParametersToMap)
	      sendCorrectionParams( *(tcxInfo.correctionParam),
				    MAP, moduleName[MAP_MODULE]);
	}
      }
      auto_update_counter++;
    }
  }
}

#define SEND_STATUS_REPORTS_FROM_SCRIPT 

/*****************************************************************************
 * Send the robot position to the planning module.
 *****************************************************************************/
void
sendBasePositionToUpdateModules( realPosition robPos)
{
  if ( ! communicationInfo.sendReports)
    return;

  /* First send the position to plan. */
  if ( connectionEstablished( &PLAN, moduleName[PLAN_MODULE])) {

    PLAN_new_robot_pos_message_type robPosMessage;

    robPosMessage.x = robPos.x;
    robPosMessage.y = robPos.y;
    robPosMessage.orientation = robPos.rot;

    writeLog( "TCX message to PLAN: base position (%f %f %f).\n",
	     robPosMessage.x, robPosMessage.y, robPosMessage.orientation);

    tcxSendMsg ( PLAN, "PLAN_new_robot_pos_message", &robPosMessage );
    return;
  }
  
  /* Now send the position to all update modules. */
  if ( n_auto_update_modules > 0) {
    BASE_update_status_reply_type baseStatus;
    int i;
    baseStatus.pos_x = robPos.x;
    baseStatus.pos_y = robPos.y;
    baseStatus.orientation = robPos.rot;

    for (i = 0; i < n_auto_update_modules; i++){
    if ( auto_update_modules[i].subscribeStatus > 0 &&
	   ( auto_update_counter % auto_update_modules[i].subscribeStatus == 0)) {

	writeLog( "# Send base position to %s.\n",
		  tcxModuleName(auto_update_modules[i].module));

	tcxSendMsg( auto_update_modules[i].module, "BASE_update_status_reply",
		    &baseStatus);
      }
    }
  }
}

/*****************************************************************************
 * Send the robot position to the planning module.
 *****************************************************************************/
void
sendLaserToUpdateModules( sensing_PROXIMITY frontLaser,
			  sensing_PROXIMITY rearLaser)
{
  if ( ! communicationInfo.sendReports)
    return;

  /* Send the laser to all update modules. */
  if ( n_auto_update_modules > 0) {
    static LASER_laser_reply_type laserStatus;
    static int firstTime = TRUE;
    int i;

    if ( firstTime) {
      firstTime = FALSE;
      laserStatus.f_numberOfReadings = frontLaser.numberOfReadings;
      laserStatus.r_numberOfReadings = rearLaser.numberOfReadings;
      laserStatus.f_reading = (int*) allocate1D( frontLaser.numberOfReadings, INT);
      laserStatus.r_reading = (int*) allocate1D( rearLaser.numberOfReadings, INT);
    }
    else if ( frontLaser.numberOfReadings != laserStatus.f_numberOfReadings ||
	      rearLaser.numberOfReadings != laserStatus.r_numberOfReadings) {
      putc( 7, stderr);
      fprintf( stderr, "Error! Number of readings changed.\n");
      return;
    }
    
    laserStatus.xPos   = tcxInfo.basePosition->x;
    laserStatus.yPos   = tcxInfo.basePosition->y;
    laserStatus.rotPos = deg2Rad(90.0 - tcxInfo.basePosition->rot);
	    
    for ( i = 0; i < frontLaser.numberOfReadings; i++)
      laserStatus.f_reading[i] = (int) frontLaser.reading[i].dist;
    for ( i = 0; i < rearLaser.numberOfReadings; i++)
      laserStatus.r_reading[i] = (int) rearLaser.reading[i].dist;

    for (i = 0; i < n_auto_update_modules; i++){
      if ( auto_update_modules[i].subscribeStatus > 0 &&
	   ( auto_update_counter % auto_update_modules[i].subscribeStatus == 0)) {

	writeLog( "# Send laser update to %s.\n",
		  tcxModuleName(auto_update_modules[i].module));

	tcxSendMsg( auto_update_modules[i].module, "LASER_laser_reply",
		    &laserStatus);
      }
    }
  }
}

/*****************************************************************************
 * Stops localization.
 *****************************************************************************/
void
LOCALIZE_quit_handler( TCX_REF_PTR  ref, void* data)
{
  writeLog(  "Received quit message from %s.\n",
	     tcxModuleName( ref->module));

  closeLogAndExit(0);
  data=data;
}

/*****************************************************************************
 * Starts active localization.
 *****************************************************************************/
void
LOCALIZE_start_active_localization_handler( TCX_REF_PTR  ref,
					    void* data)
{
  fprintf( stderr, "Received start active message from %s.\n",
	     tcxModuleName( ref->module));
  writeLog(  "Received start active message from %s.\n",
	     tcxModuleName( ref->module));

  if (0)  {
    resetGrid( tcxInfo.grid, tcxInfo.initialPositionProbs);
    tcxInfo.localMaxima->numberOfCells = 0;
  }

  startActiveLocalization();
  data=data;
}

/*****************************************************************************
 * Stops active localization.
 *****************************************************************************/
void
LOCALIZE_stop_active_localization_handler( TCX_REF_PTR  ref,
					   void* data)
{
  fprintf( stderr, "Received stop active message from %s.\n",
	   tcxModuleName( ref->module));
  
  writeLog(  "Received stop active message from %s.\n",
	     tcxModuleName( ref->module));
  stopActiveLocalization();
  data=data;
}

/*****************************************************************************
 * sets the suggested robot position in the grid.
 *****************************************************************************/
void
LOCALIZE_set_robot_position_handler( TCX_REF_PTR  ref,
				     LOCALIZE_set_robot_position_ptr position)
{
  realPosition pos;

  fprintf( stderr, "#Received robot position (%f %f %f) from %s.\n",
	   position->x, position->y, position->rot,
	   tcxModuleName( ref->module));

  writeLog( "Received robot position (%f %f %f) from %s.\n",
	    position->x, position->y, position->rot,
	    tcxModuleName( ref->module));

  pos.x   = position->x;
  pos.y   = position->y;
  pos.rot = position->rot;

  if ( *tcxInfo.useProbGrid) {

    setPosition( pos, tcxInfo.grid, 3.0);
    
    
    findLocalMaxima( tcxInfo.grid, tcxInfo.localMaxima,
		     cmToCells( XY_CUBE, tcxInfo.grid),
		     radToPlanes( Z_CUBE, tcxInfo.grid));
  }
  else {
    setSamplePosition( pos,
		       tcxInfo.samples,
		       15.0);
    displaySamples();
  }
  
  tcxInfo.mapPosition->x   = pos.x;
  tcxInfo.mapPosition->y   = pos.y;
  tcxInfo.mapPosition->rot = pos.rot;
  


	

  computeCorrectionParam( *(tcxInfo.basePosition),
			  tcxInfo.localMaxima->cell[0].pos,
			  tcxInfo.correctionParam);


  /* Send the new correction parameters. */
  broadcastStatusReport( NULL, CONSIDER_PLAN);
}


void
LOCALIZE_samples_query_handler( TCX_REF_PTR ref,
				LOCALIZE_samples_query_ptr query)
{
  fprintf(stderr, "# Received sample query from %s.\n", tcxModuleName(ref->module));
  writeLog( "# Received sample query from %s.\n", tcxModuleName(ref->module));
  
  sendSamplesReplyType(query->numberOfSamples,
		       query->timeStamp,
		       ref->module,
		       tcxModuleName(ref->module));
}

void
LOCALIZE_updated_samples_handler( TCX_REF_PTR ref,
				  LOCALIZE_updated_samples_ptr tcxSamples)
{
  int s;
  int numberOfSamples = tcxSamples->numberOfSamples;
  sampleSet* samples = tcxInfo.samples;

  if ( ! multiLocalize) {

    fprintf(stderr, "MULTI_LOCALIZE not set: ignore %d updated samples (no. %d) from %s.\n",
	    numberOfSamples, tcxSamples->numberOfSet, tcxModuleName(ref->module));
    
    writeLog( "# MULTI_LOCALIZE not set: ignore %d updated samples (no. %d) from %s.\n",
	      numberOfSamples, tcxSamples->numberOfSet, tcxModuleName(ref->module));
  
    tcxFree("LOCALIZE_updated_samples", tcxSamples);
    return;
  }
  
  fprintf(stderr, "Received %d updated samples (no. %d) from %s.\n",
	  numberOfSamples, tcxSamples->numberOfSet, tcxModuleName(ref->module));

  writeLog( "# Received %d updated samples (no. %d) from %s.\n",
	    numberOfSamples, tcxSamples->numberOfSet, tcxModuleName(ref->module));

  /* We assume that the internal size of sample sets doesn't change. */
  if ( samples->allocatedSamples < numberOfSamples) {
    fprintf(stderr, "Oops. Received more samples than I sent. Ignore.\n");
    writeLog( "Oops. Received more samples than I sent. Ignore.\n");
    tcxFree("LOCALIZE_updated_samples", tcxSamples);
    return;
  }
  
  if ( sampleSetUpToDate( tcxSamples->numberOfSet, tcxInfo.samplesHistory)) {

    /* Replace the old set in the history and extract the summed motion
     * from the history. */
    setSampleSetHistory( tcxSamples, samples,
			 tcxInfo.samplesHistory);
    
    samples->numberOfSamples = numberOfSamples;
    
    samples->minX = tcxSamples->boundingBoxMin.x;
    samples->minY = tcxSamples->boundingBoxMin.y;
    samples->maxX = tcxSamples->boundingBoxMax.x;
    samples->maxY = tcxSamples->boundingBoxMax.y;
    
    /* Copy the stuff over. */
    for ( s = 0; s < tcxSamples->numberOfSamples; s++) {
      samples->sample[s].pos    = tcxSamples->replySamples[s].pos;
      samples->sample[s].weight = tcxSamples->replySamples[s].weight;
    }
    
    samples->alreadySampled = FALSE;
    
    samples->replacedViaTcx = TRUE;
    
    displaySamples();
  }
    
  tcxFree("LOCALIZE_updated_samples", tcxSamples);
}

static void
sendSamplesReplyType( int numSamples,
		      struct timeval timeStamp,
		      TCX_MODULE_PTR module,
		      char * name)
{
  if ( connectionEstablished( &module, name)) {

    if ( multiLocalize) {

      LOCALIZE_samples_reply_type samplesReply;
      sampleSetHistory* history = tcxInfo.samplesHistory;
      
      int i;
      
      int historyPointer = getClosestSampleSet( history, timeStamp);
      
      /* Ok. We found a sample set. */
      if ( historyPointer >= 0) {
	
	static sampleType* replySamples = NULL;
	static int numberOfAllocatedReplySamples = 0;
	sampleType* historySamples;
	
	fprintf(stderr, "FOUND %d : %f\n", historyPointer,
		timeDiff(&(history->timeStamp[historyPointer]), &(timeStamp)));
	writeLog( "FOUND %d : %f\n", historyPointer,
		  timeDiff(&(history->timeStamp[historyPointer]), &(timeStamp)));
	
	/* If required samples is set to zero than send them all. */
	if ( numSamples <= 0)
	  numSamples = history->numberOfSamplesToBeSent[historyPointer];
	else if ( numSamples > history->numberOfSamplesToBeSent[historyPointer]) {
	  fprintf( stderr, "Warning: request for more samples (%d) then available (%d).\n",
		   numSamples, history->numberOfSamplesToBeSent[historyPointer]);
	  numSamples = history->numberOfSamplesToBeSent[historyPointer];
	}
	
	samplesReply.numberOfSet = history->currentSampleSetNumber -
	  ( history->currentSampleSetPosition - historyPointer);
	
	if ( samplesReply.numberOfSet > history->currentSampleSetNumber)
	  samplesReply.numberOfSet -= history->sizeOfHistory;
	
	fprintf(stderr, "Set number %d %d %d %d\n", history->currentSampleSetNumber,
		history->currentSampleSetPosition, historyPointer, samplesReply.numberOfSet);
	
	if ( numberOfAllocatedReplySamples < numSamples) {
	  if ( numberOfAllocatedReplySamples > 0)
	    free( replySamples);
	  
	  replySamples = (sampleType*) malloc ( numSamples * sizeof(sampleType));
	  numberOfAllocatedReplySamples = numSamples;
	}
	
	historySamples = history->samples[historyPointer];
	
	for(i = 0; i < numSamples; i++) {
	  replySamples[i].pos    = historySamples[i].pos;
	  replySamples[i].weight = historySamples[i].weight;
	}
	
	samplesReply.boundingBoxMin.x   = history->minPosition.x;
	samplesReply.boundingBoxMin.y   = history->minPosition.y;
	samplesReply.boundingBoxMin.rot = history->minPosition.y;
	samplesReply.boundingBoxMax.x   = history->maxPosition.x;
	samplesReply.boundingBoxMax.y   = history->maxPosition.y;
	samplesReply.boundingBoxMax.rot = history->maxPosition.y;
	
	samplesReply.numberOfSamples    = numSamples;
	samplesReply.replySamples       = replySamples; 
	
	samplesReply.timeStamp          = history->timeStamp[historyPointer];
	
	tcxSendMsg(module, "LOCALIZE_samples_reply", &samplesReply);
	writeLog( "Sending %d samples to %s.\n", numSamples, name);
      }
      else {
	fprintf( stderr, "NO set found.\n");
	writeLog( "NO set found.\n");
	
	samplesReply.numberOfSamples = 0;
	samplesReply.numberOfSet     = 0;
	
	tcxSendMsg(module, "LOCALIZE_samples_reply", &samplesReply);
	writeLog( "Sending ZERO samples to %s.\n", name);
      }
    }
    else {
      
      LOCALIZE_samples_reply_type samplesReply;
      
      int i;
      
      sampleType* replySamples;
      
      /* If required samples is set to zero than send them all. */
      if ( numSamples <= 0)
	numSamples = tcxInfo.tcxSamples->numberOfSamples;
      
      samplesReply.numberOfSet = tcxInfo.tcxSamples->numberOfSet;
      
      replySamples = (sampleType*) malloc ( numSamples * sizeof(sampleType));
      
      for(i = 0; i < numSamples; i++) {
	replySamples[i].pos    = tcxInfo.tcxSamples->sample[i].pos;
	replySamples[i].weight = tcxInfo.tcxSamples->sample[i].weight;
      }
      
      samplesReply.boundingBoxMin.x   = tcxInfo.tcxSamples->minX;
      samplesReply.boundingBoxMin.y   = tcxInfo.tcxSamples->minY;
      samplesReply.boundingBoxMin.rot = -DEG_360;
      samplesReply.boundingBoxMax.x   = tcxInfo.tcxSamples->maxX;
      samplesReply.boundingBoxMax.y   = tcxInfo.tcxSamples->maxY;
      samplesReply.boundingBoxMax.rot = +DEG_360;
      
      samplesReply.numberOfSamples = numSamples;
      samplesReply.replySamples    = replySamples; 
      
      samplesReply.timeStamp       = tcxInfo.tcxSamples->timeStamp;
      
      tcxSendMsg(module, "LOCALIZE_samples_reply", &samplesReply);
      writeLog( "Sending %d samples to %s.\n", numSamples, name);

      free(replySamples);
    }
  }
}

void
LOCALIZE_map_query_handler( TCX_REF_PTR ref,
			    LOCALIZE_map_query_ptr query)
{
  if ( query->resizedObstacles)
    sendMapReplyType( tcxInfo.initialPositionProbs,
		      NOT_INVERTED,
		      ref->module,
		      tcxModuleName(ref->module), 0);
  else
    sendMapReplyType( tcxInfo.originalMap,
		      INVERTED,
		      ref->module,
		      tcxModuleName(ref->module), 0);
}


/*****************************************************************************
 * Sends the correction parameters for the current estimated position
 * to the module.
 *****************************************************************************/
bool
sendCorrectionParams( correctionParameter corr,
		      TCX_MODULE_PTR module,
		      char* name)
{
  if ( corr.initialized && connectionEstablished( &module, name)) {

    if ( module != MAP) {
      MAP_correction_parameters_reply_type data;
      
      /* Set the values in the tcx struct and send it. */
      data.parameter_x     = corr.x;
      data.parameter_y     = corr.y;
      data.parameter_angle = corr.rot;
      data.type            = corr.type;
      data.current_wall_angle = 0.0;
      data.current_wall_angle_defined = 0;
      
      
      writeLog( "Send correction parameter to %s (", name);
      writeLog( "%f %f %f %d).\n",
		data.parameter_x, data.parameter_y, data.parameter_angle, data.type);
      tcxSendMsg( module, "MAP_correction_parameters_reply", &data);
    }
    else {
      if (communicationInfo.sendCorrectionParametersToMap){
	MAP_correction_parameters_inform_type data2;
	data2.parameter_x     = corr.x;
	data2.parameter_y     = corr.y;
	data2.parameter_angle = corr.rot;
	data2.type            = corr.type;
	
	writeLog( "Echoed correction parameter to MAP");
	fprintf( stderr, "Echoed correction parameter to MAP");
	writeLog( "%f %f %f %d).\n",
		  data2.parameter_x, data2.parameter_y, data2.parameter_angle, data2.type);
	tcxSendMsg(MAP, "MAP_correction_parameters_inform", &data2);
      }
    }
    
    return TRUE;
  }
  else
    return FALSE;
}


/*****************************************************************************
 * Sends the map to the module.
 *****************************************************************************/
bool
sendMapReplyType( probabilityGrid* map,
		  bool inverted,
		  TCX_MODULE_PTR module,
		  char* name, int type)
{
  if ( connectionEstablished( &module, name)) {

    MAP_partial_map_reply_type data;
    int x, y;
    probability min = 1e20, max = -1e20;

    for ( x = 0; x < map->sizeX; x++)
      for ( y = 0; y < map->sizeY; y++)
	if ( map->prob[x][y] != map->unknown) {
	  if (map->prob[x][y] < min)
	    min = map->prob[x][y];
	  if (map->prob[x][y] > max) {
	    max = map->prob[x][y];
	  }
	}

    /* Now copy the values in the tcx structure. */
    data.first_x = 0;
    data.first_y = 0;
    data.delete_previous_map = 1;
    data.number_of_map = type;

    data.resolution = map->resolution;
    data.size_x  = map->sizeX;
    data.size_y  = map->sizeY;

    data.char_values = (unsigned char*)
      malloc( data.size_x * data.size_y * sizeof(unsigned char));

    for ( x = 0; x < data.size_x; x++)
      for ( y = 0; y < data.size_y; y++)
	if ( map->prob[x][y] != map->unknown) {
	  if ( inverted)
	    data.char_values[x * data.size_y + y] =
	      fNorm( map->prob[x][y], min, max, 230, 1);
	  else
	    data.char_values[x * data.size_y + y] =
	      fNorm( map->prob[x][y], min, max, 1, 230);
	}
	else
	  data.char_values[x * data.size_y + y] = 0;

    writeLog( "# Send map to %s.\n", name);

    tcxSendMsg(module, "MAP_partial_map_reply", &data);

    free( data.char_values);
    return TRUE;
  }
  else {
    writeLog( "Error: attempt to send reply map to unconnected module.\n");
    return FALSE;
  }
}


/*****************************************************************************
 * Sends the map to the module.
 *****************************************************************************/
bool
sendMap( probabilityGrid* map,
	 bool inverted,
	 TCX_MODULE_PTR module,
	 char* name)
{
  if ( connectionEstablished( &module, name)) {

    MAP_sensor_interpretation_type data;
    int x, y;
    probability min = 1e20, max = -1e20;

    for ( x = 0; x < map->sizeX; x++)
      for ( y = 0; y < map->sizeY; y++)
	if ( map->prob[x][y] != map->unknown) {
	  if (map->prob[x][y] < min)
	    min = map->prob[x][y];
	  if (map->prob[x][y] > max) {
	    max = map->prob[x][y];
	  }
	}

    /* Now copy the values in the tcx structure. */
    data.delete_previous_map = 1;
    data.map_number = CAD_MAP_NUMBER;

    data.resolution = map->resolution;
    data.size_x  = map->sizeX;
    data.size_y  = map->sizeY;

    /* Set some values necessary for MAP. */
    data.robot_x = 0.0;
    data.robot_y = 0.0;
    data.robot_orientation = 0.0;
    data.translational_speed = 0.0;
    data.rotational_speed = 0.0;
    data.origin_x = 0.0;
    data.origin_y = 0.0;
    data.origin_orientation = 0.0;

    /* No sensor data with this message */
    data.num_sensor_values_enclosed = 0;
    data.sensor_values              = NULL;


    data.char_likelihoods = (unsigned char*)
      malloc( data.size_x * data.size_y * sizeof(unsigned char));

    for ( x = 0; x < data.size_x; x++)
      for ( y = 0; y < data.size_y; y++)
	if ( map->prob[x][y] != map->unknown) {
	  if ( inverted)
	    data.char_likelihoods[x * data.size_y + y] =
	      fNorm( map->prob[x][y], min, max, 230, 1);
	  else
	    data.char_likelihoods[x * data.size_y + y] =
	      fNorm( map->prob[x][y], min, max, 1, 230);
	}
	else
	  data.char_likelihoods[x * data.size_y + y] = 0;

    writeLog( "# Send map to %s.\n", name);

    tcxSendMsg(module, "MAP_sensor_interpretation", &data);
    free( data.char_likelihoods);

    return TRUE;
  }
  else {
    fprintf(stderr, "Error: attempt to send map to unconnected module.\n");
    return FALSE;
  }
}


/*****************************************************************************
 * Gets a map and stores it in the online map structure.
 *****************************************************************************/
void MAP_partial_map_reply_handler( TCX_REF_PTR                 ref,
				    MAP_partial_map_reply_ptr map)
{  
  probabilityGrid* onlineMap = tcxInfo.onlineMap;
  static int firstTime = TRUE;

  if ( map->size_x < 2 || map->size_y < 2) {
    fprintf( stderr, "# Map not yet initialized. Ignore it.\n");
    writeLog( "# Map not yet initialized. Ignore it.\n");
    tcxFree ( "MAP_correction_parameters_reply", map);
    return;
  }

  fprintf( stderr, "Got map: %d %d  (%d %d).\n",
	   map->size_x, map->size_y, map->first_x, map->first_y);

  writeLog( "# Got map: %d %d  (%d %d).\n",
	    map->size_x, map->size_y, map->first_x, map->first_y);
  
  writeLog( "# Robot position is: %f %f %f (%f %f %f).\n",
	    map->robot_x, map->robot_y, map->robot_orientation,
	    tcxInfo.samples->mean.x, tcxInfo.samples->mean.y, rad2Deg(tcxInfo.samples->mean.rot));
  
  /* Initialize the map parameters according to the received map. */
  incorporatePartialMap( map->char_values,
			 map->size_x, map->size_y,
			 map->first_x, map->first_y,
			 map->resolution,
			 onlineMap);

  /* Adjust sample region. */
  setSampleRegion( tcxInfo.samples,
		   onlineMap->maxRealX - onlineMap->sizeX *
		   onlineMap->resolution,
		   onlineMap->maxRealY - onlineMap->sizeY *
		   onlineMap->resolution,
		   onlineMap->maxRealX, onlineMap->maxRealY);
  
  /* Copy the relevant information into all other maps as well. */
  *tcxInfo.originalMap = *onlineMap;
  *tcxInfo.laserMap    = *onlineMap;
  *tcxInfo.sonarMap    = *onlineMap;
    
  if ( firstTime) {
    
    realPosition mapPos;
        
    /* Initialize the samples to the position in this map. */    
    mapPos.x = map->robot_x;
    mapPos.y = map->robot_y;
    mapPos.rot = deg2Rad(map->robot_orientation);
    
    
#define POSITION_UNCERTAINTY 15.0
    if (1) setSamplePosition( mapPos,
		       tcxInfo.samples,
		       POSITION_UNCERTAINTY);
    
    /* Update the correction parameters. */
    computeCorrectionParam( *tcxInfo.basePosition,
			    mapPos,
			    tcxInfo.correctionParam);
    
    onlineMap->initialized = TRUE;
  
    firstTime = FALSE;
  }
  
  ref=ref;
  tcxFree ( "MAP_correction_parameters_reply", map);
}


void MAP_correction_parameters_reply_handler(TCX_REF_PTR                 ref,
				     MAP_correction_parameters_reply_ptr map)
{
  ref = ref;
  fprintf( stderr, "Shouldn't get correction from map.\n");
  tcxFree ( "MAP_correction_parameters_reply", map);
}

void MAP_robot_position_reply_handler(TCX_REF_PTR                 ref,
				     MAP_robot_position_reply_ptr map)
{
  ref = ref;
  fprintf( stderr, "Shouldn't get robot positions from map.\n");
  tcxFree ( "MAP_robot_position_reply", map);
}


/**********************************************************************
 **********************************************************************
 *
 *  BASE handlers
 *
 **********************************************************************
 **********************************************************************/
void
BASE_update_status_reply_handler( TCX_REF_PTR ref,
				  BASE_update_status_reply_ptr status)
{
  extern laserParameters globalLaserParameters;

  if (1 || !globalLaserParameters.useFrontLaser){
    baseStatus.newPos.x = status->pos_x;
    baseStatus.newPos.y = status->pos_y;
    baseStatus.newPos.rot = status->orientation;
    baseStatus.newPosition = TRUE;
    baseStatus.newSensing = TRUE;
  }

  if (0) {
    char out_text[256];
    struct timeval t;
    time_t current_time;
    time( &current_time);
    strftime (out_text, 50, "\n@SENS %d-%m-%y %H:%M:%S",
	      localtime(&current_time));
    gettimeofday (&t,NULL);
    writeLog ( "%s.%d\n", out_text, t.tv_usec);
    writeLog ( "#ROBOT %f %f %f\n\n",
	       status->pos_x, status->pos_y, 90.0 - status->orientation);
  }

  *(tcxInfo.transVelocity) = status->trans_current_speed;
  *(tcxInfo.rotVelocity) = deg2Rad(status->rot_current_speed);
  
  /* Don't remove this! */
  tcxFree("BASE_update_status_reply", status);
  fprintf(stderr, "b");  
  ref=ref;
}

void
BASE_robot_position_reply_handler(TCX_REF_PTR                   ref,
				  BASE_robot_position_reply_ptr pos)
{;ref=ref;pos=pos;}


void
BASE_action_executed_reply_handler(TCX_REF_PTR                   ref,
				   BASE_action_executed_reply_ptr data)
{;ref=ref;data=data;}


/**********************************************************************
 **********************************************************************
 *
 *  SONAR handlers
 *
 **********************************************************************
 **********************************************************************/
void
SONAR_sonar_reply_handler(TCX_REF_PTR           ref,
			  SONAR_sonar_reply_ptr sonar)
{
  int i;

  ref = ref;
  
#ifdef USER_debug
  fprintf(stderr, "TCX: Received a SONAR_sonar_reply message.\n");
  for (i = 0; i < 24; i++)
    fprintf(stderr, " %5.2f", sonar->values[i]);
  fprintf(stderr, "\n");
#endif
  
  for (i = 0; i < numberOfSonars; i++){
    if ( robotType != PIONEER_II &&
	 robotType != PIONEER_ATRV &&
	 robotType != URBAN_ROBOT)
      sonar->values[i] += ROB_RADIUS;
    if (sonar->values[i] < 0.0)
      baseStatus.sonar[i] = 0.0;
    else
      baseStatus.sonar[i] = sonar->values[i];
  }
  baseStatus.newSonar = TRUE;
  baseStatus.newSensing = TRUE;

  tcxFree("SONAR_sonar_reply", sonar);
  ref=ref;
  fprintf(stderr, "s");
}


void
SONAR_ir_reply_handler(TCX_REF_PTR      ref,
		       SONAR_ir_reply_ptr ir)
{
  ref=ref;
  tcxFree("SONAR_ir_reply", ir);
}



/**********************************************************************
 **********************************************************************
 *
 *  LASER handlers
 *
 **********************************************************************
 **********************************************************************/
void
LASER_laser_reply_handler(TCX_REF_PTR           ref,
			  LASER_laser_reply_ptr laser)
{
  int i;

  if (0){
   char out_text[256];
    struct timeval t;
    time_t current_time;
    time( &current_time);
    strftime (out_text, 50, "\n@SENS %d-%m-%y %H:%M:%S",
	      localtime(&current_time));
    gettimeofday (&t,NULL);
    writeLog( "%s.%d\n", out_text, t.tv_usec);
    writeLog("#LASER %d %d:",
	     laser->f_numberOfReadings, laser->r_numberOfReadings);
    for (i = 0; i < laser->f_numberOfReadings; i++){
      if (laser->f_reading != NULL)
	writeLog(" %d", laser->f_reading[i]);
      else
	writeLog(" -1");
    }
    for (i = 0; i < laser->r_numberOfReadings; i++){
      if (laser->r_reading != NULL)
	writeLog(" %d", laser->r_reading[i]);
      else
	writeLog(" -1");
    }
    writeLog("\n");
  }
  
  baseStatus.sizeOfFrontLaserScan = laser->f_numberOfReadings;
  baseStatus.sizeOfRearLaserScan  = laser->r_numberOfReadings;

  /* Get the front readings. */
  for (i = 0; i < laser->f_numberOfReadings; i++) {
    if (laser->f_reading[i] < 0.0)
      baseStatus.frontLaser[i] = 0.0;
    else {
      baseStatus.frontLaser[i] = laser->f_reading[i];
    }
  }
  
  /* Get the rear readings. */
  for (i = 0; i < laser->r_numberOfReadings; i++) {
    if (laser->r_reading[i] < 0.0)
      baseStatus.rearLaser[i] = 0.0;
    else{
      baseStatus.rearLaser[i] = laser->r_reading[i];
    }
  }

  if (laser->f_numberOfReadings > 0) baseStatus.newFrontLaser = TRUE;
  if (laser->r_numberOfReadings > 0) baseStatus.newRearLaser = TRUE;

  /*sprintf(stderr, "LASER: %g %g %g\n", laser->xPos, laser->yPos, laser->rotPos);*/

  baseStatus.laserPos.x = laser->xPos;
  baseStatus.laserPos.y = laser->yPos;
  baseStatus.laserPos.rot = laser->rotPos;

  if (0) {
    baseStatus.newPos.x = laser->xPos;
    baseStatus.newPos.y = laser->yPos;
    baseStatus.newPos.rot = 90.0 - rad2Deg(laser->rotPos);
    baseStatus.newPosition = TRUE;

  }
  baseStatus.newSensing = baseStatus.newFrontLaser || baseStatus.newRearLaser;
    
  tcxFree("LASER_laser_reply", laser);
  ref=ref;

  fprintf(stderr, "l");
}



/**********************************************************************
 **********************************************************************
 *
 *  MAP handlers
 *
 **********************************************************************
 **********************************************************************/

bool
getSensing( rawSensings* actualSensing)
{
  int i;

  struct timeval TCX_waiting_time = {0, 0};

  baseStatus.newPosition = FALSE;
  baseStatus.newSonar = FALSE;
  baseStatus.newFrontLaser = FALSE;
  baseStatus.newRearLaser = FALSE;
  baseStatus.newButton = FALSE;
  baseStatus.newSensing = FALSE;
  baseStatus.newCamera = FALSE;

  while ( ! baseStatus.newSensing) {
    TCX_waiting_time.tv_sec = 0;
    TCX_waiting_time.tv_usec = 0;

    tcxRecvLoop((void *) &TCX_waiting_time);

    if ( BASE == NULL) {
      BASE_register_auto_update_type data;
      data.subscribe_status_report = tcxInfo.subscribeBaseReport;
      data.subscribe_sonar_report  = tcxInfo.subscribeProximityReport;
      data.subscribe_laser_report  = tcxInfo.subscribeProximityReport;
      data.subscribe_ir_report     = 0;
      data.subscribe_colli_report  = 0;

      if (communicationInfo.robotName == NULL){
	writeLog( "Reconnecting %s ...", moduleName[BASE_MODULE]);
	fprintf( stderr, "Reconnecting %s ...", moduleName[BASE_MODULE]);
      }
      else{
	writeLog( "Reconnecting %s_%s ...", moduleName[BASE_MODULE],
		  communicationInfo.robotName);
	
	fprintf( stderr, "Reconnecting %s_%s ...", moduleName[BASE_MODULE],
		 communicationInfo.robotName);
      }	
      
      BASE = tcxConnectModule(moduleName[BASE_MODULE]);
      tcxSendMsg(BASE, "BASE_register_auto_update", &data);
      writeLog( "done.\n");
      fprintf(stderr, "done.\n");
      baseStatus.newPosition = FALSE;
    }

    if ( ! baseStatus.newSensing) {
      block_wait(NULL, 1, 0);
    }
  }
  
  if ( baseStatus.newPosition) {

    if ( baseStatus.positionCount == 0){
      actualSensing->delta.forward = actualSensing->delta.sideward =
	actualSensing->delta.rotation = 0.0;
      baseStatus.currentPos.x = baseStatus.newPos.x;
      baseStatus.currentPos.y = baseStatus.newPos.y;
      baseStatus.currentPos.rot = baseStatus.newPos.rot;
      baseStatus.positionCount = 1;
    }
    else {
      actualSensing->delta =
	movementBetweenRobotPoints(baseStatus.currentPos, baseStatus.newPos);
      baseStatus.positionCount++;
      if (0) writeLog( "delta %f %f %f -- %f %f %f  --> %f %f %f\n",
		baseStatus.currentPos.x,
		baseStatus.currentPos.y,
		baseStatus.currentPos.rot,
		baseStatus.newPos.x,
		baseStatus.newPos.y,
		baseStatus.newPos.rot,
		actualSensing->delta.forward,
		actualSensing->delta.sideward,
		rad2Deg(actualSensing->delta.rotation));
    }

    baseStatus.currentPos.x = baseStatus.newPos.x;
    baseStatus.currentPos.y = baseStatus.newPos.y;
    baseStatus.currentPos.rot = baseStatus.newPos.rot;

    actualSensing->basePosition.x   = baseStatus.currentPos.x;
    actualSensing->basePosition.y   = baseStatus.currentPos.y;
    actualSensing->basePosition.rot = baseStatus.currentPos.rot;

    actualSensing->delta.isNew = TRUE;
  }
  else {
    actualSensing->delta.isNew = FALSE;
  }
  
  if (baseStatus.newSonar){
    actualSensing->sonar.numberOfReadings = numberOfSonars;

    if (robotType == PIONEER_ATRV || robotType == PIONEER_II ||	robotType == URBAN_ROBOT)
      for (i=0; i < numberOfSonars; i++) {
	actualSensing->sonar.reading[i].dist =
	  baseStatus.sonar[i];
      }
    else
      for (i=0; i < numberOfSonars; i++) {
	actualSensing->sonar.reading[i].dist =
	  baseStatus.sonar[numberOfSonars-1-i];
      }
    actualSensing->sonar.isNew = TRUE;
  }
  else {
    actualSensing->sonar.isNew = baseStatus.newSonar = FALSE;
  }

  actualSensing->frontLaser.isNew = FALSE;
  actualSensing->rearLaser.isNew = FALSE;

  if ( baseStatus.newFrontLaser || baseStatus.newRearLaser) {

    /* Check whether the position of the scan is close enough to the
     * position of the base status. */
#define DISTANCE_THRESHOLD 100.0

    float angleDist = fabs( 90.0 - baseStatus.currentPos.rot
			    - rad2Deg(baseStatus.laserPos.rot));
    if ( angleDist > 180.0)
      angleDist = 360.0 - angleDist;
    
    if ( sqrt(
	      fSqr( baseStatus.currentPos.x - baseStatus.laserPos.x) +
	      fSqr( baseStatus.currentPos.y - baseStatus.laserPos.y) +
	      fSqr( angleDist)) < DISTANCE_THRESHOLD) {
      
      if (0) fprintf( stderr, "%f %f %f %d #BASE\n%f %f %f %d #LASER\n%f %f %f #DIST\n",
		      baseStatus.currentPos.x,
		      baseStatus.currentPos.y,
		      90.0 - baseStatus.currentPos.y,
		      baseStatus.newPosition,
		      baseStatus.laserPos.x,
		      baseStatus.laserPos.y,
		      rad2Deg(baseStatus.laserPos.rot),
		      baseStatus.newFrontLaser,
		      baseStatus.currentPos.x - baseStatus.laserPos.x,
		      baseStatus.currentPos.y - baseStatus.laserPos.y,
		      angleDist);
      
      if ( baseStatus.newFrontLaser) {
	
	actualSensing->frontLaser.numberOfReadings =
	  baseStatus.sizeOfFrontLaserScan;
	
	for (i=0; i < baseStatus.sizeOfFrontLaserScan; i++)
	  actualSensing->frontLaser.reading[i].dist = baseStatus.frontLaser[i];
	
	actualSensing->frontLaser.isNew = TRUE;
      }

      if ( baseStatus.newRearLaser) {

	actualSensing->rearLaser.numberOfReadings =
	  baseStatus.sizeOfRearLaserScan;

	for (i=0; i < baseStatus.sizeOfRearLaserScan; i++)
	  actualSensing->rearLaser.reading[i].dist = baseStatus.rearLaser[i];
	actualSensing->rearLaser.isNew = TRUE;
      }
    }
    else
      writeLog( "IGNORE laser reading ((%f %f) (%f %f) (%f %f).\n",
		baseStatus.currentPos.x, baseStatus.laserPos.x,
		baseStatus.currentPos.y, baseStatus.laserPos.y,
		baseStatus.currentPos.rot, baseStatus.laserPos.rot);
  }


  if (CAMERA != NULL && baseStatus.newCamera){
    int x, y;

    for (x = 0; x < IMAGE_SIZE_X; x++){
      for (y = 0; y < IMAGE_SIZE_Y; y++){
	actualSensing->image.pix[x][y] =
	  baseStatus.pix[x][y];
	/*	fprintf(stderr, "%3d ", actualSensing->image.pix[x][y]); */
      }
      /*      fprintf(stderr, "\n"); */
    }
    actualSensing->image.sizeX = IMAGE_SIZE_X;
    actualSensing->image.sizeY = IMAGE_SIZE_Y;
    actualSensing->image.isNew = TRUE;
  }    
    
  
  if (baseStatus.newButton){
    actualSensing->answer.answerType = baseStatus.colorOfButton;
    actualSensing->answer.isNew = TRUE;
  }

  return(TRUE);
}



/**********************************************************************
 **********************************************************************
 *
 *  SOUND handlers
 *
 **********************************************************************
 **********************************************************************/

void 
SOUND_play_reply_handler(TCX_REF_PTR ref, void *data)
{
  ref = ref;
  fprintf( stderr, "Recieving SOUND_play_reply\n" );
  tcxFree( "SOUND_play_reply", data );
}

void
SOUND_playing_reply_handler(TCX_REF_PTR ref, SOUND_playing_reply_ptr data)
{
  ref = ref;
  tcxFree( "SOUND_playing_reply", data );
}




/**********************************************************************
 **********************************************************************
 *
 *  CAMERA handlers
 *
 **********************************************************************
 **********************************************************************/

void
CAMERA_image_reply_handler (TCX_REF_PTR ref, CAMERA_image_reply_ptr data)
{
  int x, y;
  unsigned char *red = data->red;
  
  ref = ref;

  /* #define DEBUG */
#ifdef DEBUG
  fprintf( stderr, "Recieving CAMERA_image_reply\n" );
  fprintf( stderr, "sizeX: %d sizeY: %d\n",
	   data->xsize, data->ysize);
#endif
  if (data->xsize != IMAGE_SIZE_X || data->ysize != IMAGE_SIZE_Y){
    fprintf(stderr, "# Error: wrong image size, ignoring image\n");
  }
  else{

    for (x = 0; x < IMAGE_SIZE_X; x++){
      for (y = 0; y < IMAGE_SIZE_Y; y++){
	baseStatus.pix[x][y] = *red++;
#ifdef DEBUG
fprintf(stderr, "%3d ", baseStatus.pix[x][y]);
#endif
      }
#ifdef DEBUG
      fprintf(stderr, "\n");
#endif
    }
  }

  baseStatus.newSensing = baseStatus.newCamera = TRUE;

  tcxFree( "CAMERA_image_reply", data );
  fprintf(stderr, "c");
  
}

void
CAMERA_load_reply_handler (TCX_REF_PTR ref, CAMERA_load_reply_ptr data)
{
  ref = ref;
  fprintf( stderr, "Recieving CAMERA_load_reply\n" );
  tcxFree( "CAMERA_load_reply", data );
  
}

void
CAMERA_shmid_reply_handler (TCX_REF_PTR ref, CAMERA_shmid_reply_ptr data)
{
  ref = ref;
  fprintf( stderr, "Recieving CAMERA_image_reply\n" );
  tcxFree( "CAMERA_shmid_reply", data );
  
}

/**********************************************************************
 **********************************************************************
 *
 *  BUTTONS handlers
 *
 **********************************************************************
 **********************************************************************/

void
BUTTONS_status_reply_handler (TCX_REF_PTR ref, BUTTONS_status_reply_ptr data)
{
  ref = ref;
  baseStatus.colorOfButton = NO_COLOR;
     
  if (data->red_button_pressed)
       baseStatus.colorOfButton = RED;
  else if (data->green_button_pressed)
       baseStatus.colorOfButton = GREEN;
  else if (data->yellow_button_pressed)
       baseStatus.colorOfButton = YELLOW;
  else if (data->blue_button_pressed)
       baseStatus.colorOfButton = BLUE;

  if (baseStatus.colorOfButton == RED || baseStatus.colorOfButton == GREEN) {
       baseStatus.newButton = TRUE;
       baseStatus.newSensing = TRUE;
       fprintf(stderr, "BUTTON pressed.\n");
    }
}


/**********************************************************************
 **********************************************************************
 *
 *  MULTI_LOCALIZE handlers
 *
 * This is a quick hack to send detections to the multi localize module.
 *
 **********************************************************************
 **********************************************************************/

void
sendDetectionToMultiLocalize( char* observer, char* detected,
			      float dist, float distUncertainty,
			      float angle, float angleUncertainty,
			      float currentRotSpeed,
			      struct timeval timeStamp,
			      int detectionFlag)
{
  writeLog( "# Send detection\n");
  if ( ! MULTI_LOCALIZE) {
    if ( communicationInfo.useTcx) {
      MULTI_LOCALIZE = tcxConnectOptional( "MULTI_LOCALIZE"); 
      if ( MULTI_LOCALIZE)
	fprintf( stderr, "# Connected to MULTI_LOCALIZE.\n");
    }
    else
      return;    
  }
    
  if ( MULTI_LOCALIZE) {
    ROBOT_DETECTION_status_reply_type detection;
    struct tm* tmp = localtime(&(timeStamp.tv_sec));
	    
    detection.robotDetected       = detectionFlag;
    detection.distance            = dist;
    detection.distanceUncertainty = distUncertainty;
    detection.angle               = angle;
    detection.angleUncertainty    = angleUncertainty;
    detection.nameOfDetectedRobot = detected;
    detection.nameOfObserverRobot = observer;
    detection.timeStamp           = timeStamp;
    detection.currentRotSpeed     = currentRotSpeed;

    tcxSendMsg( MULTI_LOCALIZE, "ROBOT_DETECTION_status_reply", & detection);
  }
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



TCX_REG_HND_TYPE LOCALIZE_handler_array[] = {

  {"LOCALIZE_register_auto_update", "LOCALIZE_register_auto_update_handler",
     LOCALIZE_register_auto_update_handler, TCX_RECV_ALL, NULL},
  {"LOCALIZE_register_update", "LOCALIZE_register_update_handler",
     LOCALIZE_register_update_handler, TCX_RECV_ALL, NULL},
  {"LOCALIZE_quit", "LOCALIZE_quit_handler",
     LOCALIZE_quit_handler, TCX_RECV_ALL, NULL},
  {"LOCALIZE_set_robot_position", "LOCALIZE_set_robot_position_handler",
     LOCALIZE_set_robot_position_handler, TCX_RECV_ALL, NULL},
  {"LOCALIZE_start_active_localization", "LOCALIZE_start_active_localization_handler",
     LOCALIZE_start_active_localization_handler, TCX_RECV_ALL, NULL},
  {"LOCALIZE_stop_active_localization", "LOCALIZE_stop_active_localization_handler",
     LOCALIZE_stop_active_localization_handler, TCX_RECV_ALL, NULL},
  {"LOCALIZE_map_query", "LOCALIZE_map_query_handler",
     LOCALIZE_map_query_handler, TCX_RECV_ALL, NULL},
  {"LOCALIZE_updated_samples", "LOCALIZE_updated_samples_handler",
     LOCALIZE_updated_samples_handler, TCX_RECV_ALL, NULL},
  {"LOCALIZE_samples_query", "LOCALIZE_samples_query_handler",
     LOCALIZE_samples_query_handler, TCX_RECV_ALL, NULL}
};


void
printConnectingMessage(FILE *fp, char *module){
    fprintf(fp, "Connecting to %s...", module);
}

void
init_tcx( actionInformation* info,
	  bool connectBase,
	  int subscribeBaseReport,
	  int subscribeProximityReport,
	  int subscribeCamera)
{
  char *tcxMachine = NULL;

  TCX_REG_MSG_TYPE TCX_message_array[] = {
    BASE_messages,
    LASER_messages,
    SONAR_messages,
    MAP_messages,
    PLAN_messages,
    BUTTONS_messages,
    LOCALIZE_messages,
    SOUND_messages,
    ROBOT_DETECTION_messages,
    CAMERA_messages
  };
  if (subscribeCamera > 0)
    fprintf(stderr, "Subscribing to CAMERA\n");

  tcxInfo.subscribeBaseReport = subscribeBaseReport;
  tcxInfo.subscribeProximityReport = subscribeProximityReport;
  tcxInfo.subscribeCameraReport = subscribeCamera;

  printConnectingMessage(stderr, "TCX");
  tcxMachine = getenv("TCXHOST");

  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }

  tcxInitialize(moduleName[USER_MODULE], (void *) tcxMachine);

  fprintf(stderr, "done.\n");

  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));

  tcxRegisterHandlers(BASE_reply_handler_array,
		      sizeof(BASE_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(SONAR_reply_handler_array,
		      sizeof(SONAR_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(LASER_reply_handler_array,
		      sizeof(LASER_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(PLAN_reply_handler_array,
		      sizeof(PLAN_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(MAP_reply_handler_array,
		      sizeof(MAP_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(BUTTONS_reply_handler_array,
		      sizeof(BUTTONS_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(SOUND_reply_handler_array,
		      sizeof(SOUND_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(CAMERA_reply_handler_array,
		      sizeof(CAMERA_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(LOCALIZE_handler_array,
		      sizeof(LOCALIZE_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterCloseHnd(LOCALIZE_close_handler);

  
  if ( connectBase) {
    BASE_register_auto_update_type data;
    data.subscribe_status_report = tcxInfo.subscribeBaseReport;
    data.subscribe_sonar_report  = tcxInfo.subscribeProximityReport;
    data.subscribe_laser_report  = tcxInfo.subscribeProximityReport;
    data.subscribe_ir_report     = 0;
    data.subscribe_colli_report  = 0;

    printConnectingMessage(stderr, moduleName[BASE_MODULE]);
    BASE = tcxConnectModule(moduleName[BASE_MODULE]);
    tcxSendMsg(BASE, "Base_register_auto_update", &data);
    fprintf(stderr, "done.\n");
  }
  else
    BASE = NULL;

  if ( connectBase && subscribeCamera > 0) {
    CAMERA_register_auto_update_type data;
    
    data.numGrabber = 1;
    data.image = subscribeCamera;
    data.orig_image_xmin = 160 - (IMAGE_SIZE_X / 2) ;
    data.orig_image_ymin = 120 - (IMAGE_SIZE_Y / 2);
    data.orig_image_xsize = IMAGE_SIZE_X;
    data.orig_image_ysize = IMAGE_SIZE_Y;
    data.return_image_xsize = IMAGE_SIZE_X;
    data.return_image_ysize = IMAGE_SIZE_Y;
    
    printConnectingMessage(stderr, moduleName[CAMERA_MODULE]);
    
    CAMERA = tcxConnectModule(moduleName[CAMERA_MODULE]);
    if (CAMERA != NULL){
      tcxSendMsg(CAMERA, "Camera_register_auto_update", &data);
      fprintf(stderr, "done.\n");
    }
    else
      fprintf(stderr, "failed.\n");
    
    /*
      if (0) do {
      CAMERA = tcxConnectOptional(moduleName[CAMERA_MODULE]);
      if (!CAMERA)
      CAMERA = tcxConnectOptional("METEOR");
      if (!CAMERA)
      CAMERA = tcxConnectOptional("METEOR1");
      if (!CAMERA)
      CAMERA = tcxConnectOptional("METEOR2");
      if (!CAMERA){
      TCX_waiting_time.tv_sec  = 1;
      TCX_waiting_time.tv_usec = 0;
      tcxRecvLoop((void *) &TCX_waiting_time);
      }
    } while (!CAMERA);
    */
  }
  else
    CAMERA = NULL;
  
  printConnectingMessage(stderr, moduleName[PLAN_MODULE]);
  PLAN = tcxConnectOptional(moduleName[PLAN_MODULE]); 
  if (PLAN == NULL)
    fprintf(stderr, "failed.\n");
  else
    fprintf(stderr, "done.\n");
  
  printConnectingMessage(stderr, moduleName[MAP_MODULE]);
  MAP = tcxConnectOptional(moduleName[MAP_MODULE]); 
  if (MAP == NULL)
    fprintf(stderr, "failed.\n");
  else {
    MAP_register_auto_update_type reg = {4,0,0};
    tcxSendMsg( MAP, "MAP_register_auto_update", &reg);
    fprintf(stderr, "done.\n");
  }
  
#ifdef CONSIDER_QUESTION
  {
    BUTTONS_register_auto_update_type data;
    data.status = 1;
    fprintf(stderr, "Connecting to %s...", moduleName[BUTTONS_MODULE]);
    BUTTONS = tcxConnectOptional(moduleName[BUTTONS_MODULE]);
    if (BUTTONS == NULL)
      fprintf(stderr, "failed.\n");
    else {
      tcxSendMsg (BUTTONS, "BUTTONS_register_auto_update", &data);
      fprintf(stderr, "done.\n");
    }

    SOUND = tcxConnectOptional(moduleName[SOUND_MODULE]);
    if (SOUND == NULL)
      fprintf(stderr, "failed.\n");
  }
#endif
  
  baseStatus.newPosition = baseStatus.newSonar = FALSE;
  baseStatus.positionCount = 0;

  baseStatus.newPos.x = baseStatus.newPos.y = baseStatus.newPos.rot = 0.0;
  baseStatus.laserPos.x = baseStatus.laserPos.y = baseStatus.laserPos.rot = 0.0;
  
  initializeTcxInfoStructure( info);

  tcx_initialized = TRUE;

}






