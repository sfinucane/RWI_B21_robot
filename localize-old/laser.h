\
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/laser.h,v $
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
 * $Log: laser.h,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.34  1999/10/02 09:06:44  thrun
 * New robot type: XR4000.
 *
 * Revision 1.33  1999/09/29 16:06:10  fox
 * Should work.
 *
 * Revision 1.32  1999/09/26 18:55:22  fox
 * Added scout robot.
 *
 * Revision 1.31  1999/06/30 11:46:57  rhino
 * Added laser settings for PIONEER_II
 *
 * Revision 1.30  1999/06/25 19:48:12  fox
 * Minor changs for the urbie.
 *
 * Revision 1.29  1999/06/24 00:21:51  fox
 * Some changes for the urbies.
 *
 * Revision 1.28  1999/01/11 19:47:50  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.26  1998/12/16 14:59:02  wolfram
 * First version without libGetDistance. Use with caution.
 *
 * Revision 1.25  1998/11/25 16:29:24  wolfram
 * Added higher resolution for vision maps. resolution is now read from file.
 * Couldn't integrate it consistently into graphic.c.
 *
 * Revision 1.24  1998/11/17 23:26:21  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.23  1998/11/16 08:35:26  wolfram
 * Cleaned up vision.c. It seems to work give appropriate maps!
 *
 * Revision 1.22  1998/11/03 21:02:19  fox
 * Lasers will not be integrated if the robot rotates faster than
 * MAX_ROT_VEL_FOR_LASERS deg/sec.
 *
 * Revision 1.21  1998/10/29 03:45:02  fox
 * Nothing special.
 *
 * Revision 1.20  1998/10/23 20:52:50  fox
 * Nothing specatacular.
 *
 * Revision 1.19  1998/10/19 18:29:55  fox
 * *** empty log message ***
 *
 * Revision 1.18  1998/10/02 15:16:39  fox
 * Several improvements of condensation. Updated the function calls to the
 * new version of the distancServer.
 *
 * Revision 1.17  1998/09/25 17:53:31  fox
 * Improved version of condensation.
 *
 * Revision 1.16  1998/09/25 04:02:56  fox
 * First version of CONDENSATION!!!!
 *
 * Revision 1.15  1998/09/18 15:44:27  fox
 * First attempt to get a running system after museum.
 *
 * Revision 1.14  1998/08/11 23:05:37  wolfram
 * Changes for the Smithsonian museum.
 *
 * WARNING: LOCALIZE now uses only one stddev for Lasers. Laser beams have 25 m
 * Maxrange now.
 * Please use ~wolfram/bee/data/localize/params/laser-0.1stdev with this version
 * of LOCALIZE
 *
 * Revision 1.13  1998/02/13 14:12:22  fox
 * Minor changes.
 *
 * Revision 1.12  1998/01/27 15:25:25  fox
 * Minor changes.
 *
 * Revision 1.11  1998/01/05 10:37:12  fox
 * Several changes especially in selection parts.
 *
 * Revision 1.10  1997/12/11 17:06:30  fox
 * Added some parameters.
 *
 * Revision 1.9  1997/11/25 17:12:55  fox
 * Should work.
 *
 * Revision 1.8  1997/08/16 22:59:50  fox
 * Last version before I change selsection.
 *
 * Revision 1.7  1997/05/26 08:47:50  fox
 * Last version before major changes.
 *
 * Revision 1.6  1997/03/14 17:58:19  fox
 * This version should run quite stable now.
 *
 * Revision 1.5  1997/03/13 17:36:36  fox
 * Temporary version. Don't use!
 *
 * Revision 1.4  1997/01/31 17:11:01  fox
 * Integrated laser reply.
 *
 * Revision 1.3  1997/01/30 17:17:24  fox
 * New version with integrated laser.
 *
 * Revision 1.2  1996/12/02 10:32:07  wolfram
 * Expected distance file now includes distances + variances
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



#ifndef LASER_INCLUDE
#define LASER_INCLUDE

#include "general.h"
#include "sensings.h"
#include "probGrid.h"
#include "proximity.h"

/*********************************************************
 *********************************************************
 * The following functions and structures are mandatory for
 * each sensor.
 *********************************************************
 *********************************************************/

/* Action to be performed for the lasers. */
#define INTEGRATE_LASER         0

#define NUMBER_OF_ACTIONS_LASER 1


/* Laser parameter */
/* the product of the latter two must be 256 */

#define MAX_SIZE_OF_LASER_SCAN 180
extern float FRONT_LASER_START_ANGLE;
extern float REAR_LASER_START_ANGLE;

#define B18_FRONT_LASER_OFFSET 34.5
#define B18_REAR_LASER_OFFSET (-10.5)

#define B21_TWO_LASERS_FRONT_LASER_OFFSET 11.5
#define B21_TWO_LASERS_REAR_LASER_OFFSET (-11.5)

/*  #define B21_ONE_LASER_OFFSET -6.0 */
#define B21_ONE_LASER_OFFSET 11.5
 
#define PIONEER_ATRV_LASER_OFFSET 11.5

#define PIONEER_II_LASER_OFFSET 11.5

#define URBAN_ROBOT_LASER_OFFSET 20.0

#define SCOUT_LASER_OFFSET 10.0

#define XR4000_LASER_OFFSET 20.0


/* These offsets will be set according to the robot type. */
extern float frontLaserOffset;
extern float rearLaserOffset;


#define LASER_OPENING_ANGLE                      M_PI / 60;
#define LASER_HEIGHT                                40

extern int numberOfLaserIntegrationsSinceStopped;

void
initialize_LASER( char* fileName,
		  actionInformation* actionInfo,
		  sensingActionMask* actionMask,
		  sensingFunctions* handlers);

void
checkIfConsider_LASER( actionInformation* actionInfo,
		       sensingActionMask* mask);

void
checkWhichActionsToPerform_LASER( actionInformation* actionInfo,
				  sensingActionMask* mask);

void
performActions_LASER( actionInformation* actionInfo,
		      sensingActionMask* mask);

/*********************************************************
 *********************************************************
 * Structures.
 *********************************************************
 *********************************************************/

typedef struct {
  float integrateThreshold;
  float maxQuotaOfPlanes;
  sensorParameters parameters;
  bool useFrontLaser;
  bool useRearLaser;
  int numberOfLasersToBeUsed;
  int numberOfLasersForConvolve;
  int maxNumberOfIntegrationsPerStop;
  int cutFeature;
  float maxRotVelForIntegration;
} laserParameters;



/* This struct stores information needed to integrate the laser information. */
typedef struct {
  informationsFor_PROXIMITY  general;
  abstractSensorVector*    abstractFrontScan;
  abstractSensorVector*    abstractRearScan;
} informationsFor_LASER;



/*********************************************************
 *********************************************************
 * Functions.
 *********************************************************
 *********************************************************/

distance
obstacleDistance_LASER( probabilityGrid m,
			int x, int y, float rot,
			distance maxRange);


float
laserRot( int laserReading, int laser);

#endif
