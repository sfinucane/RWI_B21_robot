
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/obstacleServer.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:25:05 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: obstacleServer.h,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.4  1999/07/23 19:46:37  fox
 * Added ability to read grid maps for the obstacle server.
 *
 * Revision 1.3  1997/04/17 09:19:38  fox
 * Minor changes.
 *
 * Revision 1.2  1997/04/17 09:16:20  fox
 * Added timeout for laser devices --> colliServer only needs 30% cpu.
 *
 * Revision 1.1  1997/03/26 18:42:06  fox
 * Incorporated external obstacle information into the collision avoidance.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#ifndef OBSTACLE_SERVER_INCLUDE
#define OBSTACLE_SERVER_INCLUDE

#include "BASE-messages.h"
#include "LOCALIZE-messages.h"

#define TRUE 1
#define FALSE 0

typedef struct {
  float x;
  float y;
  float rot;
  int type;
} correctionParameter;

/* This struct contains the given position / occupancy probabilities.
 */
typedef struct {
  float unknown;
  int sizeX;
  int sizeY;
  int origsizeX;
  int origsizeY;
  int shiftedX;
  int shiftedY;
  int resolution;
  float offsetX;
  float offsetY;
  float maxRealX;
  float maxRealY;
  float** prob;
} probabilityGrid;

#define DEG_90 (1.5707963705)
#define DEG_360 (6.2831854820)

float
DegToRad(float x);

float
RadToDeg(float x);

float
fSqr( float x);

int
obstacleInDirection( realPosition mapPos,
		     realPosition robPos,
		     float relativeAngle,
		     obstaclePoint* point,
		     correctionParameter* correction);

int
createScanInGrid( probabilityGrid* map,
		  realPosition mapPos,
		  realPosition robPos,
		  obstaclePoint* point,
		  int sizeOfScan);


int
readProbabilityMap( char *fileName, probabilityGrid *m);

extern realPosition robotPosition;
extern probabilityGrid gridMap;

#endif


