

/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** File name:                   laser.h
 *****
 ***** Part of:                     RHINO SOFTWARE
 *****
 ***** Creator:                     Wolfram Burgard, University of Bonn
 *****
 ***** Date of creation:            Aug 1996
 *****
 *****
 *****
 *****
 ***** $Source: /usr/local/cvs/bee/src/simulator2/laser.h,v $
 *****
 ***** $Revision: 1.1 $
 *****
 ***** $Date: 2002/09/14 16:11:07 $
 *****
 ***** $Author: rstone $
 *****
 *****
 *****
 ***** Contact thrun@carbon.cs.bonn.edu or thrun@cs.cmu.edu.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** 
 ***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED BY
 ***** APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING THE
 ***** COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM "AS IS"
 ***** WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED,
 ***** INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 ***** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE ENTIRE
 ***** RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU.
 ***** SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL
 ***** NECESSARY SERVICING, REPAIR OR CORRECTION.
 ***** 
 ***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
 ***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
 ***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
 ***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
 ***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
 ***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO LOSS OF
 ***** DATA OR DATA BEING RENDERED INACCURATE OR LOSSES SUSTAINED BY YOU
 ***** OR THIRD PARTIES OR A FAILURE OF THE PROGRAM TO OPERATE WITH ANY
 ***** OTHER PROGRAMS), EVEN IF SUCH HOLDER OR OTHER PARTY HAS BEEN
 ***** ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#define NUMBER_OF_LASERS 2
#define NUMBER_OF_LASER_READINGS 180
#define LASER_ANGLE_RESOLUTION 1                /* unfortunately in Deg */
#define LASER_RANGE		   2500		/*** centimeter ***/
#define LASER_INFINITY_RANGE       5000         /*** centimeter ***/
#define LASER_OFFSET 11.5

typedef float laserReading;

typedef struct
{
  float x;
  float y;
} position;


typedef struct {
  int numberOfReadings;
  laserReading reading[NUMBER_OF_LASER_READINGS];
  float angleResolution;
  float distanceFromCenter;
  float startAngle;
  position robotPos;
  float robotRot;
} laserRangeFinder;

#ifdef __cplusplus
extern "C" {
#endif
    
void
initLasers();


/***************************************************************************/
/* PROCEDURE :			init( LaserlaserRangeFinder *laser)       **/
/*									  **/
/* Initialize laser         						  **/
/* 									  **/
/***************************************************************************/

void 	initLaser(laserRangeFinder *laser, float startAngle);



/***************************************************************************/
/* PROCEDURE :			laserReport()      			  **/
/*									  **/
/* sends laser report via TCX 						  **/
/* 									  **/
/***************************************************************************/

void  	laserReport();

#ifdef __cplusplus
}
#endif







