

/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** File name:                   sonar.h
 *****
 ***** Part of:                     RHINO SOFTWARE
 *****
 ***** Creator:                     Peter Wallossek, University of Bonn
 *****
 ***** Date of creation:            Jan 1994
 *****
 *****
 *****
 *****
 ***** $Source: /usr/local/cvs/bee/src/simulator2/sonar.h,v $
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
 ***************************************************************************
 *
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: sonar.h,v $
 * Revision 1.1  2002/09/14 16:11:07  rstone
 * *** empty log message ***
 *
 * Revision 1.8  1997/07/17 17:31:53  tyson
 * Added wiggle, square, Changes, fixed bugs, added features.  See the new Changes file for details
 *
 * Revision 1.7  1997/04/09 17:27:08  thrun
 * Tyson changed Robot.h to oldRobot.h
 *
 * Revision 1.6  1997/04/07 10:00:12  schulz
 * some changes in saving/loading playgrounds
 *
 * Revision 1.5  1997/03/25 21:44:47  tyson
 * Many bug fixes.
 *
 * Revision 1.4  1997/02/02 22:32:44  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
 *
 * Revision 1.3  1996/12/19 15:37:45  schulz
 * -- adjusted default sonar_range to 650
 * -- introduced sonar_infinity as 3610.777832
 *
 * Revision 1.2  1996/10/25 14:27:13  ws
 * some improvements
 * - better options handling
 * - added an initialization file, many parameters are modifiable now
 * - Modified some timings to reduce CPU load
 *   (TCX is polled after every 50 ms now)
 *
 * Revision 1.1.1.1  1996/09/30 16:17:45  schulz
 * reimport on new source tree
 *
 * Revision 1.4  1996/09/11 14:03:47  schulz
 * - Speed up sonar and laser simulation by a factor of 20.
 *   We precalculate the set of obstacles which can be hit by a
 *   laser/sonar beam and check only for these few obstacles, if they are hit.
 *   Switched to S.Thruns version of line and circle intersection routines,
 *   these are a little bit faster then the old ones.
 * - Included laser visualisation  (to slow an some machines)
 *
 * Revision 1.3  1996/08/27 15:15:19  schulz
 * Many bug fixes...
 * Installed File Headers, changed c++ file names to .cc/.hh file suffix
 * There is still a great performance problem!
 *
 * Revision 1.2  1996/08/26 10:05:30  schulz
 * added laser interface (without visualization, yet) D.S.
 *
 * Revision 1.13  1995/06/02 09:44:43  wallosse
 * Changes minimal distance for sonars to 2 cm
 *
 * Revision 1.12  1995/05/31  11:01:13  wallosse
 * Neural network version / memory-based version.
 * Sonars are now modeled based on real data from the robot.
 *
 * Revision 1.11  1994/10/09  11:57:36  wallosse
 * changed some handling features. one more zoom-level. sonar display
 *
 * Revision 1.10  1994/09/30  14:23:03  wallosse
 * Now there is a minimal distance for sonar !
 *
 * Revision 1.9  1994/09/18  11:22:19  wallosse
 * Now it is possible to select between different types of surfaces.
 * I think it works !!!
 *
 * Revision 1.8  1994/09/06  11:52:20  fox
 * Now zooming is possible and scheduling works with the routines from block3.c
 *
 * Revision 1.7  1994/07/22  11:57:09  wallosse
 * Fixed a lot of bugs !
 * Vewrsion now runs again under LINUX when the optimize flag is not used when compiling.
 *
 * Revision 1.6  1994/06/22  13:45:30  wallosse
 * Changed a lot ! :	now events are scheduled with routine schedule().
 * 			obstacles can be added or chenged while robot
 * 			is runnung.
 * 			TARGET is not ok !!!
 *
 * Revision 1.4  1994/06/07  15:05:11  wallosse
 * New header files. Don't they look great?
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include "bUtils.h"

#ifdef __cplusplus
extern "C" {
#endif


#define NO_CHAINS		  4
#define NO_OF_SONARS		 bRobot.sonar_cols[0]
#define SONAR_ANGLE		 15		/*** degrees ***/
#ifdef B21
#define SONAR_OFFSET             7.5            
#endif
#ifdef B14
#define SONAR_OFFSET             11.5
#endif
#define RAYS_PER_SONAR		 3		
#define SONAR_RANGE		 650		/*** cm ***/
#define SONAR_INFINITY		 3610.777832		/*** cm ***/
#define SONAR_MIN_RANGE		  0.02		/*** meter ***/

#define CM_PER_SECOND 		33000.0
#define SECONDS_PER_CYCLE 	3.2552e-6
#define	SONAR_ANSWER_TIME	50            /* (int) (800/6)  in ms ***/ 



/**********************************************************************************************************/
/* PROCEDURE :			InitSonar()      			***********************************/
/*									***********************************/
/* Initialize sonar_readings						***********************************/
/* 									***********************************/
/**********************************************************************************************************/

void 	InitSonar();


/**********************************************************************************************************/
/* PROCEDURE :                  sonarReport()                           ***********************************/
/* Parameter :                  none                                    ***********************************/
/*                                                                      ***********************************/
/* Sends sonar report via TCX                                           ***********************************/
/*                                                                      ***********************************/
/**********************************************************************************************************/

void  	sonarReport();

extern int no_of_sonars;
extern float sonar_angle;
extern float sonar_range;
extern float sonar_infinity;
extern float sonar_offset;
extern float sonar_malfunc_rate;
extern int rays_per_sonar;


#ifdef __cplusplus
}
#endif
