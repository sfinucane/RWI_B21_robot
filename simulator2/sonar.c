
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** File:                        sonar.c
 *****
 ***** Part of:                     RHINO SOFTWARE
 *****
 ***** Creator:                     Peter Wallossek, University of Bonn
 *****                              adapted to simulatorII by Dirk Schulz
 ***** Date of creation:            Jan 1994
 *****
 *****
 *****
 *****
 ***** $Source: /usr/local/cvs/bee/src/simulator2/sonar.c,v $
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
 * $Log: sonar.c,v $
 * Revision 1.1  2002/09/14 16:11:07  rstone
 * *** empty log message ***
 *
 * Revision 1.32  2000/03/09 09:30:10  schulz
 * Added Saschas multi-robot extension
 *
 * Revision 1.31  1999/09/30 11:57:12  wolfram
 * Changed name of round( in sonar.c to fRound so that it compiles under SuSe
 *
 * Revision 1.30  1997/12/11 14:51:41  schulz
 * added missing case in inhalfplane()
 *
 * Revision 1.29  1997/07/17 17:31:52  tyson
 * Added wiggle, square, Changes, fixed bugs, added features.  See the new Changes file for details
 *
 * Revision 1.28  1997/05/25 10:30:55  thrun
 * okay, I try my very best to get this sonar thing correct....
 *
 * Revision 1.27  1997/04/27 11:12:19  thrun
 * Fixed an inconsistency: Sonar measurements used to be measured from the
 * center of the robot in the simularo, but the robot returned the raw
 * readings. This is now fixed. Both return the raw reading.
 *
 * Revision 1.26  1997/03/11 17:16:44  tyson
 * added IR simulation and other work
 *
 * Revision 1.25  1997/02/02 22:32:44  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
 *
 * Revision 1.24  1997/01/29 14:20:12  schulz
 * sonar simulation uses only one beam now, which is chosen at random out of
 * rays_per_sonar positions.
 *
 * Revision 1.23  1997/01/28 15:28:22  wolfram
 * Back to the old randMax
 *
 * Revision 1.22  1997/01/27 15:13:30  schulz
 * added added multiple sensor planes to the sensor simulation stuff
 * I hope this does not introduce to many new bugs.
 *
 * Revision 1.21  1997/01/24 14:22:41  wolfram
 * MaxRange error now also correct under Linux
 *
 * Revision 1.20  1997/01/06 13:58:07  fox
 * Fixed a bug in sonar distance.
 *
 * Revision 1.19  1997/01/06 10:51:52  schulz
 * -- (0,0) is really in the lower left corner now!
 * -- sonar simulation works again.
 *
 * Revision 1.18  1996/12/23 11:06:44  fox
 * Remove some output.
 *
 * Revision 1.17  1996/12/19 15:37:45  schulz
 * -- adjusted default sonar_range to 650
 * -- introduced sonar_infinity as 3610.777832
 *
 * Revision 1.16  1996/12/13 16:56:55  schulz
 * changed default value for sonar_malfunc_rate to 0.02
 *
 * Revision 1.15  1996/12/06 00:14:02  tyson
 * simulator<->baseServer work
 *
 * Revision 1.14  1996/12/04 05:42:30  tyson
 * simulator2 <-> baseServer partially tested.  Cleaning up bumble and sensor tests as demos.  Added frameWork module.
 *
 * Revision 1.13  1996/12/03 15:18:37  schulz
 * -- some fixes
 * -- added humans as obstacles (still buggy)
 *
 * Revision 1.12  1996/12/02 17:02:01  schulz
 * new user interface for obstacle insertion
 * some fixes
 *
 * Revision 1.11  1996/11/25 21:06:33  tyson
 * Simulator ready for baseServer?  ...time to update baseServer.
 *
 * Revision 1.10  1996/11/18 14:43:13  ws
 * default ROBOT_RADIUS is taken from rai/B21/Base.h now.
 * Played around with job priorities a little bit.          DS
 *
 * Revision 1.9  1996/11/18 04:34:59  tyson
 * More baseServer<->simulator2 work. Still not done.
 *
 * Revision 1.8  1996/11/14 13:43:33  ws
 * Obstacles are solid now!             DS
 *
 * Revision 1.7  1996/11/05 22:43:27  ws
 * Some minor defines for SUN OS 4.1
 *
 * Revision 1.6  1996/10/25 14:27:12  ws
 * some improvements
 * - better options handling
 * - added an initialization file, many parameters are modifiable now
 * - Modified some timings to reduce CPU load
 *   (TCX is polled after every 50 ms now)
 *
 * Revision 1.5  1996/10/23 08:57:17  ws
 * eliminated cross file references to struct robot
 *
 * Revision 1.4  1996/10/14 12:19:11  ws
 * added support for different surfaces (not yet complete) and sonar error
 *
 * Revision 1.3  1996/10/09 15:23:51  fox
 * Laser report is sent independently from the sonar report.
 *
 * Revision 1.2  1996/10/04 11:35:24  ws
 * some bug fixes in zooming and centering the robot
 *
 * Revision 1.1.1.1  1996/09/30 16:17:45  schulz
 * reimport on new source tree
 *
 * Revision 1.5  1996/09/11 14:03:50  schulz
 * - Speed up sonar and laser simulation by a factor of 20.
 *   We precalculate the set of obstacles which can be hit by a
 *   laser/sonar beam and check only for these few obstacles, if they are hit.
 *   Switched to S.Thruns version of line and circle intersection routines,
 *   these are a little bit faster then the old ones.
 * - Included laser visualisation  (to slow an some machines)
 *
 * Revision 1.4  1996/08/27 15:15:21  schulz
 * Many bug fixes...
 * Installed File Headers, changed c++ file names to .cc/.hh file suffix
 * There is still a great performance problem!
 *
 * Revision 1.3  1996/08/27 08:12:04  schulz
 * Changed Filename suffix to .cc for c++ files
 *
 * Revision 1.2  1996/08/26 10:05:33  schulz
 * added laser interface (without visualization, yet) D.S.
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <sonarClient.h>

#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "sundefines.h"
#include "playground.hh"
#include "robot.h"
#include "sonar.h"
#include "trigofkt.h"
#include "surface.hh"

#define ctoi(x)         ((x) - '0')



#include "SIMULATOR-messages.h"

TCX_MODULE_PTR MODULE_SONAR = NULL;


float sonar_angle = SONAR_ANGLE;
float sonar_range = SONAR_RANGE;
float sonar_infinity = SONAR_INFINITY;
float sonar_offset = SONAR_OFFSET;
float sonar_zpos = 80.0;
float sonar_malfunc_rate = 0.02;
int rays_per_sonar = RAYS_PER_SONAR;

static float sonar_readings[B_MAX_SENSOR_COLS];
float sonar_dists[B_MAX_SENSOR_COLS];


static struct SonarVar {
  long    CP;             /*** Chain Power          ***/
  char    RT[5];          /*** Read Transducers     ***/

  /*
   * MSP type sonar variables
   */

  int sonarRunning;
  int lastTactileValues;
  sonarType sonars[B_MAX_SENSOR_COLS];

} SonarVariables;

static char report[NO_CHAINS][81], commandBack[81];

/*
#define SONARDEBUG
*/



/**********************************************************************************************************/
/* PROCEDURE :			sonar()      				***********************************/
/* Parameter :			command line				***********************************/
/* 									***********************************/
/* scans command line and performs sonar action				***********************************/
/* 									***********************************/
/**********************************************************************************************************/

void
sonar(char command[])
{
   int	i, j, k;
   long	ticks;
   float dist = 0, maxdist = 0.0;
   char	s[5], buf[81], buf2[81];
   int	sonar_no[NO_CHAINS];

   static int VsonarToRsonar[] = {
   	0,1,2,3,4,5,
   	6,7,8,9,10,11,
   	12,13,14,15,16,17,
   	18,19,20,21,22,23
   };
   
   static int RsonarToVsonar[] = {
   	0,1,2,3,4,5,
   	6,7,8,9,10,11,
   	12,13,14,15,16,17,
   	18,19,20,21,22,23
   };  

   struct timeval       before, after;
   int			time_lost;
   
   float GetDistance(int cell);

#ifdef SONARDEBUG
   fprintf(stderr, "SONAR got command : %s\n", command);
#endif
   
   if (strncmp(command, "RT", 2) == 0) {		/*** Read Transducers ***/
     
     if (SonarVariables.CP == 0) {			/*** Chain Power ? ***/
       fprintf(stderr, "Tried to Read Transducers while Chain Power is OFF !!!\n");
       return;
     }

     gettimeofday(&before, NULL);		/*** used to calculate amount of time lost when ***/
     						/*** caclulating sonar responases 		***/
     sscanf(command,"RT %s", buf);

     strcpy(commandBack, command);

     *buf2='\0';					/*** leading zeros !! 	    ***/
     while (strlen(buf)+strlen(buf2) < NO_CHAINS)	/*** important when not all ***/
     	strcat(buf2, "0");				/*** chains are on          ***/

     strcat(buf2, buf);

     for (i=0; i<NO_CHAINS; i++) {
	
       j = ctoi(buf2[i]);

       if (j != 0) {
	 dist = GetDistance(VsonarToRsonar[j + (NO_CHAINS-i-1)*6 - 1]);
	 if (dist > maxdist)
		maxdist = dist;
	 ticks = (long) (dist * 2 / CM_PER_SECOND / SECONDS_PER_CYCLE);
     
	 sprintf(report[i], "%4x", ticks);
	 for (k = 0; k < (int)strlen(report[i]); k++)
		if (isspace(report[i][k]))
			report[i][k] = '0';

	 strcat(report[i], " ");
	 if (strlen(report[i]) > 5) {
	   strcpy(report[i], "FFFF ");	   
	 }
       } else 
	   strcpy(report[i], "");
       
     } /* for */		
     gettimeofday(&after, NULL);		/*** used to calculate amount of time lost when ***/
     						/*** caclulating sonar responases 		***/
     
     time_lost = (after.tv_usec - before.tv_usec) / 1000;		/*** ms ***/
     if (time_lost < 0)
     	time_lost = 1000 + time_lost;
#ifdef DEBUG

#endif
     schedule_sonarReport(SONAR_ANSWER_TIME - time_lost);
   } else if (strncmp(command, "CP", 2) == 0) {		/*** Chain Power ***/
     /*** immer ein ***/
     SonarVariables.CP = 1;
   }
   
   else
     fprintf(stderr, "Unknown base command sent to sonar (simulator):  %s !\n", command);
   return;
   
} /* sonar() */

/**********************************************************************
 *
 * MSP type sonar support - tds
 *
 **********************************************************************/

mspSonarStart(int value) {
  extern void schedule_sonarServerReport(int delay);

#ifdef SONARDEBUG
  fprintf(stderr, "%10s:%5d:%14s(): %d\n",
	  __FILE__, __LINE__, __FUNCTION__, value);
#endif

  SonarVariables.sonarRunning = value;
  if (value) {
    schedule_sonarServerReport(100);
  }
}

void
sonarServer(char *message) {
  extern void mspIrStart(int);

  /*
   * XXX - This is a total gross hack right now. -tds
   */

  mspSonarStart(message[0]);
}

/*
 * Get sonar data and send sonar messages
 */

int
sonarServerReport (void) {
  static int setNum = 0;
  int sonarNum, sonar;
  unsigned long value;
  struct timeval time;
  float dist;

  float GetDistance(int cell);

  if (!SonarVariables.sonarRunning) {
    return(0);
  }

  gettimeofday(&time, NULL);    /* get time first */

  /* make previous data old */
  for (sonar = 0; sonar < bRobot.sonar_cols[0]; sonar++) {
    SonarVariables.sonars[sonar].mostRecent = FALSE;
  }
  
  /* iterate through new data */
  sonar = 0;

  for (sonar=0; sonar<6; sonar++) {
    sonarNum = setNum + sonar*4;
    memcpy(&(SonarVariables.sonars[sonarNum].time), &time,
	   sizeof(SonarVariables.sonars[sonarNum].time));
    SonarVariables.sonars[sonarNum].mostRecent = TRUE;

    dist = GetDistance((sonarNum+(((int) bRobot.sonar_cols[0])/2))%
		       bRobot.sonar_cols[0]);

    value = (unsigned long)(dist * 10);
    value = value + (10.0 * bRobot.enclosure_radius);
  
    if (value > NO_RETURN) {
      value = NO_RETURN;
    }

    SonarVariables.sonars[sonarNum].value = value;
  }

  setNum = (++setNum) & 0x03;

  /* call callback */
  /* if (moduleOps.sonarRep) return(moduleOps.sonarRep(sonars)); */

  /*
   * Do TCX message
   */

#ifdef SONARDEBUG
  fprintf(stderr, "%10s:%5d:%14s(): \n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif
  
  tcxSendMsg(MODULE_SONAR, "SIMULATOR_message_to_sonarServer",
	     SonarVariables.sonars);

  return(1);
}


/**********************************************************************************************************/
/* PROCEDURE :			InitSonar()    				***********************************/
/* Parameter :			none					***********************************/
/* 									***********************************/
/* obvious functionality						***********************************/
/* 									***********************************/
/**********************************************************************************************************/


void
InitSonar()
{
  int	i;


  for (i = 0; i < bRobot.sonar_cols[0]; i++)
	sonar_readings[i] = 0.0;

  return;

} /* InitSonar() */




/**********************************************************************************************************/
/* PROCEDURE :			GetDistance()      			***********************************/
/* Parameter :			No. of cell				***********************************/
/* 									***********************************/
/* Pass through list of obstacles and find the closest one inside range ***********************************/
/* Check angle and surface and decide if obstacle is recognized		***********************************/
/* 									***********************************/
/**********************************************************************************************************/

static int
fRound( float x){
  return (int) x + 0.5;
}

static int
randMax(int max)
{
  return (int) (((float) (max + 1) * rand()) / (RAND_MAX + 1.0));
}


float
GetDistance(int cell)
{
  float tdist,dist;
  float PosX, PosY,EndX,EndY;
  float rx,ry,rori;
  float f,deg,rad;
  float open_angle;
  float sonarAngleDistance = 360.0 / bRobot.sonar_cols[0];

  getRobotPosition(&rx,&ry,&rori);

  deg = ( rori - sonar_offset - cell * sonarAngleDistance);

  if (sonar_malfunc_rate > 0)
    if (randMax(fRound(1/sonar_malfunc_rate)) == 0){
      sonar_dists[cell] = sonar_infinity;
      return sonar_infinity;
    }
  tdist = dist = sonar_infinity;

/* we test only one beam at random */ 
  f = -sonar_angle/2;
  f += sonar_angle/rays_per_sonar * randMax(rays_per_sonar);

  rad = (deg + f) * M_PI /180;
  PosX = rx + robot.RobotRadius * cos(rad);
  PosY = ry + robot.RobotRadius * sin(rad);
  EndX = PosX + sonar_range * cos(rad);
  EndY = PosY + sonar_range * sin(rad);
    /*  open_angle determines the z-range which is hit depending on */ 
  /* the obstacles distance */
  open_angle = sqrt(sonar_angle*sonar_angle/4 - f*f) * M_PI/180.0;
  if(get_distance(SONAR_SENSOR,
		  PosX, PosY, sonar_zpos,
		  open_angle,
		  EndX, EndY, &dist)) {
    
    sonar_dists[cell] = dist;
  }
  else
    sonar_dists[cell] = sonar_infinity;
  return dist;
} /* GetDistance() */



/**********************************************************************************************************/
/* PROCEDURE :			sonarReport()      			***********************************/
/* Parameter :			none					***********************************/
/* 									***********************************/
/* Sends sonar report via TCX						***********************************/
/* 									***********************************/
/**********************************************************************************************************/


void
sonarReport()
{
    int i;

#ifdef SONARDEBUG
    fprintf(stderr, "SIMULATOR :   SENDING  SONAR Report :  %s\n", report);
#endif

    if (MODULE_SONAR != NULL){
    	char *reply_msg;

    	reply_msg = (char *) malloc(256);
    	strcpy(reply_msg, commandBack);

#ifdef SONARDEBUG
	fprintf(stderr, "%10s:%5d:%14s(): \n",
		__FILE__, __LINE__, __FUNCTION__);
#endif
    	tcxSendMsg(MODULE_SONAR, "SIMULATOR_message_to_sonar", &reply_msg);

	for (i = 0; i < NO_CHAINS; i++) {
    	   strcpy(reply_msg, report[i]);
	   if (strcmp(reply_msg, "") != 0) {
    	   	tcxSendMsg(MODULE_SONAR, "SIMULATOR_message_to_sonar", &reply_msg);
	   }
        }

    	free (reply_msg);
    }  

    return;

} /* sonarReport() */
