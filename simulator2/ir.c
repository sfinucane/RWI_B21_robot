
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** File:                        ir.c
 *****
 ***** Part of:                     BeeSoft
 *****
 ***** Creator:                     Tyson D Sawyer, Real World Interface
 ***** 
 ***** Date of creation:            Feb 28, 1997
 *****
 *****
 *****
 *****
 ***** $Source: /usr/local/cvs/bee/src/simulator2/ir.c,v $
 *****
 ***** $Revision: 1.1 $
 *****
 ***** $Date: 2002/09/14 16:11:07 $
 *****
 ***** $Author: rstone $
 *****
 *****
 *****
 ***** Contact tyson@rwii.com
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
 * $Log: ir.c,v $
 * Revision 1.1  2002/09/14 16:11:07  rstone
 * *** empty log message ***
 *
 * Revision 1.3  2000/03/09 09:30:07  schulz
 * Added Saschas multi-robot extension
 *
 * Revision 1.2  1997/03/14 17:21:47  tyson
 * Added tactile support to simulator
 *
 * Revision 1.1  1997/03/11 17:16:43  tyson
 * added IR simulation and other work
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <irClient.h>

#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <bUtils.h>

#include "sundefines.h"
#include "playground.hh"
#include "robot.h"
#include "ir.h"
#include "trigofkt.h"
#include "surface.hh"

#include "SIMULATOR-messages.h"

float base_coord_rot();

TCX_MODULE_PTR MODULE_IR = NULL;

float ir_dists[B_MAX_SENSOR_ROWS][B_MAX_SENSOR_COLS];

static struct IrVar {
  int irRunning;
  irType irs[B_MAX_SENSOR_ROWS][B_MAX_SENSOR_COLS];
} IrVariables;

#define IR_DEBUG


/**********************************************************************
 *
 * MSP type IR support - tds
 *
 **********************************************************************/

/*------------------------------------------------------------*/

/*
 * NOTE!
 * 
 * There are assumptions here about the sensor geometry and
 * the robot type.
 *
 */

static float
simIrWorldAngle(int irRow, int irCol)
{
  float angle = 0.0;
  float heading, brh;

  if (irRow >= bRobot.ir_rows) {
    return(0.0);
  }

  if (irCol >= bRobot.ir_cols[irRow]) {
    return(0.0);
  }

  heading = (base_coord_rot() + robot.robot_start_deg) * M_PI / 180.0;
  brh     = robot.base_BRH * M_PI / 180.0;

  switch (irRow) {
  case 0:
    angle = M_PI + M_PI_2 - heading -
      ((float)irCol+.5) * (2.0 * M_PI/(float)bRobot.ir_cols[irRow]);
    break;

  case 1:
    angle = M_PI + M_PI_2 - brh -
      ((float)irCol+0.7) * (2.0 * M_PI /(float)bRobot.ir_cols[irRow]);
    break;

  case 2:
    angle = M_PI + M_PI_2 - brh -
      ((float)irCol+0.05) * (2.0 * M_PI /(float)bRobot.ir_cols[irRow]);
    break;
  }

  while (angle>M_PI) {
    angle -= 2.0 * M_PI;
  }

  while (angle<-M_PI) {
    angle += 2.0 * M_PI;
  }

#if 0
  {
    static int count = 0;

    if (--count<0) {
      count=64;
      fprintf(stderr,
	      "%s:%6d:%s() - row=%3d col=%3d "
	      "head1=%f head=%f brh=%f angle=%f\n",
	      __FILE__, __LINE__, __FUNCTION__, irRow, irCol,
	      base_coord_rot(),
	      heading*180.0/M_PI, brh*180.0/M_PI, angle*180.0/M_PI);
    }
  }
#endif

  return(angle);
}

/*------------------------------------------------------------*/

void
mspIrStart(int value)
{
  extern void schedule_irServerReport(int delay);

#ifdef IR_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): %d\n",
	  __FILE__, __LINE__, __FUNCTION__, value);
#endif

  IrVariables.irRunning = value;
  if (value) {
    schedule_irServerReport(100);
  }
}

/*------------------------------------------------------------*/

void
irServer(char *message)
{
  /*
   * XXX - This is a total gross hack right now. -tds
   */

  mspIrStart(message[0]);
}

/*------------------------------------------------------------*/

/*
 * Get ir data and send ir messages
 */

int
irServerReport (void)
{
  int row;
  int col;
  int value_index= 0;
  int bufsize;
  long values[B_MAX_SENSOR_ROWS * B_MAX_SENSOR_COLS * 3];
  SIMULATOR_message_to_irServer_type message;

  int irNum, ir;
  unsigned long value;
  struct timeval time;
  float dist;

  static float GetDistance(int row, int col);

#if 0
  fprintf(stderr, "%10s:%5d:%14s()\n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  if (!IrVariables.irRunning) {
    return(0);
  }

  gettimeofday(&time, NULL);    /* get time first */

  /* make previous data old */
  for(row=0; row<bRobot.ir_rows; row++) {
    for(col=0; col<bRobot.ir_cols[row]; col++) {
      IrVariables.irs[row][col].mostRecent = FALSE;
    }
  }

  /* we pack the index and sonar value up for each new reading */
  /* and terminate it by a sonar index out of range, which client */
  /* code checks for */

  {
    static int count = 0;

    count = 10;

    if (!count) {
      fprintf(stderr, "%s:%6d:%s() - \n", __FILE__, __LINE__, __FUNCTION__);
    }

    for(row=0; row<bRobot.ir_rows; row++) {
      for(col=0; col<bRobot.ir_cols[row]; col++) {
	values[value_index++] = row;
	values[value_index++] = col;
	values[value_index++] = IrVariables.irs[row][col].value = 
	  GetDistance(row, col);
	IrVariables.irs[row][col].mostRecent = 1;
	if (!count) {
	  fprintf(stderr, "[%3d] ",
		  (int)IrVariables.irs[row][col].value);
	}
      }
      if (!count) {
	fprintf(stderr, "\n");
      }
    }

    if (--count < 0) count = 20;
  }

  values[value_index++] = -1;

  message.values = values;
  message.count  = value_index;

  /*
   * Do TCX message
   */

  if (MODULE_IR) {
    tcxSendMsg(MODULE_IR, "SIMULATOR_message_to_irServer",
	       &message);
  }

  return(1);
}

/*------------------------------------------------------------*/

void
InitIr()
{
  int ii, jj;

  for (jj=0; jj<B_MAX_SENSOR_ROWS; jj++) {
    for (ii=0; ii< B_MAX_SENSOR_COLS; ii++) {
      ir_dists[jj][ii] = 0.0;
    }
  }

  return;
}

/*------------------------------------------------------------*/

static float
GetDistance(int row, int col)
{
  float tdist, dist;
  float PosX, PosY, EndX, EndY;
  float rx, ry, rori;
  float worldAngle;
  float open_angle;
  float irZ;

  getRobotPosition(&rx,&ry,&rori);

  worldAngle = simIrWorldAngle(row, col);

  PosX = rx + bRobot.base_radius * cos(worldAngle);
  PosY = ry + bRobot.base_radius * sin(worldAngle);
  EndX = PosX + 50.0 * cos(worldAngle);
  EndY = PosY + 50.0 * sin(worldAngle);

  /*
   * open_angle determines the z-range which is hit depending on
   * the obstacles distance
   */

  open_angle = 5.0 * M_PI/180.0;
  irZ = 20.0;

  if (row > 1) {
    ir_dists[row][col] = dist = 30.0;
  }
  else if(get_distance(IR_SENSOR,
		       PosX, PosY, irZ,
		       open_angle,
		       EndX, EndY, &dist)) {
    ir_dists[row][col] = dist;
  }
  else {
    dist = ir_dists[row][col] = 0;
  }

  return(dist);
} /* GetDistance() */
