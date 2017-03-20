/*
 * Copyright 1994, Brown University, Providence, RI
 *
 * Permission to use and modify this software and its documentation for
 * any purpose other than its incorporation into a commercial product is
 * hereby granted without fee.  Permission to copy and distribute this
 * software and its documentation only for non-commercial use is also
 * granted without fee, provided, however, that the above copyright notice
 * appear in all copies, that both that copyright notice and this permission
 * notice appear in supporting documentation, that the name of Brown
 * University not be used in advertising or publicity pertaining to
 * distribution of the software without specific, written prior permission,
 * and that the person doing the distribution notify Brown University of
 * such distributions outside of his or her organization. Brown University
 * makes no representations about the suitability of this software for
 * any purpose.  It is provided "as is" without express or implied warranty.
 * Brown University requests notification of any modifications to this
 * software or its documentation.
 *
 * Send the following redistribution information:
 *
 *	Name:
 *	Organization:
 *	Address (postal and/or electronic):
 *
 * To:
 *	Software Librarian
 *	Computer Science Department, Box 1910
 *	Brown University
 *	Providence, RI 02912
 *
 *		or
 *
 *	brusd@cs.brown.edu
 *
 * We will acknowledge all electronic notifications.
 */

#include <rai.h>
#include <mspmodule.h>
#include <utils.h>

#include <string.h>


#define PRINT_INTERVAL  2000
#define EXECUTION_TIME 1000000

int irMins[B_MAX_SENSOR_ROWS][B_MAX_SENSOR_COLS];
int irMaxes[B_MAX_SENSOR_ROWS][B_MAX_SENSOR_COLS];

int printCount = 0;

int irCallback(irType **irMatrix)
{
  int index,row;
  int value;

  for(row = 0; row < bRobot.ir_rows ;row++)
    {
      for(index = 0; index < bRobot.ir_cols[row]; index++)
	{
	  value= irMatrix[row][index].value;
	  if (value < irMins[row][index])
	    irMins[row][index]=value;	    
	  if (value > irMaxes[row][index])
	    irMaxes[row][index]=value;	    
	}
    }
  return 0;
}

/*************************************************

  Printing the tables

 *************************************************/

void printIrs(RaiModule* printer)
{
  int index,row;
  int value;

  if (printCount<3) {
    printCount++;
    for(row = 0; row < bRobot.ir_rows; row++) {
      for(index = 0; index < bRobot.ir_cols[row]; index++) {
	irMins[row][index]=0xFF;
	irMaxes[row][index]=0;
      }
    }
  } else {
    for(row = 0; row < bRobot.ir_rows; row++) {
      fprintf(stderr, "Row %d: ",row);
      for(index = 0; index < bRobot.ir_cols[row]; index++) {
	value= irs[row][index].value;
	if (irMaxes[row][index]>irMins[row][index]) {
	  value = (value-irMins[row][index])*99;
	  value /= irMaxes[row][index]-irMins[row][index];
	}
	fprintf(stderr, "%02d ", value);
      }
      fprintf(stderr, "\n");
    }
    fprintf(stderr, "\n");
  }
}

void printerInit() {
  RaiModule *printer;
  printer = makeModule("Printer",NULL);
  addPolling(printer,printIrs,PRINT_INTERVAL);
}




/*************************************************/
/*
  Know when to say when
*/
/*************************************************/

void startShutdown(RaiModule* reaper)
{
 printf("Done with calibration\n");
 RaiShutdown();
}

void irDone(RaiModule* reaper) {
/*
 * row numInRow max/min val val val ...
 *               02/01
 *
 * all numbers formated: %02X
 */

  int index,row;
  int value;

  for(row = 0; row < bRobot.ir_rows; row++) {

    fprintf(stdout, "%X %02X 1 ",row, bRobot.ir_cols[row]);
    for(index = 0; index < bRobot.ir_cols[row]; index++) {
      value = irMins[row][index];
      /* _HACK_ The IRs next to motors on the B21 don't work */
      if (row == 1 &&((index==4)||(index==10)||(index==16)||(index==22))) {
	fprintf(stdout, "000 ");
      }
      else {
	fprintf(stdout, "%02X ", value);
      }
    }
    fprintf(stdout, "\n");

    fprintf(stdout, "%X %02X 2 ",row, bRobot.ir_cols[row]);
    for(index = 0; index < bRobot.ir_cols[row]; index++) {
      value = irMaxes[row][index];
      /* _HACK_ The IRs next to motors on the B21 don't work */
      if (row == 1 &&((index==4)||(index==10)||(index==16)||(index==22))) {
	value = 0xFFF;
      }
      fprintf(stdout, "%02X ", value);
    }
    fprintf(stdout, "\n\n");

  }
}

void reaperInit() {
  RaiModule *reaper;
  reaper = makeModule("Reaper", irDone);
/*  addTimeout(reaper,startShutdown,EXECUTION_TIME); */
}

int main(int argc, char *argv[])
{
  int index,row;
  struct bParamList * params = NULL;

  /* add some parameter files */
  params = bParametersAddFile(params, "etc/beeSoft.ini");

  /* Fill the global parameter structure */
  bParametersFillParams(params);

  argv0 = argv[0];

  for(row = 0; row < bRobot.ir_rows; row++) {
    for(index = 0; index < bRobot.ir_cols[row]; index++) {
	irMins[row][index]=0xFF;
	irMaxes[row][index]=0;
    }
  }

  RaiInit();
  catchInterrupts();

  irInit();
  tactileInit();

  registerIrCallback(irCallback);

  fprintf(stderr, "Starting Rai scheduler\n");
  fprintf(stderr, "\nShould report ir limits every 2 seconds\n");

  printerInit();
  reaperInit();
  normalizeIr = 0;
  RaiStart();
  
  /* We shouldn't get to here */
  return 1;
}
