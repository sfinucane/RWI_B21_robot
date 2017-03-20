
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/utils/itrig.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:50:46 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: itrig.c,v $
 * Revision 1.1  2002/09/14 15:50:46  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:21  rhino
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

/*****************************************************
*
*  This isn't a module, but is rather just a library
*  for doing integer triginometry.
*
*  James Kurien
*
******************************************************/

#include <limits.h>
#include <itrig.h>

#ifndef M_PI
#define M_PI PI
#endif

#define TRUE 1
#define FALSE !TRUE


/* we multiply our values by this when putting them in the table */
/* then divide out when we give an answer, letting us store      */
/* non-integral values. This only makes sense because usually we */
/* are multiplying the table value by a large scalar when we     */
/* retreive it */

#define ITRIG_INT_SCALE   256

int isin_table[ITRIG_UNITS_PER_CIRCLE];
int icos_table[ITRIG_UNITS_PER_CIRCLE];
int itan_table[ITRIG_UNITS_PER_CIRCLE];

/* just so we do not waste time initializing twice */ 
int itrig_initialized = FALSE;

/* This will not work for tan because it is undefine some */
/* places.  We do tan by itself below */

void init_table(int* table, int size,double(*function)(double))
{
  int i,int_val;
  double func_val,arg;

  /* This is pretty verbose just to make sure all the implicit*/
  /* type conversion gets done */

  for(i=0;i<size;i++)
    {
      arg = (2*M_PI*i)/size;
      func_val = function(arg); /* almost by definition :-) */
      int_val  = func_val * ITRIG_INT_SCALE;
      table[i] = int_val;
    }
}


void init_tan(int* table, int size)
{
  int i,int_val;
  double func_val,arg;

  /* This is pretty verbose just to make sure all the implicit*/
  /* type conversion gets done */

  for(i=0;i<size;i++)
    {
      arg = (2*M_PI*i)/size;
      if ((i == size/4) || i == (3*size/4))
	func_val = INT_MAX / size;
      else
	func_val = tan(arg); 
      int_val  = func_val * ITRIG_INT_SCALE;
      table[i] = int_val;
    }
}


void ITrigInit(void)
{
  if (!itrig_initialized)
    {
      init_table(isin_table,ITRIG_UNITS_PER_CIRCLE,sin);
      init_table(icos_table,ITRIG_UNITS_PER_CIRCLE,cos);
      init_tan(itan_table,ITRIG_UNITS_PER_CIRCLE);
      itrig_initialized = TRUE;
    }
}


/* NOTE:  % is bogus if angle is negative and more than ITRIG_UNITS_PER_CIRCLE */

inline int iCos(int angle,int scalar)
{
  return 
    (scalar * icos_table[(angle) % ITRIG_UNITS_PER_CIRCLE]) / ITRIG_INT_SCALE;
}


inline int iSin(int angle,int scalar)
{
  return 
    (scalar * isin_table[(angle) % ITRIG_UNITS_PER_CIRCLE]) / ITRIG_INT_SCALE;
}

inline int iTan(int angle,int scalar)
{
  return
    (scalar * itan_table[(angle) % ITRIG_UNITS_PER_CIRCLE]) / ITRIG_INT_SCALE;
}



