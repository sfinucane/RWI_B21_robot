
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/ROBOT.c,v $
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
 * $Log: ROBOT.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:06  rhino
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





/*****************************************************************************
 * PROJECT: Rhino
 *
 * FILE: ROBOT.c
 *
 * ABSTRACT:
 * 
 * This file provides a set for routines for interfacing to the robot.
 *
 *****************************************************************************/

#ifdef VMS
#include "vms.h"
#include "vmstime.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#ifndef VMS
#include <sys/ioctl.h>
#endif
#include <sys/time.h>

#include "tcx.h"
#include "tcxP.h"
#include "Common.h"
#include "libc.h"
#include "ROBOT.h"

/*****************************************************************************
 *
 * FUNCTION: void signal_base(void);
 *
 * DESCRIPTION:
 *
 * Interrupt handler for the base.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/

void signal_base(void)
{
  fprintf(stderr,"shutting down the base\n");
  stop_robot();
}


/*****************************************************************************
 *
 * FUNCTION: BOOLEAN start_robot()
 *
 * DESCRIPTION:
 *
 * This routine starts base device on the robot.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/


BOOLEAN start_robot(void)
{
  BOOLEAN result=TRUE;

  
  /* do any device specific set up here. */
  
  /* Start up base device */
  result = BASE_init();

  /* connect up the signal handler. */
  if (!base_device.dev.use_simulator && !base_device.dev.use_rwi_server)
    base_device.dev.sigHnd = signal_base;
  
  return result;
}


void stop_robot()
{

  BASE_RotateHalt();
  BASE_TranslateHalt();
  
  fprintf(stderr, "Someone stopped the robot!\n");
}




