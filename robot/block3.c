
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/robot/block3.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:55:27 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: block3.c,v $
 * Revision 1.1  2002/09/14 15:55:27  rstone
 * *** empty log message ***
 *
 * Revision 1.5  1999/04/18 19:00:14  fox
 * This is just to get the most recent version into the cmu bee tree. It shouldn't do any harm (hopefully!).
 *
 * Revision 1.4  1998/02/07 23:40:14  swa
 * now works with redhat5.0
 *
 * Revision 1.3  1998/01/13 23:55:59  thrun
 * .
 *
 * Revision 1.2  1997/10/24 19:06:34  tyson
 * fixed problems with X events and abus
 *
 * Revision 1.1.1.1  1996/09/22 16:46:12  rhino
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




#ifdef VMS
#include "vms.h"
#endif

#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <X11/Xlib.h>

#include "tcx.h"
#include "tcxP.h"
#include "global.h"
#include "EZX11.h"
#include "devUtils.h"


/* extern Display *theDisplay; */ /* get this from your EZX */

/* block till something happens
 * returns 1 if there is a waiting X11 event, 2 if there is a waiting TCX
 * event and 0 if there is no event 
 */
 

int block_wait(struct timeval *timeout, int tcx_initialized,
	       int X_initialized)
{
  extern void check_redraw_screen();
  XEvent ev;

/*  extern fd_set tcxConnectionListGlobal; */
  int stat, xfd = 0;
  fd_set readMask;
  
  if (tcx_initialized)
    readMask = (Global->tcxConnectionListGlobal);/* current tcx socket list */
  else
    FD_ZERO(&readMask);

  if (X_initialized){
    xfd = XConnectionNumber(theDisplay);
    FD_SET(xfd,&readMask);
  }
      
  stat = select(FD_SETSIZE, &readMask, NULL, NULL, timeout);

  if (stat > 0) 
    {
      if (!X_initialized)
	return 2;
#ifndef UNIBONN
      else if (FD_ISSET(xfd,&readMask)) {

        /* Do we need to grab the focus? */
        if (XCheckMaskEvent(theDisplay, EnterWindowMask, &ev)) {
          XSetInputFocus(theDisplay, ev.xcrossing.window, 
                         RevertToPointerRoot, CurrentTime);
        }
	
        /* Do we need to issue a redraw? */
        check_redraw_screen(theDisplay);

	return 1;
      }
      else
	return 2;
#endif
    }
  else return 0;
}


int block_wait_with_serial_ports(struct timeval *timeout, int tcx_initialized,
				 int X_initialized)
{
  int i;
  extern int maxDevNum;
  /*  extern fd_set tcxConnectionListGlobal; */
  int stat, xfd = 0;
  fd_set readMask;
  struct timeval TCX_waiting_time = {0, 0};

  if (tcx_initialized)
    readMask = (Global->tcxConnectionListGlobal);/* current tcx socket list */
  else
    FD_ZERO(&readMask);

  if (X_initialized){
    xfd = XConnectionNumber(theDisplay);
    FD_SET(xfd,&readMask);
  }

  for (i = 0; i < maxDevNum; i++) /* add all devices */
    if (devices[i] != NULL)
      FD_SET(i, &readMask);

  stat = select(FD_SETSIZE, &readMask, NULL, NULL, timeout);

  tcxRecvLoop((void *)(&TCX_waiting_time));

}
