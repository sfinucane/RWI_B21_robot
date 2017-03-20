
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/pantilt/pantiltClient.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:26:54 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: pantiltClient.h,v $
 * Revision 1.1  2002/09/14 15:26:54  rstone
 * *** empty log message ***
 *
 * Revision 1.5  1999/10/29 09:21:45  haehnel
 * comment strange ptConnected definition
 *
 * Revision 1.4  1998/01/14 19:38:35  swa
 * Changed the client library so that it re-connects to the server once
 * the pt server dies. Changed the server so that it stays alive once a
 * client dies and allow the client to reconnect.
 *
 * Revision 1.3  1997/07/29 22:44:42  thrun
 * Improved pantilt interface
 *
 * Revision 1.2  1997/03/11 17:16:40  tyson
 * added IR simulation and other work
 *
 * Revision 1.1  1997/02/25 18:12:43  tyson
 * client lib for PANTILT and lots of little stuff
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#ifndef PANTILT_CLIENT_H
#define PANTILT_CLIENT_H

/**** TCX module name - this is a unique identifier for TCX *******/
#define TCX_PANTILT_MODULE_NAME "PANTILT"

extern float ptStatusPanVel;
extern float ptStatusPanPos;
extern float ptStatusPanAccel;

extern float ptStatusTiltVel;
extern float ptStatusTiltPos;
extern float ptStatusTiltAccel;
extern int   ptConnected;

#ifdef __cplusplus
extern "C" {
#endif

  /* int ptConnected;		   1, if there is a connection to
				 * the server, 0 if not */

void ptMoveTo(float pan2, float tilt2);
void ptMoveBy(float pan2, float tilt2);
void ptSetVel(float speed);
void ptSetAccel(float accel);
void ptTrackPoint(float x, float y, float z);
void ptStopTracking(void);
void ptRegister();

int ptConnect(int wait_till_established); /* if parameter is 1, then
					   * this will wait until
					   * connection has been 
					   * established */

void ptTerminate(void);
void ptDisconnected(void);
void ptStatusCB(void *);

#ifdef __cplusplus
}
#endif

#endif /* PANTILT_CLIENT_H */
