
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/colliClient.h,v $
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
 * $Log: colliClient.h,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1998/01/15 16:11:09  swa
 * I also changed colliClient to allow reconnects. This involved
 * re-declaring a function to be int colliConnect(int) and no longer void
 * colliConnect(void). Replace any occurence of colliConnect() in your
 * programs with colliConnect(1) and your done.
 *
 * Revision 1.2  1997/03/26 09:09:45  fox
 * Fixed a bug in colliApproachRelative. The two parameters give directions
 * relative to the robot and NOT x and y coordinates.
 *
 * Revision 1.1  1997/02/02 22:32:28  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

void colliSetMode(int mode);
void colliStopRobot(void);
void colliSetVelocity(float trans_speed, float rot_speed);
void colliSetAcceleration(float trans_accel, float rot_accel);
void colliApproachAbsolute(float x, float y,
			   float approachDist, int newTarget);

/* Direction to the target point is given by the sign of the parameters:
 * forward > 0  --> target point in front of the robot
 * sideward > 0 --> target point to the right of the robot */
void colliApproachRelative(float forward, float sideward,
			   float approachDist, int newTarget);

void colliRegister(void);

int colliConnect(int wait_till_established); /* if parameter is 1, then
					      * this will wait until
					      * connection has been 
					      * established */

int colliConnected;		/* 1, if there is a connection to
				 * the server, 0 if not    */

/*
 * The two point functions in COLLI seem to not work at this time.
 */

void colliApproachTwoPointsAbsolute(float x1, float y1, float x2, float y2,
				    float approachDist, int newTarget);
void colliApproachTwoPointsRelative(float x1, float y1, float x2, float y2,
				    float approachDist, int newTarget);
