
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
 ***** Source file:     $Source: /home/rhino/rhino/CVS/bee/src/msp/mspmodule.h,v $
 *****
 ***** Created by:      $Author: tyson $
 *****
 ***** Revision #:      $Revision: 1.4 $
 *****
 ***** Date of revision $Date: 1997/07/17 17:31:50 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: mspmodule.h,v $
 * Revision 1.4  1997/07/17 17:31:50  tyson
 * Added wiggle, square, Changes, fixed bugs, added features.  See the new Changes file for details
 *
 * Revision 1.3  1997/03/25 21:44:46  tyson
 * Many bug fixes.
 *
 * Revision 1.2  1996/11/18 04:34:57  tyson
 * More baseServer<->simulator2 work. Still not done.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:08  rhino
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
 * Portions of this code were created by and are
 * copyrighted by Real World Interface Inc. (RWI)
 *
 * The creation and continued develoment of this software
 * is sponsored and directed by RWI in the interest of
 * providing the mobile robotics and AI research communities
 * with a well designed and robust Robot Applications
 * Interface (RAI) for the complete line of RWI mobile robots. 
 *
 *   ==Contact  support@rwii.com  for further information==
 */

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

#ifndef MSP_MODULE_H
#define MSP_MODULE_H

#include <time.h>
#include <bUtils.h>
#include <sonarClient.h>     /* The generic APIs for the sensors we support*/
#include <tactileClient.h>
#include <irClient.h>
#include <acb/abus.h>
#include <msp.h>

/*
 * This MSP RAI device module supports only one client.
 * It perhaps should support the possibility of more
 * than one fd.  That can come later.
 *
 * This modules abstracts the multiple MSPs with different
 * sensor types into linear arrays of sensors.  The
 * setting of the tables to translate the sensors
 * of each MSP to the linear array for each device 
 * type will be a topic of continued research.
 * One goal is to be as independent if MSPs as reasonable.
 */

#define MAX_SONAR_CALLBACKS 1

#define ABUS_FILE "/dev/abus"

#define  NUM_BASE_MSPS     (4)
#define  NUM_ENCL_MSPS     (3)
#define  NUM_MSPS          (NUM_BASE_MSPS + NUM_ENCL_MSPS)

#define  CHAINS_PER_MSP    (2)
#define  SONARS_PER_CHAIN  (4)
#define  SONARS_PER_MSP    (CHAINS_PER_MSP * SONARS_PER_CHAIN)

/* More  IR, Sonar, Tactile info defined on a per machine basis in Robot.h */

#define SONARS_PER_SET 6
#define NUM_SETS 4		/* How many sets for full table */
#define BOGUS_THRESHOLD 0
#define NO_RETURN 0x7FFF
#define MM_PER_CLICK 0.568

extern int normalizeIr;

#endif MSP_MODULE_H
