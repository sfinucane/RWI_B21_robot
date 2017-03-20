
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/activeLocalize.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:39 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: activeLocalize.h,v $
 * Revision 1.1  2002/09/14 20:45:39  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1997/06/20 07:36:08  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.2  1997/03/13 17:36:34  fox
 * Temporary version. Don't use!
 *
 * Revision 1.1  1997/01/29 12:22:59  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.11  1997/01/07 13:14:28  wolfram
 * Movement decisions are made on the basis of a list of real positions
 *
 * Revision 1.10  1996/12/31 09:19:22  fox
 * Last version before I switched exploration mode in PLAN to random
 * mode in COLLI.
 *
 * Revision 1.9  1996/12/20 15:29:37  fox
 * Added four parameters.
 *
 * Revision 1.8  1996/12/03 12:27:39  fox
 * Seems to work with the thin corridor.
 *
 * Revision 1.7  1996/12/02 10:32:03  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.6  1996/11/28 12:40:29  wolfram
 * Wolframs version.
 *
 * Revision 1.5  1996/11/27 15:51:16  fox
 * For Woflram.
 *
 * Revision 1.4  1996/11/27 12:18:17  fox
 * Nothing special.
 *
 * Revision 1.3  1996/11/26 11:08:11  fox
 * Improved version.
 *
 * Revision 1.2  1996/11/25 19:35:40  fox
 * Test version for decisions of movements.
 *
 * Revision 1.1  1996/11/22 17:22:52  wolfram
 * Initial version by Dieter Fox
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#ifndef ACTIVE_LOCALIZE_INCLUDE
#define ACTIVE_LOCALIZE_INCLUDE

#include "general.h"
#include "sensings.h"

#define MAX_NUMBER_OF_SENSORS 400

void
initialize_ACTIVE( char* fileName, actionInformation* actionInfo,
		   sensingActionMask* actionMask);

void
performActions_ACTIVE( actionInformation* info, sensingActionMask* mask);

void
computeBestTarget( abstractSensorVector* abstractSensors,
		   realCellList* maxPositions,
		   positionProbabilityGrid* grid,
		   probabilityGrid* map);

void
startActiveLocalization();

void
stopActiveLocalization();

#endif
