
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/selection.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 17:06:29 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: selection.h,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1998/08/19 16:33:59  fox
 * Minor changes. Position is dumped upon exit of tcxServer.
 *
 * Revision 1.2  1998/06/12 10:16:45  fox
 * Implemented virutal sensor.
 *
 * Revision 1.1  1997/11/25 17:13:06  fox
 * Should work.
 *
 * Revision 1.4  1997/10/31 13:11:42  fox
 * Version for active sensing.
 *
 * Revision 1.3  1997/09/09 19:45:12  fox
 * Fixed a bug in finding local maxima and did several changes to active
 * localization.
 *
 * Revision 1.2  1997/08/22 04:16:38  fox
 * Final version before IJCAI.
 *
 * Revision 1.1  1997/05/26 09:34:13  fox
 * Replaced entropy by information.
 *
 * Revision 1.5  1997/04/30 12:25:39  fox
 * Some minor changes.
 *
 * Revision 1.4  1997/03/18 18:45:29  fox
 * First executable version with a global scan mask for different sensor types.
 *
 * Revision 1.3  1997/03/17 18:41:13  fox
 * First version with sensor selection in cluttered environments.
 *
 * Revision 1.2  1997/03/13 17:36:35  fox
 * Temporary version. Don't use!
 *
 * Revision 1.1  1997/01/29 12:36:22  fox
 * New version.
 *
 * Revision 1.8  1997/01/18 19:41:02  fox
 * Improved action selection.
 *
 * Revision 1.7  1996/12/03 12:27:39  fox
 * Seems to work with the thin corridor.
 *
 * Revision 1.6  1996/12/02 10:32:01  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.5  1996/11/28 17:56:20  fox
 * *** empty log message ***
 *
 * Revision 1.4  1996/11/25 19:35:39  fox
 * Test version for decisions of movements.
 *
 * Revision 1.3  1996/11/22 16:33:45  fox
 * First version.
 *
 * Revision 1.2  1996/11/21 13:41:55  fox
 * First version to show the main structure to find the best movement of the
 * robot.
 *
 * Revision 1.1  1996/11/21 12:40:24  fox
 * Tools for bayesian reasoning on the grids.
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#ifndef SELECTION_INCLUDE
#define SELECTION_INCLUDE

#include "general.h"
#include "activeLocalize.h"
#include "probGridTools.h"

#define CONSIDER_INACTIVE 1
#define DONT_CONSIDER_INACTIVE 0

double
computeExpectedEntropyDiff( realCellList* cellList,
			    abstractSensorVector* sensors,
			    positionProbabilityGrid* grid,
			    probabilityGrid* occupancyProbs,
			    double* entropies,
			    bool considerInactive);
void
computeEntropyDiff( realCellList* cellList,
		    abstractSensorVector* sensors,
		    positionProbabilityGrid* grid,
		    probabilityGrid* occupancyProbs,
		    double* entropies,
		    bool considerInactive);

void
computeProbOfUnexpected( realCellList* cellList,
			 abstractSensorVector* sensors,
			 positionProbabilityGrid* grid,
			 double* probOfUnexpected,
			 int mode,
			 float threshold);
#endif     




