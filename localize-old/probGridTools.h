
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/probGridTools.h,v $
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
 * $Log: probGridTools.h,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.6  1997/06/27 16:26:29  fox
 * New model of the proximity sensors.
 *
 * Revision 1.5  1997/06/20 07:36:13  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.4  1997/04/24 21:25:44  wolfram
 * Ooops, fourth parameter of CUBE's is radius and not diameter!
 *
 * Revision 1.3  1997/04/07 11:03:23  fox
 * Should be ok.
 *
 * Revision 1.2  1997/03/17 18:41:14  fox
 * First version with sensor selection in cluttered environments.
 *
 * Revision 1.1  1997/01/29 12:31:51  fox
 * New from exploreTools.
 *
 * Revision 1.16  1997/01/19 14:05:26  wolfram
 * Added counter for number of integrated readings
 *
 * Revision 1.15  1997/01/18 14:07:54  fox
 * Test version.
 *
 * Revision 1.14  1997/01/16 12:42:48  fox
 * Fixed a bug in scan masks.
 *
 * Revision 1.13  1997/01/14 16:53:22  fox
 * Removed a very very nasty bug for sensor selection.
 *
 * Revision 1.12  1997/01/14 10:38:19  wolfram
 * Test Version
 *
 * Revision 1.11  1997/01/08 15:59:11  wolfram
 * Resolved conflicts during merge
 *
 * Revision 1.10  1997/01/08 15:52:55  fox
 * Rotations are not any longer treated like movements.
 * Send parameters to PLAN.
 *
 * Revision 1.9  1997/01/07 12:31:26  wolfram
 * FindLocalMaxima additionally computes a list of localMaxima
 *
 * Revision 1.8  1997/01/07 09:51:34  wolfram
 * Added function to compute list local maxima give as real positions
 *
 * Revision 1.7  1996/12/02 10:32:04  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.6  1996/11/27 15:51:16  fox
 * For Woflram.
 *
 * Revision 1.5  1996/11/25 09:47:24  wolfram
 * Added graphic support for movment errors
 * Improved procedure for computing the entropy of a grid
 *
 * Revision 1.4  1996/11/21 14:55:28  wolfram
 * moved gridCellList to general.h
 *
 * Revision 1.3  1996/11/21 14:08:05  wolfram
 * First steps toward action selection
 *
 * Revision 1.2  1996/11/15 17:44:05  ws
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:32  rhino
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


#ifndef PROBGRID_TOOLS_INCLUDE
#define PROBGRID_TOOLS_INCLUDE

#include "general.h"

/**************************************************************************
 **************************************************************************
 * Mandatory functions.
 **************************************************************************
 **************************************************************************/

/**************************************************************************
 **************************************************************************
 * Structures.
 **************************************************************************
 **************************************************************************/


/**************************************************************************
 **************************************************************************
 * Other functions.
 **************************************************************************
 **************************************************************************/

#define MAX_NUMBER_OF_LOCAL_MAX 10

distance
realPositionDistance(realPosition pos1, realPosition pos2);

distance
angleDistance(float rot1, float rot2);

probability
posGridEntropy( positionProbabilityGrid* posGrid);

double
realCellListEntropy( realCellList* cellList);

double
gridCellListEntropy( gridCellList* cellList);


void
insertLocalMaximum(gridCell nextCell, gridCellList *list);

void
insertRealLocalMaximum(realCell nextCell, realCellList *list,
		       int maxNumberOfLocalMax);

void
findLocalMaxima( positionProbabilityGrid *posGrid,
		 realCellList *localMaxima,
		 int deltaPos, int deltaRot);


probability
bayesEstimationError( positionProbabilityGrid *posGrid, realPosition realPos1);

void
normalizeGridCellList( gridCellList *localMaxima);

void
normalizeRealCellList( realCellList *localMaxima);

realPosition
measuredRobotPosition(float expiredTime, bool* success);

#endif






