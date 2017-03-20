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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/detection-old/entropy.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:00 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: entropy.c,v $
 * Revision 1.1  2002/09/14 20:45:00  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1997/04/30 16:17:09  fox
 * Module to detect unexpected obstacles.
 *
 * Revision 1.7  1997/04/03 13:17:50  fox
 * Some minor changes.
 *
 * Revision 1.6  1997/03/19 17:52:41  fox
 * New laser parameters.
 *
 * Revision 1.5  1997/03/18 18:45:28  fox
 * First executable version with a global scan mask for different sensor types.
 *
 * Revision 1.4  1997/03/17 18:41:12  fox
 * First version with sensor selection in cluttered environments.
 *
 * Revision 1.3  1997/03/14 17:58:17  fox
 * This version should run quite stable now.
 *
 * Revision 1.2  1997/03/13 17:36:20  fox
 * Temporary version. Don't use!
 *
 * Revision 1.1  1997/01/29 12:23:04  fox
 * First version of restructured DETECTION.
 *
 * Revision 1.21  1997/01/19 19:31:15  fox
 * yeah
 *
 * Revision 1.20  1997/01/19 18:56:55  wolfram
 * Again a bug ...
 *
 * Revision 1.19  1997/01/18 19:41:02  fox
 * Improved action selection.
 *
 * Revision 1.18  1997/01/16 19:43:20  fox
 * And another bug ...
 *
 * Revision 1.17  1997/01/13 16:54:12  fox
 * Nothing special.
 *
 * Revision 1.16  1997/01/10 15:19:21  fox
 * Improved several methods.
 *
 * Revision 1.15  1997/01/08 15:52:54  fox
 * Rotations are not any longer treated like movements.
 * Send parameters to PLAN.
 *
 * Revision 1.14  1996/12/20 15:29:35  fox
 * Added four parameters.
 *
 * Revision 1.13  1996/12/04 14:29:59  fox
 * ok
 *
 * Revision 1.12  1996/12/03 15:40:24  fox
 * ok
 *
 * Revision 1.11  1996/12/03 12:27:38  fox
 * Seems to work with the thin corridor.
 *
 * Revision 1.10  1996/12/02 18:46:25  fox
 * First version with the new expected distances.
 *
 * Revision 1.9  1996/12/02 10:32:00  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.8  1996/11/29 09:31:38  fox
 * ok
 *
 * Revision 1.7  1996/11/28 17:56:20  fox
 * *** empty log message ***
 *
 * Revision 1.6  1996/11/26 16:08:25  fox
 * Nothing special.
 *
 * Revision 1.5  1996/11/26 11:08:10  fox
 * Improved version.
 *
 * Revision 1.4  1996/11/25 19:35:39  fox
 * Test version for decisions of movements.
 *
 * Revision 1.3  1996/11/21 14:28:50  fox
 * Decides which action to perform next.
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
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#include "general.h"
#include "laser.h"
#include "entropy.h"

void
computeMaxLikelihoods( gridCellList* cells,
		       distanceScan* lasers,
		       float* likelihoods)
{
  int sensorNumber; 
  int i;           /* index for the positions */

  /*--------------------------------------------------------------
   *--------------------------------------------------------------*/
  for ( sensorNumber = 0; sensorNumber < lasers->numberOfReadings; sensorNumber++) {
    
    likelihoods[sensorNumber] = 0.0;
    
    /*--------------------------------------------------------------
     * Now compute the maximal likelihood.
     *--------------------------------------------------------------*/

    for ( i = 0; i < cells->numberOfCells; i++) {

      float tmp;
      
      tmp = cells->cell[i].prob * probOfFeature( lasers->feature[sensorNumber],
						 lasers->reading[sensorNumber].rot,
						 cells->cell[i].pos);

      if ( tmp > likelihoods[sensorNumber]) 
	likelihoods[sensorNumber] = tmp;
    }
  }
}

