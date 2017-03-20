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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/localLib.c,v $
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
 * $Log: localLib.c,v $
 * Revision 1.1  2002/09/14 20:45:39  rstone
 * *** empty log message ***
 *
 * Revision 1.7  2000/03/06 20:00:44  wolfram
 * Changed the code of the vision routines.
 *
 * Revision 1.6  1998/08/14 15:22:48  wolfram
 * Cleaned up
 *
 * Revision 1.5  1998/08/11 23:05:38  wolfram
 * Changes for the Smithsonian museum.
 *
 * WARNING: LOCALIZE now uses only one stddev for Lasers. Laser beams have 25 m
 * Maxrange now.
 * Please use ~wolfram/bee/data/localize/params/laser-0.1stdev with this version
 * of LOCALIZE
 *
 * Revision 1.4  1997/06/20 07:36:10  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.3  1997/04/02 08:57:33  fox
 * Display the robot right after sensor selection.
 *
 * Revision 1.2  1997/01/30 13:11:21  fox
 * Nothing special.
 *
 * Revision 1.1  1997/01/29 12:23:08  fox
 * First version of restructured LOCALIZE.
 *
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <ctype.h>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>

#include "corr.h"
#include "tcx.h"
#include "tcxP.h"
#include "localize.h"

#include "SIMULATOR-messages.h"
#include "LOCALIZE-messages.h"


#ifndef M_PI
# define M_PI           3.14159265358979323846  /* pi */
#endif


static float
Deg2Rad(float x)
{
  return x * M_PI / 180.0;
}


static float
Rad2Deg(float x)
{
  return x * 180.0 / M_PI;
}

/* Sends the position to the simulator and localize. Tcx and both
 * modules must already be connected. */
void
setRobotPosition( float x, float y, float rot)
{
  if ( SIMULATOR != NULL) {
    SIMULATOR_set_robot_position_type simPos;
    simPos.x = x;
    simPos.y = y;
    simPos.rot = rot;
    fprintf( stderr, "Send position (%f %f %f) to SIMULATOR_II.\n",
	    simPos.x, simPos.y, simPos.rot);
    tcxSendMsg( SIMULATOR, "SIMULATOR_set_robot_position", &simPos);
  }
  if ( LOCALIZE != NULL) {
    LOCALIZE_set_robot_position_type locPos;
    locPos.x = x;
    locPos.y = y;
    locPos.rot = rot;
    fprintf( stderr, "Send position (%f %f %f) to LOCALIZE.\n",
	    locPos.x, locPos.y, locPos.rot);
    tcxSendMsg( LOCALIZE, "LOCALIZE_set_robot_position", &locPos);
  }
    
} 
