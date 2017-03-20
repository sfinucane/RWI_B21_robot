
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/robot/LIB-math.c,v $
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
 * $Log: LIB-math.c,v $
 * Revision 1.1  2002/09/14 15:55:27  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1997/02/22 05:16:48  thrun
 * Version numbers are now also checked for libraries.
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
#include <stdlib.h>
#include <math.h>
#include "Common.h"

/***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include "beeSoftVersion.h"

int  librobot_major      = BEESOFT_VERSION_MAJOR;
int  librobot_minor      = BEESOFT_VERSION_MINOR;
int  librobot_robot_type = BEESOFT_VERSION_ROBOT_TYPE;
char librobot_date[80]   = BEESOFT_VERSION_DATE;


/***************************************************************************
 ***************************************************************************
 ***************************************************************************/




#define _2pi (double)(2.0 * PI)

double _0_to_2pi (double rad)
{
  for(;;) {
    if (rad >= _2pi)
      rad -= _2pi;
    else if (rad < 0.0)
      rad += _2pi;
    else
      return (rad);
  }
}

double  _pi_to_pi (double rad)
{
  for(;;) {
    if (rad > PI)
      rad -= _2pi;
    else if (rad <= - PI)
      rad += _2pi;
    else
      return (rad);
  }
}

double _0_to_360 (double deg)
{
  for(;;) {
    if (deg >= 360.0)
      deg -= 360.0;
    else if (deg < 0.0)
      deg += 360.0;
    else
      return (deg);
  }
}


/*
  double _180_to_180 (double deg)
  {
  for(;;) {
  if (deg > 180.0)
  deg -= 360.0;
  else if (deg <= -180.0)
  deg += 360.0;
  else
  return (deg);
  }
  }
  */
double floatMod (double num1, double num2)
{
  if( num2 == 0 ) {
    fprintf( stderr, "floatMod: taking mod by 0\n" );
    return(0.0);
  }
  return (num1 - ((int)(num1/num2)) * num2);
}
