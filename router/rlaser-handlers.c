
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/router/rlaser-handlers.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 16:01:03 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: rlaser-handlers.c,v $
 * Revision 1.1  2002/09/14 16:01:03  rstone
 * *** empty log message ***
 *
 * Revision 1.1  1997/05/26 09:19:49  fox
 * Added laser support.
 *
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "libc.h"
#include "Common.h"
#include "tcx.h"
#include "robot_specifications.h"
#include "tcxP.h"

#if 0
#define TCX_define_variables /* this makes sure variables are installed */
#endif
#include "LASER-messages.h"

#include "router.h"

/*---- 'SONAR_debug' prints out messages upon receipt of a TCX message ----*/
#define SONAR_debug


/***************************************************************************
 * GLOBAL VARIABLES                                                        *
 ***************************************************************************/

extern RouterModuleReferenceType RMR[];

extern int NumberOfLaserReplyModules;
extern int LaserAutoReplyList[];
extern int verbose_on;

extern TCX_MODULE_PTR REAL_BASE;

unsigned int LASER_REPLIES = 0;

/*********************************************************************\
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
\*********************************************************************/




/**************************************************************************/
/**************************************************************************
 * LASER_laser_reply_handler                                              *
 **************************************************************************/
void LASER_laser_reply_handler(TCX_REF_PTR                  ref,
			       LASER_laser_reply_ptr        data)
{
  int i,count=0;

  for ( i=0 ; i<NumberOfLaserReplyModules ; i++)
    {
      if ( RMR[LaserAutoReplyList[i]].ModuleStatus == ACTIVE )
	{
	  if ( ( LASER_REPLIES % RMR[LaserAutoReplyList[i]].laser )==0)
	    {
	      tcxSendMsg ( RMR[LaserAutoReplyList[i]].module,
			  "LASER_laser_reply", data );
	      count++;
	    }
	}
    }
  
  LASER_REPLIES++;  
  if ( verbose_on == 1 )
    fprintf (stderr,"S%d",count);
  tcxFree ("LASER_laser_reply",data);
}

/*********************************************************************\
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
\*********************************************************************/


TCX_REG_HND_TYPE LASER_handler_array[] = {
};

/* copied from LASER-messages.h */
TCX_REG_HND_TYPE LASER_reply_handler_array[] = {
  {"LASER_laser_reply", "LASER_laser_reply_handler",
     LASER_laser_reply_handler, TCX_RECV_ALL, NULL}
};



void tcx_register_laser(void)	/* make sure we are connected to tcx! */
{
  tcxRegisterHandlers(LASER_handler_array, 
		      sizeof(LASER_handler_array) / sizeof(TCX_REG_HND_TYPE));
  tcxRegisterHandlers(LASER_reply_handler_array,
		      sizeof(LASER_reply_handler_array) /
		      sizeof(TCX_REG_HND_TYPE));

}







