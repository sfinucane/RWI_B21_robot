
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/router/rcolli-handlers.c,v $
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
 * $Log: rcolli-handlers.c,v $
 * Revision 1.1  2002/09/14 16:01:03  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1997/06/27 00:47:20  thrun
 * Excluded speech from the official BeeSoft release - this made it
 * necessary to change some of the files (those that included
 * speech stuff)
 *
 * Revision 1.1.1.1  1996/09/22 16:46:34  rhino
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





#include <stdio.h>
#include <stdlib.h>
#include <values.h>
#include "tcx.h"
#include "tcxP.h"
#include "sonar_interface.h" 

#include "SONAR-messages.h"

#define TCX_define_variables /* this makes sure variables are installed */
#include "BASE-messages.h"
#include "SONAR-messages.h"
#include "COLLI-messages.h"
#include "LASER-messages.h"
#include "IR-messages.h"


#include "collision.h"
#include "base-handlers.h"
#include "colli-handlers.h"

#include "router.h"

/*---- 'COLLI_debug' prints out messages upon receipt of a TCX message ----*/
/*#define COLLI_debug*/

/***************************************************************************
 * GLOBAL VARIABLES                                                        *
 ***************************************************************************/
COLLI_colli_reply_type        colli_tcx_status;

int COLLI_REPLIES = 0;
extern RouterModuleReferenceType RMR[];

extern int NumberOfColliReplyModules;
extern int ColliAutoReplyList[];
extern int verbose_on;
extern TCX_MODULE_PTR REAL_BASE;    


/************************************************************************
 *
 *   NAME:         COLLI_vision_line_handler
 *                 
 *   FUNCTION:     receives collision lines from the vision module
 *                 
 *   PARAMETERS:   struct no_of_lines: number of collision lines
                   array with the different lines
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void COLLI_vision_line_handler(TCX_REF_PTR                ref,
			       COLLI_vision_line_ptr      vision_lines)
{
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "COLLI_vision_line" , vision_lines );
    }
  tcxFree("COLLI_vision_line", vision_lines);
}



/************************************************************************
 *
 *   NAME:         COLLI_vision_point_handler
 *                 
 *   FUNCTION:     receives collision points from the vision module
 *                 
 *   PARAMETERS:   struct no_of_points: number of collision points
                   array with the different points
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void COLLI_vision_point_handler(TCX_REF_PTR                ref,
			       COLLI_vision_point_ptr      vision_points)
{
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "COLLI_vision_point" , vision_points );
    }
  tcxFree("COLLI_vision_point", vision_points);
}


/************************************************************************
 *
 *   NAME:         COLLI_parameter_handler
 *                 
 *   FUNCTION:     receives collision parameters.
 *                 
 *   PARAMETERS:   struct parameters:
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void COLLI_parameter_handler(TCX_REF_PTR                ref,
			       COLLI_parameter_ptr      parameters)
{
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "COLLI_parameter" , parameters );
    }
  tcxFree("COLLI_parameter", parameters);
}





/***************************************************************************
 * COLLI_colli_reply_handler                                               *
 ***************************************************************************/
void COLLI_colli_reply_handler(TCX_REF_PTR                ref,
			       COLLI_colli_reply_ptr      data)
{
  int i,count=0;
 
  for ( i=0 ; i<NumberOfColliReplyModules ; i++)
    {
      if ( RMR[ColliAutoReplyList[i]].ModuleStatus == ACTIVE )
	{
	  if ( ( COLLI_REPLIES % RMR[ColliAutoReplyList[i]].colli )==0)
	    {
	      if ( verbose_on == 1 )
		fprintf (stderr,">");
	      tcxSendMsg ( RMR[ColliAutoReplyList[i]].module,
			  "COLLI_colli_reply", data );
	      count++;
	    }
	}
    }
  
  for ( i=0 ; i<MaxNumberOfModules ; i++ )
    {
      if ( RMR[i].colli == 0 )   /* final security */
	{
	  if ( GetQuery ( i , COLLI_QUERY ) > 0 )
	    {
	      if ( verbose_on == 1 )
		fprintf (stderr,"\nSending COLLI REPLY to %d",i );
	      tcxSendMsg ( RMR[i].module,
			  "COLLI_colli_reply", data );
	      DecreaseQuery ( i , COLLI_QUERY );
	    }
	}
    }
  
  COLLI_REPLIES++;  
  if ( verbose_on == 1 )
    fprintf (stderr,"C%d",count);
  tcxFree ( "COLLI_colli_reply" , data );
}

/* copied from COLLI-messages.h */
/******* (b) Handler array ******/

TCX_REG_HND_TYPE COLLI_reply_handler_array[] = {
  {"COLLI_colli_reply", "COLLI_colli_reply_handler",
     COLLI_colli_reply_handler, TCX_RECV_ALL, NULL}
};



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void tcx_register_colli ( void )
{
  tcxRegisterHandlers(COLLI_reply_handler_array,
		      sizeof(COLLI_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));
}

