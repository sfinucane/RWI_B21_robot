
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/router/rsonar-handlers.c,v $
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
 * $Log: rsonar-handlers.c,v $
 * Revision 1.1  2002/09/14 16:01:03  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1997/05/26 09:19:50  fox
 * Added laser support.
 *
 * Revision 1.2  1997/03/11 17:16:41  tyson
 * added IR simulation and other work
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
#include <math.h>
#include "libc.h"
#include "Common.h"
#include "tcx.h"
#include "robot_specifications.h"
#include "tcxP.h"
#include "sonar_interface.h"

#if 0
#define TCX_define_variables /* this makes sure variables are installed */
#endif
#include "SONAR-messages.h"

#include "router.h"

/*---- 'SONAR_debug' prints out messages upon receipt of a TCX message ----*/
#define SONAR_debug


/***************************************************************************
 * GLOBAL VARIABLES                                                        *
 ***************************************************************************/

extern RouterModuleReferenceType RMR[];

extern int NumberOfSonarReplyModules;
extern int SonarAutoReplyList[];
extern int verbose_on;

extern TCX_MODULE_PTR REAL_BASE;

float SONAR_BUFFER[24];
unsigned int SONAR_REPLIES = 0;

/*********************************************************************\
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
\*********************************************************************/




/************************************************************************
 *
 *   NAME: SONAR_switch_on_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void SONAR_switch_on_handler(TCX_REF_PTR      ref,
			     void            *data)
{  
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "SONAR_switch_on" , NULL );
    }
}


/************************************************************************
 *
 *   NAME: SONAR_switch_off_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void SONAR_switch_off_handler(TCX_REF_PTR      ref,
			      void            *data)
{
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "SONAR_switch_off" , NULL);
    }
}

/************************************************************************
 *
 *   NAME: SONAR_activate_mask_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/
void SONAR_activate_mask_handler(TCX_REF_PTR      ref,
				 int             *mask)
{
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "SONAR_activate_mask" , mask );
    }
  tcxFree("SONAR_activate_mask", mask);
}


/************************************************************************
 *
 *   NAME: SONAR_sonar_query_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void SONAR_sonar_query_handler(TCX_REF_PTR      ref,
			       void            *data)
{
  int SenderModule,pass_thru=0;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      pass_thru = IncreaseQuery ( SenderModule , SONAR_QUERY );
      if ( pass_thru == 1 )
	{
	  tcxSendMsg ( REAL_BASE , "SONAR_sonar_query", NULL );
	  if ( verbose_on ==1 )
	    fprintf (stderr,"\nSEND A SINGLE SONAR QUERY ...!!!");
	}
    }
}

/************************************************************************
 *
 *   NAME: SONAR_define_mask_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void SONAR_define_mask_handler(TCX_REF_PTR           ref,
			       SONAR_define_mask_ptr mask)
{
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "SONAR_define_mask" , mask );
    }
 
  tcxFree("SONAR_define_mask", mask);
}

/**************************************************************************/
/**************************************************************************
 * SONAR_sonar_reply_handler                                              *
 **************************************************************************/
void SONAR_sonar_reply_handler(TCX_REF_PTR                  ref,
			       SONAR_sonar_reply_ptr        data)
{
  int i,count=0;

  /* buffer the new reading */
  for ( i=0 ; i<24 ; i++ )
    SONAR_BUFFER[i] = data->values[i];
  
  for ( i=0 ; i<NumberOfSonarReplyModules ; i++)
    {
      if ( RMR[SonarAutoReplyList[i]].ModuleStatus == ACTIVE )
	{
	  if ( ( SONAR_REPLIES % RMR[SonarAutoReplyList[i]].sonar )==0)
	    {
	      tcxSendMsg ( RMR[SonarAutoReplyList[i]].module,
			  "SONAR_sonar_reply", data );
	      count++;
	    }
	}
    }
  
  for ( i=0 ; i<MaxNumberOfModules ; i++ )
    {
      if ( RMR[i].sonar == 0 )   /* final security */
	{
	  if ( GetQuery ( i , SONAR_QUERY ) > 0 )
	    {
	      if ( verbose_on ==1 )
		fprintf (stderr,"\nSending SONAR REPLY to %d",i );
	      tcxSendMsg ( RMR[i].module,
			  "SONAR_sonar_reply", data );
	      DecreaseQuery ( i , SONAR_QUERY );
	    }
	}
    }
  
  SONAR_REPLIES++;  
  if ( verbose_on == 1 )
    fprintf (stderr,"S%d",count);
  tcxFree ("SONAR_sonar_reply",data);
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


TCX_REG_HND_TYPE SONAR_handler_array[] = {
  {"SONAR_switch_on", "SONAR_switch_on_handler",
     SONAR_switch_on_handler, TCX_RECV_ALL, NULL},
  {"SONAR_switch_off", "SONAR_switch_off_handler",
     SONAR_switch_off_handler, TCX_RECV_ALL, NULL},
  {"SONAR_activate_mask", "SONAR_activate_mask_handler",
     SONAR_activate_mask_handler, TCX_RECV_ALL, NULL},
  {"SONAR_sonar_query", "SONAR_sonar_query_handler",
     SONAR_sonar_query_handler, TCX_RECV_ALL, NULL},
  {"SONAR_define_mask", "SONAR_define_mask_handler",
     SONAR_define_mask_handler, TCX_RECV_ALL, NULL}
};

/* copied from SONAR-messages.h */
TCX_REG_HND_TYPE SONAR_reply_handler_array[] = {
  {"SONAR_sonar_reply", "SONAR_sonar_reply_handler",
     SONAR_sonar_reply_handler, TCX_RECV_ALL, NULL}
};



void tcx_register_sonar(void)	/* make sure we are connected to tcx! */
{
  tcxRegisterHandlers(SONAR_handler_array, 
		      sizeof(SONAR_handler_array) / sizeof(TCX_REG_HND_TYPE));
  tcxRegisterHandlers(SONAR_reply_handler_array,
		      sizeof(SONAR_reply_handler_array) /
		      sizeof(TCX_REG_HND_TYPE));

}







