
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/SONAR-messages.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:25:05 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: SONAR-messages.h,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1999/06/29 21:44:59  fox
 * Minor changes to make it compile with g++.
 *
 * Revision 1.2  1999/05/28 20:30:45  thrun
 * necessary modifications for BeeSoft light
 *
 * Revision 1.1.1.1  1996/09/22 16:46:06  rhino
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



#ifndef SONAR_messages_defined
#define SONAR_messages_defined





#include "tcx.h"

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* 
 *  Notice: this file is part of BASE-messages.
 */


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** TCX module name - this is a unique identifier for TCX *******/

#define TCX_SONAR_MODULE_NAME "SONAR"



#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR SONAR;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR SONAR;	/* otherwise: reference */

#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** SONAR data types (for the commands listed below)
 **** Make sure the struct defs and the format defs are consistent! ****/




/* void SONAR_switch_on(void) */

#define SONAR_switch_on_format NULL



/* void SONAR_switch_off(void) */

#define SONAR_switch_off_format NULL
  



/* void SONAR_activate_mask(int) */

#define SONAR_activate_mask_format "int"




#define SONAR_sonar_query_format NULL
#define SONAR_ir_query_format NULL



typedef struct {
  float values[24];
} SONAR_sonar_reply_type, *SONAR_sonar_reply_ptr;

#define SONAR_sonar_reply_format "{[float : 24]}"

typedef struct {
  int upperrow[24];
  int lowerrow[24];
  int drow[8];
} SONAR_ir_reply_type, *SONAR_ir_reply_ptr;
      
#define SONAR_ir_reply_format "{[int : 24],[int : 24],[int : 8]}"
      



/* void SONAR_define_mask(SONAR_define_mask_type) */

typedef struct {
  int number;			/* Reference number of the user-defined mask.
				 * must be in 2..9.
				 * Currently 0 and 1 are built-in default 
				 * masks. */
  int length;			/* Length of user-defined sonar mask */
  int *user_mask;		/* User-mask. */
} SONAR_define_mask_type, *SONAR_define_mask_ptr;

#define SONAR_define_mask_format "{int, int, <int : 2>}" 

    





/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** SONAR commands - these are the commands/queries understood by SONAR ****/


#ifdef TCX_define_variables		/* do this exactly once! */

#define SONAR_messages \
  {"SONAR_switch_on",     SONAR_switch_on_format},\
  {"SONAR_switch_off",    SONAR_switch_off_format},\
  {"SONAR_activate_mask", SONAR_activate_mask_format},\
  {"SONAR_sonar_query",   SONAR_sonar_query_format},\
  {"SONAR_sonar_reply",   SONAR_sonar_reply_format},\
  {"SONAR_ir_query",   SONAR_ir_query_format},\
  {"SONAR_ir_reply",   SONAR_ir_reply_format},\
  {"SONAR_define_mask",   SONAR_define_mask_format}



#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/




#ifdef DEFINE_REPLY_HANDLERS


/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with SONAR ******/




/******* (a) Procedure headers ******/




void SONAR_sonar_reply_handler(TCX_REF_PTR                ref,
			       SONAR_sonar_reply_ptr      data);


void SONAR_ir_reply_handler(TCX_REF_PTR                ref,
                            SONAR_ir_reply_ptr      data);
                               

/******* (b) Handler array ******/



TCX_REG_HND_TYPE SONAR_reply_handler_array[] = {
  {"SONAR_sonar_reply", "SONAR_sonar_reply_handler",
     (void (*)()) SONAR_sonar_reply_handler, TCX_RECV_ALL, NULL},
  {"SONAR_ir_reply", "SONAR_ir_reply_handler",
     (void (*)()) SONAR_ir_reply_handler, TCX_RECV_ALL, NULL},
};



#endif

#endif
