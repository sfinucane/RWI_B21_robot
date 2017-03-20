
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/faceServer/FACE-messages.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 21:06:30 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: FACE-messages.h,v $
 * Revision 1.1  2002/09/14 21:06:30  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1999/12/28 05:51:04  thrun
 * new head control module. by Mike Montemerlo
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/



#ifndef PANTILT_messages_defined
#define FACE_messages_defined





#include "tcx.h"
#include "faceClient.h"

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* 
   Notice: when including this file, you need to have the flag

   TCX_define_variables

   be defined exactly once. This will allocate memory for the
   module pointer and the message arrays.
*/

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR FACE = NULL;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR FACE;	/* otherwise: reference */

#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** FACE data types (for the commands listed below)
 **** Make sure the struct defs and the format defs are consistent! ****/



#define FACE_init_query_format "int"

#define FACE_init_reply_format "int"

#define FACE_eye_pos_format "int"
#define FACE_brows_pos_format "int"
#define FACE_mouth_pos_format "int"

#define FACE_adjustment_format "int"

typedef struct {
  int eyes;
  int brows;
  int mouth;
} FACE_state, *FACE_state_ptr;

#define FACE_state_format "{int, int, int}"
#define FACE_status_update_format "{int, int, int}"

#define FACE_reset_format NULL
#define FACE_state_query_format NULL
#define FACE_disconnect_format NULL

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** FACE commands - these are the commands/queries understood by FACE ****/


#ifdef TCX_define_variables		/* do this exactly once! */


#define FACE_messages \
  {"FACE_init_query",       FACE_init_query_format},\
  {"FACE_init_reply",       FACE_init_reply_format},\
  {"FACE_state_query",      FACE_state_query_format},\
  {"FACE_state_reply",      FACE_state_format},\
  {"FACE_set_eyes",         FACE_eye_pos_format},\
  {"FACE_set_brows",        FACE_brows_pos_format},\
  {"FACE_set_mouth",        FACE_mouth_pos_format},\
  {"FACE_set_state",        FACE_state_format},\
  {"FACE_reset",            FACE_reset_format},\
  {"FACE_adjust_mood",      FACE_adjustment_format},\
  {"FACE_adjust_eyes",      FACE_adjustment_format},\
  {"FACE_status_update",    FACE_status_update_format},\
  {"FACE_disconnect",       FACE_disconnect_format}


#endif



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


#ifdef DEFINE_REPLY_HANDLERS


/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with FACE ******/




/******* (a) Procedure headers ******/
#ifdef FACE_STATIC_HANDLERS
static void FACE_state_reply_handler(TCX_REF_PTR ref, FACE_state_ptr data);
static void FACE_init_reply_handler(TCX_REF_PTR ref, int *data);
static void FACE_status_update_handler(TCX_REF_PTR ref, FACE_state_ptr data);
#else
void FACE_state_reply_handler(TCX_REF_PTR ref, FACE_state_ptr data);
void FACE_init_reply_handler(TCX_REF_PTR ref, int *data);
void FACE_status_update_handler(TCX_REF_PTR ref, FACE_state_ptr data);
#endif

/******* (b) Handler array ******/



TCX_REG_HND_TYPE FACE_reply_handler_array[] = {
  {"FACE_init_reply", "FACE_init_reply_handler",
    (void (*)()) FACE_init_reply_handler, TCX_RECV_ALL, NULL},
  {"FACE_state_reply", "FACE_state_reply_handler",
    (void (*)()) FACE_state_reply_handler, TCX_RECV_ALL, NULL},
  {"FACE_status_update", "FACE_status_update_handler",
    (void (*)()) FACE_status_update_handler, TCX_RECV_ALL, NULL}
};


#endif

#endif
