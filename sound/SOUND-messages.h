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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/sound/SOUND-messages.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:41:10 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 *  $Log: SOUND-messages.h,v $
 *  Revision 1.1  2002/09/14 15:41:10  rstone
 *  *** empty log message ***
 *
 *  Revision 1.4  1998/08/05 01:03:56  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.3  1997/07/03 11:19:15  haehnel
 *  fixed a small bug (correct length of sound)
 *
 *  Revision 1.2  1997/06/09 06:40:50  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.1  1997/03/13 16:21:09  haehnel
 *  SOUND - include soundblaster, cd and speechboard
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#ifndef SOUND_messages_defined
#define SOUND_messages_defined


#include "tcx.h"

#define TCX_SOUND_MODULE_NAME "SOUND"

#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR SOUND;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR SOUND;	/* otherwise: reference */

#endif

#define STOP 1
#define PLAY 2
#define TALK 3

#define SOUND_play_query_format "string"

#define SOUND_play_reply_format NULL

typedef struct{
  int sec;
  int usec;
} SOUND_playing_reply_type, *SOUND_playing_reply_ptr;

#define SOUND_playing_reply_format "{ int, int }"

#define SOUND_talk_query_format "string"

#define SOUND_stop_query_format NULL

typedef struct{
  int cd;
  int dsp;
} SOUND_set_mixer_type, *SOUND_set_mixer_ptr;

#define SOUND_set_mixer_format "{ int, int }"

/**** SOUND commands ****/

#ifdef TCX_define_variables		/* do this exactly once! */

#define SOUND_messages \
  {"SOUND_play_query",        SOUND_play_query_format},\
  {"SOUND_play_reply",        SOUND_play_reply_format},\
  {"SOUND_playing_reply",     SOUND_playing_reply_format},\
  {"SOUND_talk_query",        SOUND_talk_query_format}, \
  {"SOUND_stop_query",        SOUND_stop_query_format}, \
  {"SOUND_set_mixer_query",   SOUND_set_mixer_format}
#endif

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#ifdef DEFINE_REPLY_HANDLERS

/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with PLAN ******/

/******* (a) Procedure headers ******/
void SOUND_play_reply_handler(TCX_REF_PTR ref, void *data);
void SOUND_playing_reply_handler(TCX_REF_PTR ref, SOUND_playing_reply_ptr data);

/******* (b) Handler array ******/
TCX_REG_HND_TYPE SOUND_reply_handler_array[] = {
  {"SOUND_play_reply", "SOUND_play_reply_handler",
   (void (*)()) SOUND_play_reply_handler, TCX_RECV_ALL, NULL},
  {"SOUND_playing_reply", "SOUND_playing_reply_handler",
   (void (*)()) SOUND_playing_reply_handler, TCX_RECV_ALL, NULL}
};

#endif

#endif
