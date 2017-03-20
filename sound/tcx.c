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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/sound/tcx.c,v $
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
 *  $Log: tcx.c,v $
 *  Revision 1.1  2002/09/14 15:41:10  rstone
 *  *** empty log message ***
 *
 *  Revision 1.4  1998/08/29 22:29:26  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.3  1998/08/05 01:07:34  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.2  1997/06/09 06:46:46  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.1  1997/03/13 16:21:11  haehnel
 *  SOUND - include soundblaster, cd and speechboard
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#ifdef i386

#define TCX_define_variables 
#define DEFINE_REPLY_HANDLERS

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <math.h>
#include <string.h>
#include "time.h"
#include "tcx.h"
#include "tcxP.h"

#include "SOUND-messages.h"

char* MODULE_NAME="SOUND";

extern void tcxRegisterCloseHnd(void (*closeHnd)());

extern void SOUND_play_query_handler( TCX_REF_PTR ref, char **msg );
extern void SOUND_talk_query_handler( TCX_REF_PTR ref, char **msg );
extern void SOUND_stop_query_handler( TCX_REF_PTR ref, void *data );
extern void SOUND_set_mixer_query_handler( TCX_REF_PTR ref,  SOUND_set_mixer_ptr mix );

/**************************************************************************
 * Handler
 **************************************************************************/
TCX_REG_HND_TYPE SOUND_handler_array[] = {
  {"SOUND_play_query", "SOUND_play_query_handler",
     SOUND_play_query_handler, TCX_RECV_ALL, NULL},
  {"SOUND_talk_query", "SOUND_talk_query_handler",
     SOUND_talk_query_handler, TCX_RECV_ALL, NULL},
  {"SOUND_stop_query", "SOUND_stop_query_handler",
     SOUND_stop_query_handler, TCX_RECV_ALL, NULL},
  {"SOUND_set_mixer_query", "SOUND_set_mixer_query_handler",
     SOUND_set_mixer_query_handler, TCX_RECV_ALL, NULL},
 };

/**************************************************************************
 * TCX_close_handler                                                      *
 **************************************************************************/
void 
TCX_close_handler ( char *name, TCX_MODULE_PTR module)
{
 if( !strcmp(name, "TCX Server") ) {
  fprintf( stderr, "TCX died, so will I...\n" );
  exit(1);
 }
}

/**************************************************************************
 * connect2TCX                                                            *
 **************************************************************************/
void 
connect2TCX( void )
{
  TCX_REG_MSG_TYPE TCX_message_array[] = 
  {
    SOUND_messages
  };
  /* Connect to tcxServer */
  tcxInitialize( MODULE_NAME, (char *) getenv("TCXHOST"));
  /* Register all messages. */
  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));
  /* Register all handlers. */
  tcxRegisterHandlers(SOUND_handler_array,
		      sizeof(SOUND_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));
  tcxRegisterCloseHnd( TCX_close_handler);
}

/**************************************************************************
 * REPLY_handlers                                                         *
 **************************************************************************/
void 
SOUND_play_reply_handler( TCX_REF_PTR ref, void *data )
{
  tcxFree( "SOUND_play_reply", data );
}

void 
SOUND_playing_reply_handler(TCX_REF_PTR ref, SOUND_playing_reply_ptr data)
{
  tcxFree( "SOUND_playing_reply", data );
}

#endif
