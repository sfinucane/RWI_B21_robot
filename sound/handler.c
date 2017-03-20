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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/sound/handler.c,v $
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
 *  $Log: handler.c,v $
 *  Revision 1.1  2002/09/14 15:41:10  rstone
 *  *** empty log message ***
 *
 *  Revision 1.17  2000/06/23 11:19:39  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.16  2000/06/23 11:00:57  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.15  1998/08/23 00:58:44  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.14  1998/08/11 22:03:58  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.13  1998/08/05 01:03:56  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.12  1998/03/26 10:53:47  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.11  1998/01/19 09:33:06  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.10  1997/06/09 12:16:47  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.9  1997/06/09 12:12:17  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.8  1997/06/09 06:40:51  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.7  1997/05/26 14:16:56  haehnel
 *  insert a volume control "hack"
 *
 *  Revision 1.6  1997/05/13 08:02:24  haehnel
 *  stop bug fixed
 *
 *  Revision 1.5  1997/05/13 07:48:58  haehnel
 *  stop bug
 *
 *  Revision 1.4  1997/05/02 15:54:29  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.3  1997/04/03 09:38:52  haehnel
 *  do it right
 *
 *  Revision 1.2  1997/04/03 09:33:24  haehnel
 *  reply, if there is no sound
 *
 *  Revision 1.1  1997/03/13 16:21:10  haehnel
 *  SOUND - include soundblaster, cd and speechboard
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#ifdef i386

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <math.h>
#include <fcntl.h>
#include <strings.h>
#include <unistd.h>
#include <string.h>
#include <sys/soundcard.h>

#include "tcx.h"
#include "tcxP.h"
#include "SOUND-messages.h"
#include "SOUND.h"

extern int  *track_start;
extern void play_cd( int, int, int, int, int, int );

extern int SoundsNum;

extern int        sound_type[MAX_NUM_SOUNDS];
extern cd_sound   cd[MAX_NUM_SOUNDS];
extern wav_sound  wav[MAX_NUM_SOUNDS];
extern int        loudness[MAX_NUM_SOUNDS];
extern int        CD_MIXER;
extern int        WAV_MIXER;
extern int        VOL_MIXER;
extern int        MIC_MIXER;

extern void cdplay( int, int, int, int, int, int );
extern void stop_cd( void );
extern int SPEECH_talk_text( char *plain_text );

extern int SoundsNum;
extern int use_audio;
extern int use_cd;
extern int use_speech;
extern int use_mixer;
extern int use_mic;
extern int use_volume_control;

extern int global_volume;
extern int global_mic;

extern int minsecfrm_type;

extern int SearchName( char *name );
extern void SOUND_play( int number );
extern void SOUND_talk( char *msg );
extern void change_mixer( int dev, int val );
extern int read_mixer( int dev );

TCX_MODULE_PTR actual_module = NULL; 
TCX_MODULE_PTR cd_module     = NULL; 
TCX_MODULE_PTR dsp_module    = NULL; 

int actual_device = FALSE;

char cd_active  = FALSE;
char dsp_active = FALSE;

static struct timeval current_time = {0, 0};
static struct timeval reply_time = {0, 0};

/**************************************************************************
 * UTILS
 **************************************************************************/

void
check_mixer( void ) {
  if (use_cd)
    change_mixer( CD_MIXER, 0 );
  if (use_audio)
    change_mixer( WAV_MIXER, 0 );
}

void
send_tcx_reply( TCX_MODULE_PTR ref )
{
  fprintf( stderr, "Sending SOUND_play_reply to %s\n", 
	   tcxModuleName( ref) );
  tcxSendMsg( ref, "SOUND_play_reply", NULL );
  if (use_mixer) {
    if (use_volume_control) {
      fprintf( stderr, "MIXER: VOL=%d\n", 0 );
      change_mixer( VOL_MIXER, 0 );
    }
    if (use_mic) {
      fprintf( stderr, "MIXER: MIC=%d\n", global_mic );
      change_mixer( MIC_MIXER, global_mic );
    }
  }
}

/**************************************************************************
 * HANDLERS
 **************************************************************************/

void 
SOUND_play_query_handler( TCX_REF_PTR ref, char **msg )
{
  int number;
  if (*msg!=NULL) {
    fprintf( stderr, "TCX: receive SOUND_play_query( %s )\n", *msg );
    if (sscanf( *msg, "%d", &number)==1) {
      actual_module=ref->module;
      SOUND_play( number );
    } else if ( ( number = SearchName( *msg ) ) != -1 ) {
      actual_module=ref->module;
      SOUND_play( number );
    } else {
      fprintf( stderr, "SOUND: no sound \"%s\"\n", *msg );    
      send_tcx_reply( ref->module ); 
    }
  } else {
    fprintf( stderr, "SOUND: illegal name!\n" );
  }
  tcxFree( "SOUND_play_query", msg ); 
}

void 
SOUND_talk_query_handler( TCX_REF_PTR ref, char **msg )
{
  fprintf( stderr, "TCX: receive SOUND_talk_query( %s )\n", *msg );
  SOUND_talk( *msg );
  tcxFree( "SOUND_talk_query", msg ); 
}

void 
SOUND_stop_query_handler( TCX_REF_PTR ref, void *data )
{
  fprintf( stderr, "TCX: receive SOUND_stop_query\n" );
  stop_cd();
  if (cd_active) {
    send_tcx_reply( cd_module ); 
    cd_active = FALSE;
  }
  if (dsp_active) {
    send_tcx_reply( dsp_module ); 
    dsp_active = FALSE;
  }
  tcxFree( "SOUND_stop_query", data ); 
}

void 
SOUND_set_mixer_query_handler( TCX_REF_PTR ref,  SOUND_set_mixer_ptr mix )
{
  fprintf( stderr, "TCX: receive SOUND_set_mixer_query( %d, %d )\n", 
	   mix->cd, mix->dsp );
  if (use_mixer) {
    fprintf( stderr, "MIXER: CD=%d DSP=%d\n", mix->cd, mix->dsp );
    change_mixer( CD_MIXER, mix->cd );
    change_mixer( WAV_MIXER, mix->dsp );
  } else {
    fprintf(stderr, "MIXER: you don't use mixer\n");
  }
  tcxFree( "SOUND_set_mixer_query", mix ); 
}

/**************************************************************************
 * START ACTION
 **************************************************************************/

void 
SOUND_play( int number )
{
  if (sound_type[number]==CD_SOUND) {
    if (use_cd) {
      fprintf( stderr, "SOUND: play cd-message \"%s\"\n" , 
	       cd[number].nickname );
      if (use_mixer) {
	fprintf( stderr, "MIXER: [%d]\n", loudness[number] );
	change_mixer( CD_MIXER, loudness[number] );
      }
      if (cd_active && (actual_module != cd_module))
	send_tcx_reply( cd_module );
      cd_module = actual_module;
      cdplay( cd[number].start_track, 
	      cd[number].start_time, 
	      cd[number].start_frame,
	      cd[number].end_track, 
	      cd[number].end_time, 
	      cd[number].end_frame );
    } else {
      fprintf( stderr, "SOUND: don't use CD\n" );
      send_tcx_reply( actual_module ); 
    }
  } else if (sound_type[number]==WAV_SOUND) {
    if (use_audio) {
      fprintf( stderr, "SOUND: play wav-message \"%s\"\n", 
	       wav[number].nickname );
      if (use_mixer) {
	if (use_volume_control) {
	  fprintf( stderr, "MIXER: VOL=%d\n", global_volume );
	  change_mixer( VOL_MIXER, global_volume );
	}
	if (use_mic) {
	  fprintf( stderr, "MIXER: MIC=%d\n", 0 );
	  change_mixer( MIC_MIXER, 0 );
	}
	fprintf( stderr, "MIXER: [%d]\n", loudness[number] );
	change_mixer( WAV_MIXER, loudness[number] );
      }
      if (dsp_active) {
	stop_dsp();
	if (actual_module != dsp_module)
	  send_tcx_reply( dsp_module );
      }
      dsp_module = actual_module;
      start_dsp( wav[number].filename );
      dsp_active=TRUE;
    } else {
      fprintf( stderr, "SOUND: don't use AUDIO\n" );      
      send_tcx_reply( actual_module ); 
    }
  } else {
    fprintf( stderr, "SOUND: unknown sound type\n" ); 
  }
}

void 
SOUND_talk( char *msg )
{
  fprintf( stderr, "SOUND: talk \"%s\"\n", msg );
  if (use_speech) 
    SPEECH_talk_text( msg );
  else
    fprintf( stderr, "SOUND: don't use SPEECH\n" );          
}

void 
cdplay( int start_track, int start_time, int start_frame,
	int end_track, int end_time, int end_frame  ) 
{
  int smin, ssec, sfrm, stime;
  int emin, esec, efrm, etime;
  int play_length, play_sec, play_usec;
  static struct timeval ctime = {0, 0};
  SOUND_playing_reply_type playing;
 
  if (minsecfrm_type) {
    fprintf( stderr, 
	     "SOUND: (MIN/SEC/FRM) <start> %d/%d/%d - <end> %d/%d/%d\n",
	     start_track, start_time, start_frame,
	     end_track, end_time, end_frame  );
    stime = start_track * 60 * 75 + start_time * 75 + start_frame;
    etime = end_track * 60* 75 + end_time * 75 + end_frame;
    smin = start_track;
    ssec = start_time;
    sfrm = start_frame;
    emin = end_track;
    esec = end_time;
    efrm = end_frame;
  } else {
    fprintf( stderr, 
	     "SOUND: (TRK/SEC/FRM) <start> %d/%d/%d - <end> %d/%d/%d\n",
	     start_track, start_time, start_frame,
	     end_track, end_time, end_frame  );
    stime = track_start[start_track-1] + start_time * 75 + start_frame;
    etime = track_start[end_track-1] + end_time * 75 + end_frame;
    smin = stime / (60*75);
    ssec = (stime % (60*75)) / 75;
    sfrm = stime % 75;
    emin = etime / (60*75);
    esec = (etime % (60*75)) / 75;
    efrm = etime % 75;
  }
  
  cd_active   = TRUE;
  play_length = etime-stime;
  play_sec    = play_length / 75;
  play_usec   = ( play_length % 75 ) * ( 1000000.0 / 75.0 );
  
  playing.sec = play_sec;
  playing.usec = play_usec;

  fprintf( stderr, "Send SOUND_playing_reply(%d secs, %d usecs)\n", 
	   play_sec, play_usec );
  tcxSendMsg( cd_module, "SOUND_playing_reply", &playing );
  if (stime<=etime) {
    play_cd( smin, ssec, sfrm, emin, esec, efrm );
    gettimeofday(&ctime, NULL);
    reply_time.tv_sec = ctime.tv_sec + play_sec;
    reply_time.tv_usec = ctime.tv_usec + play_usec;
  } else {
    fprintf( stderr, "CD: wrong start/end values\n" );
    reply_time.tv_sec = ctime.tv_sec;
    reply_time.tv_usec = ctime.tv_usec;
  }
}

/**************************************************************************
 * TIME TO SEND REPLY
 **************************************************************************/
void 
check_active( void )
{
  if ( cd_active ) {
    gettimeofday(&current_time, NULL);
    if ( (current_time.tv_sec*1000000+current_time.tv_usec) >
	 (reply_time.tv_sec*1000000+reply_time.tv_usec) ) {
      send_tcx_reply( actual_module );
      cd_active = FALSE;
    }
  }
  if ( dsp_active ) {
    play_dsp();
  }
}

#endif
