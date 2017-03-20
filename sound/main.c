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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/sound/main.c,v $
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
 *  $Log: main.c,v $
 *  Revision 1.1  2002/09/14 15:41:10  rstone
 *  *** empty log message ***
 *
 *  Revision 1.12  2000/06/23 11:19:39  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.11  2000/06/23 11:00:57  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.10  2000/03/10 18:38:24  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.9  1998/08/29 22:29:25  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.8  1998/08/22 23:27:02  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.7  1998/08/11 22:03:58  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.6  1998/03/26 13:02:16  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.5  1998/03/26 12:58:30  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.4  1998/01/19 09:54:25  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.3  1998/01/19 09:33:07  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.2  1997/06/09 12:12:17  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.1  1997/03/13 16:21:10  haehnel
 *  SOUND - include soundblaster, cd and speechboard
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <stdio.h>

#ifdef i386

#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/soundcard.h>
#include "tcx.h"
#include "tcxP.h"
#include "SOUND.h"

extern int dsp_fd;
extern int mixer;

extern void strmcpy (char **t, const char *s);

extern void open_cd( void );
extern void init_cd( void );
extern void init_mixer( void );
extern void init_speech( void );
extern void init_audio( void );

extern void change_mixer( int dev, int val );
extern int read_mixer( int dev );

extern void connect2TCX( void );
extern void ReadData( char *FileName );

extern int        CD_MIXER;
extern int        WAV_MIXER;
extern int        VOL_MIXER;
extern int        MIC_MIXER;

struct timeval TCX_waiting_time;

char *filename = "sounds.dat";

int use_audio  = FALSE;
int use_cd     = FALSE;
int use_speech = FALSE;
int use_mixer  = FALSE;
int use_mic    = FALSE;
int use_volume_control = FALSE;

int minsecfrm_type = FALSE;
int global_volume = 0;
int global_mic = 0;

extern char dsp_active;
extern char cd_active;


void
Quit( int sig ) 
{
  fprintf( stderr, "Terminate SOUND \n" );
  if( use_audio )
    if ( dsp_fd )
      close( dsp_fd );
  if( use_mixer )
    if ( mixer )
      close( mixer );
  exit(0);
}

int
main( int argc, char *argv[] )
{
 int i, vol;
 for (i=1; i<argc; i++) {
   if ((strcmp(argv[i],"-wav")==0))
     use_audio = FALSE;
   else if ((strcmp(argv[i],"-cd")==0))
     use_cd = FALSE;
   else if ((strcmp(argv[i],"-speech")==0))
     use_speech = FALSE;
   else if ((strcmp(argv[i],"-mixer")==0))
     use_mixer = FALSE;
   else if ((strcmp(argv[i],"-mic")==0))
     use_mic = FALSE;
   else if ((strcmp(argv[i],"-vol")==0))
     use_volume_control = FALSE;
   else if ((strcmp(argv[i],"+wav")==0))
     use_audio = TRUE;
   else if ((strcmp(argv[i],"+cd")==0))
     use_cd = TRUE;
   else if ((strcmp(argv[i],"+speech")==0))
     use_speech = TRUE;
   else if ((strcmp(argv[i],"+mixer")==0))
     use_mixer = TRUE;
   else if ((strcmp(argv[i],"+mic")==0))
     use_mic = TRUE;
   else if ((strcmp(argv[i],"+vol")==0))
     use_volume_control = TRUE;
   else if ((strcmp(argv[i],"-file")==0) && (argc>i+1)) {
     filename =
       (char *) malloc(strlen(argv[i+1])*sizeof(char));
     strcpy( filename, argv[i+1] );
     i++;
   } else { 
     fprintf( stderr, "Usage: %s [+/-wav] [+/-cd] [+/-speech] [+/-mixer] [-file FILE]\n", argv[0] );
     exit(1);
   }
 }
 signal(SIGINT, Quit);
 ReadData( filename );
 fprintf( stderr, "Initialize Hardware:\n" );
 fprintf(stderr, "   SPEECH device...");
 if (use_speech) {
   init_speech();
   fprintf( stderr, "yes\n" );
 } else {
   fprintf( stderr, "no\n" );
 }
 fprintf( stderr, "   AUDIO device....");
 if (use_audio) {
   init_audio();
   fprintf( stderr, "yes\n" );
 } else {
   fprintf( stderr, "no\n" );
 }
 fprintf(stderr, "   MIXER device....");
 if (use_mixer) {
   init_mixer();
   fprintf( stderr, "yes\n" );
 } else {
   fprintf( stderr, "no\n" );
 }
 fprintf(stderr, "   MIC   control...");
 if (use_mic) {
   global_mic = read_mixer( MIC_MIXER )/256;
   fprintf( stderr, "yes\n" );
 } else {
   fprintf( stderr, "no\n" );
 }
 fprintf(stderr, "   VOL   control...");
 if (use_volume_control) {
   global_volume = read_mixer( VOL_MIXER )/256;
   fprintf( stderr, "yes\n" );
 } else {
   fprintf( stderr, "no\n" );
 }
 fprintf( stderr, "   CD device......." );
 if (use_cd) {
   init_cd();
   open_cd();
   fprintf( stderr, "yes\n" );
 } else {
   fprintf( stderr, "no\n" );
 }
 if (use_volume_control) {
   fprintf( stderr, "MIXER: VOL=%d\n", 0 );
   change_mixer( VOL_MIXER, 0 );
 }
 connect2TCX();
 if (use_mixer) {
   fprintf( stderr, "SOUND: checking MIXER device....");
   check_mixer();
   fprintf( stderr, "done\n" );
 }
 while (TRUE) {
   TCX_waiting_time.tv_sec=0;
   if (dsp_active)
     TCX_waiting_time.tv_usec=0;
   else
     TCX_waiting_time.tv_usec=50000;
   tcxRecvLoop(&TCX_waiting_time);  
   check_active();
 }
 return(0);
}

#else

void
main( void )
{
 fprintf( stderr, "This is something for LINUX ...\n" );
}

#endif

