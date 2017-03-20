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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/sound/file.c,v $
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
 *  $Log: file.c,v $
 *  Revision 1.1  2002/09/14 15:41:10  rstone
 *  *** empty log message ***
 *
 *  Revision 1.8  2000/06/23 11:19:39  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.7  2000/06/23 11:00:56  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.6  1998/08/11 22:03:57  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.5  1998/03/26 12:58:30  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.4  1998/03/26 10:53:47  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.3  1998/01/19 09:33:06  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.2  1997/05/26 14:16:56  haehnel
 *  insert a volume control "hack"
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
#include <sys/soundcard.h>

#include "SOUND.h"

int        SoundsNum = 0;

int        sound_type[MAX_NUM_SOUNDS];
cd_sound   cd[MAX_NUM_SOUNDS];
wav_sound  wav[MAX_NUM_SOUNDS];
int        loudness[MAX_NUM_SOUNDS];

char  CD_MIXER_DEV[255]    = "cd";
int   CD_MIXER  = SOUND_MIXER_CD;
char  WAV_MIXER_DEV[255]   = "pcm";
int   WAV_MIXER = SOUND_MIXER_PCM;
int   VOL_MIXER = SOUND_MIXER_VOLUME;
int   MIC_MIXER = SOUND_MIXER_MIC;

extern int use_audio;
extern int use_cd;
extern int use_speech;
extern int use_mixer;
extern int use_mic;
extern int use_volume_control;
extern int minsecfrm_type;

void 
ReadData( char *FileName ) 
{
 FILE *iop;
 
 char ReadNickName[256];
 char ReadFileName[256];
 char FullFileName[256];

 char line[256];
 char command[256];    
 char dummy[256];    

 int std_cd_loudness  = STANDARD_CD_LOUDNESS;
 int std_wav_loudness = STANDARD_WAV_LOUDNESS;

 sprintf( FullFileName, "%s", FileName);
 if ((iop = fopen( FullFileName, "r")) == 0){
  sprintf( FullFileName, "../etc/%s", FileName );
  if ((iop = fopen( FullFileName, "r")) == 0){
   fprintf(stderr, "Could not open input file %s\n", FileName );
   exit(0);
  }
 }
 fprintf(stderr, "Read input file %s\n", FullFileName ); 

 SoundsNum=0;
 while (fgets(line,256,iop) != NULL) {
   if (sscanf(line,"%s",command)!=0) {
     if (!strcmp( command, "STD_CD_LOUDNESS") ){
       sscanf(line, "%s %d",
	      &dummy,
	      &std_cd_loudness );
     } else if (!strcmp( command, "STD_WAV_LOUDNESS") ){
       sscanf(line, "%s %d",
	      &dummy,
	      &std_wav_loudness );
     } else if (!strcmp( command, "USE_CD") ){
       use_cd = TRUE;
     } else if (!strcmp( command, "USE_WAV") ){
       use_audio = TRUE;
     } else if (!strcmp( command, "USE_MIXER") ){
       use_mixer = TRUE;
     } else if (!strcmp( command, "USE_MIC") ){
       use_mic = TRUE;
     } else if (!strcmp( command, "USE_VOLUME_CONTROL") ){
       use_volume_control = TRUE;
     } else if (!strcmp( command, "USE_SPEECH") ){
       use_speech = TRUE;
     } else if (!strcmp( command, "MIN_SEC_FRM_TYPE") ){
       minsecfrm_type = TRUE;
     } else if (!strcmp( command, "TRK_SEC_FRM_TYPE") ){
       minsecfrm_type = FALSE;
     } else if (!strcmp( command, "CD_MIXER_DEV") ){
       if (sscanf(line, "%s %s", &dummy, &CD_MIXER_DEV)==2) {
	 if (!strcmp(CD_MIXER_DEV,"cd"))
	   CD_MIXER = SOUND_MIXER_CD;
	 else if (!strcmp(CD_MIXER_DEV,"line1"))
	   CD_MIXER = SOUND_MIXER_LINE1;
	 else if (!strcmp(CD_MIXER_DEV,"line2"))
	   CD_MIXER = SOUND_MIXER_LINE2;
	 else if (!strcmp(CD_MIXER_DEV,"line3"))
	   CD_MIXER = SOUND_MIXER_LINE3;
	 else {
	   fprintf( stderr, "SOUND: CD-MIXER -> only cd, line1, line2 or line3 implemented\n" ); 	
	   exit(1);
	 }
       }
     } else if (!strcmp( command, "WAV_MIXER_DEV") ){
       if (sscanf(line, "%s %s", &dummy, &WAV_MIXER_DEV)==2) {
	 if (!strcmp(WAV_MIXER_DEV,"pcm"))
	   WAV_MIXER = SOUND_MIXER_PCM;
	 else if (!strcmp(WAV_MIXER_DEV,"line"))
	   WAV_MIXER = SOUND_MIXER_LINE;
	 else {
	   fprintf( stderr, "SOUND: WAV-MIXER -> only pcm or line implemented\n" ); 	
	   exit(1);
	 }
       }
     } else if (!strcmp( command, "WAV") ){
       if (SoundsNum<MAX_NUM_SOUNDS) {
	 if (sscanf(line, "%s %s %s %d",
		    &dummy,
		    &ReadNickName,
		    &ReadFileName,
		    &loudness[SoundsNum]) < 4 )
	   loudness[SoundsNum]=std_wav_loudness;
	 strncpy( wav[SoundsNum].nickname, ReadNickName, MAX_NAME_LENGTH );
	 strncpy( wav[SoundsNum].filename, ReadFileName, MAX_NAME_LENGTH );
	 sound_type[SoundsNum]=WAV_SOUND;
	 SoundsNum++;
       }
     } else if (!strcmp( command, "CD") ){
       if (SoundsNum<MAX_NUM_SOUNDS) {
	 if (sscanf(line, "%s %s %d %d %d %d %d %d %d", 
		    &dummy,
		    &ReadNickName,
		    &cd[SoundsNum].start_track,
		    &cd[SoundsNum].start_time,
		    &cd[SoundsNum].start_frame,
		    &cd[SoundsNum].end_track,
		    &cd[SoundsNum].end_time,
		    &cd[SoundsNum].end_frame,
		    &loudness[SoundsNum]) < 9 )
	   loudness[SoundsNum]=std_cd_loudness;
	 strncpy( cd[SoundsNum].nickname, ReadNickName, MAX_NAME_LENGTH );
	 sound_type[SoundsNum]=CD_SOUND;
	 SoundsNum++;
       }
     } else {
       if (!(command[0]=='#')){
	 fprintf( stderr, "Unknown Keyword %s\n", command );
	 fclose(iop);
	 exit(0);
       }
     }
   }
 }
 fclose(iop);
}

int
SearchName( char *name )
{
  int sndtype, i=0;
  while (i<SoundsNum ) {
    sndtype = sound_type[i];
    if (sndtype==CD_SOUND) {
      if (!strcmp( cd[i].nickname, name )) {
	return(i); 
      }
    } else {
      if (!strcmp( wav[i].nickname, name )) {
	return(i); 
      }
    }
    i++;
  }
  return(-1); 
}

#endif
