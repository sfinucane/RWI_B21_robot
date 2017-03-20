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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/sound/speech.c,v $
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
 *  $Log: speech.c,v $
 *  Revision 1.1  2002/09/14 15:41:10  rstone
 *  *** empty log message ***
 *
 *  Revision 1.6  1998/02/01 19:57:55  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.5  1998/02/01 19:34:38  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.4  1998/01/27 13:38:34  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.3  1998/01/19 09:39:43  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.2  1997/03/13 16:23:45  haehnel
 *  not important change
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

#define DEV_PORT

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h> 
#include <fcntl.h>
#include <stddef.h>
#include <sys/types.h>
#include <sys/stat.h>

#define _SPEECH_TEXT_MODE    0
#define _SPEECH_CHAR_MODE    1
#define _SPEECH_PHONEME_MODE 2
#define _SPEECH_TONE_MODE    3
#define _SPEECH_PCM_MODE     4

#define DEFAULT_TEXT_DELAY    1

#define TALK_DTR_ADDRESS		0x29e
#define TALK_DTR_LPC_PORT		0x00
#define TALK_DTR_TTS_PORT		0x01
#define TTS_BASE                       (TALK_DTR_TTS_PORT+TALK_DTR_ADDRESS)

#ifdef DEV_PORT

int portfd; /* fd for /dev/port */

static void
outb (char val, short port)
{
  lseek(portfd, port, SEEK_SET);
  write(portfd, &val, 1);
}

static unsigned char
inb (short port)
{
  unsigned int ret;

  lseek(portfd, port, SEEK_SET);
  read(portfd, &ret, 1);
  return ret;
}

#else

#define inb(y) \
({ int _tmp__; \
     asm volatile("inb %%dx, %0" : "=a" (_tmp__) : "d" (y)); \
     _tmp__; })


#define outb(y, x) \
({ int _tmp__; \
     asm volatile("outb %1, %%dx" : "=r" (_tmp__) : "a" (y) , "d" (x)); \
     y; })

#endif

static int   _speech_current_mode          = _SPEECH_TEXT_MODE;
static int   _speech_current_text_delay    = DEFAULT_TEXT_DELAY;

void 
_send_to_tts( unsigned char command )
{
  unsigned char rdy=0;
  do {
    rdy=inb(TTS_BASE);
    usleep(1);
  } while (!(rdy & 0x10));      /* While not ready to receive */
  outb(command, TTS_BASE);
  do {
    rdy=inb(TTS_BASE);
    usleep(1);
  } while (!(rdy & 0x10));      /* While not finished receiving */
}

void 
_send_string_to_tts( char *string )
{
  unsigned int i;
  for (i=0; i<strlen(string); i++)
    _send_to_tts(string[i]);
  _send_to_tts(0);
}

static void
_speechOutput( char *string )
{
  _send_string_to_tts(string);
}

void
SPEECH_init( void )
{
  _send_to_tts(0x1e);     /* Interrupt     */
  _speechOutput("\01@");  /* SPEECH_reinitialize */
  _speechOutput("\01R");  /* SPEECH_clear */
}

/************************************************************************/
/*  Parameter tuning                                                    */
/************************************************************************/

void
SPEECH_enable_intonation(void)
{
  _speechOutput("\01E");
}

void
SPEECH_disable_intonation(void)
{
  _speechOutput("\01M");
}


int
SPEECH_set_frequency(int frequency)
{
  char buff[20];
  if ((frequency<0)||(frequency>9))
    return (-1);
  sprintf(buff,"\01%iF", frequency);
  _speechOutput( buff);
  return frequency;
}

int
SPEECH_set_speed(int speed)
{
  char buff[20];
  if ((speed<0)||(speed>9))
    return (-1);
  sprintf(buff,"\01%iS", speed);
  _speechOutput( buff);
  return speed;
}


int
SPEECH_set_pitch(int pitch)
{
  char buff[20];
  if ((pitch<0)||(pitch>99))
    return (-1);
  sprintf(buff,"\01%iP", pitch);
  _speechOutput( buff);
  return pitch;
}


int
SPEECH_set_volume(int volume)
{
  char buff[20];
  if ((volume<0)||(volume>0))
    return (-1);
  sprintf(buff,"\01%iV", volume);
  _speechOutput( buff);
  return volume;
}

int
SPEECH_set_tone(int tone)
{
  char buff[20];
  if ((tone<0)||(tone>1))
    return (-1);
  sprintf(buff,"\01%iX", tone);
  _speechOutput( buff);
  return tone;
}

int
SPEECH_set_punctuation_level(int punc_level)
{
  char buff[20];
  if ((punc_level<0)||(punc_level>7))
    return (-1);
  sprintf(buff,"\01%iB", punc_level);
  _speechOutput( buff);
  return punc_level;
}

int
SPEECH_set_timeout_delay(int delay)
{
  char buff[20];
  if ((delay<0)||(delay>15))
    return (-1);
  sprintf(buff,"\01%iY", delay);
  _speechOutput( buff);
  return delay;
}

int
SPEECH_set_text_delay(int delay_between_words)
{
  char buff[20];
  if ((delay_between_words<0)||(delay_between_words>15))
    return (-1);
  sprintf(buff,"\01%iT", delay_between_words);
  _speechOutput( buff);
  return delay_between_words;
}

int
SPEECH_set_character_delay(int delay_between_letters)
{
  char buff[20];
  if ((delay_between_letters<0)||(delay_between_letters>31))
    return (-1);
  sprintf(buff,"\01%iC", delay_between_letters);
  _speechOutput( buff);
  return delay_between_letters;
}

/************************************************************************/
/************************************************************************/

void
init_speech( void )
{
#ifdef DEV_PORT
  portfd = open("/dev/port", O_RDWR);
#else
  ioperm((unsigned long) TTS_BASE, (unsigned long) 1, 1); 
#endif
  SPEECH_init();
  SPEECH_set_pitch(33);
  SPEECH_set_speed(1);
  SPEECH_set_frequency(70);
  SPEECH_set_timeout_delay(10);
  SPEECH_set_text_delay(3);
  SPEECH_set_volume(9);  /* should be set to 11 :) */ 
  SPEECH_set_tone(1);
}

void 
SPEECH_talk_text( char *plain_text )
{
  char buff[20];
  if (plain_text==NULL)
    return;
  /* Convert to text mode */
  if (_speech_current_mode!=_SPEECH_TEXT_MODE) {
    _speech_current_mode=_SPEECH_TEXT_MODE;
    sprintf(buff,"\01%iT", _speech_current_text_delay);
    _speechOutput( buff);
  }
  _speechOutput( plain_text);
}

#endif

