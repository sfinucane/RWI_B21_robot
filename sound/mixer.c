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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/sound/mixer.c,v $
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
 *  $Log: mixer.c,v $
 *  Revision 1.1  2002/09/14 15:41:10  rstone
 *  *** empty log message ***
 *
 *  Revision 1.3  2000/06/23 11:00:57  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.2  1998/08/11 22:03:59  haehnel
 *  *** empty log message ***
 *
 *  Revision 1.1  1997/06/09 12:12:36  haehnel
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

#ifdef i386

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <math.h>
#include <fcntl.h>
#include <strings.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/soundcard.h>
#include "tcx.h"
#include "tcxP.h"
#include "SOUND.h"

int mixer;
int devmask = 0;
int recmask = 0;
int recsrc = 0;
int stereodevs = 0;

void
init_mixer( void )
{
  if ((mixer = open("/dev/mixer", O_RDWR)) < 0) {
    fprintf(stderr, "MIXER: error opening /dev/mixer\n");
    exit(1);
  }
  if (ioctl(mixer, SOUND_MIXER_READ_DEVMASK, &devmask) == -1) {
    fprintf(stderr,"MIXER: error in SOUND_MIXER_READ_DEVMASK\n");
    exit(-1);
  }
  if (ioctl(mixer, SOUND_MIXER_READ_RECMASK, &recmask) == -1) {
    fprintf(stderr,"MIXER: error in SOUND_MIXER_READ_RECMASK\n");
    exit(-1);
  }
  if (ioctl(mixer, SOUND_MIXER_READ_RECSRC, &recsrc) == -1) {
    fprintf(stderr,"MIXER: error in SOUND_MIXER_READ_RECSRC\n");
    exit(-1);
  }
  if (ioctl(mixer, SOUND_MIXER_READ_STEREODEVS, &stereodevs) == -1) {
    fprintf(stderr,"MIXER: error in SOUND_MIXER_READ_STEREODEVS\n");
    exit(-1);
  }
  if (!devmask) {
    fprintf(stderr, "MIXER: no device found\n");
    exit(-1);
  }
}

void
change_mixer( int dev, int val )
{
  val = (val > 100) ? 100 : val;
  val = (val < 0) ? 0 : 257 * val;
  if (ioctl(mixer, MIXER_WRITE(dev), &val) == -1) {
    fprintf(stderr, "MIXER: can't set value !\n");
    exit(-1);
  }
}

int
read_mixer( int dev )
{
  int val;
  if (ioctl(mixer, MIXER_READ(dev), &val) == -1) {
    fprintf(stderr, "MIXER: can't read value !\n");
    exit(-1);
  }
  return(val);
}


#endif


