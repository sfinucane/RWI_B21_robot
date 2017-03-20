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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/sound/cdhardware.c,v $
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
 *  $Log: cdhardware.c,v $
 *  Revision 1.1  2002/09/14 15:41:10  rstone
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

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <mntent.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/param.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <linux/cdrom.h>
#include <signal.h>

const char *cd_device = "/dev/cdrom\0";

void open_cd( void );
void stop_cd( void );
void init_cd( void );
void play_cd( int, int, int, int, int, int );

int  cd_fd = -1;
int  *track_start = NULL;

void
play_cd( int start_min, int start_sec, int start_frm,
	 int end_min, int end_sec, int end_frm )
{
  struct cdrom_msf msf;
  if (cd_fd < 0)
    return;
  msf.cdmsf_min0 = start_min;
  msf.cdmsf_sec0 = start_sec;
  msf.cdmsf_frame0 = start_frm;
  msf.cdmsf_min1 = end_min;
  msf.cdmsf_sec1 = end_sec;
  msf.cdmsf_frame1 = end_frm;
  if (ioctl(cd_fd, CDROMSTART)) {
    perror("CDROMSTART");
    return;
  }
  if (ioctl(cd_fd, CDROMPLAYMSF, &msf)) {
    perror("CDROMPLAYMSF");
    return;
  }
}

void
init_cd( void )
{
  static struct mntent *mnt;
  static FILE *fp;
  if ((fp = setmntent(MOUNTED, "r")) == NULL) {
    fprintf (stderr, "CD: Couldn't open %s: %s\n", MOUNTED, strerror (errno));
    exit (1);
  }
  while ((mnt = getmntent(fp)) != NULL) {
    if (strcmp (mnt->mnt_type, "iso9660") == 0) {
      fprintf( stderr, "CD: CDROM already mounted. Operation aborted.\n" );
      endmntent (fp);
      exit (1);
    }
  }
  endmntent (fp);
}

void
stop_cd( void )
{
  if (cd_fd < 0)
    return;  
  ioctl(cd_fd, CDROMSTOP);
}

void
open_cd( void ) 
{
  int i, ntracks;
  struct cdrom_subchnl	   sc;
  struct cdrom_tochdr	   hdr;
  struct cdrom_tocentry	   entry; 
  if (cd_fd < 0) {
    if ((cd_fd = open(cd_device, 0)) < 0) {
      if (errno == EACCES) {
	fprintf(stderr, "CD: no permission to use %s !\n", cd_device );
      } else if (errno != ENXIO) {
	fprintf( stderr, "CD: error !\n" );
	exit(1);
      }
      return;
    }
  }  
  sc.cdsc_format = CDROM_MSF; 
  if (ioctl(cd_fd, CDROMSUBCHNL, &sc)) {
    fprintf( stderr, "CD: no cd !\n" );
    exit(0);
  }
  if (ioctl(cd_fd, CDROMREADTOCHDR, &hdr)) {
    perror("readtochdr");
    return;
  }
  ntracks = hdr.cdth_trk1;
  track_start = malloc((ntracks + 1) * sizeof(int));
  if (track_start == NULL) {
    fprintf( stderr, "CD: can't alloc memory" );
    return;
  }
  for (i = 0; i <= ntracks; i++) {
    if (i == ntracks)
      entry.cdte_track = CDROM_LEADOUT;
    else
      entry.cdte_track = i + 1;
    entry.cdte_format = CDROM_MSF;
    if (ioctl(cd_fd, CDROMREADTOCENTRY, &entry)) {
      fprintf( stderr, "CD: tocentry read" );
      return;
    }
    track_start[i] = ( entry.cdte_addr.msf.minute * 60 + entry.cdte_addr.msf.second ) * 75 +
      entry.cdte_addr.msf.frame;
  }
}

#endif
