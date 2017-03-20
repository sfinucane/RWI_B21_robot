/*    
 *  plcd 
 *  Copyright (C) 1996 by Robert Drake
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License Version 2
   as published by the Free Software Foundation.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
   
   
 */

#include <sys/ioctl.h>
#include <linux/cdrom.h>
#include <fcntl.h>
#include <time.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define VERSION "plcd v1.0beta, Copyright (C) 1996 Robert F. Drake\n"
#define INVALID "Invalid track selected\n"

#define TRUE 1
#define FALSE 0

struct cdrom_tochdr tochdr;
struct cdrom_ti ti;
struct cdrom_tocentry entry;
struct cdrom_subchnl sc;


int drive = 0;	

int stopped = TRUE;  
int paused  = FALSE;

int cdbegin, cdend;
int track;
int status;     

const char *options = 
     "options are:\n"
     "\n"
     "\n"
     "plcd #        <------- where # is any valid track number\n"
     "plcd s        <------- stops play\n"
     "plcd V        <------- version info\n"
     "plcd t        <------- display number of tracks on CD\n"
     "plcd r        <------- play a random track\n"
     "plcd p        <------- toggle pause/unpause CD\n"
     "plcd a#       <------- play all tracks starting with this number\n"
     "plcd ar       <------- play random tracks forever\n"
     "\n"
     "\n"
     "You may specify as many tracks as you like\n"
     "\n"
     "don't put dashes before or after these options\n"
     "\n";

const char *showfreegarbage =
     "\n"
     VERSION
     "plcd is free software with ABSOLUTELY NO WARRANTY\n"
     "for details, please see the file COPYING\n"
     "\n";
     
void get_state()  {
   ioctl(drive,CDROMSUBCHNL,&sc);
    status = sc.cdsc_audiostatus;
     
    if (status != CDROM_AUDIO_NO_STATUS)
          track = sc.cdsc_trk;
    stopped = TRUE;
    paused = FALSE;
	
    switch (status)  {
      case CDROM_AUDIO_PLAY:  
        stopped = FALSE;
        break;
      case CDROM_AUDIO_PAUSED:
        paused = TRUE;
    }
}

void init_cddrive()  {
    if (drive<=0) 
       drive = open("/dev/cdrom",O_RDONLY);
    
    ioctl(drive,CDROMREADTOCHDR, &tochdr);
    cdbegin = tochdr.cdth_trk0;
    cdend = tochdr.cdth_trk1;
    get_state();	
}

void stop() {
    ioctl(drive,CDROMSTOP);
}

void pausecd()  {
   get_state();
   if (paused) 
      ioctl(drive,CDROMRESUME);
   else ioctl(drive,CDROMPAUSE);
}

void
play_cd( int start_min, int start_sec, int start_frm,
	 int end_min, int end_sec, int end_frm )
{
  struct cdrom_msf msf;
  if (drive < 1) return;
  msf.cdmsf_min0 = start_min;
  msf.cdmsf_sec0 = start_sec;
  msf.cdmsf_frame0 = start_frm;
  msf.cdmsf_min1 = end_min;
  msf.cdmsf_sec1 = end_sec;
  msf.cdmsf_frame1 = end_frm;
  if (ioctl(drive, CDROMSTART)) {
    perror("CDROMSTART");
    return;
  }
  if (ioctl(drive, CDROMPLAYMSF, &msf)) {
    perror("CDROMPLAYMSF");
    return;
  }
}

void 
play_track(int tracknum)  {
     if (drive < 1) return;
     ti.cdti_trk0 = tracknum;
     ti.cdti_trk1 = tracknum++;
     ti.cdti_ind0 = ti.cdti_ind1 = 0;
     ioctl(drive,CDROMSTOP);
     ioctl(drive,CDROMPLAYTRKIND, &ti);
}

void 
play_all_track(int tracknum)  {
     if (drive < 1) return;
     ti.cdti_trk0 = tracknum;
     ti.cdti_trk1 = cdend;
     ti.cdti_ind0 = ti.cdti_ind1 = 0;
     ioctl(drive,CDROMSTOP);
     ioctl(drive,CDROMPLAYTRKIND, &ti);
}    

int 
tracklength(int tracknum)  {
    int length,pos;
    entry.cdte_track = tracknum;
    entry.cdte_format = CDROM_MSF;
    ioctl(drive,CDROMREADTOCENTRY, &entry);
    pos = entry.cdte_addr.msf.minute * 60 + entry.cdte_addr.msf.second;
    entry.cdte_track = tracknum+1;
    entry.cdte_format = CDROM_MSF;
    ioctl(drive,CDROMREADTOCENTRY, &entry);
    length = entry.cdte_addr.msf.minute * 60 + entry.cdte_addr.msf.second - pos;  
    return length;
} 

int 
ssleep(int seconds)  {
    struct timeval sleeptime;
    
    sleeptime.tv_sec = seconds;
    sleeptime.tv_usec = 0;
    select(0,0,0,0,&sleeptime);
    
    return seconds;
}

void 
main (int argc, char *argv[]) {
    int smin, ssec, sfrm, emin, esec, efrm, gap, psec, pmin, actfrm;
    unsigned long start;
    unsigned long nextsec;
    unsigned long now;

    static struct timeval start_time = {0, 0};
    static struct timeval current_time = {0, 0};

    if (argc != 7)  {   
      fprintf( stderr, 
	  "./plcd StartMin StartSec StartFrame EndMin EndSec EndFrame \n");
      exit(0);
    }
    
    init_cddrive();
    smin = atoi(argv[1]);
    ssec = atoi(argv[2]);
    sfrm = atoi(argv[3]);
    emin = atoi(argv[4]);
    esec = atoi(argv[5]);
    efrm = atoi(argv[6]);

    fprintf( stderr, "Play\n   Start: Min=%d Sec=%d Frame=%d\n", 
	     smin, ssec, sfrm);
    fprintf( stderr, "   End:   Min=%d Sec=%d Frame=%d\n\n", 
	     emin, esec, efrm );

    play_cd( smin, ssec, sfrm, emin, esec, efrm );
    
    gettimeofday(&start_time, NULL);
    gap=(75-sfrm);
    start = start_time.tv_sec*1000000+start_time.tv_usec;
    nextsec = start + (gap/75.0) * 1000000;
    psec = ssec;
    pmin = smin;
    while (1) {
      gettimeofday(&current_time, NULL);
      now = current_time.tv_sec*1000000+current_time.tv_usec;
      actfrm = sfrm + 75 * ( current_time.tv_usec / 1000000.0 );
      actfrm = actfrm % 75;
      if (now>nextsec) {
	psec++;
	if (psec==60) {
	  psec=0;
	  pmin++;
	}
	nextsec = nextsec+1000000;
	if ((pmin==emin) && (psec>=esec)) {
	  fprintf( stderr, "\n" );
	  exit(0);
	}
      }
      fprintf( stderr, "%d %d %d\r", pmin, psec, actfrm );
      usleep(10000);
    }
}

