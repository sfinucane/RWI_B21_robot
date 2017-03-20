#ifdef i386

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <strings.h>
#include <math.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/soundcard.h>
#include "wavlib.h"
#include "SOUND.h"
#include "SOUND-messages.h"

wav_sig *file_info;
long count;

extern char dsp_active;

extern TCX_MODULE_PTR dsp_module; 

void
wav_info( void )
{
  printf("WAV: [FILE: %s]\n", file_info->name );
  printf("     [LENGTH: %u]\n", file_info->len );
  printf("     [RATE: %u]\n", file_info->srate );
  printf("     [BPS: %u]\n", file_info->bps );
  printf("     [DEPTH: %u]\n", file_info->depth );
  printf("     [CHANNELS: %u]\n", file_info->chans );
  printf("     [BSIZE: %u]\n", bsize );
}

int
init_dsp( wav_sig *file_info )
{
  int i, p;
  dsp_fd = open("/dev/dsp", O_WRONLY, 0);
  ioctl(dsp_fd, SNDCTL_DSP_RESET, 0);
  p =  file_info->depth;
  i =  ioctl(dsp_fd, SOUND_PCM_WRITE_BITS, &p);
  p =  file_info->chans;
  i += ioctl(dsp_fd, SOUND_PCM_WRITE_CHANNELS, &p);
  p =  file_info->srate;
  i += ioctl(dsp_fd, SOUND_PCM_WRITE_RATE, &p);
  i += ioctl(dsp_fd, SNDCTL_DSP_GETBLKSIZE, &bsize);
  ioctl(dsp_fd, SNDCTL_DSP_SYNC, 0);
  return(i);
}

void
play_buffer( void )
{
  if (file_info->len < bsize){
    count = read(file_info->handle, buffer, file_info->len);
    file_info->len -= count;
  } else {
    count = read(file_info->handle, (int *)buffer, bsize);
    file_info->len -= count;
  }
  write(dsp_fd, buffer, count);
}

void
test_dsp( void )
{
  if (init_dsp(file_info)) {
    fprintf(stderr,"WAV: Cannot initialize DSP\n");
    exit(-1);
  }
  close(dsp_fd);
}

void
init_audio( void )
{
  file_info = (wav_sig *) malloc (sizeof(wav_sig));
  buffer = (int *) malloc (32767);
  test_dsp();
}

void
start_dsp( char *filename  )
{
  SOUND_playing_reply_type playing;
  if (init_wav_read(filename, file_info) == -1){
    fprintf(stderr,"WAV: Error opening file: %s\n", file_info->name);
    return;
  }
  if (init_dsp(file_info)) {
    fprintf(stderr,"WAV: Cannot initialize DSP\n");
    return;
  }
  wav_info();
  playing.sec =
    (int) (file_info->len/file_info->bps);
  playing.usec =
    (int) (((float)(file_info->len%file_info->bps)/
	    (float)file_info->bps)*1000000);
  fprintf( stderr, "Sending SOUND_playing_reply(%d secs, %d usecs)\n",
	   playing.sec, playing.usec );
  tcxSendMsg( dsp_module, "SOUND_playing_reply", &playing );
}

void
stop_dsp( void ){
  dsp_active=FALSE;
  close(dsp_fd);
  close(file_info->handle);
}
     
void
play_dsp( void )
{
  if (file_info->len > 0) {
    play_buffer();
  } else {
    stop_dsp();
    send_tcx_reply( dsp_module );
  }
}

#endif
