#ifdef i386

/* operations for verifying and reading wav files. */

#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include "wavlib.h"

	/* wav header is 44 bytes long */
int open_wav(char *header, char file_name[]){
	int handle;
	handle = open(file_name, O_RDONLY, S_IREAD);
	read(handle,(char *) header, 44);

	return(handle);
}

short get_wav_format(wav_sig *info){
	return(*(short *)&info->head[20]);
}

	/* mono or stereo */
short get_wav_channels(wav_sig *info){
	return(*(short *)&info->head[22]);
}

	/* sample rate */
long get_wav_srate(wav_sig *info){
	return(*(long *)&info->head[24]);
}

	/* # of bytes/sec processed (for buffer estimation) */
long get_wav_bps(wav_sig *info){
	return(*(long *)&info->head[28]);
}
	/* minimum # of bytes per block for correct processing */
short get_wav_blkalign(wav_sig *info){
	return (*(long *)&info->head[32]);
}

	/* sample depth (8bit or 16bit) */
short get_wav_depth(wav_sig *info){
	return((short)info->head[34]);
}

	/* data section only  ==  totalfile - 44 */
long get_wav_len(wav_sig *info){
	return (*(long *)&info->head[40]);	
}


int init_wav_read(char file_name[], wav_sig *info){
	info->handle = open_wav(info->head, file_name);
	strcpy(info->name,file_name);
	info->chans = get_wav_channels(info);
 	info->srate = get_wav_srate(info);
 	info->bps   = get_wav_bps(info);
 	info->blkalign = get_wav_blkalign(info);
	info->depth = get_wav_depth(info);
	info->len   = get_wav_len(info);
	return(info->handle);
	
}

void init_head(wav_sig *info){
	strcpy(info->head, "RIFF    WAVEfmt                     data    ");
	*(long  *)&info->head[4]  = (info->sofar + 32);
	*(long  *)&info->head[16] = 16;
	*(short *)&info->head[20] = 1;
	*(short *)&info->head[22] = info->chans;
	*(long  *)&info->head[24] = info->srate;
	*(long  *)&info->head[28] = info->bps;
	*(short *)&info->head[32] = info->blkalign;
	*(short *)&info->head[34] = info->depth;
	*(long  *)&info->head[40] = info->sofar;
}

int open_wav_rec(wav_sig *info){
	info->handle = open(info->name, O_CREAT | O_RDWR, 0644);
	init_head(info);
	write(info->handle, info->head, 44);
	return(info->handle);
}

int rewrite_head(wav_sig *info){
	lseek(info->handle, 0, SEEK_SET);
	init_head(info);
	write(info->handle, info->head, 44);
}

#endif
