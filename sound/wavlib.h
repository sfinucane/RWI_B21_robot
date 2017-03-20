#include <signal.h>
#include <termcap.h>
#include <sys/soundcard.h>

#define TRUE	1
#define FALSE	0

typedef struct{
	long	srate;
	short	chans;
	short	depth;
	long	bps;
	short	blkalign;
	long	len;
	long	sofar;
	int	handle;
	char	name[31];
	char   	head[43];
} wav_sig;

buffmem_desc	binfo;
count_info	cinfo;

int 	dsp_fd;
int 	*buffer;

short 	get_wav_format		(wav_sig *info);
int 	open_wav		(char *header, char file_name[]);
short 	get_wav_channels	(wav_sig *info);
long 	get_wav_srate		(wav_sig *info);
long 	get_wav_bps		(wav_sig *info);
long 	get_wav_len		(wav_sig *info);
int 	init_wav		(char file_name[], wav_sig *info);
void	get_wav_nextblock	(int handle, char *buffer);

int 	init_parm		(int argc, char *argv[], wav_sig *file_info);
int 	open_wav_rec		(wav_sig *info);

int bsize;

