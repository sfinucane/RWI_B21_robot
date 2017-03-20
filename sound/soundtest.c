#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>

#define TCX_define_variables 
#define DEFINE_REPLY_HANDLERS

#include "tcx.h"
#include "tcxP.h"
#include "SOUND-messages.h"

struct timeval TCX_waiting_time;

char* MODULE_NAME="SOUNDTEST";

void Connect2TCX ( void )
{
 TCX_REG_MSG_TYPE TCX_message_array[] = 
   {
    SOUND_messages
   };
 
 /* Connect to tcxServer */
 tcxInitialize( MODULE_NAME, (char *) getenv("TCXHOST"));
 
 /* Register all messages. */
 tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		     / sizeof(TCX_REG_MSG_TYPE));
 
 /* Register all handlers. */
 tcxRegisterHandlers(SOUND_reply_handler_array,
		     sizeof(SOUND_reply_handler_array)
		     / sizeof(TCX_REG_HND_TYPE));
}

void 
Connect2SOUND( void )
{
 fprintf( stderr, "Connecting to SOUND ..." );
 SOUND = tcxConnectModule ( TCX_SOUND_MODULE_NAME );
 fprintf( stderr, "OK \n" );
}

/**************************************************************************
 * REPLY_handlers                                                         *
 **************************************************************************/
void 
SOUND_play_reply_handler(TCX_REF_PTR ref, void *data)
{
 fprintf( stderr, "Recieving SOUND_play_reply\n" );
 tcxFree( "SOUND_play_reply", data );
 exit(0);
}

void
SOUND_playing_reply_handler(TCX_REF_PTR ref, SOUND_playing_reply_ptr data)
{
 tcxFree( "SOUND_playing_reply", data );
}

void
sound_play( char *play )
{
 fprintf( stderr, "Playing \"%s\"\n", play );
 tcxSendMsg( SOUND, "SOUND_play_query", &play );
}

void
sound_stop( void )
{
 fprintf( stderr, "Stop sound ...\n" );
 tcxSendMsg( SOUND, "SOUND_stop_query", NULL );
}

void
sound_talk( char *play )
{
 fprintf( stderr, "Talking \"%s\"\n", play );
 tcxSendMsg( SOUND, "SOUND_talk_query", &play );
}


int
main( int argc, char *argv[] )
{
 int i;
 int play = FALSE;
 int talk = FALSE;
 int stop = FALSE;
 char playstr[256];
 char talkstr[256];
 if (argc>1 ) {
  for (i=1; i<argc; i++) {
   if ((strcmp(argv[i],"-talk")==0) && (argc>i+1)) {
    strcpy( talkstr, argv[i+1] );
    talk = TRUE;
    i++;
   } else if ((strcmp(argv[i],"-stop")==0)) {
     stop = TRUE;
     i++;
   } else { 
    if ((strcmp(argv[i],"-play")==0) && (argc>i+1)) {
     strcpy( playstr, argv[i+1] );
     play = TRUE;
     i++;
    } else { 
     fprintf( stderr, "Usage: %s [-talk TEXT] [-play PLAY] [-stop]\n",
	     argv[0] );
     exit(1);
    }
   }
  }
 } else {
  fprintf( stderr, "Usage: %s [-talk TEXT] [-play PLAY] [-stop]\n",
	  argv[0] );
  exit(1);
 }
 Connect2TCX();
 Connect2SOUND();
 if( talk )
   sound_talk( talkstr );
 if ( stop )
   sound_stop();
 if( play ) {
   sound_play( playstr );
   while( TRUE ) {
     tcxRecvLoop(&TCX_waiting_time);
     usleep((unsigned long) 500000); 
   }
 }
 return(0);
}
