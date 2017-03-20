
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** Reaction - React to visitors
 ***** Brian Rudy (brudy@praecogito.com)         
 *****
 ***** 0.64 5/3/2003
 ***** Removed all `''s from cue strings, as they were causing problems
 ***** with speakit2.
 *****
 ***** 0.63 11/16/2002
 ***** Fixed cue string section, re-adding sfx support
 *****
 ***** 0.62 7/5/2002
 ***** Commented out a section in shutitdown() to prevent laser and 
 ***** pantilt tcx handles from becoming NULL. Extensive testing has not 
 ***** been able to replicate the error after making this change.
 *****
 ***** 0.61 6/14/2002
 ***** Now runs shutitdown() for proper shutdown when shutdown signals are
 ***** received. Hopefully this will keep laserServer from crashing with a 
 ***** signal 13 (broken pipe) when the client disconnects.
 ***** 
 ***** 0.60 6/7/2002
 ***** Major re-write of all TCX functions. Removed all the old RAI stuff.
 *****
 ***** 0.57 6/5/2002
 ***** Added auto-calculation of the number of elements in each emotion 
 ***** string array. Cleaned up bad string skipping routine.
 *****
 ***** 0.56 5/10/2002
 ***** Updated max random numbers for all emotions after discovering
 ***** 'shyness' bug.
 *****
 ***** 0.55 12/29/2001
 ***** Fixed usePantilt switch usage for use without pantilt head.
 *****
 ***** 0.54 7/12/2001
 ***** Added command line switches for timeDelaySpeech, and timeDelayZap 
 ***** for tweaking w/o needing to recompile. Removed last remnants of 
 ***** original wander code. 
 *****
 ***** 0.53 6/27/2001
 ***** Added catch-up capability if current emotional_state has not been 
 ***** stated after TIME_DELAY_SPEECH timer has expired.
 *****
 ***** 0.52 6/25/2001 
 ***** Added TIME_DELAY_SPEECH for max speech rate to alleviate 
 ***** frequent speech over-queueing.
 *****
 ***** 0.51 5/31/2001
 ***** Using external 'speakit2.pl' to send speech data. No more crashes
 ***** when the face is offline or crashes.
 *****
 ***** 0.50 5/24/2001
 ***** Faster responses on emotional_state change. Random phrases. 
 ***** Modification to easysocket.h to allow disconnect, integrated 
 ***** connect/disconnect for speech so 'face' is not always connected.  
 *****
 ***** 0.40 5/21/2001
 ***** Combined pan and tilt functions so motion is more fluid, added 
 ***** jitter window adjustment. Removed most of remaining wander code. 
 ***** Added speech cues for distance. Added panning offset.
 *****
 ***** 0.35 5/18/2001 
 ***** Aims cameras at most likely 'person'. Pantilt aiming now works 
 ***** properly. 
 *****
 ***** 0.30 5/17/2001
 ***** First functional version
 *****
 *****
 ***** Todo: Add more context specific responses. Speech response
 ***** causes delay in pantilt tracking in 'Dynamic' connection mode. 
 ***** 
 ***** 
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/signal.h>
#include <sys/types.h>

#include "tcx.h"
#include "tcxP.h"
#include "global.h"

#include "devUtils.h"
#include <bUtils.h>

/*
#include <termios.h>
#include <fcntl.h>
#include <string.h>
*/

#include "easysocket.h"

#define TCX_define_variables 1
/* this makes sure variables are installed */
#define DEFINE_REPLY_HANDLERS 1 /* this makes sure we'll get the handler array */
#include "LASER_SERVER-messages.h"
#include "PANTILT-messages.h"

#include "beeSoftVersion.h"
#include "librobot.h"


#define TCX_USER_MODULE_NAME "REACTION"
#define NUMBER_OF_ROBOTS 20
#define MODULE_NAME_LENGTH 80

extern int  listen_for_tcx_events;	/* in devUtils.c */

int numberOfRobots = 0;

/* module pointers */
TCX_MODULE_PTR base[NUMBER_OF_ROBOTS];
TCX_MODULE_PTR pantilt[NUMBER_OF_ROBOTS];
TCX_MODULE_PTR laser[NUMBER_OF_ROBOTS];

/* module names */
char robotName[NUMBER_OF_ROBOTS][MODULE_NAME_LENGTH];
char baseName[NUMBER_OF_ROBOTS][MODULE_NAME_LENGTH];
char laserName[NUMBER_OF_ROBOTS][MODULE_NAME_LENGTH];
char pantiltName[NUMBER_OF_ROBOTS][MODULE_NAME_LENGTH];

int tcx_initialized = FALSE;
char update = FALSE;

float timeDelayZap;
float timeDelaySpeech;

int Dynamic;
int useLaser;
int usePantilt;
int zap;
int shuttingdown = 0;

#define MAX_STEP_SIZE 700.0
#define TIME_DELAY 0.1
#define TIME_DELAY_PAN 0.25
/* 
#define TIME_DELAY_PAN 2.0 
#define TIME_DELAY_TILT 0.5 
*/
#define VERBOSE 0
#define NUMBER_LASERS 180
float value[NUMBER_LASERS];
float angle[NUMBER_LASERS];


#define MAX_LASER_RANGE 500

#define FROM_READING 0
#define TO_READING   179
#define WINDOW_SIZE 2           /* size of search window */

#define MOUTH_SMILE 40.0   
#define MOUTH_NEUTRAL -10.0
#define MOUTH_FROWN -60.0
#define DIST_FROWN 45.0
#define DIST_SMILE DIST_FROWN
#define DIST_NEUTRAL 150.0

#define DIST_NEUTRAL_TILT 100.0

#define JITT_THRESHOLD 2.0    /* 2 degree jitter threshhold */
#define TIME_DELAY_ZAP 5.0
#define TIME_DELAY_SPEECH 2.0

float nTilt, nPan;
float oTilt, oPan;
int emotional_state;
int last_emotion;
int emoted = 1;
#define DIST_SAD 250.0

#define EMOTION_HAPPY 0
#define EMOTION_NEUTRAL 1
#define EMOTION_SAD 2
#define EMOTION_ANGRY 3
int NETSOCKET;

#define PAN_SCALING 1.4

/*  Usefull for debugging TCX message traffic */
/* #define TCX_debug 1 */

char *happystr[] = {
  "Hi there, how are you?",
  "Have you seen my I bo?",
  "Cant talk now, I have to see a human about a battery.",
  "Hello, my name is Zaw Zaw, whats yours?",
  "Do you like robots?",
  "Welcome to the Tech Museum of Innovation",
  "Have a nice day!",
  "I think Ill go over there.",
  "What kind of robot are you?",
  "Can someone direct me to an electrical outlet?",
  "Robots rock!",
  "Whats your name?",
  "%3CSABLE%3E%0A%3CAUDIO+SRC=%22http%3A%2F%2Fzazaconsole.exhibits.thetech.org%2F%7Ebrudy%2Fzaza%2Fsfx%2Fhelpyou-mono.wav%22%2F%3E%0A%3C%2FSABLE%3E%0A",
  "%3CSABLE%3E%0A%3CAUDIO+SRC=%22http%3A%2F%2Fzazaconsole.exhibits.thetech.org%2F%7Ebrudy%2Fzaza%2Fsfx%2FR2D2-mono.wav%22%2F%3E%0A%3C%2FSABLE%3E%0A",
  "%3CSABLE%3E%0A%3CAUDIO+SRC=%22http%3A%2F%2Fzazaconsole.exhibits.thetech.org%2F%7Ebrudy%2Fzaza%2Fsfx%2FHellosir-mono.wav%22%2F%3E%0A%3C%2FSABLE%3E%0A"
};

char *sadstr[] = {
  "Wont someone please come play with me?",
  "Im lost, can someone help me out?",
  "Is anyone out there?",
  "Can anyone hear me?",
  "Why wont anyone play with me?",
  "%3CSABLE%3E%0A%3CAUDIO+SRC=%22http%3A%2F%2Fzazaconsole.exhibits.thetech.org%2F%7Ebrudy%2Fzaza%2Fsfx%2Fspace-mono.wav%22%2F%3E%0A%3C%2FSABLE%3E%0A"
};   

char *angrystr[] = {
  "Please give me some room!",
  "Please step aside.",
  "Hey, back off!",
  "Dont crowd me!",
  "Hey, dont make me zap you!",
  "Please be nice!",
  "Thats not nice!",
  "What would your mother think?",
  "Hey, I am not being mean to you!",
  "Please let me pass!",
  "Thats really not nice!",
  "Please do not touch me, I bruise easily.",
  "%3CSABLE%3E%0A%3CAUDIO+SRC=%22http%3A%2F%2Fzazaconsole.exhibits.thetech.org%2F%7Ebrudy%2Fzaza%2Fsfx%2Fdoomed-mono.wav%22%2F%3E%0A%3C%2FSABLE%3E%0A",
  "%3CSABLE%3E%0A%3CAUDIO+SRC=%22http%3A%2F%2Fzazaconsole.exhibits.thetech.org%2F%7Ebrudy%2Fzaza%2Fsfx%2FYourFault-mono.wav%22%2F%3E%0A%3C%2FSABLE%3E%0A"
};

char *neutralstr[] = {
  "I see you over there!",
  "Please come here, I cant see you very well.",
  "Hello?",
  "I would come over there, but Im too slow.",
  "Ahhhh, reminds me of when I was just a bunch of parts."
};

#define N_HAPPY (sizeof(happystr)/sizeof(char *))
#define N_NEUTRAL (sizeof(neutralstr)/sizeof(char *))
#define N_ANGRY (sizeof(angrystr)/sizeof(char *))
#define N_SAD (sizeof(sadstr)/sizeof(char *))

int
block_wait( struct timeval *timeout, int tcx_initialized,
            int X_initialized);


int
moduleNumber(TCX_REF_PTR ref,
             TCX_MODULE_PTR *module,
             unsigned int numberOfRobots){
  /* determine the module */
  unsigned int i = 0;
  char found = FALSE;   
  while (i < numberOfRobots && !found)
    if (ref->module != module[i])
      i++;
    else
      found = TRUE;
    
  if (!found){
    fprintf(stderr, "Error: module not found\n");
    exit(0);
  }
  return i;
}


/* Not currently used */
void
swallowStatusReports()
{
  if ( tcx_initialized) {
    struct timeval TCX_waiting_time = {0, 0};
    tcxRecvLoop((void *) &TCX_waiting_time);
  }
}


/************************************************************************
 *
 *   NAME: verbal_reaction()
 *
 *
 *   RETURN-VALUE:
 *
 ************************************************************************/

void
verbal_reaction(float dist_adj, float angle_adj)
{
  float time_difference;
  struct timeval this_time;
  static struct timeval last_time_zap  = {0, 0};
  static struct timeval last_time_speak  = {0, 0};
  char tempstring[255];
  char speakstring[255] = "/home/bee/speakit2.pl ";
  char speakstringzap[255] = "/home/bee/speakit2.pl ";
  int randvar;


  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time_speak.tv_sec))
    + (((float) (this_time.tv_usec - last_time_speak.tv_usec))
       /  1000000.0);
 

    /* Happy */
    if((DIST_FROWN < dist_adj) && (dist_adj < DIST_NEUTRAL)) {
       emotional_state = EMOTION_HAPPY;
    }
    /* Angry */
    if(DIST_FROWN >= dist_adj) {
       emotional_state = EMOTION_ANGRY;
    }
    /* Neutral */
    if((DIST_NEUTRAL < dist_adj) && (dist_adj < DIST_SAD)) {
        emotional_state = EMOTION_NEUTRAL;
    }
    /* Sad */
    if(dist_adj >= DIST_SAD) {
        emotional_state = EMOTION_SAD;
    }    


    if (emotional_state != last_emotion) {
     emoted = 0;
     switch (emotional_state) {
       case EMOTION_HAPPY:
         if (last_emotion == EMOTION_ANGRY) {
         /*  sprintf(tempstring,"/EHThanks!\r\n"); */
           sprintf(tempstring,"Thanks!"); 
         } 
         else {
            /*  sprintf(tempstring, "/EH%s\r\n", happystr[rand() % 13]); */
              sprintf(tempstring, "%s", happystr[rand() % N_HAPPY]);
         }
         break;
      
      case EMOTION_NEUTRAL:
     /*    sprintf(tempstring, "/EN%s\r\n", neutralstr[rand() % 9]); */
         sprintf(tempstring, "%s", neutralstr[rand() % N_NEUTRAL]);
         break;

      case EMOTION_ANGRY:   
     /*    sprintf(tempstring, "/EA%s\r\n", angrystr[rand() % 11]); */
         randvar = rand() % N_ANGRY; 
         sprintf(tempstring, "%s", angrystr[randvar]);
         break;

      case EMOTION_SAD:
      /*   sprintf(tempstring, "/ES%s\r\n", sadstr[rand() % 5]); */
         sprintf(tempstring, "%s", sadstr[rand() % N_SAD]);
         break;

      default:   
      /*   sprintf(tempstring, "/ENI am default. What does that mean?\r\n"); */
         sprintf(tempstring, "I am default. What does that mean?");
         break;
     }
 }
     

     /* Get verbal */
   if ((time_difference >= timeDelaySpeech) && (emoted == 0)) { 
     if (Dynamic) { 
        strcat(speakstring, tempstring);
        if (VERBOSE) { 
         fprintf(stderr, "Saying %s", speakstring);
        } 
        /* This is a kludgy workaround, for avoiding sending garbage
           to the face, can causing the MS speech engine to crash */
        if ((strlen(tempstring) >= 5) && (strcmp(tempstring, "4      ")) && (strcmp(tempstring, "4"))) {
        /*   fprintf(stderr, "Speakstring=%s, strlen=%d, emotional_state=%d\n", speakstring, strlen(tempstring), emotional_state); */
           system(speakstring);
           last_emotion = emotional_state;
           emoted = 1;
           last_time_speak.tv_sec  = this_time.tv_sec;
           last_time_speak.tv_usec = this_time.tv_usec;
        } 
        else { 
           fprintf(stderr, "tempstring is invalid, skipping: (%s)\n", tempstring);
           last_emotion = emotional_state;
           emoted = 1;
           last_time_speak.tv_sec  = this_time.tv_sec;
           last_time_speak.tv_usec = this_time.tv_usec;
        } 
     }
     else {
        if (VERBOSE) { 
         fprintf(stderr, "Saying %s", tempstring);
/*         fprintf(stderr, "dist_adj=%f, angle_adj=%f\n", dist_adj, angle_adj);  */  
        } 
        write_to_socket(NETSOCKET, tempstring, strlen(tempstring));
        last_emotion = emotional_state;   
        emoted = 1;
        last_time_speak.tv_sec  = this_time.tv_sec;
        last_time_speak.tv_usec = this_time.tv_usec;
     }
   }

  /* 5 second zap rate max */
  time_difference =
     ((float) (this_time.tv_sec - last_time_zap.tv_sec))
     + (((float) (this_time.tv_usec - last_time_zap.tv_usec))
      /  1000000.0);
  
   if (zap) {
    /* Zap the bad ones ;) */ 
    if (time_difference >= timeDelayZap) {
      if ((EMOTION_ANGRY == emotional_state) && (dist_adj <= 30.0)) {
       /*  fprintf(stderr, "\n\nI should be zapping now!\n\n"); */
        if (Dynamic) {
         /*  strcat(speakstring, "/EA/S0\r\n"); */
         /*  strcat(speakstringzap, "Excuse me!"); */
           strcat(speakstringzap,"%3CSABLE%3E%0A%3CAUDIO+SRC=%22http%3A%2F%2Fzazaconsole.exhibits.thetech.org%2F%7Ebrudy%2Fzaza%2Fsfx%2Felectricity-mono.wav%22%2F%3E%0A%3C%2FSABLE%3E%0A"); 
           system(speakstringzap);
         } else {
           write_to_socket(NETSOCKET, "/EA/S0\r\n", strlen("/EA/S0\r\n"));
         }       
      }
      last_time_zap.tv_sec  = this_time.tv_sec;
      last_time_zap.tv_usec = this_time.tv_usec;
    }
   }
}


/************************************************************************
 *
 *   NAME: set_face()
 *
 *
 *   RETURN-VALUE:
 *
 ************************************************************************/

void
set_face(float eyes, float brows, float mouth, float tilt)
{
  static float prev_eyes  = 0.0;
  static float prev_brows = 0.0;
  static float prev_mouth = 0.0;
  static float prev_pan = 0.0;
  static float prev_tilt = 0.0;
  static struct timeval last_time_eyes = {0, 0};
  static struct timeval last_time_brows = {0, 0};
  static struct timeval last_time_mouth = {0, 0};
  static struct timeval last_time_pan   = {0, 0};
  static struct timeval last_time_tilt  = {0, 0};
  struct timeval this_time;
  float diff_eyes  = eyes - prev_eyes - prev_pan;
  float diff_brows = brows - prev_brows;
  float diff_mouth = mouth - prev_mouth;
  float diff_pan   = 0.0;
  float diff_tilt  = tilt - prev_tilt;
  int s_eyes, s_brows, s_mouth, s_pan, s_tilt;   
  char commandtext[128];
  int  necksweep = 0;
  float time_difference;
  PANTILT_move_type ptdata;

  if (diff_eyes < -MAX_STEP_SIZE)
    diff_eyes = -MAX_STEP_SIZE;
  else if (diff_eyes > MAX_STEP_SIZE)   
    diff_eyes = MAX_STEP_SIZE;
  if (diff_brows < -MAX_STEP_SIZE)
    diff_brows = -MAX_STEP_SIZE;
  else if (diff_brows > MAX_STEP_SIZE)
    diff_brows = MAX_STEP_SIZE;
  if (diff_mouth < -MAX_STEP_SIZE)
    diff_mouth = -MAX_STEP_SIZE;
  else if (diff_mouth > MAX_STEP_SIZE)
    diff_mouth = MAX_STEP_SIZE;
  else if (diff_tilt > MAX_STEP_SIZE)   
    diff_tilt = MAX_STEP_SIZE;
  
    
  /*
   * check if we need to move neck
   */

  
  gettimeofday(&this_time, NULL);
  
  /*
   * move neck (pan/tilt)
   */
    
  time_difference =
    ((float) (this_time.tv_sec - last_time_pan.tv_sec))
    + (((float) (this_time.tv_usec - last_time_pan.tv_usec))
       /  1000000.0);
  if (time_difference >= TIME_DELAY_PAN){
    diff_pan = 0.8 * prev_eyes;
    prev_pan += diff_pan;
    diff_eyes -= diff_pan;
    prev_tilt += diff_tilt;
    
     
    if (prev_pan == 0.0) {
       nPan = 0.0;
    }
    else {
     /*  nPan = (-(prev_pan * M_PI * PAN_SCALING)/180); */
       nPan = -(prev_pan * PAN_SCALING);
    }

    if (prev_tilt == 0.0) {
       nTilt = 0.0;
    }
    else {
    /*   nTilt = ((prev_tilt * M_PI)/180); */
       nTilt = (prev_tilt);
    }

    /* Pan Tilt */

   /* Remove jitter */
    if(((prev_pan >= (oPan + JITT_THRESHOLD)) || (prev_pan <= (oPan - 
     JITT_THRESHOLD))) || ((prev_tilt >= (oTilt + JITT_THRESHOLD)) ||
     (prev_tilt <= (oTilt - JITT_THRESHOLD)))) {
      if (usePantilt && (pantilt[0] != NULL)) {
       /* fprintf(stderr, "Moving pantilt.\n"); */
        ptdata.pan_target  = nPan;
	ptdata.tilt_target = nTilt;
	tcxSendMsg(pantilt[0], "PANTILT_move", &ptdata);
      }
      if (VERBOSE) {
        fprintf(stderr, "\nprev_pan=%f, prev_tilt=%f\n", prev_pan,prev_tilt);
        fprintf(stderr, "nPan=%f, tTilt=%f\n", nPan, nTilt);
      }
      oPan = prev_pan;
      oTilt = prev_tilt;
    }
    else {
      if (VERBOSE) {
        fprintf(stderr, "\nPantilt request did not exceed jitter threshhold.\n");
      }
    }  
   
    necksweep = 1;
    last_time_pan.tv_sec  = this_time.tv_sec;
    last_time_pan.tv_usec = this_time.tv_usec;
  }
    
    


  /*
   * move eyes
   */
    
     
  time_difference =
    ((float) (this_time.tv_sec - last_time_eyes.tv_sec))
    + (((float) (this_time.tv_usec - last_time_eyes.tv_usec))
       /  1000000.0);
  if (time_difference >= TIME_DELAY || necksweep){
    prev_eyes += diff_eyes;
    s_eyes  = (int) prev_eyes;
/*    if (VERBOSE) fprintf(stderr, "F"); */
    /* move eyes */
    /*  head_move_eyes(s_eyes); */
  
    last_time_eyes.tv_sec  = this_time.tv_sec;
    last_time_eyes.tv_usec = this_time.tv_usec;
  }

  time_difference =
    ((float) (this_time.tv_sec - last_time_brows.tv_sec))
    + (((float) (this_time.tv_usec - last_time_brows.tv_usec))
       /  1000000.0);
  if (time_difference >= TIME_DELAY && fabs(diff_brows) >= 5.0){
    prev_brows += diff_brows;
    s_brows  = (int) prev_brows;
    if (VERBOSE) fprintf(stderr, "B");
    /* Move brows */
/*      head_move_brows(s_brows); */
    last_time_brows.tv_sec  = this_time.tv_sec;
    last_time_brows.tv_usec = this_time.tv_usec;
  }
       
  time_difference =
    ((float) (this_time.tv_sec - last_time_mouth.tv_sec))
    + (((float) (this_time.tv_usec - last_time_mouth.tv_usec))
       /  1000000.0);
  if (time_difference >= TIME_DELAY && fabs(diff_mouth) >= 5.0){
    prev_mouth += diff_mouth;
    s_mouth  = (int) prev_mouth;
    if (VERBOSE) fprintf(stderr, "M");
    /* Move mouth */
/*      head_move_mouth(s_mouth); */
    last_time_mouth.tv_sec  = this_time.tv_sec;
    last_time_mouth.tv_usec = this_time.tv_usec;
  }
    
}


/************************************************************************
 *
 *   NAME: find_person
 *
 *
 *   RETURN-VALUE:
 *
 ************************************************************************/

       
void 
find_person()
{
  int i, j;
  int div;  
  int best_i = -1;
  float v, best_value = 999999.9;
  float eyes, brows, mouth, tilt, angle_adj, dist_adj, angle, dist;
  float x, y; 
       
     
  for (i = FROM_READING; i <= TO_READING; i++){
    v = 0;
    div = 0;
    for (j = -WINDOW_SIZE; j <= WINDOW_SIZE; j++){
      if (i+j >= 0 && i+j < NUMBER_LASERS){
        v += value[i+j];
        div++;
      }
    }
    v = v / ((float) div);
    if (v < best_value){
      best_value = v;
      best_i = i;
    }
  }
  /*
   * compute person's location
   */
    
  angle = (90.0 - ((float) best_i));
  dist = best_value; 
  x =  20.0 + (dist * cos(angle * M_PI / 180.0));
  y =  0.0 + (dist * sin(angle * M_PI / 180.0));
   
  angle_adj = atan2(y, x) * 180.0 / M_PI;
  dist_adj  = sqrt((x*x)+(y*y));
  
  /* 
  if (VERBOSE)
  fprintf(stderr, "\n%g %g %g %g    %g %g\n", dist, angle, x, y, angle_adj-angle, dist_adj-dist);  
   */
  /*
   * set facial parameters
   */

  eyes = angle_adj;
  brows = 40.0;
  mouth = -60.0;

  if (dist_adj <= DIST_FROWN)
    mouth = MOUTH_FROWN;
  else if (dist_adj >= DIST_NEUTRAL)
    mouth = MOUTH_NEUTRAL;
  else
    mouth = ((dist_adj - DIST_SMILE) / (DIST_NEUTRAL - DIST_SMILE) * (MOUTH_NEUTRAL - MOUTH_SMILE)) + MOUTH_SMILE;
  
  if (dist_adj >= DIST_NEUTRAL_TILT)
    tilt = 0.0; 
  else
    tilt = 0.0 + (30.0 * (DIST_NEUTRAL_TILT - dist) / DIST_NEUTRAL_TILT);
   
    
  set_face(eyes, brows, mouth, tilt); 
  verbal_reaction(dist, angle_adj);
}


/***********************************************************************
 * Pantilt handlers
 ***********************************************************************/
   
void
PANTILT_position_reply_handler(TCX_REF_PTR                ref,
                               PANTILT_position_reply_ptr data)
{
  #ifdef TCX_debug
    fprintf(stderr, "TCX: Received a PANTILT_position_reply message.\n");
  #endif             
  tcxFree("PANTILT_position_reply", data);
  ref=ref;
}

void
PANTILT_init_reply_handler(TCX_REF_PTR             ref,
                           int                    *data)
{   
  #ifdef TCX_debug
    fprintf(stderr, "TCX: Received a PANTILT_init_reply message.\n");
  #endif
  tcxFree("PANTILT_init_reply", data);
  ref=ref;
}
  
void  
PANTILT_limits_reply_handler(TCX_REF_PTR              ref,
                             PANTILT_limits_reply_ptr data)
{   
  #ifdef TCX_debug
    fprintf(stderr, "TCX: Received a PANTILT_limits_reply message.\n");
  #endif
  tcxFree("PANTILT_limits_reply", data);
  ref=ref;
}

void
PANTILT_status_update_handler(TCX_REF_PTR              ref,
                              PANTILT_status_update_ptr data)
{     
  #ifdef TCX_debug
    fprintf(stderr, "TCX: Received a PANTILT_status_update message.\n");
    fprintf(stderr, "speed: %6.4f %6.4f\n",
          data->pan_velocity, data->tilt_velocity);
  #endif
  tcxFree("PANTILT_status_update", data);
  ref=ref;
}


/*------------------------------------------------------------*/
void LASER_SERVER_sweep_reply_handler( TCX_REF_PTR            ref,
                                   LASER_SERVER_sweep_reply_ptr sweep)
{
  int i;
    
  /*
  if (VERBOSE)
    fprintf(stderr, "%d", sweep->numLaser);
  */
  if (sweep->numLaser == 1){
    fprintf(stderr, "Currently unable to handle 2 lasers ;-( \n");
    exit(-1);
  }
  
  for (i = 0; i < NUMBER_LASERS; i++)
    if (sweep->value[i] > MAX_LASER_RANGE)
      value[i] = MAX_LASER_RANGE;
    else
      value[i] = sweep->value[i];

  /*
   * find person
   */
  
  find_person();
    
  tcxFree("LASER_SERVER_sweep_reply", sweep);
  /* very important - free memory! */
}

/************************************************************************
 * 
 *   Name:         REACTION_close_handler
 *
 *   FUNCTION:     handles a close message (special case)
 * 
 *   PARAMETERS:   standard TCX handler parameters
 *
 *   RETURN-VALUE:
 *
 ************************************************************************/  

  void
REACTION_close_handler(char *name, TCX_MODULE_PTR module)
{
  fprintf( stderr, "%s: closed connection detected: %s\n",
           TCX_USER_MODULE_NAME, name);

  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    exit(0);
  }
  else {
    if ( module == pantilt[0]) {
      fprintf( stderr, "%s disconnected.\n", pantiltName[0]);
      pantilt[0] = NULL;
    }
    if ( module == laser[0]) {
      fprintf( stderr, "%s disconnected.\n", laserName[0]);
      laser[0] = NULL;
    }
  }
  module=module;
}

/*
 * Just making sure
 */
void shutitdown() {
  shuttingdown = 1;
  /*
  fprintf( stderr, "Disconnecting %s.\n", pantiltName[0]);
  pantilt[0] = NULL;
  fprintf( stderr, "Disconnecting %s.\n", laserName[0]);
  laser[0] = NULL;
  */
  exit(0);
}

/************************************************************************
 *
 *   NAME:         init_tcx
 *
 *   FUNCTION:     connects to tcx and the two clients.
 *
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *
 *   RETURN-VALUE:
 *
 ************************************************************************/
       
void  
initTcx(char *moduleName)
{
  
  char *tcxMachine = NULL;

  /*
   * compose message array - add all messages you want TCX to recognize here
   */
   
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    LASER_SERVER_messages,
    PANTILT_messages
  };


  fprintf(stderr, "Connecting to TCX...");
  tcxMachine = getenv("TCXHOST");
    
  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }
    
  tcxInitialize(moduleName, (void *) tcxMachine);
  fprintf(stderr, "done.\n");

  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
                      / sizeof(TCX_REG_MSG_TYPE));

  tcxRegisterHandlers(LASER_SERVER_reply_handler_array, 
                      sizeof(LASER_SERVER_reply_handler_array)
                      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(PANTILT_reply_handler_array,
                      sizeof(PANTILT_reply_handler_array)
                      / sizeof(TCX_REG_HND_TYPE));
  
                      
  tcxRegisterCloseHnd(REACTION_close_handler);  
  tcx_initialized = TRUE;
}


/*****************************************************************************
 * Checks wether the connection to the module is established. If not the 
 * function tries to establish the connection.
 ****************************************************************************/
int
connectionEstablished( TCX_MODULE_PTR* module, char* name)
{ 
  if (!tcx_initialized)
    return FALSE;
  if ( *module == NULL) {
    *module = tcxConnectOptional( name);
    if (*module != NULL) {
      fprintf(stderr, "Connected to %s.\n", name);
      return TRUE;                   
    }
    else
      return FALSE;
  }
  else
    return TRUE;
}  



/*------------------------------------------------------------*/

void
main(int argc, char** argv )
{
  struct bParamList * params = NULL;
  LASER_SERVER_register_auto_update_type subscribe;
  struct timeval TCX_waiting_time = {0, 0};
  char *moduleName = TCX_USER_MODULE_NAME;
  char *name;
  struct timeval block_waiting_time;
  struct timeval tcx_waiting_time;
  int data;
  int wait;
  fd_set readMask;
  PANTILT_move_type pan_data;
  PANTILT_set_velocity_type ptVel;
  PANTILT_set_acceleration_type ptAccel;

  listen_for_tcx_events = TRUE;

  /* Shut down cleanly if we get a SIGHUP */
  signal(SIGHUP, shutitdown);
  signal(SIGTERM, shutitdown);
  signal(SIGINT, shutitdown);

  /* set defaults */

  params = bParametersAddEntry(params, "", "timeDelaySpeech", "2.0"); /* sec */
  params = bParametersAddEntry(params, "", "timeDelayZap", "5.0"); /* sec */

  params = bParametersAddEntry(params, "", "Dynamic", "Y");
  params = bParametersAddEntry(params, "", "useLaser", "Y");
  params = bParametersAddEntry(params, "", "usePantilt", "Y");
/*  params = bParametersAddEntry(params, "", "Zap", "n"); */
  params = bParametersAddEntry(params, "", "Zap", "Y"); 

  /* add some parameter files */
  params = bParametersAddFile(params, "etc/beeSoft.ini");

  /* add some enviroment variables */
  params = bParametersAddEnv(params, "", "TCXHOST");

  /* add command line arguements */
  params = bParametersAddArray(params, "", argc, argv);

  /* Fill the global parameter structure */
  bParametersFillParams(params);

  timeDelaySpeech = atof(bParametersGetParam(params, "", "timeDelaySpeech"));
  timeDelayZap = atof(bParametersGetParam(params, "", "timeDelayZap"));

  Dynamic    = bStrToTruth(bParametersGetParam(params, "", "Dynamic"));
  useLaser   = bStrToTruth(bParametersGetParam(params, "", "useLaser"));  
  usePantilt   = bStrToTruth(bParametersGetParam(params, "", "usePantilt"));  

  zap = bStrToTruth(bParametersGetParam(params, "", "Zap"));


  /* Set the robot names into the global structures. */
  numberOfRobots = 1;
  strcpy(robotName[0], "");
  fprintf(stderr, "# Displaying one robot!\n");
  if (useLaser) {
    tcxSetModuleName(TCX_LASER_SERVER_MODULE_NAME, NULL, laserName[0]);
  }
  if (usePantilt) {
    tcxSetModuleName(TCX_PANTILT_MODULE_NAME, NULL, pantiltName[0]);
  }
  name = "robot"; 

  fprintf(stderr, "Connecting to robot: %s.\n", robotName[0]);

  /* initializing TCX */
  initTcx(moduleName);


  fprintf(stderr, "\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "********  OPTIONS  **********\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "-timeDelaySpeech=%f\n", timeDelaySpeech);
  fprintf(stderr, "-timeDelayZap=%f\n",    timeDelayZap);
  fprintf(stderr, "\n");
  fprintf(stderr, "     -Dynamic=%d\n",    Dynamic);
  fprintf(stderr, "    -useLaser=%d\n",    useLaser);
  fprintf(stderr, "  -usePantilt=%d\n",    usePantilt);
  fprintf(stderr, "         -zap=%d\n",    zap);
  fprintf(stderr, "\n");
  fprintf(stderr, "*****************************\n");

  /*
   * Main loop
   */
  while(TRUE) {
    wait = TRUE;
    update = FALSE;
 
     /* Reconnection? */
   if (shuttingdown != 1) {
      if ( (usePantilt) && (pantilt[0] == NULL)) {
        pantilt[0] = tcxConnectOptional(pantiltName[0]);
        if (pantilt[0] != NULL){
          fprintf(stderr, "Connected to %s\n", pantiltName[0]);
          data = 1;
          tcxSendMsg(pantilt[0], "PANTILT_init_query", &data);
          sleep(1);
	  tcxSendMsg(pantilt[0], "PANTILT_stop_tracking", NULL);
          ptVel.pan_velocity = 70.0;
          ptVel.tilt_velocity = 70.0;
	  tcxSendMsg(pantilt[0], "PANTILT_set_velocity", &ptVel); 
          ptAccel.pan_acceleration = 135.0;  
          ptAccel.tilt_acceleration = 135.0;
	  tcxSendMsg(pantilt[0], "PANTILT_set_acceleration", &ptAccel); 
        }
      }
      if ( (useLaser) && (laser[0] == NULL)) {
        laser[0] = tcxConnectOptional(laserName[0]);
        if (laser[0] != NULL){
          fprintf(stderr, "Connected to %s\n", laserName[0]);
          subscribe.sweep0   = 1;
          subscribe.sweep1   = 1;
          tcxSendMsg(laser[0], "LASER_SERVER_register_auto_update", &subscribe);
        }
      }

    /*
     * do nothing for a while - "select" will automatically terminate
     * when a message has arrived, but won't use up precious CPU
     * time while waiting for it.
     */   
         
    block_waiting_time.tv_sec  = 0; 
    block_waiting_time.tv_usec = 100000;
    readMask = (Global->tcxConnectionListGlobal);
    select(FD_SETSIZE, &readMask, NULL, NULL, &block_waiting_time);
         
      
    /*  
     * query TCX
     */       
      
    tcx_waiting_time.tv_sec = 0;
    tcx_waiting_time.tv_usec = 0;
    tcxRecvLoop((void *) &tcx_waiting_time);
   }


    /* fprintf(stderr, "Reached end of loop...\n"); */
         	
  }
  exit(0);
}

