
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** Reaction - React to visitors
 *****
 ***** 0.51 5/31/2001
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
 ***** App crashes when 'face' crashes.
 ***** 
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <stdio.h>
#include <unistd.h>	
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include <baseClient.h>
#include <pantiltClient.h>
#include <laserClient.h>
#include <rai.h>

#include <bUtils.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <sys/signal.h>
#include <sys/types.h>
#include "easysocket.h"



float safetyMargin;
float exploreRange;
float safeDistance;
float maxSpeed;
float transAccel;
float rotSpeed;
float rotAccel;

int Dynamic;
int useLaser;
int usePantilt;
int zap;

#define MAX_STEP_SIZE 700.0
#define TIME_DELAY 0.1
#define TIME_DELAY_PAN 0.25
/* 
#define TIME_DELAY_PAN 2.0 
#define TIME_DELAY_TILT 0.5 
*/
#define VERBOSE 1
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
#define TIME_DELAY_SPEECH 5.0
float nTilt, nPan;
float oTilt, oPan;
int emotional_state;
int last_emotion;
#define EMOTION_DWELL 5.0  /* 5 seconds per emotion minimum */
#define DIST_SAD 250.0

#define EMOTION_HAPPY 0
#define EMOTION_NEUTRAL 1
#define EMOTION_SAD 2
#define EMOTION_ANGRY 3
int NETSOCKET;

char *happystr[] = {
  "Hi there, how are you?",
  "Have you seen my I bo?",
  "Can't talk now, I have to see a human about a battery.",
  "Hello, my name is Zaw Zaw, what's yours?",
  "Do you like robots?",
  "Welcome to the Tech Museum of Innovation",
  "Have a nice day!",
  "I think Ill go over there.",
  "What kind of robot are you?",
  "Can someone direct me to an electrical outlet?",
  "Robots rock!",
  "/S6",
  "What's your name?"
};

char *sadstr[] = {
  "Won't someone please come play with me?",
  "I'm lost, can someone help me out?",
  "Is anyone out there?",
  "Can anyone hear me?",
  "why wont anyone play with me?"
};   

char *angrystr[] = {
  "Please give me some room!",
  "Hey, back off!",
  "Dont crowd me!",
  "Hey, don't make me zap you!",
  "Please be nice!",
  "Thats not nice!",
  "What would your mother think",
  "Hey, I am not being mean to you!",
  "Please let me pass!",
  "That's really not nice!",
  "Please do not touch me, I bruise easily."
};

char *neutralstr[] = {
  "I see you over there!",
  "Please come here, I can't see you very well.",
  "Hello?",
  "I'd come over there, but I'm kind of slow.",
  "Ahhhh, reminds me of when I was just a bunch of parts...",
  "/S5",
  "/S7",
  "/S8",
  "/S9"
};

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
  static struct timeval last_time_speak  = {0, 0};
  char tempstring[128];
  char speakstring[128] = "/home/brudy/speakit2.pl ";


  gettimeofday(&this_time, NULL);
 
  /* 5 second response rate max */
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
     switch (emotional_state) {
       case EMOTION_HAPPY:
         if (last_emotion == EMOTION_ANGRY) {
           sprintf(tempstring,"/EHThanks!\r\n"); 
         } 
         else {
           /* sprintf(tempstring,"/EHI am happy!\r\n"); */
              sprintf(tempstring, "/EH%s\r\n", happystr[rand() % 13]);
         }
         break;
      
      case EMOTION_NEUTRAL:
        /* sprintf(tempstring, "/ENI am neutral...\r\n"); */
         sprintf(tempstring, "/EN%s\r\n", neutralstr[rand() % 9]);
         break;

      case EMOTION_ANGRY:   
         /* sprintf(tempstring, "/EAHey, back off!\r\n"); */
         sprintf(tempstring, "/EA%s\r\n", angrystr[rand() % 11]);
         break;

      case EMOTION_SAD:
       /*  sprintf(tempstring, "/ESI am sad. Please play with me!\r\n"); */
         sprintf(tempstring, "/ES%s\r\n", sadstr[rand() % 5]);
         break;

      default:   
         sprintf(tempstring, "/ENI am default. What does that mean?\r\n");
         break;
     }
     /* Get verbal */
     if (Dynamic) {
        strcat(speakstring, tempstring);
        if (VERBOSE) {
    /*   fprintf(stderr, "Number of array elements=%d, should be 9\n", size(neutralstr)); */
         fprintf(stderr, "Saying %s", speakstring);
         fprintf(stderr, "dist_adj=%f, angle_adj=%f\n", dist_adj, angle_adj);    
        }
        system(speakstring);
        last_emotion = emotional_state; 
     }
     else {
        if (VERBOSE) {
    /*   fprintf(stderr, "Number of array elements=%d, should be 9\n", size(neutralstr)); */
         fprintf(stderr, "Saying %s", tempstring);
         fprintf(stderr, "dist_adj=%f, angle_adj=%f\n", dist_adj, angle_adj);    
        }
        write_to_socket(NETSOCKET, tempstring, strlen(tempstring));
        last_emotion = emotional_state;   
     }
   }
   
   if (zap) {
    /* Zap the bad ones ;) */ 
    if (time_difference >= TIME_DELAY_SPEECH) {
      if ((EMOTION_ANGRY == emotional_state) && (dist_adj <= 30.0)) {
       /*  fprintf(stderr, "\n\nI should be zapping now!\n\n"); */
        if (Dynamic) {
           strcat(speakstring, "/EA/S0\r\n");
           sleep(1);           
           system(speakstring);
           sleep(1);           
         } else {
           write_to_socket(NETSOCKET, "/EA/S0\r\n", strlen("/EA/S0\r\n"));
         }       
      }
      last_time_speak.tv_sec  = this_time.tv_sec;
      last_time_speak.tv_usec = this_time.tv_usec;
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
       nPan = (-(prev_pan * M_PI * 1.3)/180);
    }

    if (prev_tilt == 0.0) {
       nTilt = 0.0;
    }
    else {
       nTilt = ((prev_tilt * M_PI)/180);
    }

    /* Pan Tilt */

   /* Remove jitter */
    if(((prev_pan >= (oPan + JITT_THRESHOLD)) || (prev_pan <= (oPan - 
     JITT_THRESHOLD))) || ((prev_tilt >= (oTilt + JITT_THRESHOLD)) ||
     (prev_tilt <= (oTilt - JITT_THRESHOLD)))) {
      ptMoveTo(nPan, nTilt);
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


/*------------------------------------------------------------*/
int myLaserSweepCallback ( laserSweepType *sweep ) {

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
    
/*  tcxFree("LASER_SERVER_sweep_reply", sweep); */
/* very important - free memory! */

  return(0);

}

/*------------------------------------------------------------*/

/*
void
demoPoll(RaiModule * demoModule)
{
  ageTargets();
}
*/

/*------------------------------------------------------------*/

void
createDemoModule()
{
/*  RaiModule* demoModule; */

  if (useLaser) {
    registerLaserSweepCallback(myLaserSweepCallback) ;
  }


  /* ask that your polling function be run every 500msec */
  /*  
   demoModule=makeModule("Demo", NULL); 
   addPolling(demoModule, demoPoll, 500);
  */
  
  return;
}

/*------------------------------------------------------------*/

void
commShutdown(char *name, TCX_MODULE_PTR module)
{

  fprintf( stderr, "%s(%s): %s died. Closing. \n", 
	   __FILE__, __FUNCTION__, name );

  module = module;		/* prevents gcc from barking with `-Wall' */
  fflush(NULL);
  RaiShutdown();
  exit(0);
}

/*------------------------------------------------------------*/

void
main(int argc, char** argv )
{
  struct bParamList * params = NULL;

  /* set defaults */

  params = bParametersAddEntry(params, "", "safetyMargin", "20"); /* cm */
  params = bParametersAddEntry(params, "", "exploreRange", "200"); /* cm */
  params = bParametersAddEntry(params, "", "transSpeed", "40"); /* cm/s */
  params = bParametersAddEntry(params, "", "transAccel", "80"); /* cm/s^2 */
  params = bParametersAddEntry(params, "", "rotSpeed", "30"); /* deg/s */
  params = bParametersAddEntry(params, "", "rotAccel", "80"); /* deg/s^2 */

  params = bParametersAddEntry(params, "", "Dynamic", "Y");
  params = bParametersAddEntry(params, "", "useLaser", "Y");
  params = bParametersAddEntry(params, "", "usePantilt", "Y");
  params = bParametersAddEntry(params, "", "zap", "n");

  /* add some parameter files */
  params = bParametersAddFile(params, "etc/beeSoft.ini");

  /* add some enviroment variables */
  params = bParametersAddEnv(params, "", "TCXHOST");

  /* add command line arguements */
  params = bParametersAddArray(params, "", argc, argv);

  /* Fill the global parameter structure */
  bParametersFillParams(params);

  fprintf(stderr, "safetyMargin = [%s]\n", 
	  bParametersGetParam(params, "", "safetyMargin"));

  safetyMargin = atof(bParametersGetParam(params, "", "safetyMargin"));
  exploreRange = atof(bParametersGetParam(params, "", "exploreRange"));
  maxSpeed     = atof(bParametersGetParam(params, "", "transSpeed"));
  transAccel   = atof(bParametersGetParam(params, "", "transAccel"));

  rotSpeed = atof(bParametersGetParam(params, "", "rotSpeed"))*M_PI/180.0;
  rotAccel = atof(bParametersGetParam(params, "", "rotAccel"))*M_PI/180.0;

  safeDistance = safetyMargin + bRobot.base_radius;

  Dynamic    = bStrToTruth(bParametersGetParam(params, "", "Dynamic"));
  useLaser   = bStrToTruth(bParametersGetParam(params, "", "useLaser"));  
  usePantilt   = bStrToTruth(bParametersGetParam(params, "", "usePantilt"));  

  zap = bStrToTruth(bParametersGetParam(params, "", "zap"));


  if (useLaser) {
    laserRegister();
  }

  if (usePantilt) {
     ptRegister();
  }
	
  initClient("Reaction", commShutdown); /* close function called if */
                                      /* the base server dies */


  if (usePantilt) {
    ptConnect(1);
    fprintf(stderr, "Connected to pantilt.\n");
  }
   
  if (useLaser) {
    laserConnect(1);   /* 1 -> wait until connection has been established */
    fprintf(stderr, "Connected to laserServer.\n");
    laserSubscribeSweep( 0, 1 );
    fprintf(stderr, "Subscribed to laser sweep.\n");
  } 
 
  if (usePantilt) {
    ptSetAccel(135.0*M_PI/180.0);
    ptSetVel  ( 70.0*M_PI/180.0);
  }

  RaiInit();                          /* init (but not start) scheduler*/
  catchInterrupts();

  initClientModules();                /* set up Rai modules to do      */
                                      /* communication for you */

  createDemoModule();                 /* set up your module to move robot*/

  if (!Dynamic) {
    NETSOCKET = make_client_connection("zaza2", 101, TCP);
    if (NETSOCKET <0) {perror("Hmm, bad...\n"); exit(-1); }
    fprintf(stderr, "The Socket is open...\n");
  }


  fprintf(stderr, "\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "********  OPTIONS  **********\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "-safetyMargin=%f\n",    safetyMargin);
  fprintf(stderr, "-exploreRange=%f\n",    exploreRange);
  fprintf(stderr, "  -transSpeed=%f\n",    maxSpeed);
  fprintf(stderr, "  -transAccel=%f\n",    transAccel);
  fprintf(stderr, "    -rotSpeed=%f\n",    rotSpeed * 180.0/M_PI);
  fprintf(stderr, "    -rotAccel=%f\n",    rotAccel * 180.0/M_PI);
  fprintf(stderr, "\n");
  fprintf(stderr, "     -Dynamic=%d\n",    Dynamic);
  fprintf(stderr, "    -useLaser=%d\n",    useLaser);
  fprintf(stderr, "  -usePantilt=%d\n",    usePantilt);
  fprintf(stderr, "         -zap=%d\n",    zap);
  fprintf(stderr, "\n");
  fprintf(stderr, "*****************************\n");

  RaiStart(); /* This will not return */
}

