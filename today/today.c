
#include <stdio.h>
#include <stdlib.h>
#include <rai.h>
#include <string.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <math.h>
#include <sys/time.h>
#include "devUtils.h"

#include "tcx.h"
#include "tcxP.h"
#include "global.h"
#include "headClient.h"


#include "EZX11.h"
#include "o-graphics.h"
#include "librobot.h"


#define TCX_define_variables /* this makes sure variables are installed */
#define DEFINE_REPLY_HANDLERS /* this makes sure we'll get the handler array */
#include "LASER_SERVER-messages.h"
#include "FACE-messages.h"

/* extern int listen_for_tcx_events; */

#define NUMBER_LASERS 180
float value[NUMBER_LASERS];
float angle[NUMBER_LASERS];


#define MAX_LASER_RANGE 500

int ROBOT_BACKGROUND = 0;
int ROBOT = 0;
int MARKER = 0;
int EXIT_BUTTON = 0;
int ISON_BUTTON = 0;
int UPDATE_BUTTON = 0;
#define ROBOT_RADIUS 30.0
#define TCX_CLIENT_MODULE_NAME "today"

#define FROM_READING 0
#define TO_READING   179
#define WINDOW_SIZE 2		/* size of search window */

#define MOUTH_SMILE 40.0
#define MOUTH_NEUTRAL -10.0
#define MOUTH_FROWN -60.0
#define DIST_FROWN 40.0
#define DIST_SMILE DIST_FROWN
#define DIST_NEUTRAL 150.0

#define DIST_NEUTRAL_TILT 100.0

void connect_to_FACE();

int use_headclient = 0;
int is_on = 1;
int update_display = 0;

/* ********************************************************************** */

void init_graphics(int display)
{
  int i;
  static char *myfonts[] = {"5x8", "6x10", "7x13bold", 
			    "9x15", "10x20", "12x24", 
			    "lucidasans-bold-18"};


  for (i = 0; i != NUMBER_LASERS; i++){
    angle[i] = -90.0 + (((float) i) * 180.0 /
			((float) (NUMBER_LASERS-1)));
    value[i] = MAX_LASER_RANGE * 0.5 * (1.0 + sin(((float) i) / 4.0));
  }


  G_set_display(display);

  G_initialize_fonts(7, myfonts);
  G_initialize_graphics("LASER", 50.0, 10.0, C_STEELBLUE4);
  G_set_matrix_display_style(1);
  
  
  /******** ROBOT_BACKGROUND **************************************/
  {
    int switch_num                      = 1;
    static float switch_pos[]           = {0.0, 3.9, 6.0, 9.9};
    static char *switch_texts[]         = {""};
    static int switch_fonts[]           = {2};
    static int switch_background_color[]= {C_GREY70};
    static int switch_frame_color[]     = {C_GREY40};
    static int switch_text_color[]      = {NO_COLOR};
    
    ROBOT_BACKGROUND = G_create_switch_object(switch_pos, switch_num, 
						    switch_texts,
						    switch_background_color,
						    switch_frame_color, 
						    switch_text_color,
						    switch_fonts);
    
  }
  
  /******** ROBOT *************************************/
  {
    static float pos_r[]                 = {0.0, 3.9, 6.0, 9.9};
    static char *text_r                  = "ROBOT";
    static int robot_font                = 4;
    static int colors_r[]                = {NO_COLOR, NO_COLOR, C_GREY90,
					      C_BLACK, C_TURQUOISE4, C_GREY40,
					      NO_COLOR, NO_COLOR};
    
    ROBOT =  G_create_robot_object(pos_r, text_r, 
				   -(MAX_LASER_RANGE) * 1.05,
				   (MAX_LASER_RANGE) *1.05,
				   -(MAX_LASER_RANGE) *1.05,
				   (MAX_LASER_RANGE) *1.05,
				   0.0, 0.0, 90.0, 
				   ROBOT_RADIUS,
				   NUMBER_LASERS,
				   MAX_LASER_RANGE,
				   value,
				   angle,
				   colors_r, robot_font);
  }    

  /******** MARKER *************************************************/
  {
    static float pos_mar[]                 = {0.0, 3.9, 6.0, 9.9};
      int num_mar                            = 1;
      static char *text_mar[]                = {"MARKERS"};
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = {C_YELLOW};
      static int mar_text_color[]            = {NO_COLOR};
      static int mar_fonts[]                 = {2};

      MARKER = G_create_markers_object(pos_mar, 1, ROBOT_RADIUS * 0.3,
				    -(MAX_LASER_RANGE) * 1.05,
				   (MAX_LASER_RANGE) *1.05,
				   -(MAX_LASER_RANGE) *1.05,
				   (MAX_LASER_RANGE) *1.05,
				    num_mar, text_mar, mar_background_color, 
				    mar_frame_color, mar_foreground_color, 
				    mar_text_color, mar_fonts);
      
    }
   
  /******** EXIT_BUTTON **************************************/
  {
    
    int switch_num                      = 2;
    static float switch_pos[]           = {0.0, 3.9, 5.5, 5.9};
    static char *switch_texts[]         = 
      {"exit", "exit"};
    static int switch_fonts[]           = {2,2};
    static int switch_background_color[]= {C_GREY90, C_RED};
    static int switch_frame_color[]     = {C_GREY40, C_GREY40};
    static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
    EXIT_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						switch_texts,
						switch_background_color,
						switch_frame_color, 
						switch_text_color,
						switch_fonts);
  } 
  /******** ISON_BUTTON **************************************/
  {
    
    int switch_num                      = 2;
    static float switch_pos[]           = {0.0, 3.9, 5.0, 5.4};
    static char *switch_texts[]         = 
      {"toggle (is OFF)", "toggle (is ON)"};
    static int switch_fonts[]           = {2,2};
    static int switch_background_color[]= {C_GREY90, C_YELLOW};
    static int switch_frame_color[]     = {C_GREY40, C_GREY40};
    static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
    ISON_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						switch_texts,
						switch_background_color,
						switch_frame_color, 
						switch_text_color,
						switch_fonts);
  } 
  /******** UPDATE_BUTTON **************************************/
  {
    
    int switch_num                      = 2;
    static float switch_pos[]           = {0.0, 3.9, 4.5, 4.9};
    static char *switch_texts[]         = 
      {"update display (is OFF)", "update display (is ON)"};
    static int switch_fonts[]           = {2,2};
    static int switch_background_color[]= {C_GREY90, C_YELLOW};
    static int switch_frame_color[]     = {C_GREY40, C_GREY40};
    static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
    UPDATE_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						switch_texts,
						switch_background_color,
						switch_frame_color, 
						switch_text_color,
						switch_fonts);
  } 
  
  G_display_all();
  G_display_switch(ISON_BUTTON, is_on);
  G_display_switch(UPDATE_BUTTON, update_display);

}


/************************************************************************
 *   FUNCTION:     tests mouse and executes whatever we'd like
 *   RETURN-VALUE: 1, if meaningful mouse event found, 0 otherwise
 ************************************************************************/

int test_mouse(int display) {
  G_mouse_ptr mouse_events;
  int num_mouse_events = 0, button = 0;
  float mouse_x = 0.0, mouse_y = 0.0;
  int  number = 0;
  int return_value = 0;
  int mouse_event_detected = 0;

#if TOTAL_debug
  fprintf(stdout,"\n--->test_mouse\n");
  fflush( stdout );
#endif


  if (display){
    /****************** CHECK FOR MOUSE EVENT *******************/

    mouse_event_detected = G_test_mouse(0);
    if (mouse_event_detected){


      mouse_events = G_evaluate_mouse(&mouse_x, &mouse_y, 
				      &button, &num_mouse_events);
      
      if (mouse_event_detected == 1){ /* reglar mouse press */

	return_value = 1;
	
	
	/****************** EVALUATE MOUSE EVENT *******************/

	/* ******* EXIT_BUTTON ******** */
	
	if (G_mouse_event_at(EXIT_BUTTON, mouse_events, &number)){ 
	  if (use_headclient)
	    head_disconnect();
	  exit(-1);
	}	  

	/* ******* ISON_BUTTON ******** */
	
	else if (G_mouse_event_at(ISON_BUTTON, mouse_events, &number)){ 
	  is_on = 1 - is_on;
	  G_display_switch(ISON_BUTTON, is_on);
	  if (is_on)
	    fprintf(stderr, "Head animation is now ON.\n");
	  else
	    fprintf(stderr, "Head animation is now OFF.\n");

	}	  

	/* ******* UPDATE_BUTTON ******** */
	
	else if (G_mouse_event_at(UPDATE_BUTTON, mouse_events, &number)){ 
	  update_display = 1 - update_display;
	  G_display_switch(UPDATE_BUTTON, update_display);

	}	  

	/********* JUST UPDATE DISPLAY *********/
	
	else if (button == RIGHT_BUTTON){
	  G_display_all();
	}
	
      }
    }
  }
  return return_value;

}



#ifdef JUNK

void
process_input(char *command)
{
  int i;


  /* clean up command */

  for (i = 0; i < (int) strlen(command); i++)
    if (command[i] == '\n') 
      command[i] = 0x0;

  /* acknowledge */

  /*fprintf(stderr, "Received User Command: [%s]\n", command);*/

  /* act */


  if (command[0] == 0x0){
    is_on = 1 - is_on;
    if (is_on)
      fprintf(stderr, "Head animation is now ON.\n");
    else
      fprintf(stderr, "Head animation is now OFF.\n");
  }
  else if (command[0] == '.'){
    printf("Done.\n");
    exit(-1);
  }

  
}

void 
stdin_inputHnd(int fd, long chars_available)
{
  static char buffer[DEFAULT_LINE_LENGTH+1];
  static char *startPos = buffer; /* position to start parsing from */
  static char *endPos = buffer; /* position to add more characters */
  char *lineEnd;
  int numRead = 0;
  /* should handle characters output by the base */
  
  /* never expect more than DEFAULT_LINE_LENGTH characters on a line.
   * read the first DEFAULT_LINE_LENGTH and let the function get called 
   * again for any remaining characters.  This can be changed.
   */
  
  if (startPos == endPos)
    { 
      startPos = endPos = buffer;
      bzero(buffer, DEFAULT_LINE_LENGTH+1);
    }
  
  /* read in the command. */
  numRead = readN(&stdin_device, endPos, 
		  MIN(chars_available,(DEFAULT_LINE_LENGTH 
				       - (endPos - startPos))));
  endPos += numRead;
  if (numRead == 0)
    { /* handle error here. The port is already closed. */
    }
  else {
    /* see if we have a \n */
    lineEnd = (char *) strpbrk(startPos,"\n");
    while (lineEnd != NULL)
      {/* found a string, pass it to the parsing routines. */

	process_input(startPos);
	
	*lineEnd = '\0';
	startPos = lineEnd+1;
	lineEnd = (char *) strpbrk(startPos,"\n");
      }
    /* Fix up the buffer. Throw out any consumed lines.*/
    if (startPos >= endPos) 
      { /* parsed the whole thing, just clean it all up */
	bzero(buffer, DEFAULT_LINE_LENGTH+1);
	startPos = endPos = buffer;
      }
    else if (startPos != buffer)
      { /* slide it back and wait for more characters */
	bcopy(startPos, buffer, (endPos - startPos));
	endPos = buffer + (endPos - startPos);
	startPos = buffer;
      }
  }
  fprintf (stderr, "\n>");
}

#endif


/************************************************************************
 *
 *   Name:         CLIENT_close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void 
CLIENT_close_handler(char *name, TCX_MODULE_PTR module)
{
  fprintf(stderr, "CLIENT: closed connection detected: %s\n", name);
  
  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    if (use_headclient)
      head_disconnect();
    exit(0);
  }
  else if (!strcmp(name, TCX_LASER_SERVER_MODULE_NAME)){ /* server shut down */
    LASER_SERVER = NULL;		/* prevents us from sending messages */
  }
  else if (!strcmp(name, TCX_FACE_MODULE_NAME)){ /* server shut down */
    FACE = NULL;		/* prevents us from sending messages */
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

#define MAX_STEP_SIZE 700.0
#define TIME_DELAY 0.1
#define TIME_DELAY_PAN 2.0
#define TIME_DELAY_TILT 0.5
#define VERBOSE 0

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

  if (!is_on)
    return;

  if (!use_headclient && !FACE)
    connect_to_FACE();
  if (!use_headclient && !FACE)
    return;

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
   * move neck (pan)
   */

  time_difference = 
    ((float) (this_time.tv_sec - last_time_pan.tv_sec))
    + (((float) (this_time.tv_usec - last_time_pan.tv_usec))
       /  1000000.0);
  if (time_difference >= TIME_DELAY_PAN){
    diff_pan = 0.8 * prev_eyes;
    prev_pan += diff_pan;
    diff_eyes -= diff_pan;

    s_pan     = (int) (prev_pan * 1700.0 / 90.0);
    if (VERBOSE) fprintf(stderr, "p");
    sprintf(commandtext, "PP%d", s_pan);
    head_pantilt_command(commandtext);

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
    if (VERBOSE) fprintf(stderr, "F");

    if (use_headclient)
      head_move_eyes(s_eyes);
    else if (FACE)
      tcxSendMsg(FACE, "FACE_set_eyes", &s_eyes);

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
    if (use_headclient)
      head_move_brows(s_brows);
    else if (FACE)
      tcxSendMsg(FACE, "FACE_set_brows", &s_brows);
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
    if (use_headclient)
      head_move_mouth(s_mouth);
    else if (FACE)
      tcxSendMsg(FACE, "FACE_set_mouth", &s_mouth);
    last_time_mouth.tv_sec  = this_time.tv_sec;
    last_time_mouth.tv_usec = this_time.tv_usec;
  }



  /*
   * move neck (tilt)
   */

  time_difference = 
    ((float) (this_time.tv_sec - last_time_tilt.tv_sec))
    + (((float) (this_time.tv_usec - last_time_tilt.tv_usec))
       /  1000000.0);
  if (time_difference >= TIME_DELAY_TILT){
    prev_tilt += diff_tilt;

    s_tilt     = (int) (prev_tilt * 1700.0 / 90.0);
    if (VERBOSE) fprintf(stderr, "t");
    sprintf(commandtext, "TP%d", s_tilt);
    head_pantilt_command(commandtext);

    last_time_tilt.tv_sec  = this_time.tv_sec;
    last_time_tilt.tv_usec = this_time.tv_usec;
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
   * display selection
   */
  
  if (update_display){
    G_clear_markers(MARKER);
    G_add_marker(MARKER, 0.0, 0.0, 0);
    G_add_marker(MARKER, 
		 best_value * cos(((float) best_i) * M_PI / 180.0),
		 best_value * sin(((float) best_i) * M_PI / 180.0),
		 0);
    G_display_markers(MARKER);
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


  /*  fprintf(stderr, "\n%g %g %g %g    %g %g\n", dist, angle, x, y, angle_adj-angle, dist_adj-dist); */

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
}



/************************************************************************
 *
 *   NAME: LASER_SERVER_sweep_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/




void LASER_SERVER_sweep_reply_handler( TCX_REF_PTR            ref,
				 LASER_SERVER_sweep_reply_ptr sweep)
{
  /* 
     char *xx;
     xx = (char *) malloc(1024);
     fprintf(stderr, " %p ", xx);
     free(xx);
     */
  int i;


  if (VERBOSE)
    fprintf(stderr, "%d", sweep->numLaser);
  if (sweep->numLaser == 1){
    fprintf(stderr, "Currently unable to handle 2 lasers ;-( \n");
    exit(-1);
  }

  for (i = 0; i < NUMBER_LASERS; i++)
    if (sweep->value[i] > MAX_LASER_RANGE)
      value[i] = MAX_LASER_RANGE;
    else
      value[i] = sweep->value[i];


  if (update_display){
    G_display_switch(ROBOT_BACKGROUND, 0);
    G_display_robot(ROBOT, 0.0, 0.0, 90.0, NUMBER_LASERS, &(value[0]));
  }

  /*
   * find person
   */ 

  find_person();
  
  tcxFree("LASER_SERVER_sweep_reply", sweep); /* very important - free memory! */
}

void 
FACE_state_reply_handler(TCX_REF_PTR ref, FACE_state_ptr data)
{
  fprintf(stderr, "-1-");
  tcxFree("FACE_state_reply", data);
}

void 
FACE_init_reply_handler(TCX_REF_PTR ref, int *data)
{
  fprintf(stderr, "-2-");
  tcxFree("FACE_init_reply", data);
}

void 
FACE_status_update_handler(TCX_REF_PTR ref, FACE_state_ptr data)
{
  fprintf(stderr, "-3-");
  tcxFree("FACE_status_update", data);
}


/************************************************************************
 *
 *   Name:         connect_to_LASER_SERVER()
 *                 
 *   FUNCTION:     Attempts to establish connection to server.
 *                 subscribes to server messages, if connection established.
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void
connect_to_LASER_SERVER()
{
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference;
  LASER_SERVER_register_auto_update_type subscribe;

  /*
   * are we already connected? If so, don't do anything
   */

  if (LASER_SERVER)
    return;


  /*
   * check time - return if we tried this too recently
   */

  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);
  if (time_difference < 3.0)	/* only once every 3 seconds */
    return;


  /*
   * okay, let's try to connect
   */


  LASER_SERVER = tcxConnectOptional(TCX_LASER_SERVER_MODULE_NAME);


  /*
   * did we succeed?
   */


  if (LASER_SERVER){


    /*
     * Yes! Then let's subscribe to the server messages
     */

    subscribe.sweep0   = 1;
    subscribe.sweep1   = 1;


    tcxSendMsg(LASER_SERVER, "LASER_SERVER_register_auto_update", &subscribe);
  }


  /*
   * update internal timer
   */

  last_time.tv_sec  = this_time.tv_sec;
  last_time.tv_usec = this_time.tv_usec;
}


/************************************************************************
 *
 *   Name:         connect_to_FACE()
 *                 
 *   FUNCTION:     Attempts to establish connection to server.
 *                 subscribes to server messages, if connection established.
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void
connect_to_FACE()
{
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference;
  
  /*
   * headclient currently doesn't use TCX
   */

  if (use_headclient)
    return;

  /*
   * are we already connected? If so, don't do anything
   */

  if (FACE)
    return;


  /*
   * check time - return if we tried this too recently
   */

  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);
  if (time_difference < 3.0)	/* only once every 3 seconds */
    return;


  /*
   * okay, let's try to connect
   */


  FACE = tcxConnectOptional(TCX_FACE_MODULE_NAME);


  /*
   * did we succeed?
   */


  if (FACE){

    /*
     * Yes! Then let's subscribe to the server messages
     */

    set_face(0.0, 0.0, MOUTH_NEUTRAL, 10.0);



  }


  /*
   * update internal timer
   */

  last_time.tv_sec  = this_time.tv_sec;
  last_time.tv_usec = this_time.tv_usec;
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
init_tcx()
{
  static int initialized = 0;
  const char *tcxMachine = NULL;

  /*
   * compose message array - add all messages you want TCX to recognize here
   */ 

  TCX_REG_MSG_TYPE TCX_message_array[] = {
    LASER_SERVER_messages,
    FACE_messages
    /*
     * add further messages here
     */
    };

  /*
   * sanity check
   */

  if (initialized){
    fprintf(stderr, "TCX already intialized.\n");
    return;
  }
  initialized = 1;


  /*
   * where is TCX running
   */

  tcxMachine = getenv("TCXHOST");   

  /*
   * sanity check
   */

  if (!tcxMachine){
    fprintf(stderr, "ERROR: You must specify the environment variable TCXHOST.\n");
    exit(-1);
  }

  
  /*
   * connect to TCX
   */

  fprintf(stderr, "Connecting to TCX...");
  tcxInitialize(TCX_CLIENT_MODULE_NAME, (char *) tcxMachine);
  fprintf(stderr, "done.\n");

  /*
   * register messages - this must be done exactly once!
   */
    
  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));


    
  /*
   * register handler - there might be multiple calls of this command
   */ 
  

  tcxRegisterHandlers(LASER_SERVER_reply_handler_array, 
			sizeof(LASER_SERVER_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(FACE_reply_handler_array, 
			sizeof(FACE_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));


  /*
   * add other handler arrays here
   */
  
  /*
   * register close handler
   */ 
  

  tcxRegisterCloseHnd(CLIENT_close_handler);

  /*
   * connect to server
   */

  connect_to_LASER_SERVER();
  connect_to_FACE();
}


/************************************************************************
 *   FUNCTION:     main routine
 ************************************************************************/


int main(int argc, char *argv[]) {
  int i, display = 1;
  struct timeval block_waiting_time;
  struct timeval tcx_waiting_time;
  fd_set readMask;


  for (i = 1; i < argc; i++) {
    if ((strcasecmp(argv[i],"-nodisplay")==0) ||
	(strcasecmp(argv[i],"-nd")==0)){
      display = 0;
    }
    else if ((strcasecmp(argv[i],"-headclient")==0) ||
	     (strcasecmp(argv[i],"-hc")==0)){
      use_headclient = 1;
    }
    else {
      fprintf(stderr, "Usage: %s [-nodisplay] [-headclient]\n", argv[0]);
      exit(0);
    }
  }


  init_tcx();

  if (use_headclient){
    char commandtext[128];
    if (head_connect()){
      fprintf(stderr, "Failure to establish connection to the headClient.\n");
      fprintf(stderr, "You have to start up 'headClient' first.\n");
      exit(-1);
    }      
    sprintf(commandtext, "PS2900");
    head_pantilt_command(commandtext);
    sprintf(commandtext, "TS2900");
    head_pantilt_command(commandtext);
  }

  init_graphics(display);

#ifdef JUNK
  listen_for_tcx_events = 1;
  devInit();
  connectDev(&stdin_device);
  stdin_device.outputHnd = stdin_inputHnd;
#endif

  if (is_on)
    fprintf(stderr, "\nHead animation is now ON.\n");
  else
    fprintf(stderr, "\nHead animation is now OFF.\n");

  for (;;){
    /*
     * connect to server
     */
    
    connect_to_FACE();
    connect_to_LASER_SERVER();

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

    /*
     * do stuff
     */
    test_mouse(display);
  }
}
