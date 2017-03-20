#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <math.h>

#include <bUtils.h>

#include "devUtils.h"
#include "pioneer.h"
#include "bee_interface.h"


struct timeval last_ping_time = {0, 0};

int use_tcx = 1;

struct bParamList * bParamList = NULL;

STATUS_TYPE status[NUMBER_STATUS];

int actual_status = 0;
int ignore_status_report = 3;	/* ignore the first 3! */
unsigned long   total_sonar_count = 0;

extern int listen_for_tcx_events;

extern uint useRobotServer;   // FRANK

int print_sonar_values = 0;
int print_status = 0;

static int communication_verbose = 0;

float actual_target_translate_velocity = 0.0;
float actual_target_rotate_velocity = 0.0;
int   last_command_type = 0;

static int pioneerVersion = 1;

/* These vars are init through the Pioneer.ini file */
int num_sonars= 0;
static float counts_per_cm= 0.0;
static float command_translational_velocity_factor= 0.0;
static float command_rotational_velocity_factor= 0.0;
float counts_per_degree = 0.0;
float drift_in_degree_per_cm = 0.0;

static float ROBOT_RADIUS= 0.0;
float VELCONV_FACTOR =0.0;
float STATUS_VELOCITY_FACTOR =0.0;
float MAX_VELOCITY =0.0;
float MAX_ORIENTATION =0.0;
float MAX_X_Y = 0.0;
float PIONEER_POSITION_OFFSET =0.0;
float MAX_TRANSLATIONAL_VELOCITY =0.0;
float MAX_ROTATIONAL_VELOCITY =0.0;
float MIN_BATTERY =0.0;
float MAX_BATTERY =0.0;
/* end */

extern struct timeval watchdog_timestamp;
extern int global_watchdog;

extern int global_set_tvel;
extern int global_tvel_sign;
extern int global_set_tvel_new;

extern int global_set_rvel;
extern int global_rvel_sign;
extern int global_set_rvel_new;

extern int global_relative_command;
extern float global_relative_origin_x;
extern float global_relative_origin_y;
extern float global_relative_dist;

extern int n_auto_update_modules;

#define BASE_BUFFER_SIZE DEFAULT_LINE_LENGTH

BASE_TYPE    base_device = 
{ 
  { FALSE,
    { "", DEFAULT_PORT},
    /* 14 is for a baud rate of 19200 */
    /*    { "/dev/cua1", 13}, */
    { "", 13}, 
    RWIBASE_DEV_NAME,
    -1,
    TRUE,
    FALSE,
    (FILE *) NULL,
    (fd_set *) NULL,
    (DEVICE_OUTPUT_HND) BASE_outputHnd,
    BASE_timeoutHnd,  
    (DEVICE_SET_TIMEOUT)  setTimeout,  
    (DEVICE_CANCEL_TIMEOUT) cancelTimeout,
    (void (*)(void)) NULL,  
    {0, 0},
    {LONG_MAX, 0},
    {0x2FFFFFFF, 0},
    (void (*)(void)) NULL,
    FALSE
  }
};


void
BASE_outputHnd(int fd, long chars_available)
{
  static unsigned char buffer[BASE_BUFFER_SIZE+1];
  static unsigned char *startPos = buffer; /* position to start parsing from */
  static unsigned char *startPos2 = buffer; 
  static unsigned char *endPos = buffer; /* position to add more characters */
  int numRead = 0;
  int i= 0, length= 0;
  unsigned int check= 0;
  int nextch= 0;
  int count= 0;

#if TOTAL_debug
  fprintf(stdout,"\n--->BASE_outputHnd\n");
  fflush( stdout );
#endif
  
  count = (int) chars_available;
  
  while (chars_available > 0) {
    
    if (startPos == endPos)
      { 
	startPos = endPos = buffer;
	bzero(buffer, BASE_BUFFER_SIZE+1);
      }
    
    
    /* read in the output. */
    numRead = readN(&base_device.dev, endPos, 
		    MIN(chars_available,(BASE_BUFFER_SIZE - 
					 (endPos - startPos))));

      
    endPos += numRead;
    
    if (numRead == 0)
      fprintf(stderr, "\n\t??? empty message ???\n");

    /*
     * Show buffer
     */
    
    if (communication_verbose){
      fprintf(stderr, "\nBuffer: [");
      for (startPos2 = startPos; startPos2 < endPos; startPos2++)
	fprintf(stderr, " %.2x", *startPos2);
      fprintf(stderr, "]\n"); 
    }

    /*
     * Remove irrelevant initial bytes
     */

    while ((*startPos != 0xfa || *(startPos+1) != 0xfb) &&
	   startPos + 1 < endPos){
      if (communication_verbose)
	fprintf(stderr, " <%.2x>", *startPos);
      startPos++;
    }

    /*
     * Check if we got a complete message here
     */

    while (endPos - startPos >= 5 && /* minimum length */
	   endPos - startPos >= *(startPos+2) + 3){ /* complete message */

      /*
       * Okay, we got a complete message, let's parse it
       */

      if (communication_verbose){
	fprintf(stderr, "\nParsing: [");
	for (startPos2 = startPos; startPos2 < endPos; startPos2++)
	  fprintf(stderr, " %.2x", *startPos2);
	fprintf(stderr, "]\n"); 
      }

      length = *(startPos+2);
      check = calc_checksum(startPos+2);
      if (communication_verbose)
	fprintf(stderr, "message length= %.2x\n",length); 

      if (check != ((unsigned int) (*(startPos+length+1))) * 256
	  + ((unsigned int) *(startPos+length+2))){
	if (communication_verbose)
	  fprintf(stderr, "CHECKSUM ERROR: (%.2x %.2x) %d != %d",
		  *(startPos+length+1),
		  *(startPos+length+2),
		  (*(startPos+length+1)) * 256
		  + *(startPos+length+2),
		  check);
	}
      else{
	if (communication_verbose)
	  fprintf(stderr, "success\n");
	ProcessLine(startPos+2);
      }
      /*
       * Okay, message parsed, remove it
       */
      startPos += length + 3;
      
      /*
       * Show the remaining buffer
       */
      if (communication_verbose){
	fprintf(stderr, "\nRemaining buffer: [");
	for (startPos2 = startPos; startPos2 < endPos; startPos2++)
	  fprintf(stderr, " %.2x", *startPos2);
	fprintf(stderr, "]\n"); 
      }

    }

    /* 
     * Fix up the buffer. Throw out any consumed lines.
     */
    if (startPos >= endPos){	/* parsed the whole thing, 
				 * just clean it all up */
      bzero(buffer, BASE_BUFFER_SIZE+1);
      startPos = endPos = buffer;
    }
    else if (startPos != buffer){/* slide it back and wait for 
				  * more characters */
      bcopy(startPos, buffer, (endPos - startPos));
      endPos = buffer + (endPos - startPos);
      startPos = buffer;
    }

    /*
     * Show the remaining buffer
     */
    if (communication_verbose){
      fprintf(stderr, "\nRemaining buffer: [");
      for (startPos2 = startPos; startPos2 < endPos; startPos2++)
	fprintf(stderr, " %.2x", *startPos2);
      fprintf(stderr, "]\n"); 
    }

    /*
     * Check if new characters arrived in the meantime
     */
    chars_available = numChars(fd);
    count--;
  }
}

void BASE_timeoutHnd(void)
{
  ;
}

void signal_base(void)
{
  fprintf(stderr,"shutting down the base\n");
}

/*	Function Name: ProcessLine
 *	Arguments:     line -- output line from the base
 *	Description:   process the base output
 *	Returns: 
 */


static void 
ProcessLine(unsigned char *line) 
{
  int length= 0;
  int type= 0;			/* not defined for status report */
  int i= 0, j= 0;
  int v = 0;			/* verbose command */
  int got_num_sonars= 0;
  unsigned char *ptr = line;
  static int count = 0;
  int verbose = 0;		/* verbose status */
  float help= 0.0;
  struct timeval this_status_time;
  float time_diff= 0.0, angle= 0.0;
  float diff_x= 0.0, diff_y= 0.0, diff_orientation= 0.0, diff= 0.0;
  float prev_vel_l= 0.0, prev_vel_r= 0.0;
  int  height= 0, width= 0, row_num= 0, num_rows= 0, compr_type= 0, num_bytes= 0;
  float R = 0.0, G = 0.0, B = 0.0;
  int status_error= 0;
  static int number_consecutive_status_errors = 0;

  static float total_l = 0.0;
  static float total_r = 0.0;
  static float total_v = 0.0;
  static float total = 0.0;

  unsigned short tmp_ushort= 0;
  short tmp_sshort= 0;

  int temp_status_defined= 0;
  float temp_status_target_translate_velocity= 0.0;
  float temp_status_target_rotate_velocity= 0.0;
  unsigned long temp_status_count= 0;
  int temp_status_wall_following_mode= 0;
  int temp_status_motors_status= 0;
  float temp_status_internal_x= 0.0;
  float temp_status_internal_y= 0.0;
  float temp_status_internal_orientation= 0.0;
  float temp_status_vel_r= 0.0;
  float temp_status_vel_l= 0.0;
  float temp_status_accel_l= 0.0;
  float temp_status_accel_r= 0.0;
  float temp_status_orientation= 0.0;
  float temp_status_x= 0.0;
  float temp_status_y= 0.0;
  float temp_status_vel_orientation= 0.0;
  float temp_status_vel_x= 0.0;
  float temp_status_vel_y= 0.0;
  float temp_status_battery= 0.0;
  float temp_left_whisker= 0.0;
  float temp_right_whisker= 0.0;
  int temp_status_left_stalled= 0;
  int temp_status_right_stalled= 0;
  float temp_status_sonars[num_sonars];
  int temp_status_sonar_confidence[num_sonars];
  float temp_status_sonar_new_flag[num_sonars];
  struct timeval temp_status_last_status_time;
  struct timeval temp_status_last_sync_time;
  float temp_total_l= 0.0;
  float temp_total_r= 0.0;
  float temp_total_v= 0.0;
  float temp_total= 0.0;
  unsigned long temp_total_sonar_count=0;
  
#if TOTAL_debug
  fprintf(stdout,"\n--->ProcessLine\n");
  fflush( stdout );
#endif

  length = ((int) *ptr++) - 2;
  type   = (int) *ptr;



  if (type == sfSYNC0 && length == 1){
    fprintf(stderr, "0");
    if (status[actual_status].sync_level == 0){
      gettimeofday(&status[actual_status].last_sync_time, NULL);
      status[actual_status].sync_level = 1;
    }
  }

  else if (type == sfSYNC1 && length == 1){
    fprintf(stderr, "1");
    if (status[actual_status].sync_level == 1){
      gettimeofday(&status[actual_status].last_sync_time, NULL);
      status[actual_status].sync_level = 2;
    }
    else
      status[actual_status].sync_level = 0;
  }

  else if (type == sfSYNC2 /*&& length == 1*/){

    /*
    fprintf(stderr, "2");
    if (length != 1){
      fprintf(stderr, "Status message: [");
    for (; length > 0; length--)
      fprintf(stderr, " %.2x", *ptr++);
    fprintf(stderr, "]\n");
    }
    */
    if (status[actual_status].sync_level == 2){
      gettimeofday(&status[actual_status].last_sync_time, NULL);
      status[actual_status].sync_level = 3;
    }
    else
      status[actual_status].sync_level = 0;
  }

  else if (type == DISTpax)
    {
      putc(7, stderr);
      fprintf(stderr,
	      "MESSAGE NOT IMPLEMENTED (type=%.2x, length=%.2x: [",
	      (unsigned char) type, (unsigned char) length);
      for (; length > 0; length--)
	fprintf(stderr, " %.2x", *ptr++);
      fprintf(stderr, "]\n");

    }
  
  
  else if (length == 0){
    /* fprintf(stderr, "sfEMPTY\n"); */
  }
  
  else if (type != 0x32 && type != 0x33 &&
	   type != 0x34){	/* this might be overly specific.
				 * the status report may change. But
				 * this makes sure we won't confuse anything 
				 * that we don't know. */
    fprintf(stderr,
	    "Received unidentified message (type=%.2x, length=%.2x): [",
	    (unsigned char) type, (unsigned char) length);
    for (; length > 0; length--)
      fprintf(stderr, " %.2x", *ptr++);
    fprintf(stderr, "]\n>");
    
  }

  else if (ignore_status_report == 0){ /* status report */
    if (verbose)
      fprintf(stderr, "sfSTATUS (type=%.2x length=%.2x)\n",
	      (unsigned char) type, (unsigned char) length);

    status_error = 0;

    /*
     * copy current status report
     */

    temp_status_defined                        = status[actual_status].defined;
    temp_status_target_translate_velocity      = status[actual_status].target_translate_velocity;
    temp_status_target_rotate_velocity         = status[actual_status].target_rotate_velocity;
    temp_status_count                          = status[actual_status].count;
    temp_status_motors_status                  = status[actual_status].motors_status;
    temp_status_internal_x                     = status[actual_status].internal_x;
    temp_status_internal_y                     = status[actual_status].internal_y;
    temp_status_internal_orientation           = status[actual_status].internal_orientation;
    temp_status_vel_r                          = status[actual_status].vel_r;
    temp_status_vel_l                          = status[actual_status].vel_l;
    temp_status_accel_l                        = status[actual_status].accel_l;
    temp_status_accel_r                        = status[actual_status].accel_r;
    temp_status_orientation                    = status[actual_status].orientation;
    temp_status_x                              = status[actual_status].x;
    temp_status_y                              = status[actual_status].y;
    temp_status_vel_orientation                = status[actual_status].vel_orientation;
    temp_status_vel_x                          = status[actual_status].vel_x;
    temp_status_vel_y                          = status[actual_status].vel_y;
    temp_status_battery                        = status[actual_status].battery;
    temp_status_left_stalled                   = status[actual_status].left_stalled;
    temp_status_right_stalled                  = status[actual_status].right_stalled;
    for (i = 0; i < num_sonars; i++){
      temp_status_sonars[i]                    = status[actual_status].sonars[i];
      temp_status_sonar_confidence[i]          = status[actual_status].sonar_confidence[i];
      temp_status_sonar_new_flag[i]            = status[actual_status].sonar_new_flag[i];
    }
    temp_status_last_status_time.tv_sec        = status[actual_status].last_status_time.tv_sec;
    temp_status_last_status_time.tv_usec       = status[actual_status].last_status_time.tv_usec;
    temp_status_last_sync_time.tv_sec          = status[actual_status].last_sync_time.tv_sec;
    temp_status_last_sync_time.tv_usec         = status[actual_status].last_sync_time.tv_usec;

    temp_total_l                               = total_l;
    temp_total_r                               = total_r;
    temp_total_v                               = total_v;
    temp_total                                 = total;
    temp_total_sonar_count                     = total_sonar_count;

    /*
     * GENERIC BASICS
     */
    temp_status_defined                    = 1;
    temp_status_target_translate_velocity  = actual_target_translate_velocity;
    temp_status_target_rotate_velocity     = actual_target_rotate_velocity;
    temp_status_count                      = temp_total_sonar_count;

    /*
     * TAKE TIME AND SAVE COORDINATES
     */
    gettimeofday(&this_status_time, NULL);
    diff_x           = -temp_status_internal_x;
    diff_y           = -temp_status_internal_y;
    diff_orientation = -temp_status_internal_orientation;

    /*
     * HEADER
     */
    if (verbose){
      fprintf(stderr, "--------- sfSTATUS (%d) ------------------\n", length);
      fprintf(stderr, "Detailled report: length=%d, body=[", length);
      for (i = 0; i < length; i++)
	fprintf(stderr, " %.2x", *(ptr+i));
      fprintf(stderr, "]\n\n");
      fprintf(stderr, "\tmotor status:   %.2x\n", *ptr);
    }
    if (*ptr < 0x32 || *ptr > 0x33)
      status_error = 1;
    temp_status_motors_status = ((int) *ptr - 50);
    ptr++;

    /*
     * X
     */
    if (verbose)
      fprintf(stderr, "\tx               %.2x%.2x\n", *(ptr+1), *ptr);
    /*     fprintf(stderr, "\tx: %g\n",   */
    /* 	    (((float) *(ptr+1)) * 256.0 + ((float) *ptr))); */
    tmp_ushort = 256 * ptr[1] + ptr[0];
    help = (float)tmp_ushort / (counts_per_cm);
    while (help < temp_status_internal_x 
	   - ((MAX_X_Y) / (counts_per_cm) *0.5)) 
      help += (MAX_X_Y) / (counts_per_cm);
    while (help > temp_status_internal_x 
	   + ((MAX_X_Y) / (counts_per_cm) *0.5)) 
      help -= (MAX_X_Y) / (counts_per_cm);
    temp_status_internal_x = help;
    diff_x           += temp_status_internal_x;
    ptr += 2;

    /*
     * Y
     */
    if (verbose)
      fprintf(stderr, "\ty               %.2x%.2x\n", *(ptr+1), *ptr);
    /*     fprintf(stderr, "\ty: %g\n",   */
    /* 	    (((float) *(ptr+1)) * 256.0 + ((float) *ptr))); */
    tmp_ushort = 256 * ptr[1] + ptr[0];
    help = (float)tmp_ushort / (counts_per_cm);
    while (help < temp_status_internal_y
	   - ((MAX_X_Y) / (counts_per_cm) *0.5)) 
      help += (MAX_X_Y) / (counts_per_cm);
    while (help > temp_status_internal_y
	   + ((MAX_X_Y) / (counts_per_cm) *0.5)) 
      help -= (MAX_X_Y) / (counts_per_cm);
    temp_status_internal_y = help;
    diff_y           += temp_status_internal_y;
    ptr += 2;

    /*
     * ORIENTATION
     */
    if (verbose)
      fprintf(stderr, "\torientation     %.2x%.2x\n", *(ptr+1), *ptr);
    /*     fprintf(stderr, "\torientation: %g\n",  */
    /* 	    (((float) *(ptr+1)) * 256.0 + ((float) *ptr))); */
    help = (((float) *(ptr+1)) * 256.0 + ((float) *ptr)) / (counts_per_degree);
    tmp_ushort = 256 * ptr[1] + ptr[0];
    tmp_ushort &= 0x7fff;
    help = (float)tmp_ushort / (counts_per_degree);
    while (help < temp_status_internal_orientation - 
	   ((MAX_ORIENTATION) / (counts_per_degree) * 0.5))
      help += (MAX_ORIENTATION) / (counts_per_degree);
    while (help > temp_status_internal_orientation +
	   ((MAX_ORIENTATION) / (counts_per_degree) * 0.5))
      help -= (MAX_ORIENTATION) / (counts_per_degree);
    temp_status_internal_orientation = help;
    diff_orientation           += temp_status_internal_orientation;
    while (diff_orientation >  180.0) diff_orientation -= 360.0;
    while (diff_orientation < -180.0) diff_orientation += 360.0;
    ptr += 2;


    /*    fprintf(stderr, "%6.4f\t%6.4f\t%6.4f\n",
	    temp_status_internal_x, 
	    temp_status_internal_y, 
	    temp_status_internal_orientation);
	    */



    /*
     * RIGHT VELOCITY
     */
    if (verbose)
      fprintf(stderr, "\tright velocity  %.2x%.2x\n", *(ptr+1), *ptr);
    prev_vel_r = temp_status_vel_r;
    if(pioneerVersion < 2) {
      if (*(ptr+1) != 0x00 && *(ptr+1) != 0x01 &&
	  *(ptr+1) != 0xff && *(ptr+1) != 0xfe)
	status_error = 2;
      if (*(ptr+1) == 0x00 || *(ptr+1) == 0x01)
	temp_status_vel_r =
	  ((float) *(ptr+1)) * 256.0 + ((float) *ptr);
      else
	temp_status_vel_r = 
	  - 65536.0 + (((float) *(ptr+1)) * 256.0 + ((float) *ptr));
      temp_status_vel_r = temp_status_vel_r / 
	counts_per_cm * STATUS_VELOCITY_FACTOR;
    }
    else {
      tmp_ushort = 256 * ptr[1] + ptr[0];
      tmp_sshort = *((signed short*) &tmp_ushort); 
      temp_status_vel_r = tmp_sshort;
      temp_status_vel_r *= VELCONV_FACTOR;
    }
    if (verbose)
      fprintf(stderr, "\tright velocity  %.2x%.2x -> %g\n", *(ptr+1), *ptr,
	      temp_status_vel_r);
    ptr += 2;
    help = fabs(temp_status_vel_r - prev_vel_r);
    if (help > temp_status_accel_r)
      temp_status_accel_r = help;

    /*
     * LEFT VELOCITY
     */
    if (verbose)
      fprintf(stderr, "\tleft velocity   %.2x%.2x\n", *(ptr+1), *ptr);
    prev_vel_l = temp_status_vel_l;
    if(pioneerVersion < 2) {
      if (*(ptr+1) != 0x00 && *(ptr+1) != 0x01 &&
	  *(ptr+1) != 0xff && *(ptr+1) != 0xfe)
	status_error = 3;
      if (*(ptr+1) == 0x00 || *(ptr+1) == 0x01)
	temp_status_vel_l = 
	  ((float) *(ptr+1)) * 256.0 + ((float) *ptr);
      else
	temp_status_vel_l = 
	  - 65536.0 + (((float) *(ptr+1)) * 256.0 + ((float) *ptr));
      temp_status_vel_l = temp_status_vel_l /
	counts_per_cm * STATUS_VELOCITY_FACTOR;
    }
    else {
      tmp_ushort = 256 * ptr[1] + ptr[0];
      tmp_sshort = *((signed short*) &tmp_ushort); 
      temp_status_vel_l = tmp_sshort;
      temp_status_vel_l *= VELCONV_FACTOR;
    }
    if (verbose)
      fprintf(stderr, "\tleft velocity   %.2x%.2x -> %g\n", *(ptr+1), *ptr,
	      temp_status_vel_l);
    ptr += 2;
    help = fabs(temp_status_vel_l - prev_vel_l);
    if (help > temp_status_accel_l)
      temp_status_accel_l = help;


    /*
     * INTEGRATE POSITION AND ESTIMATE VELOCITY
     */


    if ((temp_status_last_status_time.tv_sec != 0 ||
	temp_status_last_status_time.tv_usec != 0) &&
	temp_status_count != 0){

      diff = sqrt((diff_x*diff_x)+(diff_y*diff_y));
      if (temp_status_vel_r + temp_status_vel_l < 0.0)
	diff *= -1.0;

      /* work in the drift */
      diff_orientation += drift_in_degree_per_cm * diff; /* drift */
      while (diff_orientation >   180.0) diff_orientation -= 360.0;
      while (diff_orientation <= -180.0) diff_orientation += 360.0;

      temp_status_orientation += 0.5 * diff_orientation; /* first half */
      while (temp_status_orientation >  180.0) 
	temp_status_orientation -= 360.0;
      while (temp_status_orientation < -180.0) 
	temp_status_orientation += 360.0;
      angle = temp_status_orientation * M_PI / 180.0;
      temp_status_x += (diff * cos(angle));
      temp_status_y += (diff * sin(angle));
      temp_status_orientation += 0.5 * diff_orientation; /* second half */
      while (temp_status_orientation >  180.0) 
	temp_status_orientation -= 360.0;
      while (temp_status_orientation < -180.0) 
	temp_status_orientation += 360.0;

      time_diff = ((float) (this_status_time.tv_sec 
			    - temp_status_last_status_time.tv_sec))
	+ (((float) (this_status_time.tv_usec
		     - temp_status_last_status_time.tv_usec))
	   / 1000000.0);
      if (time_diff <= 0.0){
	fprintf(stderr, "\n\n\t### ERROR IN CODE OR TIMER ###\n\n");
	putc(7, stderr);
      }
      else{
	temp_status_vel_orientation  = diff_orientation / time_diff;
	temp_status_vel_x            = diff_x / time_diff;
	temp_status_vel_y            = diff_y / time_diff;
      }      
    }

    /*

      fprintf(stderr, "%6.4f (%6.4f)\t%6.4f (%6.4f)\t%6.4f\n",
      temp_status_x, temp_status_vel_x, 
      temp_status_y, temp_status_vel_y, 
      temp_status_orientation);
      
	    */
#ifdef CHECK_SPEED
    /*
     * let this run for a while (robot must go forward!) and
     * you'll get the coefficient between the robot's internal
     * speed, and speed in cm/sec 
     */
    if ((temp_status_last_status_time.tv_sec != 0 ||
	temp_status_last_status_time.tv_usec != 0) &&
	actual_target_translate_velocity != 0.0){
      temp_total_l += time_diff * temp_status_vel_l;
      temp_total_r += time_diff * temp_status_vel_r;
      temp_total_v += time_diff * actual_target_translate_velocity;
      temp_total   += sqrt((diff_x*diff_x)+(diff_y*diff_y));
      if (0) fprintf(stderr, "SPECIAL\t%6.4f %6.4f   \t%6.4f\n",
	      temp_total_l / temp_total,
	      temp_total_r / temp_total, 
	      temp_total_v / temp_total);
    }
#endif

    /*
     * BATTERY
     */
    if (verbose)
      fprintf(stderr, "\tbattery         %.2x (%g V)\n", *ptr, 
	    ((float) (*ptr)) * 0.1);
    if (*(ptr) == 0x00)
      status_error = 4;
    if (temp_status_battery == 0.0)
      temp_status_battery = ((float) (*ptr)) * 0.1;
    else
      temp_status_battery = (0.99 * temp_status_battery)
	+ (0.01 * ((float) (*ptr)) * 0.1);
    ptr++;

    /*
     * STALLED
     */
    if (verbose)
      fprintf(stderr, "\tmotors stalled  l=%.2x r=%.2x\n", *(ptr+1), *ptr);
    if (*(ptr+1) != 0x00 && *(ptr+1) != 0x01)
      status_error = 5;
    if (*(ptr) != 0x00 && *(ptr) != 0x01)
      status_error = 6;
    temp_status_left_stalled  = (int) *(ptr+1);
    temp_status_right_stalled = (int) *ptr;
    ptr+= 2;

    /*
     * CONTROL
     */
    if (verbose)
      fprintf(stderr, "\tcontrol(?)      %.2x%.2x\n", *(ptr+1), *ptr);
    ptr += 2;

    /*
     * PTU
     */
    if (verbose)
      fprintf(stderr, "\tPTU (unused)    %.2x%.2x\n", *(ptr+1), *ptr);
    ptr += 2;

    /*
     * SAY
     */
    if (verbose)
      fprintf(stderr, "\tsay (unused)    %.2x\n", *ptr);

    ptr++;
    got_num_sonars = (int) (*ptr);

    /*
     * NUM_SENSORS
     */
    if (verbose)
      fprintf(stderr, "\tnumber sonars   %.2x\n", *ptr);
    ptr++;
    
    /* fprintf(stderr, "\n\n--->ptr:%d\tns:%d\tgns:%d<---\n\n",
               *ptr,num_sonars,got_num_sonars); */

    /*
     * SENSORS
     */
    if ( (pioneerVersion < 2 && (got_num_sonars < 2 || got_num_sonars > 3)) ||
	 (pioneerVersion == 2 && (got_num_sonars < 4 || got_num_sonars > 6)))
      status_error = 7;
    
    else {
      for (i = 0; i < got_num_sonars; i++){
	
	if (*ptr < num_sonars){
	  if (pioneerVersion < 2) {
	    help =
	      0.017337 * ((float) ((((int) *(ptr+2)) * 256) + ((int) *(ptr+1)))) - 3.0;		/* Magic! */
	  }
	  else if (pioneerVersion == 2) {
	    help =
	      0.0268 * ((float) ((((int) *(ptr+2)) * 256) + ((int) *(ptr+1)))) - 3.0; /* really magic */
	  }

	  /*	if (!(temp_status_sonar_new_flag[*ptr]) ||
		help < temp_status_sonars[*ptr])*/

	  temp_status_sonars[*ptr] = help; /* unconditional update */
	  
	  if (temp_status_sonars[*ptr] < 0.0)
	    temp_status_sonars[*ptr] = 0.0;

	  temp_status_sonar_new_flag[*ptr] = 1;
	  /*
	    {
	    static c = 0;
	    if (!c){
	    c = 1;
	    fprintf(stderr, "\n\n\n\t### WARNING SONAR DISACTIVATED ###\n\n\n");
            }
	    temp_status_sonars[*ptr] = 10000.0;
	    }
	  */
        }  /* (*ptr < num_sonars) */
	else
	  status_error = 8;
	
	if (verbose)
	  fprintf(stderr, "\tsonar           %.2x: %.2x%.2x (%g)\n", 
		  *ptr, *(ptr+2), *(ptr+1), temp_status_sonars[*ptr]);
	ptr += 3;
      } /* for */
    } /* else */

    /*
     * TIMER
     */
    if (verbose)
      fprintf(stderr, "\tinput timer     %.2x%.2x\n", *(ptr+1), *ptr);
    ptr += 2;

    /*
     * A/D
     */
    if (verbose)
      fprintf(stderr, "\tA/D             %.2x\n", *ptr);
    ptr++;

    /*
     * DIGITAL INPUT
     */
    if (verbose)
      fprintf(stderr, "\tdigital input   %.2x\n", *ptr);
    ptr++;

    /*
     * DIGITAL OUTPUT
     */
    if (verbose)
      fprintf(stderr, "\tdigital output  %.2x\n", *ptr);
    ptr++;


    /*
     * UPDATE TIME
     */
    temp_status_last_status_time.tv_sec = this_status_time.tv_sec;
    temp_status_last_status_time.tv_usec = this_status_time.tv_usec;

    /*
     * COUNT UP
     */
    temp_total_sonar_count           += 1;

    /*
     * PRINT SENSOR VALUE AND FINISH
     */
    if (verbose){
      fprintf(stderr, "Sonars:");
      for (i = 0; i < num_sonars; i++)
	fprintf(stderr, " %6.4f", temp_status_sonars[i]);
      fprintf(stderr, "\n");
    }
    if (verbose)
      fprintf(stderr, "-------------------------------------------\n");

    /*
     * copy the temp values back to the status report
     */
    
    if (status_error == 0){
      status[actual_status].defined                 = temp_status_defined;
      status[actual_status].count                   = temp_status_count;
      status[actual_status].motors_status           = temp_status_motors_status;
      status[actual_status].internal_x              = temp_status_internal_x;
      status[actual_status].internal_y              = temp_status_internal_y;
      status[actual_status].internal_orientation    = temp_status_internal_orientation;
      status[actual_status].vel_r                   = temp_status_vel_r;
      status[actual_status].vel_l                   = temp_status_vel_l;
      status[actual_status].accel_l                 = temp_status_accel_l;
      status[actual_status].accel_r                 = temp_status_accel_r;
      status[actual_status].orientation             = temp_status_orientation;
      status[actual_status].x                       = temp_status_x;
      status[actual_status].y                       = temp_status_y;
      status[actual_status].vel_orientation         = temp_status_vel_orientation;
      status[actual_status].vel_x                   = temp_status_vel_x;
      status[actual_status].vel_y                   = temp_status_vel_y;
      status[actual_status].battery                 = temp_status_battery;
      status[actual_status].left_stalled            = temp_status_left_stalled;
      status[actual_status].right_stalled           = temp_status_right_stalled;
      status[actual_status].target_translate_velocity = temp_status_target_translate_velocity;
      status[actual_status].target_rotate_velocity = temp_status_target_rotate_velocity;


      for (i      = 0; i < num_sonars; i++){
	status[actual_status].sonars[i]               =   temp_status_sonars[i];
	status[actual_status].sonar_confidence[i]     =   temp_status_sonar_confidence[i];
	status[actual_status].sonar_new_flag[i]       =   temp_status_sonar_new_flag[i];
      }
      status[actual_status].last_status_time.tv_sec = temp_status_last_status_time.tv_sec;
      status[actual_status].last_status_time.tv_usec = temp_status_last_status_time.tv_usec;
      status[actual_status].last_sync_time.tv_sec   = temp_status_last_sync_time.tv_sec;
      status[actual_status].last_sync_time.tv_usec  = temp_status_last_sync_time.tv_usec;

      total_l                                       = temp_total_l;
      total_r                                       = temp_total_r;
      total_v                                       = temp_total_v;
      total                                         = temp_total;
      total_sonar_count                             = temp_total_sonar_count;

      number_consecutive_status_errors = 0;
      
      {
	static int x = 0;
	
	if (x == 1){
	  if (print_sonar_values){
	    printf("Sonars:");
	    for (i = 0; i < num_sonars; i++)
	      printf(" %6.2f", status[actual_status].sonars[i]);
	    printf("\n");
	  }
	  
	  bee_send_sonar_update(status[actual_status].sonars);
	  x = 0;
	}
	else
	  x++;
      }

      if (print_status)
	printf("Status: x=%5.1f y=%5.1f o=%5.2f tv=%5.3f rv=%5.3f\n",
		status[actual_status].x,
		status[actual_status].y,
	       status[actual_status].orientation,
	       0.5 * (status[actual_status].vel_l
		      + status[actual_status].vel_r),
	       status[actual_status].vel_orientation);

      bee_check_trans_motion_terminator(status[actual_status].x,
					status[actual_status].y,
					status[actual_status].orientation
					+ 90.0);
      bee_check_rot_motion_terminator(status[actual_status].x,
				      status[actual_status].y,
				      status[actual_status].orientation
				      + 90.0);
      bee_send_status_update(status[actual_status].x,
			     status[actual_status].y,
			     status[actual_status].orientation,
			     0.5 * (status[actual_status].vel_l
				    + status[actual_status].vel_r),
			     status[actual_status].vel_orientation,
			     &(status[actual_status].last_status_time));


    }
    else{
      number_consecutive_status_errors++;
      fprintf(stderr, "############### STATUS ERROR: type %d, number %d\n",
	      status_error, number_consecutive_status_errors);
    }


    /*
     * PING ROBOT
     */

    ping_robot();



    /*
     * UPDATE_VELOCITIES
     */
    {
      struct timeval this_time;
      float  time_difference= 0.0; 
      int    watchdog_change = 0;
      static float prev_time_diff= 0.0;

      
      if (last_command_type == 0 && n_auto_update_modules > 0){
	gettimeofday(&this_time, NULL);
	time_difference = 
	  ((float) (this_time.tv_sec - watchdog_timestamp.tv_sec))
	  + (((float) (this_time.tv_usec - watchdog_timestamp.tv_usec))
	     /  1000000.0);
	if (time_difference > 1.0){
	  if (global_watchdog == 1){
	    watchdog_change = 1;
	    fprintf(stderr, "WATCHDOG STOP: no message for %g seconds.\n",
		    time_difference);
	  }
	  global_watchdog = 0;
	  prev_time_diff = time_difference;
	}
	else{
	  if (global_watchdog == 0){
	    watchdog_change = 1;
	    fprintf(stderr, "WATCHDOG RESUME: after %g seconds.\n",
		    prev_time_diff);
	  }
	  global_watchdog = 1;
	}
      }
      else
	global_watchdog = 1;
      
      if (global_set_tvel_new || watchdog_change){
	actual_target_translate_velocity =
	  0.1 * ((float) global_set_tvel) *
	  ((float) global_watchdog) * 
	  ((float) global_tvel_sign);
	if (actual_target_translate_velocity > MAX_TRANSLATIONAL_VELOCITY)
	  actual_target_translate_velocity = MAX_TRANSLATIONAL_VELOCITY;
	if (actual_target_translate_velocity < -(MAX_TRANSLATIONAL_VELOCITY))
	  actual_target_translate_velocity = -(MAX_TRANSLATIONAL_VELOCITY);
	if (v)
	  printf("TCX: translational velocity: %f.\n",
		 actual_target_translate_velocity); 
	global_set_tvel_new = 0;
	last_command_type = 0;
      }
      
      if (global_set_rvel_new || watchdog_change){
	actual_target_rotate_velocity =
	  - 1.0 / 2.83 * ((float) global_set_rvel) *
	   ((float) global_watchdog) * 
	  ((float) global_rvel_sign);
	if (actual_target_rotate_velocity > MAX_ROTATIONAL_VELOCITY)
	  actual_target_rotate_velocity = MAX_ROTATIONAL_VELOCITY;
	if (actual_target_rotate_velocity < -(MAX_ROTATIONAL_VELOCITY))
	  actual_target_rotate_velocity = -(MAX_ROTATIONAL_VELOCITY);
	if (v)
	  printf("TCX: rotational velocity: %f.\n",
		 actual_target_rotate_velocity); 
	global_set_rvel_new = 0;
	last_command_type = 0;
      }
      
      {
	static float actual_target_translate_velocity2 = 0.0;
	static float actual_target_rotate_velocity2 = 0.0;
      
	if (1 || actual_target_translate_velocity !=
	    actual_target_translate_velocity2){
	  WriteCommand(sfCOMVEL,
		       (int) (command_translational_velocity_factor *
			      actual_target_translate_velocity),
		       NULL);
	  actual_target_translate_velocity2 = actual_target_translate_velocity;
	  /*fprintf(stderr, "-1-");*/
	}
	if (1 || actual_target_rotate_velocity != actual_target_rotate_velocity2){
	    WriteCommand(sfCOMRVEL,
			 (int) (command_rotational_velocity_factor *
				actual_target_rotate_velocity),
			 NULL);
	  actual_target_rotate_velocity2 = actual_target_rotate_velocity;
	  /*fprintf(stderr, "-2-");*/
	}
      }
    }

    /*
     * DISPLAY AND ACT
     */

    if (status_error)
      fprintf(stderr, "?");
  }
  else
    ignore_status_report--;

}
  
void
process_input(char *command)
{
  int i= 0;


  float tv= 0.0, rv= 0.0;
  /* clean up command */

#if TOTAL_debug
  fprintf(stdout,"\n--->process_input\n");
  fflush( stdout );
#endif

  for (i = 0; i < (int) strlen(command); i++)
    if (command[i] == '\n') 
      command[i] = 0x0;

  /* acknowledge */

  /*fprintf(stderr, "Received User Command: [%s]\n", command);*/

  /* act */


  if (command[0] == 0x0){
    actual_target_translate_velocity = 0.0;
    actual_target_rotate_velocity    = 0.0;
    last_command_type = 1;
    printf("Stop.\n");
  }
  else if (command[0] == '.'){
    printf("Done.\n");
    shutdown_robot();
    exit(-1);
  }
  else if (sscanf(command, "tv %f", &actual_target_translate_velocity) ||
	   sscanf(command, "TV %f", &actual_target_translate_velocity)){
    if (actual_target_translate_velocity > MAX_TRANSLATIONAL_VELOCITY)
      actual_target_translate_velocity = MAX_TRANSLATIONAL_VELOCITY;
    if (actual_target_translate_velocity < -(MAX_TRANSLATIONAL_VELOCITY))
      actual_target_translate_velocity = -(MAX_TRANSLATIONAL_VELOCITY);
    printf("COMMAND: translational velocity: %f.\n",
	   actual_target_translate_velocity); 
    last_command_type = 1;
  }
  else if (sscanf(command, "rv %f", &actual_target_rotate_velocity) ||
	   sscanf(command, "RV %f", &actual_target_rotate_velocity)){
    if (actual_target_rotate_velocity > MAX_ROTATIONAL_VELOCITY)
      actual_target_rotate_velocity = MAX_ROTATIONAL_VELOCITY;
    if (actual_target_rotate_velocity < -(MAX_ROTATIONAL_VELOCITY))
      actual_target_rotate_velocity = -(MAX_ROTATIONAL_VELOCITY);
    printf("COMMAND: rotational velocity: %f.\n",
	   actual_target_rotate_velocity); 
    last_command_type = 1;
  }
  
  else if (command[0] == 's' || command[0] == 'S')
    print_sonar_values ^= 1;
  else if (command[0] == 'p' || command[0] == 'P')
    print_status ^= 1;
  else if (command[0] == 'x' || command[0] == 'X'){
    status[actual_status].x                       = 0.0;
    status[actual_status].y                       = 0.0;
    status[actual_status].orientation             = 0.0;
  }
  else if (command[0] == 'b' || command[0] == 'B')
    printf("Battery: %3.1f Volt\n",
	    status[actual_status].battery);


  else if (command[0] == '?'){ 
    printf("<RETURN>        Stop\n");
    printf(".               Quit program\n");
    printf("tv <number>     Set translational velocity\n");
    printf("rv <number>     Set rotational velocity\n");
    printf("s               Toggle printing sonar measurements\n");
    printf("p               Toggle printing status (position/velocity)\n");
    printf("b               Print battery voltage\n");
    printf("x               Reset Odometry\n");
  }
  
}

void 
stdin_inputHnd(int fd, long chars_available)
{
  static char buffer[DEFAULT_LINE_LENGTH+1];
  static char *startPos = buffer; /* position to start parsing from */
  static char *endPos = buffer; /* position to add more characters */
  char *lineEnd=NULL;
  int numRead = 0;
  /* should handle characters output by the base */
  
  /* never expect more than DEFAULT_LINE_LENGTH characters on a line.
   * read the first DEFAULT_LINE_LENGTH and let the function get called 
   * again for any remaining characters.  This can be changed.
   */
#if TOTAL_debug
  fprintf(stdout,"\n--->stdin_inputHnd\n");
  fflush( stdout );
#endif
  
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




unsigned int
calc_checksum(unsigned char *ptr)
{
  int n= 0;
  int c = 0;
  n = *(ptr++);
  n -= 2;
#if TOTAL_debug
  fprintf(stdout,"\n--->calc_checksum\n");
  fflush( stdout );
#endif


  while (n > 1){
    c += (*(ptr)<<8) | *(ptr+1);
    c = c & 0xffff;
    n -= 2;
    ptr += 2;
  }
  if (n > 0) 
    c = c ^ (int) *(ptr++);
  return (c);
}





int 
WriteCommand(unsigned char command, int argument, unsigned char *text)
{
  static unsigned char buffer[DEFAULT_LINE_LENGTH];
  unsigned char bytecount = 3;
  int      pos = 0;
  unsigned int      check = 0;
  int i= 0, n= 0;
  unsigned char *ptr=NULL;

#if TOTAL_debug
  fprintf(stdout,"\n--->WriteCommand\n");
  fflush( stdout );
#endif

  /*
   * SYNC CHARS
   */
  buffer[pos++] = 0xfa;
  buffer[pos++] = 0xfb;

  /*
   * BYTE COUNT
   */

  if (command == sfCOMROTATE || 
      command == sfCOMVEL || 
      command == sfCOMRVEL || 
      command == sfCOMVEL2 || 
      command == sfCOMDHEAD ||
      command == sfCOMSETRV){
    bytecount += 3;
  }
  else if (command == sfCOMPOLLING)
    bytecount += (num_sonars + 2); 

  /* Change depending on the number of sonars you have */
  /* To make it easier for differnt pioneers it is now */
  /* related to the number of sonars. */
  /* # sonars + argument type + number-of-sonars-byte (bytecount is */
  /* already 3 for command and checksum)  21.05.99 Frank*/ 

  else if (command == sfCOMDIGOUT)
    bytecount += 3;


  buffer[pos++] = bytecount;

  /*
   * COMMAND NAME
   */

  buffer[pos++] = command;


  /*
   * ARGUMENT
   */

  if (command == sfCOMROTATE || 
      command == sfCOMVEL || 
      command == sfCOMRVEL || 
      command == sfCOMVEL2 || 
      command == sfCOMDHEAD ||
      command == sfCOMSETRV){
    if (argument >= 0){
      buffer[pos++] = 0x3B;
      buffer[pos++] = (unsigned char) (argument & 0xff);
      buffer[pos++] = (unsigned char) (argument / 256);
    }
    else {
      buffer[pos++] = 0x1B;
      argument = 65536-argument;
      buffer[pos++] = (unsigned char) (argument & 0xff);
      buffer[pos++] = (unsigned char) (argument / 256);
    }
  }
  
  else if (command == sfCOMPOLLING){
    /* There is a mistake in the doc! The second number */
    /* in this string is the # of sonar to fire! */
    /* Remember the spec. is in octal, but the printout is in hex! */
    /* So the command string for sfCOMPOLLING is*/
    /* fa fb 0d 03 2b 08 01 02 03 04 05 06 07 08 18 3c */
    /* >ph < bc cm dt #s >   firing pattern    < >cs < */

    /* ph: packet header
       bc: byte count (13 if you use 8 sensors, 12 for 7 and so on)
       cm: command (sfCOMPOLLING)
       dt: data type (sfARGSTR)
       #s: number of sonars to fire (in oct!!!)
       fp: firing patter (# of sonar in oct!!!) even 001 005 001 005 works :)
       cs: checksum
       
       fa fb 0d 03 2b 08 01 02 03 04 05 06 07 08 18 3c
       1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 length of data stream
                1  2  3  4  5  6  7  8  9  10 11 12 13 l. of command (bytecount) */

    
    buffer[pos++] = 0x2b;
    buffer[pos++] = (unsigned char) num_sonars;
    
    /* Set the firing pattern as given in the 'text' array.
       if no array specified use serial pattern. */

    for(i = 0; i < num_sonars; i++)
      buffer[pos++] = (text != NULL ? text[i] : i+1);
    
  }

  else if (command == sfCOMDIGOUT){
    buffer[pos++] = 0x2b;
    buffer[pos++] = (unsigned char) argument;
    buffer[pos++] = '\000';
  }
  
  /*
   * CHECKSUM
   */

  check = calc_checksum(&(buffer[2]));
  ptr = (unsigned char *) &check;
  buffer[pos++] = check / 256;
  buffer[pos++] = check & 0x00ff;
  
  flushChars(&(base_device.dev));



  /*
   * ECHO - FOR SOME(!) MESSAGES ONLY
   */

  if (communication_verbose &&
      (command == sfCOMROTATE || 
       command == sfCOMPULSE || 
       command == sfCOMVEL || 
       command == sfCOMRVEL || 
       command == sfCOMVEL2 || 
       command == sfCOMDHEAD ||
       command == sfCOMSETRV ||
       command == sfCOMPOLLING)){
    fprintf(stderr, "\nWriteCommand (%d %d) [", command, argument);
    for (i = 0; i < pos; i++)
      fprintf(stderr, " %.2x", buffer[i]);  
      fprintf(stderr, "]\n");
  }



  /*
   * WRITE TO PORT
   */

  n = ((int) writeN(&(base_device.dev), buffer, pos));


  return n;
}



void
connect_to_robot(int force)
{
  int i= 0;
  struct timeval actual_time;
  int  prev_sync_level = -1;

#if TOTAL_debug
  fprintf(stdout,"\n--->connect_to_robot\n");
  fflush( stdout );
#endif

  for (actual_status = 0; actual_status < NUMBER_STATUS; actual_status++){
    status[actual_status].defined                 = 0;
    status[actual_status].motors_status           = 0;
    status[actual_status].left_stalled            = 0;
    status[actual_status].right_stalled           = 0;
    status[actual_status].x                       = 0.0;
    status[actual_status].y                       = 0.0;
    status[actual_status].orientation             = 0.0;
    status[actual_status].internal_x              = 0.0;
    status[actual_status].internal_y              = 0.0;
    status[actual_status].internal_orientation    = 0.0;
    status[actual_status].vel_l                   = 0.0;
    status[actual_status].vel_r                   = 0.0;
    status[actual_status].vel_x                   = 0.0;
    status[actual_status].vel_y                   = 0.0;
    status[actual_status].vel_orientation         = 0.0;
    status[actual_status].accel_l                 = 0.0;
    status[actual_status].accel_r                 = 0.0;
    status[actual_status].battery                 = 0.0;
    status[actual_status].control                 = 0.0;
    status[actual_status].count                   = 0;    
    for (i = 0; i < num_sonars; i++){
      status[actual_status].sonars[i]             = 0.0;
      status[actual_status].sonar_new_flag[i]     = 0;
      status[actual_status].sonar_confidence[i]   = 0;
    }
    status[actual_status].sync_level              = 0;
    
    status[actual_status].last_sync_time.tv_sec   = 0;
    status[actual_status].last_sync_time.tv_usec  = 0;
    status[actual_status].last_status_time.tv_sec = 0;
    status[actual_status].last_status_time.tv_usec= 0;
    status[actual_status].target_translate_velocity= 0.0;
    status[actual_status].target_rotate_velocity  = 0.0;

  }

  actual_status = 0;
  
  fprintf(stderr, "Connecting to Pioneer");
  
  WriteCommand(sfCOMVEL, 0, NULL);
  WriteCommand(sfCOMVEL, 0, NULL);
  WriteCommand(sfCOMCLOSE,  0, NULL);
  usleep(300000);

  if (force){
    do{
      gettimeofday(&actual_time, NULL);
      if (actual_time.tv_sec - 
	  status[actual_status].last_sync_time.tv_sec >= 2 ||
	  (actual_time.tv_sec - 
	   status[actual_status].last_sync_time.tv_sec == 1 &&
	   actual_time.tv_usec >= status[actual_status].last_sync_time.tv_usec)){
	status[actual_status].sync_level  = 0;
	prev_sync_level    = -1;
	status[actual_status].last_sync_time.tv_sec = actual_time.tv_sec;
	status[actual_status].last_sync_time.tv_usec = actual_time.tv_usec;
	fprintf(stderr, ".");
      }
      
      if (prev_sync_level != status[actual_status].sync_level){
	WriteCommand((unsigned char) (status[actual_status].sync_level),
		     0, NULL);
	prev_sync_level = status[actual_status].sync_level;
      }
      
      ProcessDevices();
      
      
    } while (status[actual_status].sync_level < 3);
  }

  WriteCommand(sfCOMPULSE, 0, NULL);
  WriteCommand(sfCOMOPEN,  0, NULL);
  WriteCommand(sfCOMSETO,  0, NULL);

#ifdef FGAN
  if (strcasecmp(bRobot.pio_type,"PIONEER_IF")==0)
  {
    /* Firing pattern for the 8-sonar-PIONEER-I */
    
    unsigned char sonar_pattern[8] = {4,7,2,6,8,3,5,1};
    
    WriteCommand(sfCOMPOLLING, 0, sonar_pattern);
    
    fprintf(stderr,"\n\n--->configure_eight_sonars<---\n\n");
  }

#endif

/*
  {
    static c = 0;
    if (!c){
      c = 1;
      fprintf(stderr, "\n\n\n\t### WARNING SONAR DISACTIVATED ###\n\n\n");
    }
    WriteCommand(sfCOMPOLLING,  0, NULL);
  }
*/
  fprintf(stderr, ".done.\n");
}




void
alarm_handler()
{
  fprintf(stderr, "$");
  alarm(1, 0);
}

void
ping_robot()
{
  float time_diff= 0.0;
  struct timeval actual_time;

#if TOTAL_debug
  fprintf(stdout,"\n--->ping_robot\n");
  fflush( stdout );
#endif
  
  gettimeofday(&actual_time, NULL);
  time_diff = ((float) (actual_time.tv_sec 
			- last_ping_time.tv_sec))
    + (((float) (actual_time.tv_usec
		 - last_ping_time.tv_usec)) / 1000000.0);
  if (last_ping_time.tv_sec == 0 || time_diff > 0.5){
    WriteCommand(sfCOMPULSE, 0, NULL);
    last_ping_time.tv_sec  = actual_time.tv_sec;
    last_ping_time.tv_usec = actual_time.tv_usec;
  }
}
	   
void
shutdown_robot()
{

#if TOTAL_debug
  fprintf(stdout,"\n--->shutdown_robot\n");
  fflush( stdout );
#endif

  fprintf(stderr, "\nShooting down robot...");
  actual_target_translate_velocity = 0.0;
  actual_target_rotate_velocity = 0.0;
  WriteCommand(sfCOMVEL, 0, NULL);
  WriteCommand(sfCOMRVEL, 0, NULL);
  WriteCommand(sfCOMCLOSE,  0, NULL);
  WriteCommand(sfCOMVEL, 0, NULL);
  WriteCommand(sfCOMRVEL, 0, NULL);
  WriteCommand(sfCOMCLOSE,  0, NULL);
  fprintf(stderr, "done!\n");
  exit(-1);
}

main( int argc, char* argv[])
{
  char robotName[64] = "";
  char* devName   = NULL;
  int robotType   = PIONEER_1_ROBOT;  /* War vorher _AT_ Dennis */
  int i = 0, bug = 0, test = 1;

  bParamList = bParametersAddFile( bParamList, "etc/beeSoft.ini" );
  bParamList = bParametersAddEntry( bParamList, "robot", "pioneer", "yes" );

  for (i=1; i<argc && !bug; i++) {
    if ((strcmp(argv[i],"-robot")==0)){
      if ((i+1<argc) && (argv[i+1][0]!='-')) {
	i++;
	strncpy( robotName, argv[i], 64 );
      }
      else {
	fprintf(stderr, "ERROR: robot name must follow keyword robot.\n");
	exit(0);
      }
    }
    else if ((strcmp(argv[i],"-dev")==0)){
      if (i < argc - 1) {
	i++;
	devName = argv[i];
	fprintf(stderr, "Connecting to %s.\n", devName);
      }
      else {
	fprintf(stderr, "ERROR: device name must follow keyword -dev.\n");
	exit(0);
      }
    }
    /* FRANK start */
    else if(strcmp(argv[i],"-RobotServer")==0) {
      useRobotServer = TRUE;
      fprintf(stderr,"baseServer connecting to RobotServer.\n");
    }
    /* FRANK stop */
    else if(strcmp(argv[i],"-pioneer2")==0) {
      /* add some parameter files */
      bParamList = bParametersAddEntry(bParamList, "robot", "name", "PIONEER_II");
      test = 0;
    }
    else if(strcmp(argv[i],"-pioneer")==0) {
      /* add some parameter files */
      bParamList = bParametersAddEntry(bParamList, "robot", "name", "PIONEER_I");
      test = 0;
    }
    else if(strcmp(argv[i],"-pioneerf")==0) {
      /* add some parameter files */
      bParamList = bParametersAddEntry(bParamList, "robot", "name", "PIONEER_IF");
      test = 0;
    }
    else if(strcmp(argv[i],"-pioneerat")==0) {
      /* add some parameter files */
      bParamList = bParametersAddEntry(bParamList, "robot", "name", "PIONEER_AT");
      test = 0;
    }
    else 
      bug =1;
  }  

  if ( (bug == 1) || ((test == 1) && (!strcasecmp(robotName, "")) ) ) {
    fprintf(stderr, "\n--Usage: %s [-pioneer2] [-pioneer] [-pioneerf] [-pioneerat] [-robot <name>] [-dev <device_path>] [-RobotServer]\n", argv[0]);
    exit(0);
  }

  if (test && (strcasecmp(robotName,"")) ) {
    bParamList = bParametersAddEntry(bParamList, "robot", "name", robotName );
  }

  /* add some enviroment variables */
  bParamList = bParametersAddEnv(bParamList, "", "TCXHOST");

  /* add command line arguements */
  bParamList = bParametersAddArray(bParamList, "", argc, argv);

  /* Fill the global parameter structure */
  bParametersFillParams(bParamList);

  if ( devName == NULL)  /* No name explicitly set. */
    devName=strdup(bRobot.pio_dev);

  if ((strcasecmp(bRobot.pio_type,"PIONEER_I")==0) ||(strcasecmp(bRobot.pio_type,"PIONEER_IF")==0))
    robotType = pioneerVersion = 1;    
  else if (strcasecmp(bRobot.pio_type,"PIONEER_II")==0)
    robotType = pioneerVersion = 2;
  else if (strcasecmp(bRobot.pio_type,"PIONEER_AT")==0)
    robotType = pioneerVersion = 0;
  else if (strcasecmp(bRobot.pio_type,"PIONEER_AT_small")==0)
    robotType = pioneerVersion = 0;
  else if (strcasecmp(bRobot.pio_type,"PIONEER_AT_big")==0)
    robotType = pioneerVersion = 0;

  num_sonars                = bRobot.pio_sonar_number;
  ROBOT_RADIUS              = bRobot.pio_radius;
  counts_per_cm             = bRobot.pio_cntperCm;
  counts_per_degree         = bRobot.pio_defcntperdeg;
  command_translational_velocity_factor = bRobot.pio_tranvelfac;
  command_rotational_velocity_factor    = bRobot.pio_rotvelfac;
  drift_in_degree_per_cm    = bRobot.pio_defdriftdegperCm;

  VELCONV_FACTOR            = bRobot.pio_velconvfac;
  STATUS_VELOCITY_FACTOR    = bRobot.pio_statvelfac;
  MAX_VELOCITY              = bRobot.pio_maxvel;
  MAX_ORIENTATION           = bRobot.pio_maxorient;
  MAX_X_Y                   = bRobot.pio_max_xy;
  PIONEER_POSITION_OFFSET   = bRobot.pio_posoffset;
  MAX_TRANSLATIONAL_VELOCITY= bRobot.pio_max_tranvel;
  MAX_ROTATIONAL_VELOCITY   = bRobot.pio_max_rotvel;
  MIN_BATTERY               = bRobot.pio_volt_warn;
  MAX_BATTERY               = bRobot.pio_volt_max;

#ifdef FGAN
fprintf(stderr,"\n\nsonars--->%i<--robottype->%s<---\n\n\n",num_sonars,bRobot.pio_type);
#endif

  devInit();

  connectDev(&stdin_device);
  stdin_device.outputHnd = stdin_inputHnd;

  fprintf( stderr, "\nUse device >%s<\n", devName);

  base_device.dev.ttydev.ttyPort = devName;

  connectTotty(&base_device.dev);

  if (base_device.dev.fd == -1)
    exit(-1);

  connectDev(&base_device.dev);

  base_device.dev.sigHnd = signal_base;

  if (bRobot.pio_bps == 14)
    m_setparms(base_device.dev.fd, "19200", "NONE", "8", 0, 0);
  else
    m_setparms(base_device.dev.fd, "9600", "NONE", "8", 0, 0);

  connect_to_robot(1);

  if (use_tcx){
    bee_init(robotName, robotType);
    listen_for_tcx_events = 1;
  }

  signal(SIGINT,  (void *) shutdown_robot);
  signal(SIGTERM, (void *) shutdown_robot);
  
  for (;;) {
    ProcessDevices();
    usleep(10);
  }
  
}
