

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <math.h>

#include "devUtils.h"
#include "scout.h"
#include "bee_interface.h"
#include "Nclient.h"


struct timeval last_ping_time = {0, 0};

int use_tcx = 1;


/* Set the conversion parameters to pioneer ats with small wheels. */



STATUS_TYPE status;

void set_vel(int force);

extern int listen_for_tcx_events;


int print_sonar_values = 0;
int print_status = 0;

static int communication_verbose = 1;

float actual_target_translate_velocity = 0.0;
float actual_target_rotate_velocity = 0.0;
int   last_command_type = 0;

static int pioneerVersion = 1;
static float robot_radius;
int num_sonars = MAX_NUM_SONARS;
static float counts_per_cm;
static float command_translational_velocity_factor;
static float command_rotational_velocity_factor;

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

#define SCOUT_DEVICE "/dev/ttyS0"


void 
turn_sonar_on(void)
{
  int sn_order[16] = {0, 2, 15, 1, 14, 3, 13, 4, 12, 5, 11, 6, 10, 7, 9, 8};  /* Remainder are irrelevant */
  
  /* turn on 5 front sonars and set them to fire in certain order */
  conf_sn(15, sn_order);
  
  conf_tm(5); 
}

void 
turn_sonar_off(void)
{
  int sn_order[16] = {255, 2, 15, 1, 14, 3, 13, 4, 12, 5, 11, 6, 10, 7, 9, 8};  /* Remainder are irrelevant */
  
  /* turn on 5 front sonars and set them to fire in certain order */
  conf_sn(15, sn_order);
  
  conf_tm(5); 
}
   
void
shutdown_robot()
{
  fprintf(stderr, "\nShooting down robot...");
  actual_target_translate_velocity = 0.0;
  actual_target_rotate_velocity = 0.0;
  st();
  turn_sonar_off();
  usleep(500000);
  lp();
  disconnect_robot(1);
  fprintf(stderr, "done.\n");
  exit(-1);
}


void
process_input(char *command)
{
  int i;


  float tv, rv;
  /* clean up command */

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
    set_vel(1);
    st();
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
    set_vel(1);

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
    set_vel(1);

  }
  
  else if (command[0] == 's' || command[0] == 'S')
    print_sonar_values ^= 1;
  else if (command[0] == 'p' || command[0] == 'P')
    print_status ^= 1;
  else if (command[0] == 'x' || command[0] == 'X'){
    zr();
  }
  else if (command[0] == 'b' || command[0] == 'B')
    printf("Battery: %3.1f %3.1f Volt\n",
	   voltMotorGet(), voltCpuGet());



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




unsigned int
calc_checksum(unsigned char *ptr)
{
  int n;
  int c = 0;
  n = *(ptr++);
  n -= 2;
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





void
reset_status_report()
{
  int i;
  struct timeval actual_time;
  int  prev_sync_level = -1;

  status.motors_status           = 0;
  status.x                       = 0.0;
  status.y                       = 0.0;
  status.orientation             = 0.0;
  status.vel_trans               = 0.0;
  status.vel_rot                 = 0.0;
  status.vel_l                   = 0.0;
  status.vel_r                   = 0.0;
  status.accel_l                 = 0.0;
  status.accel_r                 = 0.0;
  status.battery                 = 0.0;
  status.control                 = 0.0;
  status.count                   = 0;    
  status.bump                    = 0;
  status.motor                   = 0;
  for (i = 0; i < num_sonars; i++)
    status.sonars[i]             = 0.0;
    
  status.target_translate_velocity= 0.0;
  status.target_rotate_velocity  = 0.0;

}


void
set_vel(int force)
{
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float time_difference;
  float set_l, set_r;

  
  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);
  if (time_difference > 2.0 || force){
    last_time.tv_sec = this_time.tv_sec;
    last_time.tv_usec = this_time.tv_usec;

    set_l = actual_target_translate_velocity;
    set_r = actual_target_translate_velocity;
    set_l -= (0.5 * actual_target_rotate_velocity / ROT_VEL_FACT);
    set_r += (0.5 * actual_target_rotate_velocity / ROT_VEL_FACT);
    if (!(BACKWARDS_FLAG))
      vm((int) (set_r / .254), (int) (set_l / .254), 0.0);
    else
      vm((int) (-set_l / .254), (int) (-set_r / .254), 0.0);
  }
}

void
update_status(long *state)
{
  int i;
  static struct timeval last_time = {0, 0};
  static struct timeval last_time2 = {0, 0};
  static struct timeval last_time3 = {0, 0};
  struct timeval this_time;
  float time_difference;
  static float last_x, last_y;
  static float last_orientation, sum_angle = 0.0, sum_dist = 0.0;
  static int initialized = 0;
  static struct timeval start_time = {0, 0};
  float diff_angle, diff_dist;

  status.motors_status           = (int) state[STATE_MOTOR_STATUS];
  status.x                       = ((float) state[STATE_CONF_X]) * .254;
  status.y                       = ((float) state[STATE_CONF_Y]) * .254;
  status.orientation             = ((float) state[STATE_CONF_STEER]) * 0.1;
  if (!(BACKWARDS_FLAG)){
    status.vel_l                 = ((float) state[STATE_VEL_TRANS]) * .254;
    status.vel_r                 = ((float) state[STATE_VEL_STEER]) * .254;
  }
  else{
    status.orientation += 180.0;
    for (; status.orientation <= -180.0; ) status.orientation += 360.0;
    for (; status.orientation >   180.0; ) status.orientation -= 360.0;
    status.vel_l                 = -((float) state[STATE_VEL_STEER]) * .254;
    status.vel_r                 = -((float) state[STATE_VEL_TRANS]) * .254;
  }
  status.vel_trans = 0.5 * (status.vel_l + status.vel_r);
  status.vel_rot   = ROT_VEL_FACT * (status.vel_l - status.vel_r);
  status.accel_l                 = 0.0;
  status.accel_r                 = 0.0;
  status.battery                 = 0.0;
  status.control                 = 0.0;
  status.bump                    = (int) state[STATE_BUMPER];
  status.motor                   = (int) state[STATE_MOTOR_STATUS];
  status.count                   = status.count + 1;    
  if (!(BACKWARDS_FLAG))
    for (i = 0; i < num_sonars; i++)
      status.sonars[i]           = ((float) state[i+STATE_SONAR_0]) * 2.54;
  else
    for (i = 0; i < num_sonars; i++)
      status.sonars[(i+((MAX_NUM_SONARS)/2))%(MAX_NUM_SONARS)]
	= ((float) state[i+STATE_SONAR_0]) * 2.54;

    
  status.target_translate_velocity= 0.0;
  status.target_rotate_velocity  = 0.0;

  
  bee_check_trans_motion_terminator(status.x,
				    status.y,
				    status.orientation
				    + 90.0);
  bee_check_rot_motion_terminator(status.x,
				  status.y,
				  status.orientation
				  + 90.0);






  /* empirical distance measurement */
  if (!initialized){
    last_x = status.x;
    last_y = status.y;
    last_orientation = status.orientation;      
    initialized =1;
    gettimeofday(&start_time, NULL);
    gettimeofday(&last_time2, NULL);
  }
  else{
    gettimeofday(&this_time, NULL);
    time_difference = 
      ((float) (this_time.tv_sec - last_time2.tv_sec))
      + (((float) (this_time.tv_usec - last_time2.tv_usec))
	 /  1000000.0);
    if (time_difference > TIME_FREQUENCY_STATUS){

      diff_dist = sqrt(((last_x - status.x) * (last_x - status.x)) +
		       ((last_y - status.y) * (last_y - status.y)));
      sum_dist += diff_dist;
      last_x = status.x;
      last_y = status.y;
      diff_angle = status.orientation - last_orientation;
      last_orientation = status.orientation;
      for (; diff_angle <= -180.0; ) diff_angle += 360.0;
      for (; diff_angle >   180.0; ) diff_angle -= 360.0;
      sum_angle += fabs(diff_angle);
    

      bee_send_status_update(status.x, status.y, status.orientation,
			     status.vel_trans, status.vel_rot);
      if (print_status){
	fprintf(stderr, "Pos= %4.1f %4.1f %4.1f ", 
		status.x, status.y, status.orientation);
	fprintf(stderr, "Vel= L:%4.1f R:%4.1f trans:%4.1f(%4.1f) rot:%4.1f(%4.1f)  ", 
		status.vel_l, status.vel_r,
		status.vel_trans, diff_dist / time_difference,
		status.vel_rot, diff_angle / time_difference);
	fprintf(stderr, "Bump= %d ", status.bump);
	fprintf(stderr, "Motor= %d\n", status.motor);
      }
      last_time2.tv_sec = this_time.tv_sec;
      last_time2.tv_usec = this_time.tv_usec;
    }

  }

  

  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);
  if (time_difference > TIME_FREQUENCY_SONAR){

    /* transmit sonars */
    bee_send_sonar_update(&(status.sonars[0]));
    
    if (print_sonar_values){
      printf("Sonars:");
      for (i = 0; i < num_sonars; i++)
	printf(" %4.1f", status.sonars[i]);
      printf("\n");
    }
    last_time.tv_sec = this_time.tv_sec;
    last_time.tv_usec = this_time.tv_usec;


  }
 
  
  /* set velocity */


  if (global_set_tvel_new || global_set_rvel_new){

    if (global_set_tvel_new){
      actual_target_translate_velocity =
	0.1 * ((float) global_set_tvel) *
	((float) global_watchdog) * 
	((float) global_tvel_sign);
      if (actual_target_translate_velocity > MAX_TRANSLATIONAL_VELOCITY)
	actual_target_translate_velocity = MAX_TRANSLATIONAL_VELOCITY;
      if (actual_target_translate_velocity < -(MAX_TRANSLATIONAL_VELOCITY))
	actual_target_translate_velocity = -(MAX_TRANSLATIONAL_VELOCITY);
      if (1)
	printf("TCX: translational velocity: %f.\n",
	       actual_target_translate_velocity); 
      global_set_tvel_new = 0;
      last_command_type = 0;
    }
      
    if (global_set_rvel_new){
      actual_target_rotate_velocity =
	- 1.0 / 2.83 * ((float) global_set_rvel) *
	((float) global_watchdog) * 
	((float) global_rvel_sign);
      if (actual_target_rotate_velocity > MAX_ROTATIONAL_VELOCITY)
	actual_target_rotate_velocity = MAX_ROTATIONAL_VELOCITY;
      if (actual_target_rotate_velocity < -(MAX_ROTATIONAL_VELOCITY))
	actual_target_rotate_velocity = -(MAX_ROTATIONAL_VELOCITY);
      if (1)
	printf("TCX: rotational velocity: %f.\n",
	       actual_target_rotate_velocity); 
      global_set_rvel_new = 0;
      last_command_type = 0;
    }
    set_vel(1);  
  }
  else
    set_vel(0);

}

void
alarm_handler()
{
  fprintf(stderr, "$");
  alarm(1, 0);
}



main( int argc, char* argv[])
{
  char* robotName = NULL;
  char* devName = NULL;
  int i;
  
  for (i=1; i<argc; i++) {
    if ((strcmp(argv[i],"-robot")==0)){
      if (i + 1 < argc && argv[i+1][0]!='-'){
	i++;
	robotName = argv[i];
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
    else if(strcmp(argv[i],"-pioneer2")==0) {
      pioneerVersion = 2;
    }
  }
  

  devInit();

  connectDev(&stdin_device);
  stdin_device.outputHnd = stdin_inputHnd;

  /* Set the name of the device if we use a pioneer 1. */
  if ( devName == NULL) { /* No name explicitly set. */
      devName = SCOUT_DEVICE;
  }

  reset_status_report();

  if (use_tcx){
    bee_init(robotName);
    listen_for_tcx_events = 1;
  }

  signal(SIGINT,  (void *) shutdown_robot);
  signal(SIGTERM, (void *) shutdown_robot);
  
  /* connect to the robot */
  connect_robot(1, MODEL_SCOUT2, devName, 38400);

  /* turn on the front sonars */
  zr();
  turn_sonar_on();

  // vm(50.0 / .254, 0.0 / .254, 0.0);


  for (;;) {
    gs();			/* get curennt state */
    update_status(State);
    ProcessDevices();
  }
  
}
