

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <math.h>

#include "devUtils.h"
#include "robot_arg.h"
#include "xr4000.h"
#include "bee_interface.h"


xr4000_reset_odometry(){ }
xr4000_get_voltage(){ }




struct timeval last_ping_time = {0, 0};

int use_tcx = 1;

struct N_RobotState *rs;
struct N_AxisSet *as;

  long robot_id;

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



#define XY_ACCEL 1500 /* was: 300 */
#define PSI_ACCEL 5000 /* was: 1100 */




static void Translate(int speed)
{
  int result;
  long x, y;
  struct N_AxisSet *as;

  as = &rs->AxisSet;

  if ((result = N_GetIntegratedConfiguration(rs->RobotID)) != N_NO_ERROR)
  {
    fprintf(stderr, "get_integrated_conf error: %d\n", result);
    exit(-1);
  }

  x = rs->Integrator.x;
  y = rs->Integrator.y;

  as->Axis[N_YTRANSLATION].Update = TRUE;
  as->Axis[N_YTRANSLATION].DesiredPosition = 0;	// not used for velocity mode
  as->Axis[N_YTRANSLATION].DesiredSpeed = speed;
  as->Axis[N_YTRANSLATION].Acceleration = XY_ACCEL;
  as->Axis[N_YTRANSLATION].Mode = N_AXIS_VELOCITY;
 
  if ((result = N_SetAxes(rs->RobotID)) != N_NO_ERROR)
  {
    fprintf(stderr, "set_axis error: %d\n", result);
    exit(-1);
  }

  return;
}

static void Rotate(int speed)
{
  int result;
  struct N_AxisSet *as;

  /*printf("rotate %d\n", speed);*/

  as = &rs->AxisSet;

  if ((result = N_GetIntegratedConfiguration(rs->RobotID)) != N_NO_ERROR)
  {
    fprintf(stderr, "get_integrated_conf error: %d\n", result);
    exit(-1);
  }

  //  psi = rs->Integrator.Rotation;

  as->Axis[N_ROTATION].Update = TRUE;
  as->Axis[N_ROTATION].DesiredPosition = 0;	/* not used in velocity mode */
  as->Axis[N_ROTATION].DesiredSpeed = speed;
  as->Axis[N_ROTATION].Acceleration = PSI_ACCEL;
  as->Axis[N_ROTATION].Mode = N_AXIS_VELOCITY;

  if ((result = N_SetAxes(rs->RobotID)) != N_NO_ERROR)
  {
    fprintf(stderr, "set_axis error: %d\n", result);
    exit(-1);
  }

  return;
}




   
void
shutdown_robot()
{
  int error;

  fprintf(stderr, "\nShooting down robot...");
  actual_target_translate_velocity = 0.0;
  actual_target_rotate_velocity = 0.0;
  /* stop the robot */
  rs->AxisSet.Global = FALSE;
  rs->AxisSet.Axis[N_XTRANSLATION].Mode = N_AXIS_STOP;
  rs->AxisSet.Axis[N_YTRANSLATION].Mode = N_AXIS_STOP;
  rs->AxisSet.Axis[N_ROTATION].Mode = N_AXIS_STOP;
  rs->AxisSet.Axis[N_XTRANSLATION].Update = TRUE;
  rs->AxisSet.Axis[N_YTRANSLATION].Update = TRUE;
  rs->AxisSet.Axis[N_ROTATION].Update = TRUE;
  if ((error = N_SetAxes(robot_id)) != N_NO_ERROR){
      fprintf(stderr, "error stopping the robot (%d).\n", error);
      exit(-1);
    }
  rs->SonarController.SonarSet[0].FiringOrder[0] = 255;
  rs->SonarController.SonarSet[1].FiringOrder[0] = 255;

  usleep(500000);
  N_DisconnectRobot(robot_id);
  fprintf(stderr, "done.\n");
  exit(-1);
}


void
process_input(char *command)
{
  int i, error;


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
    /* stop the robot */
    rs->AxisSet.Global = FALSE;
    rs->AxisSet.Axis[N_XTRANSLATION].Mode = N_AXIS_STOP;
    rs->AxisSet.Axis[N_YTRANSLATION].Mode = N_AXIS_STOP;
    rs->AxisSet.Axis[N_ROTATION].Mode = N_AXIS_STOP;
    rs->AxisSet.Axis[N_XTRANSLATION].Update = TRUE;
    rs->AxisSet.Axis[N_YTRANSLATION].Update = TRUE;
    rs->AxisSet.Axis[N_ROTATION].Update = TRUE;
    if ((error = N_SetAxes(robot_id)) != N_NO_ERROR)
      {
      fprintf(stderr, "error stopping the robot (%d).\n", error);
	exit(-1);
      }
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
    xr4000_reset_odometry();
  }
  else if (command[0] == 'b' || command[0] == 'B')
    printf("Battery: %3.1f Volt\n", xr4000_get_voltage());




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

  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);
  if (time_difference > 0.5 || force){
    last_time.tv_sec = this_time.tv_sec;
    last_time.tv_usec = this_time.tv_usec;


    Rotate((int) (actual_target_rotate_velocity/180.0*M_PI*1000.0));
    Translate((int) (actual_target_translate_velocity * 10.0));

    /*vm((int) (set_r / .254), (int) (set_l / .254), 0.0);*/
  }
}



/* array of sensor indices in a circle.  first index is the set
 * number, second index is the sonar number.  we only describe the
 * top three sensor sets, as the bottom sets can be had by adding 1
 * to the set number */
static const int sw_sensor_circle[24][2] = 
{
  {4,6}, {4,7},
  {0,0}, {0,1}, {0,2}, {0,3}, {0,4}, {0,5}, {0,6}, {0,7},
  {2,0}, {2,1}, {2,2}, {2,3}, {2,4}, {2,5}, {2,6}, {2,7},
  {4,0}, {4,1}, {4,2}, {4,3}, {4,4}, {4,5} 
};

struct N_SonarController *scp;

/* ------------------------------------------------------------------------
 * Function:     SonarReadingAtIndex
 * Purpose:      return the reading at a given sonar index in [0, 48]
 *                 corresponding to sonar positions about the robot.
 * Arguments:    *scp : from the state structure
 *               sn_index : the index to check
 * Return:       long : the minimum of the values from the top sonar ring
 *                 and the bottom sonar ring, at the given index
 * ------------------------------------------------------------------------ */

static long SonarReadingAtIndex(int sn_index)
{
  int set, sn;
  long r1, r2;

  set = sw_sensor_circle[sn_index][0];
  sn  = sw_sensor_circle[sn_index][1];

  r1 = scp->SonarSet[set + 1].Sonar[sn].Reading;
  if (r1 > 100000)
    r1 = 100000;
  r2 = scp->SonarSet[set].Sonar[sn].Reading;
  if (r2 > 100000)
    r2 = 100000;
  if (r1 < r2)
    return r1;
  else
    return r2;
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


  status.motors_status           = 0;
  status.x                       = ((float) rs->Integrator.y) * 0.1;
  status.y                       = - ((float) rs->Integrator.x) * 0.1;
  status.orientation             = ((float) rs->Integrator.Rotation) 
    * 180.0 * 0.001 / M_PI;
  for (; status.orientation <= -180.0; ) status.orientation += 360.0;
  for (; status.orientation >   180.0; ) status.orientation -= 360.0;

  status.vel_trans =
    0.1 * ((float) rs->AxisSet.Axis[N_YTRANSLATION].ActualVelocity);
  status.vel_rot   = ((float) rs->AxisSet.Axis[N_ROTATION].ActualVelocity)
    * 180.0 * 0.001 / M_PI;


  for (i = 0; i < num_sonars; i++)
    status.sonars[i]           = 0.1 * ((float) SonarReadingAtIndex((i+5)%num_sonars));

  /*
  status.vel_l                 = ((float) state[STATE_VEL_TRANS]) * .254;
  status.vel_r                 = ((float) state[STATE_VEL_STEER]) * .254;
  status.vel_trans = 0.5 * (status.vel_l + status.vel_r);
  status.vel_rot   = - ROT_VEL_FACT * (status.vel_l - status.vel_r);
  status.accel_l                 = 0.0;
  status.accel_r                 = 0.0;
  status.battery                 = 0.0;
  status.control                 = 0.0;
  status.bump                    = (int) state[STATE_BUMPER];
  status.motor                   = (int) state[STATE_MOTOR_STATUS];
  status.count                   = status.count + 1;    
  */


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
	fprintf(stderr, "Raw= %ld %ld %ld ", 
		rs->Integrator.x, rs->Integrator.y,
		rs->Integrator.Rotation);


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
  char *sched_host;
  const char *argv0;
  unsigned short sched_port;
  int error;
  struct N_SonarSet *set;

  char* robotName = NULL;
  char* devName = NULL;
  int i, j;
  long State;
  
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



  argv0 = ARG_GetProgramName(argv[0]);

  /* get the robot ID and scheduler from the command line */
  sched_host = "localhost";
  sched_port = 7073;
  robot_id = 1;
  ARG_GetStandardArgs(&argc, &argv, &sched_host, &sched_port, &robot_id);

  /* initialize the client library and connect to our robot */
  if ((error = N_InitializeClient(sched_host, sched_port)) != N_NO_ERROR ||
      (error = N_ConnectRobot(robot_id)) != N_NO_ERROR)
  {
    fprintf(stderr, "%s: error connecting (%d).\n", argv0, error);
    exit(-1);
  }

  /* get a pointer to the state structure and the AxisSet substructure */
  rs = N_GetRobotState(robot_id);
  as = &rs->AxisSet;
  
  /* here, we zero the robot.  before setting the integrated configuration
   * to zero, we must make sure that none of the axes have a non-zero
   * goal position so that the zeroing does not induce a move */
  as->Global = FALSE;
  as->Axis[N_XTRANSLATION].Mode = N_AXIS_POSITION_ABSOLUTE;
  as->Axis[N_XTRANSLATION].DesiredPosition = 0;
  as->Axis[N_XTRANSLATION].DesiredSpeed = 0;
  as->Axis[N_XTRANSLATION].Acceleration = 0;
  as->Axis[N_XTRANSLATION].Update = TRUE;
  as->Axis[N_YTRANSLATION].Mode = N_AXIS_POSITION_ABSOLUTE;
  as->Axis[N_YTRANSLATION].DesiredPosition = 0;
  as->Axis[N_YTRANSLATION].DesiredSpeed = 0;
  as->Axis[N_YTRANSLATION].Acceleration = 0;
  as->Axis[N_YTRANSLATION].Update = TRUE;
  as->Axis[N_ROTATION].Mode = N_AXIS_POSITION_ABSOLUTE;
  as->Axis[N_ROTATION].DesiredPosition = 0;
  as->Axis[N_ROTATION].DesiredSpeed = 0;
  as->Axis[N_ROTATION].Acceleration = 0;
  as->Axis[N_ROTATION].Update = TRUE;
  if ((error = N_SetAxes(robot_id)) != N_NO_ERROR)
  {
    fprintf(stderr, "%s: error setting axes to zero (%d).\n", argv0, error);
    exit(-1);
  }

  /* zero the integrated configuration */
  rs->Integrator.x = 0;
  rs->Integrator.y = 0;
  rs->Integrator.Rotation = 0;
  if ((error = N_SetIntegratedConfiguration(robot_id)) != N_NO_ERROR)
  {
    fprintf(stderr, "%s: error zeroing (%d).\n", argv0, error);
    exit(-1);
  }

  /* set up unchanging parameters */
  as->Global = FALSE;
  as->Axis[N_XTRANSLATION].Mode = N_AXIS_VELOCITY;
  as->Axis[N_YTRANSLATION].Mode = N_AXIS_VELOCITY;
  as->Axis[N_ROTATION].Mode = N_AXIS_VELOCITY;


  /* initialize sonar */
  scp = &(rs->SonarController);
  /* configure the sonars for full firing order */
  for (j = 0; j < rs->SonarController.SonarSetCount; j++)
  {
    set = &rs->SonarController.SonarSet[j]; 
    set->FiringDelay = 0;
    set->TimeStampActive = FALSE;
    for (i = 0; i < set->SonarCount; i++)
    {
      set->FiringOrder[i] = i;
    }
  }
  if ((error = N_SetSonarConfiguration(robot_id)) != N_NO_ERROR)
  {
    fprintf(stderr, "%s: error initializing sonars (%d).\n", argv[0], error);
    exit(-1);
  }

  for (;;) {
    N_GetAxes(robot_id);
    N_GetSonarConfiguration(robot_id);
    N_GetSonar(robot_id);
    update_status(&State);
    ProcessDevices();
  }
  
}
