extern struct bParamList * bParamList;

#define NUMBER_STATUS 20	/* number of status reports that will be
				 * memorized */

#define NUMBER_WALLS 20
#define MAX_NUM_SONARS 16

#define PIONEER_AT_ROBOT 0
#define PIONEER_1_ROBOT  1
#define PIONEER_2_ROBOT  2     /* Dennis 17.5.99 */

/* NOT longer need since we use Pioneer.ini now */
/*
#define MAX_VELOCITY 160.0
#define MIN_BATTERY 11.4
#define MAX_BATTERY 12.6
#define MAX_X_Y 32768.0
#define MAX_ORIENTATION 1024.0	
#define ROBOT_RADIUS 33.0	
#define MAX_ORIENTATION 1024.0	
#define MAX_TRANSLATIONAL_VELOCITY 130.0
#define MAX_ROTATIONAL_VELOCITY 360.0
#define VELCONV_FACTOR 0.13
#define STATUS_VELOCITY_FACTOR 50.0
*/

/* parameters for robots with big/outdoor wheels */
/* NOT longer need since we use Pioneer.ini now */
/*
#define AT_BIG_COUNTS_PER_CM 161.3	
#define AT_BIG_DEFAULT_COUNTS_PER_DEGREE 3.1219 
#define AT_BIG_DEFAULT_DRIFT_IN_DEGREE_PER_CM 0.0
#define AT_BIG_ROBIN_COUNTS_PER_DEGREE 3.1858 
#define AT_BIG_ROBIN_DRIFT_IN_DEGREE_PER_CM 0.048 
#define AT_BIG_MARION_COUNTS_PER_DEGREE 3.05805 
#define AT_BIG_MARION_DRIFT_IN_DEGREE_PER_CM 0.015 /
*/

/* parameters for AT robots with small/indoor wheels */
/* NOT longer need since we use Pioneer.ini now */
/*
#define AT_SMALL_COUNTS_PER_CM 158.6403	
#define AT_SMALL_DEFAULT_COUNTS_PER_DEGREE 2.61972 
#define AT_SMALL_DEFAULT_DRIFT_IN_DEGREE_PER_CM 0.0
#define AT_SMALL_ROBIN_COUNTS_PER_DEGREE (AT_SMALL_DEFAULT_COUNTS_PER_DEGREE)
#define AT_SMALL_ROBIN_DRIFT_IN_DEGREE_PER_CM (AT_SMALL_DEFAULT_DRIFT_IN_DEGREE_PER_CM)
#define AT_SMALL_MARION_COUNTS_PER_DEGREE (AT_SMALL_DEFAULT_COUNTS_PER_DEGREE)
#define AT_SMALL_MARION_DRIFT_IN_DEGREE_PER_CM (AT_SMALL_DEFAULT_DRIFT_IN_DEGREE_PER_CM)
*/

/* parameters for Pioneer 1 robot (not name specific) */
/* NOT longer need since we use Pioneer.ini now */
/*
#define P1_COUNTS_PER_CM 197.107	
#define P1_COUNTS_PER_DEGREE 2.889166 
#define P1_DRIFT_IN_DEGREE_PER_CM 0.0 


#define PIONEER_AT_NUM_SONARS 7
#define PIONEER_AT_ROBOT_RADIUS 33.0	
#define PIONEER_AT_COUNTS_PER_CM 161.3	
#define PIONEER_AT_COUNTS_PER_DEGREE 3.15388
#define PIONEER_AT_COMMAND_TRANSLATIONAL_VELOCITY_FACTOR 8.572
#define PIONEER_AT_COMMAND_ROTATIONAL_VELOCITY_FACTOR    3.8


#define PIONEER_II_NUM_SONARS 16
#define PIONEER_II_ROBOT_RADIUS 27.0	
#define PIONEER_II_COUNTS_PER_CM 11.9618	
#define PIONEER_II_COUNTS_PER_DEGREE 11.4074 
#define PIONEER_II_COMMAND_TRANSLATIONAL_VELOCITY_FACTOR 10.3 
#define PIONEER_II_COMMAND_ROTATIONAL_VELOCITY_FACTOR    1.0 
*/

#define PIONEER_AT_ROBOT 0
#define PIONEER_1_ROBOT  1
#define PIONEER_2_ROBOT  2     /* Dennis 17.5.99 */


#define sfCOMPULSE   0
#define sfCOMOPEN    1
#define sfCOMCLOSE   2
#define sfCOMPOLLING 3
#define sfCOMENABLE  4
#define sfCOMSETA    5
#define sfCOMSETV    6
#define sfCOMSETO    7
#define sfCOMMOVE    8
#define sfCOMROTATE  9
#define sfCOMSETRV   10
#define sfCOMVEL     11
#define sfCOMHEAD    12
#define sfCOMDHEAD   13
#define sfCOMDROTATE 14
#define sfCOMSAY     15
#define sfCOMVISION  16
#define sfCOMVWINDOW 17
#define sfCOMUDP     20
#define sfCOMRVEL    21
#define sfCOMDCHEAD  22
#define sfCOMDIGOUT  30
#define sfCOMVEL2    32
#define sfCOMPTUVEL  40
#define sfCOMPTUPOS  41
#define sfCOMPTUNOD  42
#define sfCOMPTUAPOS 43
#define sfCOMSTEP    64

#define DISTpax      0xd0

#define sfSYNC0     0
#define sfSYNC1     1
#define sfSYNC2     2

extern float COUNTS_PER_CM;
extern float DEFAULT_COUNTS_PER_DEGREE;
extern float DEFAULT_DRIFT_IN_DEGREE_PER_CM;

/* NOT longer need since we use Pioneer.ini now */
/*
extern float ROBIN_COUNTS_PER_DEGREE;
extern float ROBIN_DRIFT_IN_DEGREE_PER_CM;
extern float MARION_COUNTS_PER_DEGREE;
extern float MARION_DRIFT_IN_DEGREE_PER_CM;
*/

typedef struct {
  int defined;
  unsigned long count;
  int    motors_status, left_stalled, right_stalled;
  float  internal_x, internal_y, internal_orientation;
  float  x, y, orientation;
  float  vel_l, vel_r, vel_x, vel_y, vel_orientation;
  float  accel_l, accel_r;
  float  battery, control;
  float  sonars[MAX_NUM_SONARS];
  int    sonar_confidence[MAX_NUM_SONARS];
  int    sonar_new_flag[MAX_NUM_SONARS];
  int    sync_level;
  struct timeval last_sync_time;
  struct timeval last_status_time;
  float  target_translate_velocity;
  float  target_rotate_velocity;
} STATUS_TYPE, *STATUS_PTR;


typedef struct {
  DEV_TYPE dev;
  /* put in here whatever is needed for starting up the real device.*/
}  BASE_TYPE, *BASE_PTR;

extern BASE_TYPE    base_device;


extern unsigned long   total_sonar_count;


extern float counts_per_degree;
extern float drift_in_degree_per_cm;

int configure_eight_sonars();

void
BASE_outputHnd(int fd, long chars_available);

void BASE_timeoutHnd(void);

void signal_base(void);

static void 
ProcessLine(unsigned char *line);

void 
stdin_inputHnd(int fd, long chars_available);

unsigned int
calc_checksum(unsigned char *ptr);

int 
WriteCommand(unsigned char command, int argument, unsigned char *text);

void
connect_to_robot(int force);

int 
next_status();


void
alarm_handler();

void
ping_robot();

void
shutdown_robot();









