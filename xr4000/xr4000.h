
#define TIME_FREQUENCY_STATUS 0.125
#define TIME_FREQUENCY_SONAR 0.3


#define MAX_NUM_SONARS 24		/* number of sonar sensors */

#define MAX_VELOCITY 160.0
#define MIN_BATTERY 11.4
#define MAX_BATTERY 12.6

#define MAX_TRANSLATIONAL_VELOCITY 150.0
#define MAX_ROTATIONAL_VELOCITY 360.0

#define WHEELBASE (13.4*2.54)
#define ROT_VEL_FACT (360.0/((WHEELBASE)*2.0*M_PI))


typedef struct {
  unsigned long count;
  int    motors_status;
  float  x, y, orientation;
  float  vel_trans, vel_rot;
  float  vel_l, vel_r;
  float  accel_l, accel_r;
  float  battery, control;
  float  sonars[MAX_NUM_SONARS];
  float  target_translate_velocity;
  float  target_rotate_velocity;
  int    bump, motor;
} STATUS_TYPE, *STATUS_PTR;





typedef struct {
  DEV_TYPE dev;
  /* put in here whatever is needed for starting up the real device.*/
}  BASE_TYPE, *BASE_PTR;

extern BASE_TYPE    base_device;


extern unsigned long   total_sonar_count;


extern float counts_per_degree;
extern float drift_in_degree_per_cm;



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
