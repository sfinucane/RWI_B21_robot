#ifndef LASER_INTERFACE_LOADED
#define LASER_INTERFACE_LOADED

#define NUMBER_LASERS          180
#define MAX_LASER_RANGE        5000.0
#define ROBOT_RADIUS           35.0
#define DUMMY_LASER_DEVICE     "DUMMY_DEV"

#define UKN_TYPE               -1
#define PLS_TYPE               0
#define LMS_TYPE               1

/************************************************************************
 *  Specifications of the range finders. 
 ************************************************************************/

#define FRONT_LASER            0
#define REAR_LASER             1

#define NOT_SET                99

/* 14 is for a baud rate of 19200 */

#define LASER_BAUD_RATE        13
#define FRONT_LASER_ANGLE      0.0         /* Angle relative to the robot */  
#define REAR_LASER_ANGLE       DEG_180     /* Angle relative to the robot */  

/************************************************************************
 *  COnstants
 ************************************************************************/


#define F_ERROR MAXFLOAT
#define I_ERROR (MAXINT)

#define ROB_RADIUS (bRobot.base_radius) 
#define COLLI_SONAR_RADIUS 2.48
#define SONAR_OPEN_ANGLE (DEG_TO_RAD(7.5))
#define MAX_TRANS_SPEED 90.0
#define MAX_ROT_SPEED (DEG_TO_RAD(60.0))
#define NO_OF_SONARS bRobot.sonar_cols[0]

#define DEG_90 (1.5707963705)
#define DEG_180 (3.1415927410)
#define DEG_270 (4.7123891115)
#define DEG_360 (6.2831854820)
#define SGN(x)    ((x) >= 0.0 ? (1) : (-1))

#define LASER_POLLSECONDS          1

/* Events of the lasers and the corresponding handler functions. */
#define SINGLE_LASER_REPORT     0  /* laser_readings of one laser complete */
#define COMPLETE_LASER_REPORT   1  /* laser_readings of all lasers complete */
#define LASER_REPORT_MISSED     2  /* not used */
#define LASER_NUMBER_EVENTS     3

#define BOOLEAN int

/* All these values used to be defines. Changed them so that we can set them
 * arbitrarily. */
extern int   NUMBER_OF_LASERS;
extern int   USE_FRONT_LASER;
extern int   USE_REAR_LASER;

extern int   USE_LMS;

extern int ARG_USE_FRONT_LASER;
extern int ARG_USE_REAR_LASER;
extern int ARG_USE_TWO_LASERS;

extern int ARG_NUM_VALUES;
extern int USE_DATAFILE;
extern int CORR_DIST;
extern FILE *datafile;

extern int CONFIG_LASER_MODE;

/* Will be set in laser_interface.c. */

typedef struct _dev *DEV_PTR;

typedef void  *Pointer;
typedef void (*DEVICE_OUTPUT_HND)(int, long );
typedef void (*DEVICE_SET_TIMEOUT)(DEV_PTR, int);
typedef void (*DEVICE_CANCEL_TIMEOUT)(DEV_PTR);
typedef void (*Handler)(Pointer, Pointer);

typedef struct _dev {
  struct {
    char *ttyPort;
    int baudCode;   /* one of the codes B0 .. B9600 in ttydev.h */
    char *parity;
  } ttydev;
  char *devName;
  char *passwd;
  char *typestr;
  int type;          /* 0: PLS, 1: LMS */
  int fd;
}  DEV_TYPE;

typedef struct {
  DEV_TYPE dev;
  /* put in here whatever is needed for starting up the real device.*/
} LaserDeviceType, *LaserDevicePtr;

typedef struct {
  int   defined;
  float angles[NUMBER_LASERS];
  float values[NUMBER_LASERS];
  float display_values[NUMBER_LASERS];
  int   blendung[NUMBER_LASERS];
  int   wfv[NUMBER_LASERS];
  int   sfv[NUMBER_LASERS];
  int   num_values;
  struct timeval time;
  struct timeval timestamp;
} LaserSensorValueType, *LaserSensorValuePtr;

void LaserInit(int laser_no);
void LaserShutdownLaser();

/************************************************************************
 *  Type for the laser readings.
 ************************************************************************/

typedef struct Point {
  float x;
  float y;
} Point;

typedef struct LASER_reading {
  int new;
  int numberOfReadings;
  float* reading;
  int*  blendung;
  int*  wfv;
  int*  sfv;
  float startAngle;
  float angleResolution;
  Point rPos;
  float rRot;
  struct timeval time;
} LASER_reading;

extern LASER_reading frontLaserReading;
extern LASER_reading rearLaserReading;

typedef struct {
  int laserNumber;
  LASER_reading* scan;
  DEV_TYPE dev;
}  LASER_TYPE, *LASER_PTR;

BOOLEAN start_laser(char* frontDevice, char* rearDevice);
void    requestNextLaserScan( int laserNumber);
void    stop_laser(void);
void    LASER_look_for_laser_device(void);
void    LASER_InstallHandler(Handler handler, int event, Pointer client_data);
void    LASER_RemoveHandler(Handler handler, int event);
void    LASER_RemoveAllHandlers(int event);

/* Functions called by the device. */
void    FRONT_LASER_outputHnd( int fd, long chars_available);
void    REAR_LASER_outputHnd( int fd, long chars_available);
void    LASER_timeoutHnd(void);
void    LaserPing(LASER_TYPE* laserDevice);

extern LaserSensorValueType LaserSensors[2];
extern LaserDeviceType LaserDevice[2];

#ifdef DECLARE_LASER_VARS

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/************************************************************************
 *  Laser device type.
 ************************************************************************/
#define DEFAULT_PORT    1621
#define LASER_DEV_NAME "laser"

LASER_TYPE    frontLaserDevice = 
{
  FRONT_LASER,
  &frontLaserReading,
  { 
    { DUMMY_LASER_DEVICE, LASER_BAUD_RATE, "EVEN"},
    LASER_DEV_NAME,
    PLS_PASSWORD,
    "",
    -1,
    -1
  }
};


LASER_TYPE    rearLaserDevice = 
{
  REAR_LASER,
  &rearLaserReading,
  {
    { DUMMY_LASER_DEVICE, LASER_BAUD_RATE, "EVEN"},
    LASER_DEV_NAME,
    PLS_PASSWORD,
    "",
    -1,
    -1
  }
};

#else

extern LASER_TYPE   frontLaserDevice;
extern LASER_TYPE   rearLaserDevice;

#endif

#endif
