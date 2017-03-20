/*
 * $Id: io.h,v 1.1 2002/09/14 16:33:01 rstone Exp $
 *
 */


#ifndef LASER_INTERFACE_LOADED
#define LASER_INTERFACE_LOADED



#define NUMBER_LASERS 180

#define MAX_LASER_RANGE 500.0

#define ROBOT_RADIUS 35.0

/************************************************************************
 *  Specifications of the range finders. 
 ************************************************************************/

#define FRONT_LASER 0
#define REAR_LASER  1

/* 14 is for a baud rate of 19200 */
#define LASER_BAUD_RATE 13

#define FRONT_LASER_ANGLE 0.0   /* Angle relative to the robot */  
#define REAR_LASER_ANGLE DEG_180  /* Angle relative to the robot */  

/* All these values used to be defines. Changed them so that we can set them
 * arbitrarily. */
extern int NUMBER_OF_LASERS;
extern int USE_FRONT_LASER;
extern int USE_REAR_LASER;

#ifdef FGAN
#define PARITY "NONE"
#else
#define PARITY "EVEN"
#endif

extern float FRONT_LASER_OFFSET; /* Offset in cm from the center of the robot */
extern float REAR_LASER_OFFSET; /* Offset in cm from the center of the robot */

extern struct bParamList *bParamList;


/* Will be set in laser_interface.c. */
#define DUMMY_LASER_DEVICE "DUMMY_DEV"

#define LASER_MAX_RANGE 5000

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
  struct timeval time;
} LaserSensorValueType, *LaserSensorValuePtr;

LaserSensorValueType LaserSensors[2];
LaserDeviceType LaserDevice[2];
void LaserInit(int laser_no);
void LaserShutdownLaser();


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


/************************************************************************
 *  Type for the laser readings.
 ************************************************************************/

typedef struct Point {
  float x;
  float y;
} Point;

typedef struct LASER_reading {
 /* Have the values been converted to obstacle points? */
  int new;

  /* Values specific to the mode of the scans. */
  int numberOfReadings;
  float* reading;
  int*  blendung;
  int*  wfv;
  int*  sfv;
  float startAngle;
  float angleResolution;
  Point rPos;
  float rRot;

  /* Time of the scan. */
  struct timeval time;
} LASER_reading;

/* Readings of the two laser range finders. */
extern LASER_reading frontLaserReading;
extern LASER_reading rearLaserReading;

typedef struct {
  int laserNumber;
  LASER_reading* scan;
  DEV_TYPE dev;
}  LASER_TYPE, *LASER_PTR;

#define LASER_POLLSECONDS          1


/************************************************************************
 *  Simple laser commands
 *  NOTE : LASER_init must be called prior to any other LASER command.
 ************************************************************************/

BOOLEAN start_laser(char* frontDevice, char* rearDevice);
void
requestNextLaserScan( int laserNumber);
void stop_laser(void);
void LASER_look_for_laser_device(void);

/* Events of the lasers and the corresponding handler functions. */
#define SINGLE_LASER_REPORT     0  /* laser_readings of one laser complete */
#define COMPLETE_LASER_REPORT   1  /* laser_readings of all lasers complete */
#define LASER_REPORT_MISSED     2  /* not used */
#define LASER_NUMBER_EVENTS     3



void  LASER_InstallHandler(Handler handler, int event, Pointer client_data);
void  LASER_RemoveHandler(Handler handler, int event);
void  LASER_RemoveAllHandlers(int event);

/* Functions called by the device. */
void FRONT_LASER_outputHnd( int fd, long chars_available);
void REAR_LASER_outputHnd( int fd, long chars_available);
void LASER_timeoutHnd(void);
void LaserPing(LASER_TYPE* laserDevice);

#ifdef DECLARE_LASER_VARS

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/************************************************************************
 *  Laser device type.
 ************************************************************************/

LASER_TYPE    frontLaserDevice = 
{
  FRONT_LASER,
  &frontLaserReading,
  { FALSE,
    { "", DEFAULT_PORT},
    /* 14 is for a baud rate of 19200 */
    { DUMMY_LASER_DEVICE, LASER_BAUD_RATE},
    LASER_DEV_NAME,
    -1,
    TRUE,
    FALSE,
    (FILE *) NULL,
    (fd_set *) NULL,
    (DEVICE_OUTPUT_HND) FRONT_LASER_outputHnd,
    LASER_timeoutHnd,
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


LASER_TYPE    rearLaserDevice = 
{
  REAR_LASER,
  &rearLaserReading,
  { FALSE,
    { "", DEFAULT_PORT},
    /* 14 is for a baud rate of 19200 */
    { DUMMY_LASER_DEVICE, LASER_BAUD_RATE},
    LASER_DEV_NAME,
    -1,
    TRUE,
    FALSE,
    (FILE *) NULL,
    (fd_set *) NULL,
    (DEVICE_OUTPUT_HND) REAR_LASER_outputHnd,
    LASER_timeoutHnd,
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

#else

extern LASER_TYPE   frontLaserDevice;
extern LASER_TYPE   rearLaserDevice;

#endif

#endif




/*
 * $Log: io.h,v $
 * Revision 1.1  2002/09/14 16:33:01  rstone
 * *** empty log message ***
 *
 * Revision 1.8  1999/07/20 14:25:42  schneid1
 * laserServer now uses beeSoft.ini & -display works again
 *
 * Revision 1.7  1999/04/18 19:00:09  fox
 * This is just to get the most recent version into the cmu bee tree. It shouldn't do any harm (hopefully!).
 *
 * Revision 1.6  1998/10/30 18:58:03  fox
 * Added support for pioneers and multiple robots.
 *
 * Revision 1.5  1998/08/16 19:02:22  thrun
 * Will this run at 38400 baud?
 * THere is still stuff missing. Right now, it basically
 * ignores beeSoft.ini and runs with both lasers always.
 *
 * Revision 1.4  1998/08/14 16:30:45  thrun
 * .
 *
 * Revision 1.3  1998/08/06 03:22:55  thrun
 * supports 2 lasers.
 *
 * Revision 1.2  1997/08/07 02:45:51  swa
 * First version of a laserServer. Compiles nicely, yet produces nothing useful. ;)
 * Just to copy things to a safe place ...
 *
 * Revision 1.1  1997/08/06 15:12:32  swa
 * Very first and incomplete version of a laserServer. No TCX comm yet.
 *
 *
 */
