/* /include/Nclient.h
 *
 * Nomadic client library
 * Public declarations
 *
 *                             ** COPYRIGHT NOTICE **
 *
 * Copyright (c) 1996, Nomadic Technologies, Inc.
 *
 * The contents of this  file is copyrighted by  Nomadic Technologies,
 * Inc, and is protected by United States copyright laws. You may make
 * as  many copies   of  the software   as  deemed  necessary  for the
 * effective programming of the Nomad  series robots provided that all
 * copyright and other proprietary   notices are reproduced  in  their
 * entirety.
 *
 */

#ifndef _N_INCLUDE_CLIENT_H_
#define _N_INCLUDE_CLIENT_H_

#include <limits.h>


/* --- Symbols ------------------------------------------------------------- */

/* General */

#define N_IGNORE                                Nan

#ifndef BOOL
#define BOOL unsigned char
#endif
#ifndef FALSE
#define FALSE ((BOOL)0)
#endif
#ifndef TRUE
#define TRUE  (!FALSE)
#endif

#define N_MAX_HOSTNAME_LENGTH   80

/* Measurement types for 'Unit' */
#define N_MILLIMETERS   'm'
#define N_SIXTEENTHS    'x'


/* Error codes for user functions. */
#define N_NO_ERROR                       0
#define N_UNKNOWN_ERROR                 -1
#define N_MISSING_ARGUMENT              -2
#define N_INVALID_ARGUMENT              -3
#define N_COMMAND_NOT_FOUND             -4
#define N_ROBOT_NOT_FOUND               -5
#define N_CONNECTION_FAILED             -6
#define N_ABNORMAL_ROBOT_POSITION       -7
#define N_TIMEOUT_ERROR                 -8
#define N_WRONG_OS_ERROR                -9
#define N_AXES_NOT_READY                -10
#define N_OUT_OF_MEMORY                 -11
#define N_UNSUPPORTED                   -12
#define N_SENSOR_NOT_READY              -13
#define N_DEVICE_NOT_FOUND              -14
#define N_UNINITIALIZED                 -15
#define CONNECTION_EXISTS                -16

/* Robot types, used in the RobotType field of the N_RobotState structure. */
#define N_INVALID_ROBOT_TYPE    '?'
#define N_N200_ROBOT_TYPE       'n'
#define N_XR4000_ROBOT_TYPE     'x'


/* --- Axes --- */

/* N_AxisSet.Status bit masks. */
#define N_AXES_READY         0
#define N_ESTOP_DOWN         1
#define N_JOYSTICK_IN_USE    2
#define N_MOTION_ERROR       4

/* Array bound for the Axis array field of the N_AxisSet structure. */
#define N_MAX_AXIS_COUNT     4

/* Indices into the Axis array field of the N_AxesSet structure for
 * XR4000 series robots. */
#define N_XTRANSLATION       0
#define N_YTRANSLATION       1
#define N_ROTATION           2

/* Axis modes. */
#define N_AXIS_NONE                  0
#define N_AXIS_ACCELERATION          1
#define N_AXIS_POSITION_RELATIVE     2
#define N_AXIS_POSITION_ABSOLUTE     3
#define N_AXIS_VELOCITY              4
#define N_AXIS_STOP                  5


/* --- Lift --- */

/* Array bound for the Axis array field of the N_LiftController structure. */
#define N_LIFT_AXIS_COUNT 2

/* Indices into the Axis array field of the N_LiftController structure for
   XR4000 series robots. */
#define N_LIFT               0
#define N_GRIP               1

/* N_LiftAxis.Mode values */
#define N_LIFT_NONE                  0
#define N_LIFT_ACCELERATION          1
#define N_LIFT_POSITION_RELATIVE     2
#define N_LIFT_POSITION_ABSOLUTE     3
#define N_LIFT_VELOCITY              4
#define N_LIFT_STOP                  5

/* N_LiftAxis.Status bit field masks */
#define N_LIFT_POS_LIMIT             0x01
#define N_LIFT_NEG_LIMIT             0x02
#define N_LIFT_MOTION_ERROR          0x04
#define N_LIFT_ESTOP                 0x08
#define N_LIFT_CANNOT_MOVE           0x10



/* --- Sonars --- */

/* Array bound for the SonarSet array field of the N_SonarController
 * structure. */
#define N_MAX_SONAR_SET_COUNT     6

/* Array bound for the Sonar array field of the N_SonarSet structure. */
#define N_MAX_SONAR_COUNT         16

/* Terminator for sonar set firing orders, used in the FiringOrder array field
 * of the N_SonarSet structure. */
#define N_END_SONAR_FIRING_ORDER  255

/* Value which indicates that a sonar timed out */
#define N_SONAR_TIMEOUT           LONG_MAX


/* --- Infrareds --- */

/* Array bound for the InfraredSet array field of the N_InfraredController
 * structure */
#define N_MAX_INFRARED_SET_COUNT  6

/* Array bound for the Infrared array field of the N_InfraredSet structure. */
#define N_MAX_INFRARED_COUNT      16

/* Terminator for infrared set firing orders, used in the FiringOrder array
 * field of the N_InfraredSet structure. */
#define N_END_INFRARED_FIRING_ORDER     255


/* --- Bumpers --- */

/* Array bound for the BumperSet array field of the N_BumperController
 * structure. */
#define N_MAX_BUMPER_SET_COUNT    6

/* Array bound for the Bumper array field of the N_BumperSet structure. */
#define N_MAX_BUMPER_COUNT        12

/* Possible bumper states, used in the Reading field of the N_Bumper
 * strucutre. */
#define N_BUMPER_NONE 0x0
#define N_BUMPER_LOW  0x1
#define N_BUMPER_HIGH 0x2


/* --- Sensus 500 laser stripers --- */

/* Array bound for the Laser field of the N_LaserSet structure. */
#define N_MAX_LASER_COUNT         2

/* Array bound for the Data field of the N_Laser structure. */
#define N_MAX_SCANLINE_COUNT      482

/* Flags for the Power field of the N_Laser structure. */
#define N_ON                      '1'
#define N_OFF                     '0'

/* Data types, used in the DataType field of the N_Laser structure. */
#define N_LINE                    'l'
#define N_POINT                   'p'

/* Point types, used in the PointType field of the N_Laser structure. */
#define N_RAISE                   'r'
#define N_PEAK                    'p'
#define N_FALL                    'f'
#define N_MAGNITUDE               'm'
#define N_COORDINATES             'c'


/* --- Sensus 550 (SICK) laser rangefinders --- */

/* Array bound for the S550 field of the N_S550Set structure. */
#define N_MAX_S550_COUNT      2

/* Array bound for the Readings array field of the N_S550 structure. */
#define N_MAX_S550_POINTS     361

/* Array bound for the WarningField array field of the N_S550 structure. */
#define N_MAX_S550_WF_COUNT   181

/* Array bound for the SafeField array field of the N_S550 structure. */
#define N_MAX_S550_SF_COUNT   181

/* Safefield types, used in the SafeFieldType field of the N_S550 structure. */
#define N_S550_SAFEFIELD_ARC  'a'
#define N_S550_SAFEFIELD_RECT 'r'
#define N_S550_SAFEFIELD_SEGS 's'


/* --- Batteries --- */

/* Array bound on the Battery field of the N_BatterySet structure. */
#define N_MAX_BATTERY_COUNT       4


/* Fields declared N_CONST are not meant to be modified by clients of this
 * API and will never be changed once initialized by N_ConnectRobot().  Do
 * not modify this definition. */
#ifndef N_CONST
#ifdef __cplusplus
#define N_CONST
#else
#define N_CONST const
#endif
#endif


/* --- Data structures ----------------------------------------------------- */


/* --- Integrated configuration --- */

struct N_Integrator
{
  BOOL DataActive;
  BOOL TimeStampActive;

  long x;
  long y;
  long Steering;
  long Rotation;
  unsigned long TimeStamp;
};



/* --- Axes --- */

struct N_Axis 
{
  BOOL DataActive;       /* Set to FALSE to make N_GetAxes ignore this axis. */
  BOOL TimeStampActive;  /* Set to TRUE to get time stamps with the data. */
  BOOL Update;           /* Set to TRUE to send this axis to the robot. */

  unsigned long TimeStamp;

  /* the Mode parameter describes how the settable parameters should be
   * interpreted when N_SetAxes is called, and is not retrieved by
   * N_GetAxes */
  char Mode;

  /* settable/retrievable parameters */
  long DesiredPosition;            /* Ignored for velocity moves. */
  long DesiredSpeed;               /* Scalar, greater than zero. */
  long Acceleration;               /* Scalar, greater than zero. */

  /* retrievable parameters */
  long TrajectoryPosition;         /* Instantaneous goal position. */
  long TrajectoryVelocity;         /* Instantaneous goal velocity. */
  long ActualPosition;             /* Current position. */
  long ActualVelocity;             /* Current velocity. */
  BOOL InProgress;                 /* FALSE if no move is being executed. */
};


struct N_AxisSet
{
  /* The global flag determines if the XR4000's X and Y axis use the global
   * reference frame (the x axis and y axis are determined by the robot's
   * orientation when it was zeroed), or the local reference frame (where
   * the y axis always points in the same direction as the front of the
   * robot).  This flag does not affect the IntegratedConfiguration
   * values.  Global mode is only supported by the XR4000. */
  BOOL Global;

  /* The status field can be used to check for unusual conditions in the
   * motor controller (e.g. e-stop down or joystick in use). */
  unsigned char Status;

  N_CONST unsigned int AxisCount;
  struct N_Axis Axis[N_MAX_AXIS_COUNT];
};



struct N_LiftAxis
{
  BOOL DataActive;       /* Set to FALSE to make N_GetLift ignore this axis. */
  BOOL TimeStampActive;  /* Set to TRUE to get time stamps with the data. */
  BOOL Update;           /* Set to TRUE to send this axis to the robot. */

  unsigned long TimeStamp;

  /* the Mode parameter describes how the settable parameters should be
   * interpreted when N_SetLift is called, and is not retrieved by
   * N_GetLift */
  char Mode;

  /* a bitmask of status flags */
  long Status;

  /* settable/retrievable parameters */
  long DesiredPosition;            /* Ignored for velocity moves. */
  long DesiredVelocity;
  long MaxMotor;                   /* Force limit. */
  long Acceleration;

  /* retrievable parameters */
  long Position;                   /* Current position. */
  long Velocity;                   /* Current velocity. */
};


struct N_LiftController
{
  /* TRUE if the lift is deployed */
  BOOL Deployed;

  /* TRUE if a deploy or zero operation is currently in progress. */
  BOOL InProgress;

  N_CONST unsigned int AxisCount;
  struct N_LiftAxis Axis[N_LIFT_AXIS_COUNT];
};




/* --- Joystick --- */

struct N_Joystick
{
  double X;
  double Y;
  double Theta;
  BOOL   ButtonA;
  BOOL   ButtonB;
  BOOL   ButtonC;
};



/* --- Sonars ---- */

struct N_Sonar
{
  long Reading;
  unsigned long TimeStamp;
};


struct N_SonarSet
{
  unsigned int FiringOrder[N_MAX_SONAR_COUNT + 1];
  long FiringDelay;
  long BlankingInterval;

  BOOL DataActive;
  BOOL TimeStampActive;
  N_CONST unsigned int SonarCount;
  struct N_Sonar Sonar[N_MAX_SONAR_COUNT];
};


struct N_SonarController
{
  N_CONST unsigned int SonarSetCount;
  struct N_SonarSet SonarSet[N_MAX_SONAR_SET_COUNT];
  BOOL SonarPaused;
};



/* --- Infrareds --- */

struct N_Infrared
{
  long Reading;
  unsigned long TimeStamp;
};


struct N_InfraredSet
{
  BOOL DataActive;
  BOOL TimeStampActive;
  N_CONST unsigned int InfraredCount;
  struct N_Infrared Infrared[N_MAX_INFRARED_COUNT];
};


struct N_InfraredController     
{
  BOOL InfraredPaused;
  N_CONST unsigned int InfraredSetCount;
  struct N_InfraredSet InfraredSet[N_MAX_INFRARED_SET_COUNT];
};



/* --- Bumpers --- */

struct N_Bumper
{
  char Reading;
  unsigned long TimeStamp;
};


struct N_BumperSet
{
  BOOL DataActive;
  BOOL TimeStampActive;
  N_CONST unsigned int BumperCount;
  struct N_Bumper Bumper[N_MAX_BUMPER_COUNT];
};


struct N_BumperController
{
  N_CONST unsigned int BumperSetCount;
  struct N_BumperSet BumperSet[N_MAX_BUMPER_SET_COUNT];
};



/* --- Compass modules --- */

struct N_Compass
{
  long Reading;
  unsigned long TimeStamp;
  BOOL DataActive;
  BOOL TimeStampActive;
};



/* --- Sensus 500 laser stripers --- */

struct N_Laser
{
  /* Generic configuration parameters */
  unsigned char Power;
  unsigned char DataType;
  unsigned char PointType;
  BOOL IntegrityChecking;       /* TRUE if checking for integrity */
  double Threshold;             /* Energy level above which a pixel is seen */
  unsigned short Width;         /* Minimum width for a peak to be considered */
  unsigned short SamplingCount; /* Number of sampling points */
  unsigned short MedianWindow;  /* Half size (in pixels) of the
				 * median-extracting window */

  /* Line mode configuration parameters */
  double Fitting;            /* Segment is broken if the error term in least
			      * square fitting goes above this value */
  unsigned short MinPoint;   /* Minimum number of points to make a segment */
  double Gap;                /* Max acceptable gap between two segments to
			      * be considered one */

  /* Data retrieval parameters */
  BOOL DataActive;
  BOOL TimeStampActive;
  unsigned long TimeStamp;

  /* Data returned by the laser */
  short DataCount;
  double Data[4*N_MAX_SCANLINE_COUNT];
};


struct N_LaserSet
{
  N_CONST unsigned int LaserCount;
  struct N_Laser Laser[N_MAX_LASER_COUNT];
};



/* --- Sensus 550 (SICK) laser rangefinders --- */

struct N_S550
{
  N_CONST unsigned int TotalPoints;
  unsigned int RequestedPoints;

  unsigned long Readings[N_MAX_S550_POINTS];
  unsigned char StatusFlags[N_MAX_S550_POINTS];
  unsigned char SummaryFlags;

  unsigned long TimeStamp;
  BOOL DataActive;
  BOOL TimeStampActive;
};


struct N_S550Set
{
  N_CONST unsigned int S550Count;
  struct N_S550 S550[N_MAX_S550_COUNT];
};



/* --- Batteries --- */

struct N_Battery
{
  long Voltage;
};


struct N_BatterySet
{
  N_CONST unsigned int BatteryCount;
  struct N_Battery Battery[N_MAX_BATTERY_COUNT];
  BOOL DataActive;
};



/* --- Timers --- */

struct N_Timer
{
  long Timeout;         /* Limp timeout value in msec. */
  unsigned long Time;   /* Current timestamp in msec. */
};



/* --- The main state structure -------------------------------------------- */


struct N_RobotState
{
  N_CONST long RobotID;
  N_CONST char RobotType;
  struct N_Integrator Integrator;
  struct N_AxisSet AxisSet;
  struct N_LiftController LiftController;
  struct N_Joystick Joystick;
  struct N_SonarController SonarController;
  struct N_InfraredController InfraredController;
  struct N_BumperController BumperController; 
  struct N_Compass Compass;
  struct N_LaserSet LaserSet;
  struct N_S550Set S550Set;
  struct N_BatterySet BatterySet; 
  struct N_Timer Timer;
};


typedef void (*N_ErrorFunc)(long RobotID);



/* --- User functions ------------------------------------------------------ */

#ifdef __cplusplus
extern "C"
{
#endif


/* Initialize the client library. */
int N_InitializeClient(const char *scheduler_hostname,
		       unsigned short scheduler_socket);

/* Set and retrieve the synchronisity of the library. */
void N_SetSynchronous(BOOL Value);
BOOL N_GetSynchronous(void);

/* Connect and disconnect from a robot. */
int N_ConnectRobot(long RobotID);
int N_DisconnectRobot(long RobotID);

/* Retrieve a pointer to a robot's state structure. */
struct N_RobotState *N_GetRobotState(long RobotID);

/* Move the robot and retrieve axis configurations. */
int N_SetAxes(long RobotID);
int N_GetAxes(long RobotID);

/* Move the lift and retrieve lift axis configurations. */
int N_SetLift(long RobotID);
int N_GetLift(long RobotID);
int N_DeployLift(long RobotID);
int N_RetractLift(long RobotID);
int N_ZeroLift(long RobotID, BOOL Force);

/* Move the robot as though it were being controlled by a joystick. */
int N_SetJoystick(long RobotID);

/* Set and retrieve the current axis limp timeout value. */
int N_SetTimer(long RobotID);
int N_GetTimer(long RobotID);

/* Configure sonars and retrieve the current configuration; retrieve the
 * latest sonar readings. */
int N_SetSonarConfiguration(long RobotID);
int N_GetSonarConfiguration(long RobotID);
int N_GetSonar(long RobotID);

/* Configure infrareds and retrieve the current configuration; retrieve the
 * latest infrared readings. */
int N_SetInfraredConfiguration(long RobotID);
int N_GetInfraredConfiguration(long RobotID);
int N_GetInfrared(long RobotID);

/* Retrieve the latest bumper readings. */
int N_GetBumper(long RobotID);

/* Configure the S500 laser and retrieve the current configuration;
 * retrieve the current laser readings. */
int N_SetLaserConfiguration(long RobotID);
int N_GetLaserConfiguration(long RobotID);
int N_GetLaser(long RobotID);

/* Retrieve the latest S550 (SICK) laser readings. */
int N_GetS550(long RobotID);

/* Retrieve the latest battery state. */
int N_GetBattery(long RobotID);

/* Retrieve the latest compass readings. */
int N_GetCompass(long RobotID);

/* Set and retrieve the current integrator values. */
int N_SetIntegratedConfiguration(long RobotID);
int N_GetIntegratedConfiguration(long RobotID);

/* Get all "DataActive" sensor readings at once. */
int N_GetState(long RobotID);

/* Send a text string to the speech system. */
int N_Speak(long RobotID, const char *Text);


#ifdef __cplusplus
}
#endif


#endif /* _N_INCLUDE_CLIENT_H_ */
