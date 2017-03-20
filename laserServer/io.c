/*
 * $Id: io.c,v 1.1 2002/09/14 16:33:00 rstone Exp $
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include "tcx.h"
#include "tcxP.h"
#include "global.h"
#include "devUtils.h"
#include "Common.h"
#include "mainlaser.h"
#include "handlers.h"
#include "bUtils.h"

#define DECLARE_LASER_VARS
#include "io.h"

#define SEBASTIAN_DEBUG_LEVEL 0

/*****************************************************************************
 * Global Constants
 *****************************************************************************/

#define LASER_DEVICE_TIMEOUT 10

#define MAX_NUMBER_OF_LASERS         2
#define NUMBER_OF_LASER_READINGS 180
#define FRONT_LASER_START_ANGLE   DEG_270
#define REAR_LASER_START_ANGLE   DEG_90

int NUMBER_OF_LASERS = 1;
int USE_FRONT_LASER = 1;
int USE_REAR_LASER = 0;

#define MAX_DEVICE_NAME 32
char FRONT_LASER_DEVICE[MAX_DEVICE_NAME];
char REAR_LASER_DEVICE[MAX_DEVICE_NAME];


/*****************************************************************************
 * Defines for the format of the protocol
 *****************************************************************************/

#define LASER_BUFFER_SIZE 8196

#define LASER_MESSAGE_LENGTH ((NUMBER_OF_LASER_READINGS) * 2 + 10)

#define INITIAL_ACKNOWLEDGE -1
#define ACKNOWLEDGED     0
#define NOT_ACKNOWLEDGED 1
#define TIME_OUT         2

/*****************************************************************************
 * Global variables 
 *****************************************************************************/

static int use_laser_server = 0;
static struct timeval last_ping_time[2] = {{0, 0}, {0, 0}};

static int communication_verbose = 0;
static int sensors_verbose = 0;

/* Is necessary to delete the static defined buffer after having sent a
 * command to the laser. The buffer is deleted in Laser_outputHnd. */
static BOOLEAN commandwritten[MAX_NUMBER_OF_LASERS];

LASER_reading frontLaserReading =
{0, 0, NULL, NULL, NULL, NULL, 0.0, 0.0, {0.0, 0.0}, 0.0};
LASER_reading rearLaserReading  =
{0, 0, NULL, NULL, NULL, NULL, 0.0, 0.0, {0.0, 0.0}, 0.0};

HandlerList laser_handlers;

static Point positionAtLastRequest[MAX_NUMBER_OF_LASERS];
static float rotationAtLastRequest[MAX_NUMBER_OF_LASERS];

/*****************************************************************************
 * Forward procedure declarations
 *****************************************************************************/

static void initLaserReadingStructures( char* frontDevice, char* rearDevice);

/* Same functions for both lasers. */
static unsigned short computeCRC( unsigned char* CommData, unsigned short uLen);
static void ProcessLaserStatus(   unsigned char *infos);

/* Same functions for both lasers. Need LASER_TYPE as parameter. */
static void 
ProcessLine(LASER_TYPE* laserDevice,
	    LASER_reading* scan, 
	    unsigned char *line, 
	    int length);
static void LASER_outputHnd( int fd, long chars_available,LASER_TYPE* laserDevice);
static BOOLEAN initAndStartLaserDevice( LASER_TYPE* laserDevice);
static int WriteLaserCommand( LASER_TYPE* laserDevice,
			      unsigned char command, char *argument, int arg_length);
static int waitforACK( LASER_TYPE* laserDevice);
static void requestLaserStatus( LASER_TYPE* laserDevice);
static int laserRequestReading( LASER_TYPE* laserDevice, int num);
static void adaptBaudRate( LASER_TYPE* laserDevice);
static void laser_9600_baud( LASER_TYPE* laserDevice);
static void laser_19200_baud( LASER_TYPE* laserDevice);
static void laser_38400_baud( LASER_TYPE* laserDevice);
static void closeLaserDevice( LASER_TYPE* laserDevice);
static void signalLaser(void);




/*****************************************************************************
 * Timer
 *****************************************************************************/

#define MAX_NUMBER_OF_TIMERS 20
#define EVERYBODIES_TIMER 9
static struct timeval startTime[MAX_NUMBER_OF_TIMERS];

float
timeDiff( struct timeval* t1, struct timeval* t2)
{
   float diff = 0.0;

   diff =  (float) (t1->tv_usec - t2->tv_usec) / 1000000.0;
   diff += (float) (t1->tv_sec - t2->tv_sec);

   return diff;
}

void
setStartTime( int i)
{
   if ( i < MAX_NUMBER_OF_TIMERS)
      gettimeofday( &(startTime[i]), 0);
   else
      fprintf( stderr, "setStartTime: index (%d) too high!\n", i);
}


float
timeExpired( int i)
{
   static struct timeval now;
   
   if ( i < MAX_NUMBER_OF_TIMERS) {
     gettimeofday( &now, 0);
     return timeDiff( &now, &(startTime[i]));
   }
   else {
      fprintf( stderr, "setStartTime: index (%d) too high!\n", i);
      return 0.0;
   }
}

void
printExpiredTime( int i, BOOLEAN longFormat)
{
  if (longFormat)
    fprintf( stderr, "%f seconds of interval %d expired.\n", timeExpired( i), i);
  else
    fprintf( stderr, "%f\n", timeExpired( i)); 
}

float 
normed_angle(float angle)
{
  while (angle < 0.0)
    (angle) += DEG_360;
  
  while (angle >= DEG_360)
    (angle) -= DEG_360;

  return(angle);
}



/*****************************************************************************
 *****************************************************************************
 * GLOBAL functions. 
 *****************************************************************************
 *****************************************************************************/

/*****************************************************************************
 *
 * FUNCTION: BOOLEAN start_laser()
 *
 * DESCRIPTION:
 *
 * This routine starts laser device on the robot.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/
BOOLEAN
start_laser(char* frontDevice, char* rearDevice)
{
  static BOOLEAN firstTime = TRUE;
  BOOLEAN success = TRUE;

#if TOTAL_debug
  fprintf(stdout,"\n--->start_laser\n");
  fflush( stdout );
#endif

  if ( firstTime) {
    
    /* Create the list for the handlers. */
    laser_handlers = CreateHandlerList(LASER_NUMBER_EVENTS);
    
    initLaserReadingStructures( frontDevice, rearDevice);

    /* Do the device stuff (only if the simulator is not used). */
    if ( USE_FRONT_LASER && ! frontLaserDevice.dev.use_simulator &&
	 ! use_laser_server) {
      if ( ! initAndStartLaserDevice( &frontLaserDevice)) {
	fprintf( stderr, "Cannot initialize front laser.\n");
	success = FALSE;
      }
    }

    if ( USE_REAR_LASER && ! rearLaserDevice.dev.use_simulator &&
	 ! use_laser_server) {
      if ( ! initAndStartLaserDevice( &rearLaserDevice)) {
	fprintf( stderr, "Cannot initialize rear laser.\n");
	success = FALSE;
      }
    }
  }

  firstTime = FALSE;
  return success;
}


void stop_laser(void)
{
#if TOTAL_debug
  fprintf(stdout,"\n--->stop_laser\n");
  fflush( stdout );
#endif

  if ( USE_FRONT_LASER)
    closeLaserDevice( &frontLaserDevice);
  if ( USE_REAR_LASER)
    closeLaserDevice( &rearLaserDevice);
}


/* Checks wether the laser returned some information */
void LASER_look_for_laser_device(void)
{
#if TOTAL_debug
  fprintf(stdout,"\n--->LASER_look_for_laser_device\n");
  fflush( stdout );
#endif

  if ( USE_FRONT_LASER && ! frontLaserDevice.dev.use_simulator &&
       ! use_laser_server)
    ProcessSingleDevice(&frontLaserDevice.dev);
  if ( USE_REAR_LASER && ! rearLaserDevice.dev.use_simulator &&
       ! use_laser_server)
    ProcessSingleDevice(&rearLaserDevice.dev);
}



/*****************************************************************************
 *
 * FUNCTION: Functions to deal with the handlers of certain laser events. 
 * see devUtils.c for explanation 
 *****************************************************************************/
void
LASER_InstallHandler( Handler handler, int event, Pointer client_data)
{
#if TOTAL_debug
  fprintf(stdout,"\n--->LASER_InstallHandler\n");
  fflush( stdout );
#endif

  InstallHandler( laser_handlers, handler, event, client_data);
}


void
LASER_RemoveHandler(Handler handler, int event)
{
#if TOTAL_debug
  fprintf(stdout,"\n--->LASER_RemoveHandler\n");
  fflush( stdout );
#endif

  RemoveHandler(laser_handlers, handler, event);
}


void
LASER_RemoveAllHandlers(int event)
{
#if TOTAL_debug
  fprintf(stdout,"\n--->LASER_RemoveAllHandlers\n");
  fflush( stdout );
#endif

  RemoveAllHandlers(laser_handlers, event);
}


/*****************************************************************************
 *****************************************************************************
 * LOCAL functions. 
 *****************************************************************************
 *****************************************************************************/

/*****************************************************************************
 *
 * FUNCTION: BOOLEAN initAndStartLaserDevice();
 *
 * DESCRIPTION: Open the laser device and initialize it.
 *
 * INPUTS: see devUtils.h for explanation
 *
 * OUTPUTS: Returns true if device opened successfully, false otherwise.
 *
 *****************************************************************************/
static BOOLEAN
initAndStartLaserDevice( LASER_TYPE* laserDevice)
{

  static void sendInitToLaser( LASER_TYPE* laserDevice);
  
#if TOTAL_debug
  fprintf(stdout,"\n--->initAndStartLaserDevice\n");
  fflush( stdout );
#endif

  /* Initialize the structure in devUtils.c */
  devInit();
  
  /* Open real device */
  connectTotty(&(laserDevice->dev));
  if( laserDevice->dev.fd == -1)
    return FALSE;

  connectDev(&(laserDevice->dev));
  laserDevice->dev.sigHnd = signalLaser;

  sendInitToLaser( laserDevice);
  
  return TRUE;
}


/*****************************************************************************
 *
 * FUNCTION: BOOLEAN closeLaserDevice(void);
 *
 * DESCRIPTION: Close the laser device.
 *
 * OUTPUTS: 
 *
 *****************************************************************************/
static void
closeLaserDevice( LASER_TYPE* laserDevice)
{
#if TOTAL_debug
  fprintf(stdout,"\n--->closeLaserDevice\n");
  fflush( stdout );
#endif

  close(laserDevice->dev.fd);
  laserDevice->dev.fd = -1;
}


#define CRC16_GEN_POL 0x8005
#define CRC16_GEN_POL0 0x80
#define CRC16_GEN_POL1 0x05

static
unsigned short
computeCRC(unsigned char* CommData, unsigned short uLen)
{
  unsigned char abData[2];
  unsigned char uCrc16[2];

#if TOTAL_debug
  fprintf(stdout,"\n--->computeCRC\n");
  fflush( stdout );
#endif

  
  abData[1] = 0;
  abData[0] = 0;
  uCrc16[0] = 0;
  uCrc16[1] = 0;

  while(uLen--) {
    abData[0] = abData[1];
    abData[1] = *CommData++;

    if(uCrc16[0] & 0x80) {
      uCrc16[0] <<= 1;
      if (uCrc16[1] & 0x80)
	uCrc16[0] |= 0x01;
      uCrc16[1] <<= 1;

      uCrc16[0] ^= CRC16_GEN_POL0;
      uCrc16[1] ^= CRC16_GEN_POL1;
    }
    else{
      uCrc16[0] <<= 1;
      if (uCrc16[1] & 0x80)
	uCrc16[0] |= 0x01;
      uCrc16[1] <<= 1;
    }
    uCrc16[0] ^= abData[0];
    uCrc16[1] ^= abData[1];
  }
  
  return (((unsigned short) uCrc16[0]) * 256
	  + ((unsigned short)  uCrc16[1]));
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void
LASER_timeoutHnd(void)
{
  ;
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define MAX_LASER_MESSAGE_LENGTH 400



static void
LASER_outputHnd( int fd, long chars_available, LASER_TYPE* laserDevice)
{
  static unsigned char buffer[MAX_NUMBER_OF_LASERS][LASER_BUFFER_SIZE+1];
  static unsigned char *startPos[MAX_NUMBER_OF_LASERS];
  static unsigned char *endPos[MAX_NUMBER_OF_LASERS];
  static unsigned char *startPos2=NULL;
  int numRead = 0;
  int length = 0;
  unsigned int check1 = 0, check2 = 0;
  static int error_free_message[MAX_NUMBER_OF_LASERS];
  static int firstPosition[MAX_NUMBER_OF_LASERS];
  int laserNumber = laserDevice->laserNumber;
  LASER_reading* scan = laserDevice->scan;
  struct timeval this_time[MAX_NUMBER_OF_LASERS];
  static struct timeval last_time[MAX_NUMBER_OF_LASERS];
  float  time_difference = 0.0; 

  
  
  /* We have to store the positions of the scans to interpolate the
   * next position. */
  Point actualPosition;
  float actualRot = 0.0;

  static BOOLEAN firstTime = TRUE;

#if TOTAL_debug
  fprintf(stdout,"\n--->LASER_outputHnd:%d\n",laserNumber);
  fflush( stdout );
#endif

  if ( firstTime) {
    int i = 0;
    
    setStartTime( FRONT_LASER);
    setStartTime( REAR_LASER);
    
    for ( i = 0; i != NUMBER_OF_LASERS; i++) {
      gettimeofday(&last_time[i], NULL);
      startPos[i]  = &(buffer[i][0]);
      endPos[i]  = &(buffer[i][0]);
      error_free_message[i] = FALSE;
      firstPosition[i] = TRUE;
      commandwritten[i] = FALSE;
    }
    firstTime = FALSE;
  }

  while (chars_available > 0) {
    if (SEBASTIAN_DEBUG_LEVEL >= 2)
      fprintf( stderr, "%d", laserNumber);
    setTimeout (&(laserDevice->dev), 0);

    gettimeofday(&(last_ping_time[laserNumber]), NULL); 
    
    if ((startPos[laserNumber] == endPos[laserNumber]) || commandwritten[laserNumber])
      { 
	if (communication_verbose)
	  fprintf(stderr,"Buffer cleared \n");
	startPos[laserNumber] = endPos[laserNumber] = buffer[laserNumber];
	bzero(buffer[laserNumber], LASER_BUFFER_SIZE+1); /* 8196 +1*/
	commandwritten[laserNumber] = FALSE;
      }
    
    
    /* read in the output. */
    if (communication_verbose)
      fprintf(stderr, "readN: %d (%d %d %d)\n",
	      (int) MIN( (int) chars_available, (int) (LASER_BUFFER_SIZE - 
				   (endPos[laserNumber] - startPos[laserNumber]))),
	      (int) chars_available, (int) LASER_BUFFER_SIZE, endPos[laserNumber] - startPos[laserNumber]);

    
    numRead = readN( &(laserDevice->dev), endPos[laserNumber], 
		    MIN(chars_available,
			(LASER_BUFFER_SIZE - 
			 (endPos[laserNumber] - startPos[laserNumber]))));

    if(communication_verbose)
      fprintf(stderr, "Read: %d\n",numRead);


    endPos[laserNumber] += numRead;

    if (numRead == 0)
      fprintf(stderr, "\n\t??? empty message ???\n");


    /*
     * Show buffer
     */
    
    length = 0;
    if (communication_verbose)
      fprintf(stderr, "\nBuffer: [ %d",endPos[laserNumber]-startPos[laserNumber]);
    for (startPos2 = startPos[laserNumber]; startPos2 < endPos[laserNumber]; startPos2++){
      if (*startPos2 == 0x02){
	if (communication_verbose)
	  fprintf(stderr, " (%d)\n", length);
	length = 0;
      }
      if (communication_verbose)
	if (length < 10)
	  fprintf(stderr, " %.2x", *startPos2);
      length++;
    }
    if (communication_verbose)
      fprintf(stderr, "]\n"); 


    /*
     * Remove irrelevant initial bytes
     */

    
    while ((*startPos[laserNumber] != 0x02 && 
	    startPos[laserNumber] + 1 < endPos[laserNumber])
	   || (endPos[laserNumber] - startPos[laserNumber] >= 6 &&
	       (((int) *(startPos[laserNumber]+3)) * 256) 
	       + ((int) *(startPos[laserNumber]+2)) + 6 >
	       MAX_LASER_MESSAGE_LENGTH)){
      if (communication_verbose)
	fprintf(stderr, " <<%.2x>>", *startPos[laserNumber]);
      startPos[laserNumber]++;
    }

    /*
     * Calculate length
     */
    
    if (endPos[laserNumber] - startPos[laserNumber] < 6)
      length = 0;
    else
      length = (((int) *(startPos[laserNumber]+3)) * 256) + ((int) *(startPos[laserNumber]+2)) + 6;


    /*
     * Check if we got a complete message here
     */
    
    while (endPos[laserNumber] - startPos[laserNumber] >= 6 && /* minimum length */
	   endPos[laserNumber] - startPos[laserNumber] >= length){ /* complete message */
      
      /*
       * Okay, we got a complete message, let's parse it
       */
      
      if (communication_verbose){
	fprintf(stderr, "\nmessage length= %d (%.2x)\n", length, length); 
	fprintf(stderr, "Parsing: [");
	for (startPos2 = startPos[laserNumber]; startPos2 < endPos[laserNumber]; startPos2++)
	  fprintf(stderr, " %.2x", *startPos2);
	fprintf(stderr, "]\n"); 
      }
      

      /*
       * Print the checksum
       */

      check1 = (*(startPos[laserNumber]+length-1) * 256) + *(startPos[laserNumber]+length-2);
      check2 = computeCRC((startPos[laserNumber]), (unsigned short) length-2);

      if (check1 != check2)
	fprintf(stderr, "Laser checksum error: %d %d (length=%d)\n",
		check1, check2, length);
      
      else {

	/* We interpolate between the position when the last reading came
	 * and the actual position. */
	/*updateActualPosition( &actualPosition, &actualRot,
	  DONT_CONSIDER_DIRECTION);*/
	
	/* Not too much time expired since the last reading. */
	if ( timeExpired( laserNumber) < 0.5 && ! firstPosition[laserNumber]) {

#define LAST_POS_WEIGHT 0.0
#define CURRENT_POS_WEIGHT 1.0
	    /* Do the interpolation. */
	  scan->rPos.x = CURRENT_POS_WEIGHT * actualPosition.x
	    + LAST_POS_WEIGHT * positionAtLastRequest[laserNumber].x;
	  scan->rPos.y = CURRENT_POS_WEIGHT * actualPosition.y
	    + LAST_POS_WEIGHT * positionAtLastRequest[laserNumber].y;
	  if ( fabs( actualRot - rotationAtLastRequest[laserNumber]) < DEG_180)
	    scan->rRot = (CURRENT_POS_WEIGHT * actualRot
			  + LAST_POS_WEIGHT * rotationAtLastRequest[laserNumber]);
	  else {
	    if ( actualRot < DEG_180) 
	      scan->rRot =
		normed_angle( CURRENT_POS_WEIGHT * (actualRot + DEG_360) +
			      LAST_POS_WEIGHT * rotationAtLastRequest[laserNumber]);
	    else
	      scan->rRot =
		normed_angle( CURRENT_POS_WEIGHT * actualRot +
			      LAST_POS_WEIGHT * (rotationAtLastRequest[laserNumber] + DEG_360));
	  }
	}
	else {
	  /* Position too old. Use the actual position. */
	  /*updateActualPosition( &(scan->rPos),
				&(scan->rRot),
				DONT_CONSIDER_DIRECTION);*/
	  if ( sensors_verbose)
	    fprintf( stderr, "too old : %f      ---> ",
		     timeExpired( laserNumber));

	  firstPosition[laserNumber] = FALSE;
	}
	  
	/* Set timer for the next reading. */
	setStartTime( laserNumber);


	if (SEBASTIAN_DEBUG_LEVEL >= 1){
	  gettimeofday(&this_time[laserNumber], NULL);
	  time_difference = 
	    ((float) (this_time[laserNumber].tv_sec - 
		      last_time[laserNumber].tv_sec))
	    + (((float) (this_time[laserNumber].tv_usec - 
			 last_time[laserNumber].tv_usec))
	       /  1000000.0);
	  last_time[laserNumber].tv_sec = this_time[laserNumber].tv_sec;
	  last_time[laserNumber].tv_usec = this_time[laserNumber].tv_usec;
	  if (time_difference > 0.3)
	    fprintf( stderr, "[%d:%g]", laserNumber, time_difference);
	  else
	    fprintf( stderr, "[%d", laserNumber);
	}

	/*
	if (laserNumber == 0){
	  printExpiredTime(FRONT_LASER, 0);
	  setStartTime( FRONT_LASER);
	}
	else{
	  printExpiredTime(REAR_LASER, 0);
	  setStartTime( REAR_LASER);
	}
	*/
	ProcessLine(laserDevice, scan, 
		    startPos[laserNumber]+4, length - 6);
      }


      /*
       * Ask for next sensor value
       */
      laserRequestReading( laserDevice, NUMBER_OF_LASER_READINGS);
	
      
      /*
       * Okay, message parsed, remove it
       */

      startPos[laserNumber] += length;

      
      /*
       * Show the remaining buffer
       */
      if (communication_verbose){
	fprintf(stderr, "\nRemaining buffer Parsing: [");
	for (startPos2 = startPos[laserNumber]; startPos2 < endPos[laserNumber]; startPos2++)
	  fprintf(stderr, " %.2x", *startPos2);
	fprintf(stderr, "]\n"); 
      }
      
    }
    
    /* 
     * Fix up the buffer. Throw out any consumed lines.
     */
    if (startPos[laserNumber] >= endPos[laserNumber]){	/* parsed the whole thing, 
				 * just clean it all up */
      bzero(buffer[laserNumber], LASER_BUFFER_SIZE+1);
      startPos[laserNumber] = endPos[laserNumber] = buffer[laserNumber];
    }
    else if (startPos[laserNumber] != buffer[laserNumber]){/* slide it back and wait for 
				  * more characters */
      bcopy(startPos[laserNumber], buffer[laserNumber], (endPos[laserNumber] - startPos[laserNumber]));
      endPos[laserNumber] = buffer[laserNumber] + (endPos[laserNumber] - startPos[laserNumber]);
      startPos[laserNumber] = buffer[laserNumber];
    }
    
    /*
     * Show the remaining buffer
     */
    if ( communication_verbose){
      fprintf(stderr, "\nRRemaining buffer: [");
      for (startPos2 = startPos[laserNumber]; startPos2 < endPos[laserNumber]; startPos2++)
	fprintf(stderr, " %.2x", *startPos2);
      fprintf(stderr, "]\n"); 
    }
    
    /*
     * Check if new characters arrived in the meantime
     */
    
    chars_available = numChars(fd);
    
  }
}


void
requestNextLaserScan( int laserNumber)
{
#if TOTAL_debug
  fprintf(stdout,"\n--->requestNextLaserScan\n");
  fflush( stdout );
#endif

  if ( laserNumber == FRONT_LASER)
    laserRequestReading( &frontLaserDevice, NUMBER_OF_LASER_READINGS);
  else
    laserRequestReading( &rearLaserDevice, NUMBER_OF_LASER_READINGS);
}
/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void LaserPing(LASER_TYPE* laserDevice) 
{
  struct timeval this_time;
  float  time_difference = 0.0; 
  int laser_no = laserDevice->laserNumber;

#if TOTAL_debug
  fprintf(stdout,"\n--->LaserPing:%d\n",laser_no);
  fflush( stdout );
#endif

  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_ping_time[laser_no].tv_sec))
    + (((float) (this_time.tv_usec - last_ping_time[laser_no].tv_usec))
       /  1000000.0);
  
  if (time_difference > 0.2){
    laserRequestReading( laserDevice, NUMBER_OF_LASER_READINGS);
    last_ping_time[laser_no].tv_sec = this_time.tv_sec;
    last_ping_time[laser_no].tv_usec = this_time.tv_usec;
  }
}
/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void
FRONT_LASER_outputHnd( int fd, long chars_available)
{
#if TOTAL_debug
  fprintf(stdout,"\n--->FRONT_LASER_outputHnd\n");
  fflush( stdout );
#endif

  LASER_outputHnd( fd, chars_available, &frontLaserDevice);
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void
REAR_LASER_outputHnd( int fd, long chars_available)
{
#if TOTAL_debug
  fprintf(stdout,"\n--->REAR_LASER_outputHnd\n");
  fflush( stdout );
#endif

  LASER_outputHnd( fd, chars_available, &rearLaserDevice);
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/*	Function Name: ProcessLine
 *	Arguments:     line -- output line from the laser
 *	Description:   process the laser output
 *	Returns: 
 */


static void 
ProcessLine(LASER_TYPE* laserDevice,
	    LASER_reading* scan, 
	    unsigned char *line, 
	    int length) 
{
  unsigned char type = 0 ;
  unsigned char *ptr = line;
  int body_length = 0, i = 0;
  int num_measurements = 0;
  struct timeval actual_time;
  float time_diff = 0.0;
  int laser_no = laserDevice->laserNumber;

  /* To check wether the scans of both finders are complete. */
  BOOLEAN bothComplete     = FALSE;
  static BOOLEAN frontRead;
  static BOOLEAN rearRead;
  static BOOLEAN firstTime = TRUE;
 
#if TOTAL_debug
  fprintf(stdout,"\n--->ProcessLine:%d\n",laser_no);
  fflush( stdout );
#endif
 
  if ( firstTime) {
    frontRead = ! USE_FRONT_LASER;
    rearRead  = ! USE_REAR_LASER;
    firstTime = FALSE;
  }
  
  type        = *ptr++;
  body_length = length -1;

  if (type == 0xb0){
    num_measurements = *(ptr+1) * 256 + *ptr;
    if (sensors_verbose){
      fprintf(stderr, "\n----------- SENSOR MEASUREMENT --------\n");
      fprintf(stderr, "  num_measurements      = %d (%.2x%.2x)\n",
	      num_measurements, *ptr, *(ptr+1));
    }
    
    ptr += 2;

    if (num_measurements != NUMBER_OF_LASER_READINGS)
      fprintf(stderr, "WARNING: Unexpected number of sensor values: %d\n", 
	      num_measurements);
    else{
      scan->new = TRUE;
      LaserSensors[laser_no].defined = 1;
      for (i = 0; i != num_measurements; i++){
	LaserSensors[laser_no].display_values[i] = 
	  LaserSensors[laser_no].values[i] = ((*(ptr+1)) & 0x1f) * 256 + *ptr;
	LaserSensors[laser_no].blendung[i] = ((int) ((*(ptr+1)) & 0x20)) / 32;
	LaserSensors[laser_no].wfv[i]      = ((int) ((*(ptr+1)) & 0x40)) / 64;
	LaserSensors[laser_no].sfv[i]      = ((int) ((*(ptr+1)) & 0x80)) / 128;
	scan->reading[i]    = ((*(ptr+1)) & 0x1f) * 256 + *ptr;
	scan->blendung[i] = ((int) ((*(ptr+1)) & 0x20)) / 32;
	scan->wfv[i]      = ((int) ((*(ptr+1)) & 0x40)) / 64;
	scan->sfv[i]      = ((int) ((*(ptr+1)) & 0x80)) / 128;

	if (sensors_verbose)
	  fprintf(stderr, "%d: %6.3f (blend=%d wfv=%d sfv=%d) (%.2x%.2x)\n",
		  i, scan->reading[i], scan->blendung[i], 
		  scan->wfv[i], scan->sfv[i], *ptr, *(ptr+1));
	if (scan->reading[i] > LASER_MAX_RANGE)
	  scan->reading[i] = LASER_MAX_RANGE;
	if ( LaserSensors[laser_no].display_values[i] > LASER_MAX_RANGE )
	  LaserSensors[laser_no].display_values[i] = LASER_MAX_RANGE;
	ptr += 2;
      }

      /*
       * Check timing
       */
      
      gettimeofday(&actual_time, NULL);
      if (scan->time.tv_sec != 0){
	time_diff = ((float) (actual_time.tv_sec - scan->time.tv_sec))
	  + (((float) (actual_time.tv_usec - scan->time.tv_usec)) 
	     / 1000000.0);
	if (sensors_verbose)
	  fprintf(stderr, "Time: %6.3f sec\n", time_diff);
      }
      scan->time.tv_sec  = actual_time.tv_sec;
      scan->time.tv_usec = actual_time.tv_usec;
      
      /* Check wether both range finders have been read. */
      if ( USE_FRONT_LASER && frontLaserReading.new)
	frontRead = TRUE;
      if ( USE_REAR_LASER && rearLaserReading.new)
	rearRead  = TRUE;
      
      if ( frontRead && rearRead) {
	bothComplete = TRUE;
	frontRead    = ! USE_FRONT_LASER;
	rearRead     = ! USE_REAR_LASER;
      }	
      
      /* Done. */ 
      FireHandler(laser_handlers, SINGLE_LASER_REPORT, (Pointer) NULL);
      if ( bothComplete) {
	FireHandler(laser_handlers, COMPLETE_LASER_REPORT, (Pointer) NULL);
      }
    }
  }

  else
    if(type == 0xb1)
      ProcessLaserStatus(ptr++);  /* STATUS-Meldung */
    else
      fprintf(stderr, "WARNING: Unknown message type: %.2x\n", type);
}





/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

static int 
WriteLaserCommand( LASER_TYPE* laserDevice,
		   unsigned char command, char *argument, int arg_length)
{
  unsigned char buffer[DEFAULT_LINE_LENGTH];
  int      pos = 0;
  int      i = 0, n = 0, answer = 0;
  unsigned short check = 0;
  unsigned short length = 0;

#if TOTAL_debug
  fprintf(stdout,"\n--->WriteLaserCommand\n");
  fflush( stdout );
#endif

  /*
   * SYNC CHARS
   */

  buffer[pos++] = 0x02;

  /*
   * ADDRESS
   */

  buffer[pos++] = 0x00;		/* broadcast */

  /*
   * MESSAGE LENGTH
   */

  length = 1 + arg_length;


  buffer[pos++] = length & 0x00ff;
  buffer[pos++] = length / 256;	/* I wonder if that works */

  /*
   * COMMAND
   */
  
  
  buffer[pos++] = command;

  
  /*
   * ARGUMENT
   */
  
  if (arg_length > 0)
    for (i = 0; i != arg_length; i++)
      buffer[pos++] = argument[i];
      
  
  /*
   * CHECKSUM
   */


  check = computeCRC(buffer, length + 4);
  buffer[pos++] = check & 0x00ff;
  buffer[pos++] = check / 256;
 
  if (communication_verbose){
    fprintf(stderr, "\nWriteLaserCommand (%d) [", arg_length);
    for (i = 0; i != pos; i++)
      fprintf(stderr, " %.2x", buffer[i]);  
    fprintf(stderr, "]\n");
  }
  
  /* set the timeout, if one is available */
  if (laserDevice->dev.setTimeout != NULL)
    (* laserDevice->dev.setTimeout)(&(laserDevice->dev), LASER_DEVICE_TIMEOUT);
  
  flushChars(&(laserDevice->dev));
  
  /*
   * WRITE TO PORT
   */
 
  n = ((int) writeN(&(laserDevice->dev), buffer, pos));

  answer = waitforACK( laserDevice);  /* Software-Handshake */

  if (sensors_verbose) {
    if (answer == NOT_ACKNOWLEDGED)
      fprintf(stderr, "NAK-Fehler beim Senden von: %.2x (%.2x)\n", command,*(buffer+5));
    else if (answer == TIME_OUT)
      fprintf(stderr, "Timeout-Fehler beim Senden von: %.2x (%.2x)\n", command,*(buffer+5));
  }
  
  commandwritten[laserDevice->laserNumber] = TRUE; /* internal buffer */

  return answer;
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

static void
sendInitToLaser( LASER_TYPE* laserDevice) 
{

#if TOTAL_debug
  fprintf(stdout,"\n--->sendInitToLaser\n");
  fflush( stdout );
#endif

  adaptBaudRate( laserDevice);

  laser_38400_baud( laserDevice); 
}


static void
adaptBaudRate( LASER_TYPE* laserDevice)
{
#if TOTAL_debug
  fprintf(stdout,"\n--->adaptBaudRate\n");
  fflush( stdout );
#endif

  m_setparms(laserDevice->dev.fd, "9600", PARITY, "8", 0, 0);
  if ( laserRequestReading( laserDevice, NUMBER_OF_LASER_READINGS)
       != TIME_OUT){
    fprintf(stderr, "\nLaser %d currently 9600 baud.\n",
	    laserDevice->laserNumber);
    return;
  }
  
  m_setparms(laserDevice->dev.fd, "19200", PARITY, "8", 0, 0);
  if ( laserRequestReading( laserDevice, NUMBER_OF_LASER_READINGS) 
       != TIME_OUT){
    fprintf(stderr, "\nLaser %d currently 19200 baud.\n",
	    laserDevice->laserNumber);
    return;
  }

  m_setparms(laserDevice->dev.fd, "38400", PARITY, "8", 0, 0);
  if ( laserRequestReading( laserDevice, NUMBER_OF_LASER_READINGS)
       != TIME_OUT){
    fprintf(stderr, "\nLaser %d currently 38400 baud.\n",
	    laserDevice->laserNumber);
    return;
  }
}


void
laser_38400_baud( LASER_TYPE* laserDevice)
{
  char arg[9];

#if TOTAL_debug
  fprintf(stdout,"\n--->laser_38400_baud\n");
  fflush( stdout );
#endif

#ifdef FGAN
  arg[0] = 0x00;
  arg[1] = 'S';
  arg[2] = 'I';
  arg[3] = 'C';
  arg[4] = 'K';
  arg[5] = '_';
  arg[6] = 'L';
  arg[7] = 'M';
  arg[8] = 'S';
#else
  arg[0] = 0x00;
  arg[1] = 'S';
  arg[2] = 'I';
  arg[3] = 'C';
  arg[4] = 'K';
  arg[5] = '_';
  arg[6] = 'P';
  arg[7] = 'L';
  arg[8] = 'S';
#endif

  WriteLaserCommand( laserDevice, 0x20, arg, 9);
  sleep(3);

  arg[0] = 0x40;
  WriteLaserCommand( laserDevice, 0x20, arg, 1);
  sleep(3);

  arg[0] = 0x25;
  WriteLaserCommand( laserDevice, 0x20, arg, 1);
  sleep(3);

  m_setparms(laserDevice->dev.fd, "38400", PARITY, "8", 0, 0);
  sleep(3);

  laserRequestReading( laserDevice, NUMBER_OF_LASER_READINGS);

  fprintf(stderr, "\nLaser %d currently 38400 baud.\n",laserDevice->laserNumber);
}


void
laser_19200_baud( LASER_TYPE* laserDevice)
{
  char arg[9];

#if TOTAL_debug
  fprintf(stdout,"\n--->laser_19200_baud\n");
  fflush( stdout );
#endif

#ifdef FGAN
  arg[0] = 0x00;
  arg[1] = 'S';
  arg[2] = 'I';
  arg[3] = 'C';
  arg[4] = 'K';
  arg[5] = '_';
  arg[6] = 'L';
  arg[7] = 'M';
  arg[8] = 'S';
#else
  arg[0] = 0x00;
  arg[1] = 'S';
  arg[2] = 'I';
  arg[3] = 'C';
  arg[4] = 'K';
  arg[5] = '_';
  arg[6] = 'P';
  arg[7] = 'L';
  arg[8] = 'S';
#endif

  WriteLaserCommand( laserDevice, 0x20, arg, 9);
  sleep(3);

  arg[0] = 0x41;
  WriteLaserCommand( laserDevice, 0x20, arg, 1);
  sleep(3);

  arg[0] = 0x25;
  WriteLaserCommand( laserDevice, 0x20, arg, 1);
  sleep(3);

  m_setparms(laserDevice->dev.fd, "19200", PARITY, "8", 0, 0);
  sleep(3);

  laserRequestReading( laserDevice, NUMBER_OF_LASER_READINGS);
  fprintf(stderr, "\nLaser %d currently 19200 baud.\n",laserDevice->laserNumber);
}

void
laser_9600_baud( LASER_TYPE* laserDevice)
{
  char arg[9];

#if TOTAL_debug
  fprintf(stdout,"\n--->laser_9600_baud\n");
  fflush( stdout );
#endif

#ifdef FGAN
  arg[0] = 0x00;
  arg[1] = 'S';
  arg[2] = 'I';
  arg[3] = 'C';
  arg[4] = 'K';
  arg[5] = '_';
  arg[6] = 'L';
  arg[7] = 'M';
  arg[8] = 'S';
#else
  arg[0] = 0x00;
  arg[1] = 'S';
  arg[2] = 'I';
  arg[3] = 'C';
  arg[4] = 'K';
  arg[5] = '_';
  arg[6] = 'P';
  arg[7] = 'L';
  arg[8] = 'S';
#endif

  WriteLaserCommand( laserDevice, 0x20, arg, 9);
  sleep(3);

  arg[0] = 0x42;

  WriteLaserCommand( laserDevice, 0x20, arg, 1);
  sleep(3);

  arg[0] = 0x25;
  WriteLaserCommand( laserDevice, 0x20, arg, 1);
  sleep(3);

  m_setparms(laserDevice->dev.fd, "9600", PARITY, "8", 0, 0);
  sleep(3);

  laserRequestReading( laserDevice, NUMBER_OF_LASER_READINGS);
  fprintf(stderr, "\nLaser %d currently 9600 baud.\n",laserDevice->laserNumber);
}


static int
laserRequestReading( LASER_TYPE* laserDevice, int num)
{
  unsigned char sens_text[128];
  int laser_no = laserDevice->laserNumber;

#if TOTAL_debug
  fprintf(stdout,"\n--->laserRequestReading:%d\n",laser_no);
  fflush( stdout );
#endif
 
  /* Save the corresponding position of the robot. */
  /*updateActualPosition( &(positionAtLastRequest[laserDevice->laserNumber]),
			&(rotationAtLastRequest[laserDevice->laserNumber]),
			DONT_CONSIDER_DIRECTION);*/
  
  if (SEBASTIAN_DEBUG_LEVEL >= 2)
    fprintf(stderr, " ?%d ", laser_no);
  sens_text[0] = 0x05;
  sens_text[1] = (unsigned char) num;
  return WriteLaserCommand( laserDevice, 0x30, sens_text, 2);
}


/* Fragt den Laser-Status ab und wertet ihn via LASER_outputHnd aus */
 
static void
requestLaserStatus(LASER_TYPE* laserDevice)
{
  char arg[1];
 
#if TOTAL_debug
  fprintf(stdout,"\n--->requestLaserStatus\n");
  fflush( stdout );
#endif
 
  arg[0] = 0x10;

  WriteLaserCommand( laserDevice, 0x20, arg ,1);
  sleep(3);
  WriteLaserCommand( laserDevice, 0x31, NULL ,0);
  sleep(3);
  LASER_outputHnd( laserDevice->dev.fd,130, laserDevice);
  arg[0] = 0x25;
  WriteLaserCommand( laserDevice, 0x20, arg ,1);
  sleep(3);
  laserRequestReading( laserDevice, NUMBER_OF_LASER_READINGS);
}

/* Wertet die Antwort des Lasers auf request_laser_status aus */

static void
ProcessLaserStatus(unsigned char *infos)
{
  unsigned short baud = 0;
  unsigned char text[7];
  
#if TOTAL_debug
  fprintf(stdout,"\n--->ProcessLaserStatus\n");
  fflush( stdout );
#endif

  text[0]=*infos++;
  text[1]=*infos++;
  text[2]=*infos++;
  text[3]=*infos++;
  text[4]=*infos++;
  text[5]=*infos++;
  text[6]=*infos++;
  fprintf(stdout,"Version: %s\n",text);
  

  baud = *(infos+108)+(*(infos+109))*256;

  switch(baud)
    {
    case 0x8019 : fprintf(stdout,"38400 Baud\n"); break; 
    case 0x8033 : fprintf(stdout,"19200 Baud\n"); break;
    case 0x8067 : fprintf(stdout,"9600 Baud\n"); break;
    default : fprintf(stdout,"Baudrate nicht korrekt ermittelbar: %.2x\n",baud); 
    }
}

/* 
Waits for acknowledge from the laser.
Laser answer if correct: ACK (006H) else NAK (015H).
Returns 0=ACK, or 1=NAK, or 2=Timeout (>1 sec).
*/

static int
waitforACK( LASER_TYPE* laserDevice)
{
  unsigned char answer; 
  short r_wert = INITIAL_ACKNOWLEDGE;

#if TOTAL_debug
  fprintf(stdout,"\n--->waitforACK\n");
  fflush( stdout );
#endif

  setStartTime( EVERYBODIES_TIMER);

  do {

    if (numChars(laserDevice->dev.fd) > 0) {
      readN(&(laserDevice->dev), &answer, 1);
      if (answer == 0x06)
	r_wert = ACKNOWLEDGED;
      else
	if (answer == 0x15)
	  r_wert = NOT_ACKNOWLEDGED;
    }
    else if ( timeExpired( EVERYBODIES_TIMER) > 0.1) 
      r_wert = TIME_OUT;

    usleep(5); 
    
  } while (r_wert < 0);

  return(r_wert);
}


/*****************************************************************************
 *
 * FUNCTION: initLaserReadingStructures
 *
 *****************************************************************************/
static void
initLaserReadingStructures( char* frontDevice, char* rearDevice)
{
  int i = 0;
  struct timeval now;


#if TOTAL_debug
  fprintf(stdout,"\n--->initLaserReadingStructures\n");
  fflush( stdout );
#endif

  gettimeofday( &now, 0);

  /* Initialize the global structs for the laser readings. */

  /* First consider the type of robot. */

#if (RHINO)
  if ((robotType == B18_ROBOT) || (robotType == B21_ROBOT))
    fprintf(stderr, "Use settings for robot with two laser range finders. 1\n");
  NUMBER_OF_LASERS  = 2;
  USE_FRONT_LASER   = TRUE;
  USE_REAR_LASER    = TRUE;
#else
  fprintf(stderr, "Use settings for robot with one laser range finder. 2\n");
  NUMBER_OF_LASERS  = 1;
  USE_FRONT_LASER   = TRUE;
  USE_REAR_LASER    = FALSE;
#endif

  
  if ((robotType == B18_ROBOT) || (robotType == B21_ROBOT)) {
    if ( frontDevice == NULL) {
      frontDevice =  strdup(bRobot.laser_front_dev);
    }
    strcpy( FRONT_LASER_DEVICE, frontDevice);
    if (NUMBER_OF_LASERS > 1) {
      if ( rearDevice == NULL)
	rearDevice = strdup( bRobot.laser_rear_dev);
      strcpy( REAR_LASER_DEVICE, rearDevice);
    }
  }
  else {
    if ( frontDevice == NULL) 
      frontDevice = strdup( bRobot.pio_laser_dev);
    strcpy( FRONT_LASER_DEVICE, frontDevice);
    USE_REAR_LASER    = FALSE;      
    NUMBER_OF_LASERS  = 1;
    fprintf(stderr, "Use settings for robot with one laser range finder at device %s.\n",
	    FRONT_LASER_DEVICE);
  }

  /* FRONT LASER */
  frontLaserReading.new = FALSE;
  frontLaserDevice.dev.ttydev.ttyPort = FRONT_LASER_DEVICE;

  if ( USE_FRONT_LASER && ! frontLaserDevice.dev.use_simulator &&
       ! use_laser_server) {
    frontLaserReading.numberOfReadings = NUMBER_OF_LASER_READINGS;
    frontLaserReading.startAngle       = FRONT_LASER_START_ANGLE;
    frontLaserReading.angleResolution  =
      DEG_180 / (float) (NUMBER_OF_LASER_READINGS - 1);
    frontLaserReading.time             = now;

    /* Allocate memroy. */
    if ( frontLaserReading.reading == NULL)
      frontLaserReading.reading          = (float*)
      calloc( NUMBER_OF_LASER_READINGS , (sizeof (float)));
    if ( frontLaserReading.blendung == NULL)
      frontLaserReading.blendung          = (int*)
      calloc( NUMBER_OF_LASER_READINGS , (sizeof (int)));
    if ( frontLaserReading.wfv == NULL)
      frontLaserReading.wfv          = (int*)
      calloc( NUMBER_OF_LASER_READINGS , (sizeof (int)));
    if ( frontLaserReading.sfv == NULL)
      frontLaserReading.sfv          = (int*)
      calloc( NUMBER_OF_LASER_READINGS , (sizeof (int)));
  }
  
  /* REAR LASER */
  rearLaserReading.new = FALSE;
  rearLaserDevice.dev.ttydev.ttyPort = REAR_LASER_DEVICE;
  
  if ( USE_REAR_LASER && ! rearLaserDevice.dev.use_simulator &&
       ! use_laser_server) {
    rearLaserReading.numberOfReadings = NUMBER_OF_LASER_READINGS;
    rearLaserReading.startAngle       = REAR_LASER_START_ANGLE;
    rearLaserReading.angleResolution  = DEG_180
      / (float) (NUMBER_OF_LASER_READINGS - 1);
    rearLaserReading.time             = now;  
    
    /* Allocate memory. */
    if ( rearLaserReading.reading == NULL)
      rearLaserReading.reading          = (float*)
      calloc( NUMBER_OF_LASER_READINGS , (sizeof (float)));
    if ( rearLaserReading.blendung == NULL)
      rearLaserReading.blendung          = (int*)
      calloc( NUMBER_OF_LASER_READINGS , (sizeof (int)));
    if ( rearLaserReading.wfv == NULL)
      rearLaserReading.wfv          = (int*)
      calloc( NUMBER_OF_LASER_READINGS , (sizeof (int)));
    if ( rearLaserReading.sfv == NULL)
      rearLaserReading.sfv          = (int*)
      calloc( NUMBER_OF_LASER_READINGS , (sizeof (int)));

    /* Now set the values to undefined. */
    for (i=0; i != frontLaserReading.numberOfReadings; i++)
      frontLaserReading.reading[i] = -1.0;

    for (i=0; i != rearLaserReading.numberOfReadings; i++)
      rearLaserReading.reading[i] = -1.0;
  }
}

/*****************************************************************************
 *
 * FUNCTION: void signalLaser(void);
 *
 * DESCRIPTION:
 *
 * Interrupt handler for the laser.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/

static void
signalLaser(void)
{
#if TOTAL_debug
  fprintf(stdout,"\n--->signalLaser\n");
  fflush( stdout );
#endif

  fprintf(stderr,"shutting down the laser\n");
}






/*
 * $Log: io.c,v $
 * Revision 1.1  2002/09/14 16:33:00  rstone
 * *** empty log message ***
 *
 * Revision 1.20  1999/12/17 12:09:24  fox
 * Removed a comment.
 *
 * Revision 1.19  1999/12/17 10:46:44  fox
 * Fixed a minor bug.
 *
 * Revision 1.18  1999/09/24 14:35:33  fox
 * Added support for scout.
 *
 * Revision 1.17  1999/07/27 13:07:11  schneid1
 * fixed minor bug
 *
 * Revision 1.16  1999/07/20 14:25:42  schneid1
 * laserServer now uses beeSoft.ini & -display works again
 *
 * Revision 1.15  1999/06/25 19:49:56  fox
 * Added urban robot.
 *
 * Revision 1.14  1999/04/19 09:19:35  rhino
 * Fixed conflicts with dieter's last commit.
 *
 * Revision 1.13  1999/04/18 19:00:08  fox
 * This is just to get the most recent version into the cmu bee tree. It shouldn't do any harm (hopefully!).
 *
 * Revision 1.12  1999/01/15 21:40:25  nickr
 * Dieter was too lazy to fix this bug.
 *
 * Revision 1.11  1998/11/08 17:28:01  fox
 * changed a sleep so that baud rate is successfully set.
 *
 * Revision 1.10  1998/11/04 00:02:58  fox
 * Added usleep in waitForAck.
 *
 * Revision 1.9  1998/10/30 18:58:03  fox
 * Added support for pioneers and multiple robots.
 *
 * Revision 1.8  1998/08/16 19:02:22  thrun
 * Will this run at 38400 baud?
 * THere is still stuff missing. Right now, it basically
 * ignores beeSoft.ini and runs with both lasers always.
 *
 * Revision 1.7  1998/08/14 16:30:45  thrun
 * .
 *
 * Revision 1.6  1998/08/08 21:26:10  thrun
 * Handles now both lasers correctly.
 *
 * Revision 1.5  1998/08/06 03:22:55  thrun
 * supports 2 lasers.
 *
 * Revision 1.4  1997/09/19 00:59:22  swa
 * The laserServer now honours the entries in beeSoft.ini, that is device and
 * baudrate. Internally however, it only works with 9600 baud. It does some
 * moderate error checking on the entries.
 *
 * Revision 1.3  1997/08/07 03:50:20  swa
 * Fixed a bunch of bugs. Still not working.
 *
 * Revision 1.2  1997/08/07 02:45:50  swa
 * First version of a laserServer. Compiles nicely, yet produces nothing useful. ;)
 * Just to copy things to a safe place ...
 *
 * Revision 1.1  1997/08/06 15:12:32  swa
 * Very first and incomplete version of a laserServer. No TCX comm yet.
 *
 *
 */
