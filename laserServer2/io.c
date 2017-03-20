/*
 * $Id: io.c,v 1.1 2002/09/14 16:33:04 rstone Exp $
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>

#include "bUtils.h"
#include "librobot.h"

#include "tcx.h"
#include "tcxP.h"
#include "global.h"
#include "mainlaser.h"

#define DEF_FRONT_DEVICE "/dev/ttyS0"

#define DECLARE_LASER_VARS
#include "io.h"

/*****************************************************************************
 * Global Constants
 *****************************************************************************/

#define LASER_DEVICE_TIMEOUT 10

#define MAX_NUMBER_OF_LASERS         2
#define NUMBER_OF_LASER_READINGS     180
#define FRONT_LASER_START_ANGLE      DEG_270
#define REAR_LASER_START_ANGLE       DEG_90

int NUMBER_OF_LASERS=2;
int USE_FRONT_LASER=1;
int USE_REAR_LASER=1;

extern int ARG_USE_ALL_FRONT_VALUES;
extern int ARG_USE_ALL_REAR_VALUES;

#define MAX_DEVICE_NAME 32
char FRONT_LASER_DEVICE[MAX_DEVICE_NAME];
char REAR_LASER_DEVICE[MAX_DEVICE_NAME];

char password[8];

LaserSensorValueType LaserSensors[2];
LaserDeviceType LaserDevice[2];

/*****************************************************************************
 * Defines for the format of the protocol
 *****************************************************************************/

#define LASER_BUFFER_SIZE 8196

#define LASER_MESSAGE_LENGTH ((NUMBER_OF_LASER_READINGS) * 2 + 10)

#define INI -1
#define TIO  0
#define ACK  6
#define NAK  15
#define DLE  10
#define STX  2
#define UKN  5
#define INIT_ACK_TGM 7
#define PWON_TGM     8
#define NACK_TGM     9
#define BMACK_TGM    11
#define RES_ACK      12
#define NOT_ACK      13

char *bytenames[]={"STX ","ADR ", "LENL", "LENH", "CMD ", "DATA", "CRCL", "CRCH" };

char *idlechar = "|/-\\";

/* Is necessary to delete the static defined buffer after having sent a
 * command to the laser. The buffer is deleted in Laser_outputHnd. */

LASER_reading frontLaserReading =
{0, 0, NULL, NULL, NULL, NULL, 0.0, 0.0, {0.0, 0.0}, 0.0, {0, 0}};
LASER_reading rearLaserReading  =
{0, 0, NULL, NULL, NULL, NULL, 0.0, 0.0, {0.0, 0.0}, 0.0, {0, 0}};

/*****************************************************************************
 * Forward procedure declarations
 *****************************************************************************/

int  computeCRC( unsigned char* CommData, int uLen);
void ProcessLine( int LaserNo, unsigned char *line, int length,
		  int laserType, struct timeval reqtime );

void initLaserDeviceNames( char* frontDevice, char* rearDevice );
void initLaserReadings( char* frontDevice, char* rearDevice );
int initAndStartLaserDevice( LASER_TYPE* laserDevice);
int WriteLaserCommand( LASER_TYPE* laserDevice, 
		       unsigned char command,
		       char *argument, int arg_length );

int waitForAnswer( LASER_TYPE* laserDevice, unsigned char *buffer, 
		   int TimeOut );
int adaptBaudRate( LASER_TYPE* laserDevice);
void laser_38400_baud( LASER_TYPE* laserDevice, int brate, int laserType );
void closeLaserDevice( LASER_TYPE* laserDevice);
void signalLaser(void);
int ClearInputBuffer( LASER_TYPE* laserDevice );
void sendInitToLaser( LASER_TYPE* laserDevice);
long numChars(int sd);
int writeN(DEV_PTR dev, char *buf, int nChars);
int connectPLS_TTY( DEV_PTR dev);


#ifndef TIOCGETP 

#define TIOCGETP        0x5481
#define TIOCSETP        0x5482
#define RAW             1
#define CBREAK          64

struct sgttyb
{
    unsigned short sg_flags;
    char sg_ispeed;
    char sg_ospeed;
    char sg_erase;
    char sg_kill;
    struct termios t;
    int check;
};

#endif

/*****************************************************************************
 * Timer
 *****************************************************************************/

#define MAX_NUMBER_OF_TIMERS 20
#define EVERYBODIES_TIMER 9

struct timeval startTime[MAX_NUMBER_OF_TIMERS];

float
timeDiff( struct timeval* t1, struct timeval* t2)
{
   float diff=0.0;
   diff =  (float) (t1->tv_usec - t2->tv_usec) / 1000000.0;
   diff += (float) (t1->tv_sec - t2->tv_sec);
   return diff;
}

struct timeval
timeStamp( struct timeval req, 
	   struct timeval recieve )
{
  struct timeval timest;
  double d_req, d_rec, d_tst, dummy;
  d_req =  (double) req.tv_sec + (double) (req.tv_usec/ 1000000.0);
  d_rec =  (double) recieve.tv_sec + (double) (recieve.tv_usec/ 1000000.0);
  d_tst = d_req + ((d_rec-d_req)/2.0);
  timest.tv_sec = (long) fabs(d_tst);
  timest.tv_usec = (long) (modf(d_tst,&dummy)*1000000.0);
  return(timest);
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


long 
numChars(int sd)
{
  long available=0;
  
  if (ioctl(sd, FIONREAD, &available) == 0)
    return available;
  else
    return -1;
}    

int
writeN(DEV_PTR dev, char *buf, int nChars)
{
  int amountWritten = 0;
  while (nChars > 0) {
    amountWritten = write(dev->fd, buf, nChars);
    if (amountWritten < 0) {
      if (errno == EWOULDBLOCK) {
	fprintf(stderr, "\nWARNING: writeN: EWOULDBLOCK: trying again!\n");
      } else {
	return FALSE;
      }
    }
    else {
      nChars -= amountWritten;
      buf += amountWritten;
    }
  }

  return TRUE;
}

void 
m_setrts( int fd )
{
#if defined(TIOCM_RTS) && defined(TIOCMODG)
  int mcs=0;

  ioctl(fd, TIOCMODG, &mcs);
  mcs |= TIOCM_RTS;
  ioctl(fd, TIOCMODS, &mcs);
#endif
#ifdef _COHERENT
  ioctl(fd, TIOCSRTS, 0);
#endif
}

#define _POSIX

void 
m_setparms( int fd, char *baudr, char *par, char *bits, int hwf, int swf )
{
  int spd = -1;
  int newbaud=0;
  int bit = bits[0];

#ifdef _POSIX
  struct termios tty;
  tcgetattr(fd, &tty);
#else
  struct sgttyb tty;
  ioctl(fd, TIOCGETP, &tty);
#endif

  /* We generate mark and space parity ourself. */
  if (bit == '7' && (par[0] == 'M' || par[0] == 'S'))
    bit = '8';
  /* Check if 'baudr' is really a number */
  if ((newbaud = (atol(baudr) / 100)) == 0 && baudr[0] != '0')
    newbaud = -1;

  switch(newbaud) {
  case 0:
#ifdef B0
		spd = B0;	break;
#else
		spd = 0;	break;
#endif
  case 3:	spd = B300;	break;
  case 6:	spd = B600;	break;
  case 12:	spd = B1200;	break;
  case 24:	spd = B2400;	break;
  case 48:	spd = B4800;	break;
  case 96:	spd = B9600;	break;
#ifdef B19200
  case 192:	spd = B19200;	break;
#else
#ifdef EXTA
  case 192:	spd = EXTA;	break;
#else
  case 192:	spd = B9600;	break;
#endif	
#endif	
#ifdef B38400
  case 384:	spd = B38400;	break;
#else
#ifdef EXTB
  case 384:	spd = EXTB;	break;
#else
  case 384:	spd = B9600;	break;
#endif
#endif	
#ifdef B57600
  case 576:	spd = B57600;	break;
#endif
#ifdef B115200
  case 1152:	spd = B115200;	break;
#endif
  }
  
#if defined (_BSD43) && !defined(_POSIX)
  if (spd != -1) tty.sg_ispeed = tty.sg_ospeed = spd;
  /* Number of bits is ignored */
  tty.sg_flags = RAW | TANDEM;
  if (par[0] == 'E')
	tty.sg_flags |= EVENP;
  else if (par[0] == 'O')
	tty.sg_flags |= ODDP;
  else
  	tty.sg_flags |= PASS8 | ANYP;
  ioctl(fd, TIOCSETP, &tty);
#ifdef TIOCSDTR
  /* FIXME: huh? - MvS */
  ioctl(fd, TIOCSDTR, 0);
#endif
#endif

#if defined (_V7) && !defined(_POSIX)
  if (spd != -1) tty.sg_ispeed = tty.sg_ospeed = spd;
  tty.sg_flags = RAW;
  if (par[0] == 'E')
	tty.sg_flags |= EVENP;
  else if (par[0] == 'O')
	tty.sg_flags |= ODDP;
  ioctl(fd, TIOCSETP, &tty);
#endif

#ifdef _POSIX
  if (spd != -1) {
	cfsetospeed(&tty, (speed_t)spd);
	cfsetispeed(&tty, (speed_t)spd);
  }
  switch (bit) {
  case '5':
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS5;
    break;
  case '6':
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS6;
    break;
  case '7':
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS7;
    break;
  case '8':
  default:
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    break;
  }		
  /* Set into raw, no echo mode */
#if !defined(_DGUX_SOURCE)
  tty.c_iflag &= ~(IGNBRK | IGNCR | INLCR | ICRNL | IUCLC | 
  	IXANY | IXON | IXOFF | INPCK | ISTRIP);
  tty.c_iflag |= (BRKINT | IGNPAR);
  tty.c_oflag &= ~OPOST;
  tty.c_lflag = ~(ICANON | ISIG | ECHO | ECHONL | ECHOE | ECHOK | IEXTEN);
  tty.c_cflag |= CREAD | CRTSCTS;
#else /* Okay, this is better. XXX - Fix the above. */
  tty.c_iflag =  IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cflag |= CLOCAL | CREAD;
#endif
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 5;

  /* Flow control. */
  if (hwf) {
    tty.c_cflag |= CRTSCTS;
    tty.c_cflag &= ~CLOCAL;
  }
  else {
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CLOCAL;
  }

  if (swf) {
    tty.c_iflag |= (IXON | IXOFF);
  }
  else {
    tty.c_iflag &= ~(IXON | IXOFF);
  }

  tty.c_cflag &= ~(PARENB | PARODD);

  if (par[0] == 'E')
	tty.c_cflag |= PARENB;
  else if (par[0] == 'O')
	tty.c_cflag |= PARODD;

  tcsetattr(fd, TCSANOW, &tty);

  m_setrts(fd);
#  ifdef _DGUX_SOURCE
  m_sethwf(fd, hwf);
#  endif
#endif
}

/*****************************************************************************
 *****************************************************************************
 *****************************************************************************
 *****************************************************************************/

int
startLaser(char* frontDevice, char* rearDevice)
{
  BOOLEAN success = TRUE;

  if ( USE_FRONT_LASER ) {
    if ( frontDevice == NULL ) {
      fprintf(stderr, "Set front laser to default device %s\n", 
	      DEFAULT_F_DEVICE );
      frontDevice = DEFAULT_F_DEVICE;
    }
  }
  if ( USE_REAR_LASER ) {
    if ( rearDevice == NULL ) {
      fprintf(stderr, "Set rear laser to default device %s\n",
	      DEFAULT_R_DEVICE );
      rearDevice = DEFAULT_R_DEVICE;
    }
  }

  initLaserDeviceNames( frontDevice, rearDevice );
  initLaserReadings( frontDevice, rearDevice );

  if ( USE_FRONT_LASER ) {
    fprintf( stderr, "---------------------------------------------------\n" );
    fprintf( stderr, " start initializing FRONT laser (%s)\n", frontDevice );
    fprintf( stderr, "---------------------------------------------------\n" );
    if ( ! initAndStartLaserDevice( &frontLaserDevice)) {
      fprintf( stderr, "Cannot initialize front laser.\n");
      success = FALSE;
    }
  }

  if ( USE_REAR_LASER ) {
    fprintf( stderr, "---------------------------------------------------\n" );
    fprintf( stderr, " start initializing REAR laser (%s)\n", rearDevice );
    fprintf( stderr, "---------------------------------------------------\n" );
    if ( ! initAndStartLaserDevice( &rearLaserDevice)) {
      fprintf( stderr, "Cannot initialize rear laser.\n");
      success = FALSE;
    }
  }
  return success;
}

void 
stop_laser(void)
{
  if ( USE_FRONT_LASER)
    closeLaserDevice( &frontLaserDevice);
  if ( USE_REAR_LASER)
    closeLaserDevice( &rearLaserDevice);
}


int
connectTTY(DEV_PTR dev)
{
  int BAUDRATE = B9600;
  static struct termios newtio;
  /* 0x8000 ist SYNC */  
  if ( (dev->fd = open( (dev->ttydev.ttyPort), (O_RDWR | 0x8000 | O_NOCTTY),0)) < 0 ) {
    fprintf(stderr,"Serial I/O Error:  Could not open port %s",dev->ttydev.ttyPort);
    perror(dev->ttydev.ttyPort);
    return (dev->fd);
  }
  
  tcgetattr((dev->fd),&newtio);
  
  cfsetispeed (&newtio, BAUDRATE);
  
  cfsetospeed (&newtio, BAUDRATE);
  
  newtio.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  newtio.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON | IXOFF);
  newtio.c_cflag &= ~(CSIZE | PARENB | PARODD);
  newtio.c_cflag |= (CS8);       
  newtio.c_oflag &= ~(OPOST);
  
  newtio.c_cc[VTIME] = 1;      
  newtio.c_cc[VMIN] = 0;       
  
  tcflush((dev->fd), TCIFLUSH);
  
  tcsetattr((dev->fd), TCSANOW, &newtio);
  
 return (dev->fd);
}


int
connectTTY_Dirk(DEV_PTR dev)
{
  struct termios term_info;
  
  /*.. Open the port for both ReaDing and WRiting ..*/
  /* ???? */
  if ((dev->fd = open(dev->ttydev.ttyPort, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0) {
    fprintf(stderr,"could not open port %s\n", dev->ttydev.ttyPort); 
    exit(-1);
  };

  /* Make the file descriptor asynchronous (the manual page says only
     O_APPEND and O_NONBLOCK, will work with F_SETFL...) */
  fcntl(dev->fd, F_SETFL, FASYNC);

  cfmakeraw(&term_info);
  if(tcsetattr(dev->fd, TCSANOW, &term_info)<0) {
    /* complain - fd is not a terminal */
    fprintf(stderr, "ERROR: Can't set control bits.\n" );
    exit(-1);
  }

  return(dev->fd);
}

int
initAndStartLaserDevice( LASER_TYPE* laserDevice)
{
  /* Open real device */
  fprintf( stderr, "connect TTY %s ... ", laserDevice->dev.ttydev.ttyPort );
  connectTTY(&(laserDevice->dev));
  if( laserDevice->dev.fd == -1) {
    fprintf( stderr, " failed\n" );
    return FALSE;
  }
  fprintf( stderr, " ok\n" );
  sendInitToLaser(laserDevice);
  return TRUE;
}

void
closeLaserDevice( LASER_TYPE* laserDevice)
{
  close(laserDevice->dev.fd);
  laserDevice->dev.fd = -1;
}

#define CRC16_GEN_POL 0x8005
#define CRC16_GEN_POL0 0x80
#define CRC16_GEN_POL1 0x05

int
computeCRC(unsigned char* CommData, int uLen)
{
  unsigned char abData[2];
  unsigned char uCrc16[2];
  
  abData[0] = 0;
  abData[1] = 0;
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
    } else {
      uCrc16[0] <<= 1;
      if (uCrc16[1] & 0x80)
	uCrc16[0] |= 0x01;
      uCrc16[1] <<= 1;
    }
    uCrc16[0] ^= abData[0];
    uCrc16[1] ^= abData[1];
  }
  return (((int) uCrc16[0]) * 256 + ((int) uCrc16[1]));
}

void 
ProcessLine( int laserNo, unsigned char *line, int length, 
	     int laserType, struct timeval reqtime ) 
{
  struct timeval now,timest;
  int i=0, LoB=0, HiB=0;
  int numMeasurements = ((int) (line[6] & 0x3F)) * 256 + ((int) line[5]);
  float convval=1.0;
  int bit14 = (int) (line[6] & 0x40);
  int bit15 = (int) (line[6] & 0x80);

  if (laserType==LMS_TYPE) {
    if (!bit15)
      if (!bit14)
	convval=1.0;
      else
	convval=0.1;
    else
      convval=10.0;
  }
#ifdef VERBOSE
  fprintf( stderr, "\n>RECIEVE:(%d bytes - %.2fcm)\n>", length, convval );
  for (i=0; i != length; i++)
    fprintf( stderr, "0x%.2x ", line[i] );
  fprintf( stderr, "<\n" );
  fflush(stderr);
#endif

  gettimeofday(&now, NULL);
  timest = timeStamp( reqtime, now );

  
  if (numMeasurements == 361) {
    if (!(ARG_USE_ALL_FRONT_VALUES || ARG_USE_ALL_REAR_VALUES)) { 
      fprintf(stderr, "WARNING: Unexpected number of sensor values (361-mode): %d\n", 
	      numMeasurements );
      return;
    }
  } else if ( numMeasurements != NUMBER_OF_LASER_READINGS) {
    fprintf(stderr, "WARNING: Unexpected number of sensor values: %d\n", 
	    numMeasurements );
    return;
  }
  if ((numMeasurements*2+10)==length) {
    LaserSensors[laserNo].defined = 1;
    for (i = 0; i != numMeasurements; i++){
      LoB = (int) line[(i*2)+7]; 
      HiB = (int) line[(i*2)+8];
      LaserSensors[laserNo].display_values[i] = 
	LaserSensors[laserNo].values[i] = 
	  ((HiB & 0x1f) * 256 + LoB + CORR_DIST) * convval;
      LaserSensors[laserNo].blendung[i] = (HiB & 0x20) / 32;  /* BIT 13 */
      LaserSensors[laserNo].wfv[i]      = (HiB & 0x40) / 64;  /* BIT 14 */
      LaserSensors[laserNo].sfv[i]      = (HiB & 0x80) / 128; /* BIT 15 */
    }
    LaserSensors[laserNo].timestamp.tv_sec = timest.tv_sec;
    LaserSensors[laserNo].timestamp.tv_usec = timest.tv_usec;
    LaserSensors[laserNo].num_values = numMeasurements;
  } else {
    fprintf(stderr, "WARNING: Unexpected message length: %d\n", length );
  }
}


int 
WriteLaserCommand( LASER_TYPE* laserDevice,
		   unsigned char command,
		   char *argument, int arg_length)
{
  unsigned char buffer[512];
  int      pos = 0;
  int      i=0, n=0, answer=0, loop=0, val=0, counter=0;
  unsigned short check=0;
  unsigned short length = 0;

  /* SYNC CHARS */
  buffer[pos++] = 0x02;
  /* ADDRESS    */
  buffer[pos++] = 0x00;
  /* MESSAGE LENGTH */
  length = 1 + arg_length;
  buffer[pos++] = length & 0x00ff;
  buffer[pos++] = length / 256;	/* I wonder if that works */
  /* COMMAND */
  buffer[pos++] = command;

  /* ARGUMENT */
  if (arg_length > 0)
    for (i = 0; i != arg_length; i++)
      buffer[pos++] = argument[i];
  /* CHECKSUM */
  check = computeCRC(buffer, length + 4);
  buffer[pos++] = check & 0x00ff;
  buffer[pos++] = check / 256;
  ClearInputBuffer( laserDevice );
  /* WRITE TO PORT  */

#ifdef VERBOSE
  fprintf( stderr, "\n    -> " );
  for (i=0; i != pos; i++) {
    if (i>4) 
      if (pos-i>3)
	fprintf( stderr, "%s ", bytenames[5] );
      else
	fprintf( stderr, "%s ", bytenames[i-pos+8] );
    else
      fprintf( stderr, "%s ", bytenames[i] );
  }
  fprintf( stderr, "->\n    -> " );
  for (i=0; i != pos; i++)
    fprintf( stderr, "0x%.2x ", buffer[i] );
  fprintf( stderr, "->\n" );
  fflush(stderr);
#endif

  n = ((int) writeN(&(laserDevice->dev), buffer, pos));

  /*
  tcflush(laserDevice->dev.fd, TCIOFLUSH);
  write(laserDevice->dev.fd, buffer, pos);
  tcdrain(laserDevice->dev.fd);
  */

  loop = TRUE;
  answer = INI;
  while (loop) {
    counter++;
    val = numChars(laserDevice->dev.fd);
    if (val>0) {
      read(laserDevice->dev.fd,&buffer,val);
      switch (buffer[0]) {
      case 0x02:
	answer=STX;
	break;
      case 0x06:
	answer=ACK;
	break;
      case 0x10:
	answer=DLE;
	break;
      case 0x15:
	answer=NAK;
	break;
      default:
#ifdef VERBOSE
	fprintf( stderr, "    <- UKN (0x%.2x) <-\n",buffer[0]);
#endif
	answer=UKN;
      }
      loop=FALSE;
    } else if (counter>5) {
      answer=TIO;
      loop=FALSE;
    }
    usleep(1000); 
  }
  return answer;
}

int
ClearInputBuffer( LASER_TYPE* laserDevice )
{
  unsigned char buffer[4096];
  int val=0;
  val = numChars(laserDevice->dev.fd);
  if (val>0) {
    read(laserDevice->dev.fd,&buffer,val);
  }
  return(val);
}

void
switchBaudRate( int rate,  LASER_TYPE* laserDevice )
{
  switch(rate) {
  case 9600: 
    m_setparms(laserDevice->dev.fd, "9600", 
	       laserDevice->dev.ttydev.parity, "8", 0, 0);
    break;
  case 19200: 
    m_setparms(laserDevice->dev.fd, "19200", 
	       laserDevice->dev.ttydev.parity, "8", 0, 0);
    break;
  case 38400: 
    m_setparms(laserDevice->dev.fd, "38400", 
	       laserDevice->dev.ttydev.parity, "8", 0, 0);
    break;
  }
}

void
recognizeSICK( LASER_TYPE* laserDevice, int brate )
{
  char ReqLaser[2];
  char *arg = "";
  unsigned char banswer[4096];
  int lanswer = 0;
  char name[12];

  ReqLaser[0] = 0x05;
  ReqLaser[1] = (unsigned char) NUMBER_OF_LASER_READINGS;


  lanswer=INI;
  laserDevice->dev.passwd = (char *) malloc(9*sizeof(char));
  laserDevice->dev.typestr = (char *) malloc(21*sizeof(char));
  if (laserDevice->dev.type==UKN_TYPE) {
    fprintf( stderr, "recognize LASER ............... " );
    WriteLaserCommand( laserDevice, 0x30, ReqLaser, 2);
    lanswer = waitForAnswer( laserDevice, banswer, 3 );
    while (lanswer!=STX){
      if (lanswer==UKN) {
	laserDevice->dev.ttydev.parity = (char *) malloc(5*sizeof(char));
	strcpy(laserDevice->dev.ttydev.parity,"NONE");
	switchBaudRate(brate,laserDevice);
      }
      WriteLaserCommand( laserDevice, 0x30, ReqLaser, 2);
      lanswer = waitForAnswer( laserDevice, banswer, 3 );
    }
    WriteLaserCommand( laserDevice, 0x3A, arg, 0);
    lanswer = waitForAnswer( laserDevice, banswer, 3 );
    strncpy( name, banswer, 6 );
    name[strlen(name)-1]='\0';
    if (!strncmp(name, "LMS200",6)) {
      laserDevice->dev.type = LMS_TYPE;
      strncpy(laserDevice->dev.typestr,banswer,20);
      strncpy(laserDevice->dev.passwd,LMS_PASSWORD,8);
      fprintf( stderr, " LMS\n" );
    } else {
      laserDevice->dev.type = PLS_TYPE;
      strncpy(laserDevice->dev.typestr,"PLS",20);
      strncpy(laserDevice->dev.passwd,PLS_PASSWORD,8);
      fprintf( stderr, " PLS\n" );
    }
  } else {
    if (laserDevice->dev.type==PLS_TYPE) {
      strncpy(laserDevice->dev.typestr,"PLS",20);
      strncpy(laserDevice->dev.passwd,PLS_PASSWORD,8);
      fprintf( stderr, "set LASER type ................  PLS\n" );
    } else {
      strncpy(laserDevice->dev.typestr,"LMS",20);
      strncpy(laserDevice->dev.passwd,LMS_PASSWORD,8);
      laserDevice->dev.ttydev.parity = (char *) malloc(5*sizeof(char));
      strcpy(laserDevice->dev.ttydev.parity,"NONE");
      switchBaudRate(brate,laserDevice);
      fprintf( stderr, "set LASER type ................  LMS\n" );
    }
  }
  laser_38400_baud( laserDevice, brate, laserDevice->dev.type );
}

void
setLMSrange( LASER_TYPE* laserDevice )
{
  int i;
  unsigned char banswer[4096];
  int lanswer = 0;

  char arg[9];
  char conf[32];

  arg[0] = 0x00;
  for (i=0; i<8; i++)
    arg[i+1]=laserDevice->dev.passwd[i];
  for (i=0; i<32; i++)
    conf[i]=0x00;

  lanswer=INI;
  fprintf( stderr, "set LASER in config-mode ...... " );

  while ((lanswer!=BMACK_TGM) && (lanswer!=ACK)){
    WriteLaserCommand( laserDevice, 0x20, arg, 9);
    lanswer = waitForAnswer( laserDevice, banswer, 1 );
  }
  fprintf( stderr, " ok\n" );

  lanswer=INI;
  fprintf( stderr, "get LMS-LASER configuration ... " );

  while (lanswer!=STX){
    WriteLaserCommand( laserDevice, 0x74, arg, 0);
    lanswer = waitForAnswer( laserDevice, banswer, 1 );
  }
  fprintf( stderr, " ok\n" );

  if (conf[6]!=0x00) {
    for (i=0; i<32; i++) {
      if (i==6)
	conf[i]=0x00;
      else
	conf[i]=banswer[i];
    }
    lanswer=INI;
    fprintf( stderr, "set LMS-LASER configuration ... " );
    while ((lanswer!=STX) && (lanswer!=NOT_ACK)){
      WriteLaserCommand( laserDevice, 0x77, conf, 32);
      lanswer = waitForAnswer( laserDevice, banswer, 5 );
    }
    fprintf( stderr, " ok\n" );
  }
}


void
sendInitToLaser( LASER_TYPE* laserDevice) 
{
  int cur_brate=adaptBaudRate( laserDevice );

  if (cur_brate==0) {
    fprintf( stderr, "ERROR: no device found\n" );
    exit(-1);
  } else {
    recognizeSICK( laserDevice, cur_brate );
    if (laserDevice->dev.type==LMS_TYPE)
      setLMSrange( laserDevice );
  }
}

int
adaptBaudRate( LASER_TYPE* laserDevice)
{
  int answ,i;
  unsigned char banswer[4096];
  char ReqLaser[2];
  /*  int baudrate=38400; */
  int baudrate=9600;

  ReqLaser[0] = 0x05;
  ReqLaser[1] = (unsigned char) NUMBER_OF_LASER_READINGS;
  /* test laser-request with 9600/19200/38400 BAUD */
  for (i=0;i<3;i++) {
    fprintf( stderr, " -> %5d ...", baudrate );
    switchBaudRate(baudrate,laserDevice);
    WriteLaserCommand( laserDevice, 0x30, ReqLaser, 2);
    answ=INI;
    usleep(100000);
    if (waitForAnswer( laserDevice, banswer, 1 )!=TIO) {
      fprintf( stderr, " ok\n" );
      return(baudrate);
    }
    fprintf( stderr, " no\n" );
    baudrate=baudrate*2;
  }
  return(0);
}


void
laser_38400_baud( LASER_TYPE* laserDevice, int brateset, int laserType )
{
  unsigned char banswer[4096];
  int lanswer = 0;
  int i;
  char arg[9];
  char ReqLaser[2];
  int panswer = 0;

  ReqLaser[0] = 0x05;
  ReqLaser[1] = (unsigned char) NUMBER_OF_LASER_READINGS;

  arg[0] = 0x00;
  for (i=0; i<8; i++)
    arg[i+1]=laserDevice->dev.passwd[i];

  if (brateset!=38400) {
    arg[0] = 0x00;
    lanswer=INI;
    fprintf( stderr, "set LASER in config-mode ...... " );
#ifdef VERBOSE
    fprintf( stderr, "  -> password: %s <-\n", laserDevice->dev.passwd );
#endif
    while ((lanswer!=BMACK_TGM) && (lanswer!=ACK)){
      WriteLaserCommand( laserDevice, 0x20, arg, 9);
      lanswer = waitForAnswer( laserDevice, banswer, 5 );
    }
    fprintf( stderr, " ok\n" );

    arg[0] = 0x40;
    lanswer=INI;
    fprintf( stderr, "set LASER to 38400 baud ....... " );
    while ((lanswer!=BMACK_TGM) && (lanswer!=ACK)){
      WriteLaserCommand( laserDevice, 0x20, arg, 1);
      usleep(100000);
      switchBaudRate(38400,laserDevice);
      WriteLaserCommand( laserDevice, 0x30, ReqLaser, 2);
      panswer = waitForAnswer( laserDevice, banswer, 3 );
      if (panswer==STX){
	break;
      }
    }
    fprintf( stderr, " ok\n" );
  }

  if (CONFIG_LASER_MODE) {

    arg[0] = 0x00;
    lanswer=INI;
    fprintf( stderr, "set LASER in config-mode ...... " );
    while ((lanswer!=BMACK_TGM) && (lanswer!=ACK)){
      WriteLaserCommand( laserDevice, 0x20, arg, 9);
      lanswer = waitForAnswer( laserDevice, banswer, 1 );
    }
    fprintf( stderr, " ok\n" );
    
    arg[0] = 0x25;
    lanswer=INI;
    fprintf( stderr, "set LASER in mode (0x25h) ..... " );
    
    while((lanswer!=BMACK_TGM) && (lanswer!=ACK)){
      WriteLaserCommand( laserDevice, 0x20, arg, 1);
      lanswer = waitForAnswer( laserDevice, banswer, 1 );
    }
    fprintf( stderr, " ok\n" );
  }
}

int
waitForAnswer( LASER_TYPE* laserDevice, unsigned char *abuffer, int TimeOut )
{
  unsigned char answer=0; 
  unsigned char buffer[4096];
  short r_wert = INI;
  int curBy=1;               /* current FilePointer */
  int ByRead=0;              /* #read Bytes */
  int AnswLen=4096;          /* length of answer */
  int i=0;
  int tcounter=0;
  /* max time for ACK is 40ms */
  setStartTime( EVERYBODIES_TIMER);
  /* five possibilities: TimeOut,  Ack,   NoAck,  StartX,   DLE(?)
     always one byte:       -     0x06h   0x15h    0x02h    0x10h   */
  do {
    fprintf(stderr,"%c\b",idlechar[tcounter++]);
    if (tcounter>3) tcounter=0;
    if (numChars(laserDevice->dev.fd) > 0) {
      read(laserDevice->dev.fd, &answer, 1);
      switch(answer) {
      case 0x02: {
	buffer[0]=answer;
	while ( (curBy<AnswLen) && (timeExpired(EVERYBODIES_TIMER)<TimeOut) ) {
	  ByRead = read( laserDevice->dev.fd, &buffer[curBy],512);
	  curBy = curBy + ByRead;
	  if (curBy>2) {
	    AnswLen = (((int) buffer[3])*256)+((int) buffer[2])+6;
	  }
	  usleep(1000);
	}
	if (curBy>=AnswLen) {
	  switch(buffer[4]) {
	  case 0xa0:
#ifdef VERBOSE
	    fprintf( stderr, "    <- BMACK_TGM <-\n" );
#endif
	    r_wert = BMACK_TGM;
	    break;
	  case 0x91:
#ifdef VERBOSE
	    fprintf( stderr, "    <- RES_ACK <-\n" );
#endif
	    r_wert = RES_ACK;
	    break;
	  case 0x90:
#ifdef VERBOSE
	    fprintf( stderr, "    <- RES_ACK <-\n" );
#endif
	    r_wert = RES_ACK;
	    break;
	  case 0x92:
#ifdef VERBOSE
	    fprintf( stderr, "    <- NOT_ACK <-\n" );
#endif
	    r_wert = NOT_ACK;
	    break;
	  default:
#ifdef VERBOSE
	    fprintf( stderr, "    <- STX <-\n" );
#endif
	    r_wert = STX;
	  }
	} else {
#ifdef VERBOSE
	  fprintf( stderr, "    <- TIO <-\n" );
#endif
	  r_wert = TIO; 
	}
#ifdef VERBOSE
	fprintf( stderr, "    <- " );
	for (i=0; i<(curBy>10?10:curBy); i++)
	  fprintf( stderr, "0x%.2x ", buffer[i] );
	fprintf( stderr, (curBy>10?"... <-\n":"<-\n") );
#endif
	break;
      }
      case 0x06: {
#ifdef VERBOSE
	fprintf( stderr, "    <- ACK <-\n" );
#endif
	r_wert = ACK;
	break;
      }
      case 0x10: {
#ifdef VERBOSE
	fprintf( stderr, "    <- DLE <-\n" );
#endif
	r_wert = DLE;
	break;
      }
      case 0x15: {
#ifdef VERBOSE
	fprintf( stderr, "    <- NAK <-\n" );
#endif
	r_wert = NAK;
	break;
      }
      case 0x91: {
#ifdef VERBOSE
	fprintf( stderr, "    <- INIT_ACK_TGM <-\n" );
#endif
        r_wert = INIT_ACK_TGM;
        break;
      }
      case 0x90: {
#ifdef VERBOSE
	fprintf( stderr, "    <- PWON_TGM <-\n" );
#endif
        r_wert = PWON_TGM;
        break;
      }
      case 0x92: {
#ifdef VERBOSE
	fprintf( stderr, "    <- NACK_TGM <-\n" );
#endif
        r_wert = NACK_TGM;
        break;
      }
      case 0xa0: {
#ifdef VERBOSE
	fprintf( stderr, "    <- BMACK_TGM <-\n" );
#endif
        r_wert = BMACK_TGM;
        break;
      }
      default: {
#ifdef VERBOSE
	fprintf( stderr, "    <- UKN (%.2xh) <-\n",answer);
#endif
	r_wert = UKN;
      }
      }
    } else if (timeExpired(EVERYBODIES_TIMER)>TimeOut) {
#ifdef VERBOSE
      fprintf( stderr, "    <- TIMEOUT <-\n" );
#endif
      r_wert = TIO;
    }
    usleep(1000);

  } while (r_wert < 0);

#ifdef VERBOSE
  fprintf( stderr, "    <- " );
#endif
  for (i=5;i<(curBy-3);i++) {
    abuffer[i-5]=buffer[i];
#ifdef VERBOSE
    fprintf( stderr, "0x%.2x ", abuffer[i-5] );
#endif
  }
#ifdef VERBOSE
  fprintf( stderr, "<-\n    <- copied %d bytes <-\n", i-5 );
#endif
  
  return(r_wert);
}

/*****************************************************************************
 *
 * FUNCTION: initLaserReadingStructures
 *
 *****************************************************************************/
void
initLaserDeviceNames( char* frontDevice, char* rearDevice)
{
  if ((ARG_USE_TWO_LASERS==TRUE) ||
      ((ARG_USE_FRONT_LASER==TRUE)&&(ARG_USE_REAR_LASER==TRUE))) {
    fprintf(stderr, "Use settings for robot with two laser range finders.\n");
    NUMBER_OF_LASERS  = 2;
    USE_FRONT_LASER   = TRUE;
    USE_REAR_LASER    = TRUE;
  } else {
    fprintf(stderr, "Use settings for robot with one laser range finder.\n");
    NUMBER_OF_LASERS  = 1;
    USE_FRONT_LASER   = TRUE;
    USE_REAR_LASER    = FALSE;
  }

  if (ARG_USE_FRONT_LASER != NOT_SET) {
    USE_FRONT_LASER   = ARG_USE_FRONT_LASER;
    fprintf(stderr, " arg -> frontlaser %s\n", 
	    USE_FRONT_LASER ? "enabled" : "disabled" );
  }
  if (ARG_USE_REAR_LASER != NOT_SET) {
    USE_REAR_LASER    = ARG_USE_REAR_LASER;
    fprintf(stderr, " arg -> rearlaser %s\n", 
	    USE_REAR_LASER ? "enabled" : "disabled" );
  }
  
  if ( (!USE_FRONT_LASER) && (!USE_REAR_LASER) ) {
    fprintf(stderr, "no laser activated ... exit\n" );
    exit(0);
  }

}

void
initLaserReadings( char *frontDevice, char *rearDevice ) 
{
  struct timeval now;
  int i=0;
  gettimeofday( &now, 0);
  /* FRONT LASER */
  frontLaserReading.new = FALSE;
  frontLaserDevice.dev.ttydev.ttyPort = frontDevice;
  if ( USE_FRONT_LASER ) {
    frontLaserReading.numberOfReadings = NUMBER_OF_LASER_READINGS;
    frontLaserReading.startAngle       = FRONT_LASER_START_ANGLE;
    frontLaserReading.angleResolution  = DEG_180 / (float) (NUMBER_OF_LASER_READINGS - 1);
    frontLaserReading.time.tv_sec      = now.tv_sec;
    frontLaserReading.time.tv_usec     = now.tv_usec;
    /* Allocate memroy. */
    if ( frontLaserReading.reading == NULL)
      frontLaserReading.reading          = (float*)
      calloc( NUMBER_OF_LASER_READINGS, (sizeof (float)));
    if ( frontLaserReading.blendung == NULL)
      frontLaserReading.blendung          = (int*)
      calloc( NUMBER_OF_LASER_READINGS, (sizeof (int)));
    if ( frontLaserReading.wfv == NULL)
      frontLaserReading.wfv          = (int*)
      calloc( NUMBER_OF_LASER_READINGS, (sizeof (int)));
    if ( frontLaserReading.sfv == NULL)
      frontLaserReading.sfv          = (int*)
      calloc( NUMBER_OF_LASER_READINGS, (sizeof (int)));
  }
  /* REAR LASER */
  rearLaserReading.new = FALSE;
  rearLaserDevice.dev.ttydev.ttyPort = rearDevice;
  if ( USE_REAR_LASER ) {
    rearLaserReading.numberOfReadings = NUMBER_OF_LASER_READINGS;
    rearLaserReading.startAngle       = REAR_LASER_START_ANGLE;
    rearLaserReading.angleResolution  = DEG_180 / (float) (NUMBER_OF_LASER_READINGS - 1);
    rearLaserReading.time.tv_sec      = now.tv_sec;
    rearLaserReading.time.tv_usec     = now.tv_usec;
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
  }
  /* Now set the values to undefined. */
  for (i=0; i != frontLaserReading.numberOfReadings; i++)
    frontLaserReading.reading[i] = -1.0;
  for (i=0; i != rearLaserReading.numberOfReadings; i++)
    rearLaserReading.reading[i] = -1.0;
}

int
connectPLS_TTY( DEV_PTR dev)
{
  struct termios term_info;
  /* 0x8000 ist SYNC */  
  if ( (dev->fd = open( (dev->ttydev.ttyPort), (O_RDWR | 0x8000 | O_NOCTTY),0)) < 0 ) {
    fprintf(stderr,"Serial I/O Error:  Could not open port %s",dev->ttydev.ttyPort);
    perror(dev->ttydev.ttyPort);
    return (dev->fd);
  }

  if(tcgetattr(dev->fd,&term_info) <0) {
    /* complain - fd is not a terminal */
    fprintf(stderr, "%s:%6d:%s() - ERROR: Can't get control bits.\n", 
	    __FILE__, __LINE__, __FUNCTION__);
    exit(-1);
  }
  
  /* turn on echo, canonical mode, extended processing, signals */
  term_info.c_lflag |= IEXTEN | ISIG;
  
  /* turn on break sig, cr->nl, parity, 8 bit strip, flow control */
  term_info.c_lflag |= BRKINT | ICRNL | INPCK | ISTRIP;
  
  /* turn off echo, canonical mode, extended processing, signals */
  term_info.c_lflag &= ~(ECHO | ICANON | IXON);
  
  /* clear size, turn off parity bit */
  term_info.c_cflag &= ~(CSIZE | PARENB);
  
  /* set size to 8 bits */
  term_info.c_cflag |= CS8;
  
  /* Set time and bytes to read at once */
  term_info.c_cc[VTIME] = 1;
  term_info.c_cc[VMIN] = 0;
  
  if (cfsetospeed(&term_info, dev->ttydev.baudCode)) {
    fprintf(stderr, "cfsetospeed(%ld): ",
	    (long)dev->ttydev.baudCode);
    perror(NULL);
    exit(-1);
  }
  
  if (cfsetispeed(&term_info, dev->ttydev.baudCode)) {
    fprintf(stderr, "cfsetispeed(%ld): ",
	    (long)dev->ttydev.baudCode);
    perror(NULL);
    exit(-1);
  }
  
  if(tcsetattr(dev->fd,TCSAFLUSH,&term_info) <0) {
    fprintf(stderr, "%s:%6d:%s() - ERROR: Can't initialize control bits.\n", 
	    __FILE__, __LINE__, __FUNCTION__);
    exit(-1);
  }
  return( dev->fd);
  
}

void
signalLaser(void)
{
  fprintf(stderr,"shutting down the laser\n");
}

