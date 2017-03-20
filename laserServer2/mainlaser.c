#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <sys/time.h>

#include "bUtils.h"
#include "librobot.h"

#include "tcx.h"
#include "tcxP.h"
#include "global.h"

#include "LASER_SERVER-messages.h"

#include "io.h"
#include "mainlaser.h"
#include "laserHandlers.h"

int ARG_USE_FRONT_LASER  = NOT_SET;
int ARG_USE_REAR_LASER   = NOT_SET;
int ARG_USE_TWO_LASERS   = NOT_SET;
int ARG_USE_ALL_FRONT_VALUES = FALSE;
int ARG_USE_ALL_REAR_VALUES = FALSE;
int CONFIG_LASER_MODE = FALSE;

int USE_DATAFILE   = FALSE;
int CORR_DIST      = 0;

FILE *datafile;

int USE_LMS  = FALSE;

extern int computeCRC(unsigned char* CommData, int uLen);
extern int startLaser(char* frontDevice, char* rearDevice);
extern int WriteLaserCommand( LASER_TYPE* laserDevice,
			      unsigned char command,
			      char *argument, int arg_length );
extern void ProcessLine( int LaserNo, unsigned char *line, 
			 int length, int laserType, 
			 struct timeval reqtime );
extern long numChars(int sd);
extern void stop_laser( void );

void 
commShutdown( void ) {
  stop_laser();
  if (USE_DATAFILE) {
    fflush(datafile);
    fclose(datafile);
  }
  fprintf( stderr, "%s: TCX Server died. Exiting.\n", __FILE__ );
  exit(-1);
}

void
SaveLaserData( int numLaser, float *values, struct timeval timestamp )
{
  int i;
  fprintf( datafile, "LASER %d %ld %ld ", 
	   numLaser, timestamp.tv_sec, timestamp.tv_usec );
  for (i=0; i != NUMBER_LASERS; i++)
    fprintf( datafile, " %4.1f", values[i] );
  fprintf( datafile, "\n" );
}

int 
main( int argc, char *argv[] ) {

  char DATA_FILE_NAME[255] = "";
  char* robotName = NULL;
  char* frontDevice = NULL;
  char* rearDevice = NULL;

  struct timeval TCX_waiting_time = {0, 0};

#ifdef BEE_INI
  struct bParamList * bParamList = NULL;
  int pioneer_test = 0;
  int nameSet = FALSE;
#endif

  unsigned char Fbuffer[4096];
  unsigned char Rbuffer[4096];

  char FReqLaser[2];
  int FReqLaserLen;
  char RReqLaser[2];
  int RReqLaserLen;

  int i=0, numB=0, val = 0;
  int numFp = 0;
  int numRp = 0;
  int numFc = 0;
  int numRc = 0;

  struct timeval f_this_time;
  struct timeval f_last_time;
  struct timeval r_this_time;
  struct timeval r_last_time;

  float  time_difference=0.0; 

  TCX_waiting_time.tv_sec  = 0;
  TCX_waiting_time.tv_usec = 0;

  /* anaylse arguments */

#ifndef BEE_INI
  for (i=1; i<argc; i++) {
    if ((strcmp(argv[i],"-robot")==0)){
	if (i + 1 < argc && argv[i+1][0]!='-'){
	  i++;
	  robotName = argv[i];
	} else {
	  fprintf(stderr, "ERROR: robot name must follow keyword robot.\n");
	  exit(0);
	}
    } else if ((strcmp(argv[i],"-nofrontlaser")==0)) {
      ARG_USE_FRONT_LASER = FALSE;
    } else if ((strcmp(argv[i],"-frontlaser")==0)) {
      ARG_USE_FRONT_LASER = TRUE;
    } else if ((strcmp(argv[i],"-norearlaser")==0)) {
      ARG_USE_REAR_LASER = FALSE;
    } else if ((strcmp(argv[i],"-rearlaser")==0)) {
      ARG_USE_REAR_LASER = TRUE;
    } else if ((strcmp(argv[i],"-frontDev")==0)){
      if (i < argc - 1) {
	i++;
	frontDevice = argv[i];
	ARG_USE_FRONT_LASER = TRUE;
      } else {
	fprintf(stderr, "ERROR: device name must follow keyword -frontDev.\n");
	exit(0);
      }
    } else if ((strcmp(argv[i],"-rearDev")==0)){
      if (i < argc - 1) {
	i++;
	rearDevice = argv[i];
	ARG_USE_REAR_LASER = TRUE;
      } else {
	fprintf(stderr, "ERROR: device name must follow keyword -rearDev.\n");
	exit(0);
      }
    } else if ((strcmp(argv[i],"-pls")==0)){
      frontLaserDevice.dev.type=PLS_TYPE;
      rearLaserDevice.dev.type=PLS_TYPE;
    } else if ((strcmp(argv[i],"-lms")==0)){
      frontLaserDevice.dev.type=LMS_TYPE;
      rearLaserDevice.dev.type=LMS_TYPE;
    } else if ((strcmp(argv[i],"-rhino")==0)){
      frontDevice = RHINO_F_DEVICE;
      rearDevice  = RHINO_R_DEVICE;
      ARG_USE_TWO_LASERS = TRUE;
    } else if ((strcmp(argv[i],"-defiant")==0)){
      frontDevice = DEFI_F_DEVICE;
      rearDevice  = DEFI_R_DEVICE;
      ARG_USE_TWO_LASERS = TRUE;
    } else if ((strcmp(argv[i],"-b18")==0)){
      frontDevice = B18_DEVICE;
    } else if ((strcmp(argv[i],"-b21")==0)){
      frontDevice = B21_DEVICE;
    } else if ((strcmp(argv[i],"-b21r")==0)){
      frontDevice = B21R_DEVICE;
    } else if ((strcmp(argv[i],"-pioneer")==0)){
      frontDevice = PIONEER_DEVICE;
    } else if ((strcmp(argv[i],"-pioneer2")==0)){
      frontDevice = PIONEER2_DEVICE;
    } else if ((strcmp(argv[i],"-urban")==0)){
      frontDevice = URBAN_DEVICE;
    } else if ((strcmp(argv[i],"-scout")==0)){
      frontDevice = SCOUT_DEVICE;
    } else if ((strcmp(argv[i],"-f360")==0)) {
      ARG_USE_ALL_FRONT_VALUES = TRUE;
    } else if ((strcmp(argv[i],"-r360")==0)) {
      ARG_USE_ALL_REAR_VALUES = TRUE;
    } else if ((strcmp(argv[i],"-config")==0)) {
      CONFIG_LASER_MODE = TRUE;
    } else if ((strcmp(argv[i],"-corr")==0)){
      if (i < argc - 1) {
	i++;
	CORR_DIST = atoi(argv[i]);
      } else {
	fprintf(stderr, "ERROR: correction dist must follow keyword -corr.\n");
	exit(0);
      }
    } else if ((strcmp(argv[i],"-datafile")==0)) {
      USE_DATAFILE = TRUE;
      if (i<argc-1) {
	i++;
	strncpy( DATA_FILE_NAME, argv[i], 255 );
      }
    } else {
      fprintf(stderr, "Usage: %s [-robot name] [-frontlaser] [-nofrontlaser] [-frontDev dev] [-rearlaser] [-norearlaser] [-rearDev dev] [-pls] [-lms] [-f360] [-r360] [-config] [-datafile FileName] [-rhino] [-b18] [-b21] [-pioneer] [-pioneer2] [-urban] [-scout] [-defiant]\n", argv[0]);
      exit(0);
    }
  }

#else

  /* add some parameter files */
  bParamList = bParametersAddFile(bParamList, "etc/beeSoft.ini");

  /* add some enviroment variables */
  bParamList = bParametersAddEnv(bParamList, "", "TCXHOST");

  /* add command line arguements */
  bParamList = bParametersAddArray(bParamList, "", argc, argv);

  for (i=1; i<argc; i++) {
    if ((strcmp(argv[i],"-type")==0)) {
      if (i < argc - 1) {
	i++;
        if ( (!(strcmp(argv[i],"PIONEER_I"))) || (!(strcmp(argv[i],"PIONEER_II"))) || 
	     (!(strcmp(argv[i],"PIONEER_IF"))) || (!(strcmp(argv[i],"PIONEER_AT"))) ) {
	  pioneer_test = 1;
	}
	bParamList = bParametersAddEntry( bParamList, "robot", "name", argv[i] );
	nameSet = TRUE;
      } else {
	fprintf(stderr, "ERROR: robot type must follow keyword robot.\n");
	exit(0);
      }
     } else if ((strcmp(argv[i],"-robot")==0)){
       if (i + 1 < argc && argv[i+1][0]!='-'){
	 i++;
	 robotName = argv[i];
       } else {
	 fprintf(stderr, "ERROR: robot name must follow keyword robot.\n");
	 exit(0);
       }
     } else {
       fprintf(stderr, "Usage: %s [-robot name] [-type TYPE]\n", argv[0]);
       exit(0);
     } 
  }

  if (!nameSet)
    bParamList = bParametersAddEntry( bParamList, "robot", "name", "B21" );

  if (pioneer_test)
    /* add some parameter files */
    bParamList = bParametersAddEntry(bParamList, "robot", "pioneer","yes");
  else
    bParamList = bParametersAddEntry(bParamList, "robot", "pioneer","no");

  bParametersFillParams(bParamList);  

  if (pioneer_test)
    if ( bRobot.pio_laser_dev != NULL) {
      frontDevice = strdup( bRobot.pio_laser_dev);
    } else {
      fprintf(stderr, "Oops! bRobot.pio_laser_dev laser device not correctly set.\n");
      exit(0);
    }
  else {
    if ( bRobot.laser_front_dev != NULL) {
      frontDevice = strdup(bRobot.laser_front_dev);
    } else {
      fprintf(stderr, "Oops! bRobot.laser_front_dev not correctly set.\n");
      exit(0);
    }
    if ( bRobot.laser_rear_dev != NULL) {
      rearDevice = strdup(bRobot.laser_rear_dev);
      ARG_USE_TWO_LASERS=TRUE;
    }
  }
#endif

  if (ARG_USE_ALL_FRONT_VALUES) {
    FReqLaser[0] = 0x01; /* alle 360 Werte */
    FReqLaserLen = 1;
  } else {
    FReqLaser[0] = 0x05; /* minimaler Wert im Segment */
    FReqLaser[1] = (unsigned char) DEFAULT_NUM_VALUES;
    FReqLaserLen = 2;
  }

  if (ARG_USE_ALL_REAR_VALUES) {
    RReqLaser[0] = 0x01; /* alle 360 Werte */
    RReqLaserLen = 1;
  } else {
    RReqLaser[0] = 0x05; /* minimaler Wert im Segment */
    RReqLaser[1] = (unsigned char) DEFAULT_NUM_VALUES;
    RReqLaserLen = 2;
  }

  if (USE_DATAFILE) {
    fprintf( stderr, "use data-file %s\n", DATA_FILE_NAME );
    if ((datafile = fopen( DATA_FILE_NAME, "a")) == 0){
      fprintf(stderr, "ERROR: could not open data file %s\n", DATA_FILE_NAME);
      exit(0);
    }
  }
  
  LaserInitializeTCX( robotName);
  
  startLaser( frontDevice, rearDevice);

  /* init TimeOuts */
  gettimeofday(&f_last_time, NULL);
  gettimeofday(&r_last_time, NULL);

  fprintf( stderr, "--------- START --------\n" );

  while(TRUE) {
   tcxRecvLoop((void *) &TCX_waiting_time);
    
    if (USE_FRONT_LASER) {
      /* TIMEOUT FRONT-LASER */
      gettimeofday(&f_this_time, NULL);
      time_difference = 
	((float) (f_this_time.tv_sec - f_last_time.tv_sec)) + 
	(((float) (f_this_time.tv_usec - f_last_time.tv_usec)) /  1000000.0);
      if (time_difference > MAX_TIME_DIFF){
	fprintf( stderr, "f" );
	WriteLaserCommand( &frontLaserDevice, 0x30, FReqLaser, FReqLaserLen);
	f_last_time.tv_sec = f_this_time.tv_sec;
	f_last_time.tv_usec = f_this_time.tv_usec;
      }
      
      /* THERE ARE DATA FROM FRONT-LASER */
      if ((val=numChars(frontLaserDevice.dev.fd))) {
	numB = read(frontLaserDevice.dev.fd,&Fbuffer[numFp], 512);
	numFp = numFp + numB;
	if (!(Fbuffer[0] == 0x02)){
	  numFp=0;
	} else {
	  numFc = (((int) Fbuffer[3])*256)+((int) Fbuffer[2])+6;
	}
	if ((numFp>=numFc) && (numFc>2)) {
	  numFp=0;
	  if( ((int) Fbuffer[numFc-1]) * 256 + ((int) Fbuffer[numFc-2]) ==
	      computeCRC(Fbuffer, numFc-2 )) {
	    ProcessLine(frontLaserDevice.laserNumber,
			Fbuffer,
			numFc,
			frontLaserDevice.dev.type,
			f_last_time);
	    fprintf( stderr, "+" );
	  } else {
	    fprintf( stderr, "." );
	  }
	  WriteLaserCommand( &frontLaserDevice, 0x30, FReqLaser, FReqLaserLen);
	  gettimeofday(&f_last_time, NULL);
	}
      }
    } /* front laser */
    
    if (USE_REAR_LASER) {
      /* TIMEOUT REAR-LASER */
      gettimeofday(&r_this_time, NULL);
      time_difference = 
	((float) (r_this_time.tv_sec - r_last_time.tv_sec)) + 
	(((float) (r_this_time.tv_usec - r_last_time.tv_usec)) /  1000000.0);
      if (time_difference > MAX_TIME_DIFF){
	fprintf( stderr, "r" );
	WriteLaserCommand( &rearLaserDevice, 0x30, RReqLaser, FReqLaserLen);
	r_last_time.tv_sec = r_this_time.tv_sec;
	r_last_time.tv_usec = r_this_time.tv_usec;
      }
      /* THERE ARE DATA FROM REAR-LASER */
      if (numChars(rearLaserDevice.dev.fd)) {
	numB = read(rearLaserDevice.dev.fd,&Rbuffer[numRp],512);
	numRp = numRp + numB;
	if (!(Rbuffer[0] == 0x02)){
	  numRp=0;
	} else {
	  numRc = (((int) Rbuffer[3])*256)+((int) Rbuffer[2])+6;
	}	  
	if ((numRp>=numRc) && (numRc>2)) {
	  numRp=0;
	  if( ((int) Rbuffer[numRc-1]) * 256 + ((int) Rbuffer[numRc-2]) ==
	      computeCRC(Rbuffer, numRc-2 )) {
	    ProcessLine(rearLaserDevice.laserNumber,
			Rbuffer,
			numRc,
			rearLaserDevice.dev.type,
			r_last_time);
	    fprintf( stderr, "-" );
	  } else {
	    fprintf( stderr, ":" );
	  }
	  WriteLaserCommand( &rearLaserDevice, 0x30, RReqLaser, RReqLaserLen);
	  gettimeofday(&r_last_time, NULL);
	}
      }
    } /* use rear */

    /* SEND MESSAGES */
    for (i = 0; i != 2; i++){
      if ( (LaserSensors[i].defined) && (LaserSensors[i].num_values>0 )) {

	if (USE_DATAFILE) {
	  SaveLaserData( i, LaserSensors[i].values, 
			 LaserSensors[i].timestamp );
	}
  
	if ( (i == 0 && n_auto_sweep0_update_modules ) ||
	     (i == 1 && n_auto_sweep1_update_modules )) {
	  send_automatic_sweep_update( i, LaserSensors[i].num_values, LaserSensors[i].values,
				       LaserSensors[i].timestamp );
	}
	LaserSensors[i].defined = 0;
      }
    }
    usleep(2000);
 } /* while (TRUE) */
}

