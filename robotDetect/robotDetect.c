/*
 * (hk)
 * Multi-Robot Relative Localization
 * use X -bpp 8 or set flag -nodisplay
 *
 * TODO 
 * extend for Matrox: i.e. use variable grabberDev again, add compiler
 * directives, fix inital parameter reading
 * There are many global variables. They seem to be necessary, because
 * data needs to be shared between e.g. the laserCallbackfn and
 * the image processing routines. At least bundle them in a struct.
 * Or think about a nice c++ object hierarchie.
 * 
 * $Id: robotDetect.c,v 1.1 2002/09/14 16:40:52 rstone Exp $
 */
#include <stdlib.h>
#include <math.h>
#include <tcx.h>
#include <tcxP.h>
#include <rai.h>
#include <raiClient.h>
#include <bUtils.h>

#if (!defined(sun))
#include <sys/ipc.h> /* shm stuff */
#include <sys/shm.h> /* shm stuff */
#endif

#include <signal.h>
#include <laserClient.h>
#include <cameraClient.h>
#include <Common.h>
#include <EZX11.h>
#include <display.h>
#include <imagesize.h>
#include <misc.h>
#include <qcamspecs.h>
#include <colorFilter.h>
#include <functions.h>

#define TCX_define_variables  /* this makes sure variables are installed */ 

#include "ROBOT_DETECTION-messages.h"


#define LASER_UPDATEFREQ 4
#define LASERWINSIZE 300
#define LASERX LASERWINSIZE/2
#define LASERY LASERWINSIZE
#define MAXLINELENGTH LASERWINSIZE/2 -3
#define LASERMAXRANGE 500.0


int display = 1;
int showMarker = 0;
int markerDetected = 0;     /* BOOLEAN */
int markerX = COLS/2;
float markerDistance=0.0;       /* cm */
int markerAngle = 0;            /* 0-179 */
int laserUpdated = 0;

char *pCameraImage[2];
char image[COLS*ROWS*4];
int shmid[2];			/* the id of the shared mem segment */
int shmid_attached[2];		/* did we receive the id, yet? */
int window[2];
EZXW_p ezWindow;
int idecimation=1;
char* targetName = "robin";

const char *grabberConfig;
const char *grabberType;
const char *grabberWhichGrabber;
const char *grabberDev0;
const char *grabberDev1;

int useGrabber;

char *patternFileName=NULL;
int startPatternNumber = 0;


#define MAX_STRING_LENGTH 80
#define NUM_LASER_SENSORS 180

TCX_MODULE_PTR  multiLocalizeModule = NULL;

//---------------------------------------------------------------------
// Initializes tcx connection.
//---------------------------------------------------------------------
void
ROBOT_DETECTION_register_auto_update_handler( TCX_REF_PTR                 ref,
					      ROBOT_DETECTION_register_auto_update_ptr subscribe)
{
  multiLocalizeModule = ref->module;
  
  fprintf( stderr, "Status update desired from %s.\n", tcxModuleName(multiLocalizeModule));
}


/* ---------------------------------------------------------
 *
 * commShutdown() implements graceful release (TCX communication)
 * --------------------------------------------------------*/
void commShutdown(char *name, TCX_MODULE_PTR module)
{
return;
/*
#if ( defined(G_DEBUG_WHERE_AM_I) )
  fprintf( stderr, "%s: %s\n", __FILE__, __FUNCTION__ );
#endif

  fprintf( stderr, "%s(%s): %s died. Closing. \n", 
	   __FILE__, __FUNCTION__, name );


  module = module;

  RaiShutdown();

  exit( -1 );
  */
}

/* ---------------------------------------------------------
 *
 * 
 * ctrlcShutdown() implements graceful release
 * --------------------------------------------------------*/
void ctrlcShutdown() 
{
#if ( defined(G_DEBUG_WHERE_AM_I) )
  fprintf( stderr, "%s: %s\n", __FILE__, __FUNCTION__ );
#endif

  fprintf( stderr, "%s(%s): %s died. Closing. \n", 
	   __FILE__, __FUNCTION__, "Ctrl-C" );

  RaiShutdown();
  exit( -1 );
}


/* ==========================================================
 * Laser-relevant functions
 * =========================================================*/



/* ---------------------------------------------------------
 * updateLaserDisplay(laserSweepType *data, int angleToBeMarked)
 * Displays the laserSweep in an EZWindow and colors the
 * beam at angleToBeMarked.
 *
 * --------------------------------------------------------*/
void updateLaserDisplay(float *data, int angleToBeMarked) {

  int i, lineLength;
  float laserMaxRange;
  int x2, y2;

  EZX_ClearWindow (ezWindow);  
  EZX_SetColor(C_RED);

//  if ((markerDetected))
//    laserMaxRange = data[angleToBeMarked] + 50.0;
//      laserMaxRange = 1200.0;
//  else 
  laserMaxRange = getMax(data, NUM_LASER_SENSORS);

  for (i=0; i< NUM_LASER_SENSORS; i++) {

    lineLength = (data[i] / laserMaxRange) * MAXLINELENGTH;
    
    if (lineLength > MAXLINELENGTH) 
      lineLength = MAXLINELENGTH;    
    /*
    fprintf(stderr, "lineLength %d\n", lineLength);
    fprintf(stderr, "angle %d\n", i);    TCX_waiting_time.tv_sec  = 0;
    TCX_waiting_time.tv_usec = 0;
    tcxRecvLoop((void *) &TCX_waiting_time);   
    fprintf(stderr, "sin() %f\n", sin(deg2Rad(i)));
    fprintf(stderr, "cos() %f\n", cos(deg2Rad(i)));
    */
    y2 = sin(deg2Rad(i)) * lineLength;
    x2 = cos(deg2Rad(i)) * lineLength;
    /*
    fprintf(stderr, "x2 %d\n", x2);
    fprintf(stderr, "y2 %d\n", y2);
    */
    if ((markerDetected) && (i == angleToBeMarked)) {
      EZX_SetColor(C_BLACK);
      EZX_DrawLine (ezWindow, LASERX, LASERY, LASERX+x2, LASERY-y2);  
      EZX_DrawLine (ezWindow, LASERX+1, LASERY, LASERX+x2+1, LASERY-y2);
      EZX_SetColor(C_RED);
    } else EZX_DrawLine (ezWindow, LASERX, LASERY, LASERX+x2, LASERY-y2);
  }
}





/* ---------------------------------------------------------
 * myLaserSweepCallback returns the minimum distance measured
 * by the laser. Units?
 * looking in the robot's direction, the right-most laser is numbered 0
 * and the left-most laser is numbered 179.
 * The function updates the current markerDistance and markerAngle values 
 * according to the camera-X value.
 * Angular degrees are printed relative to an imaginary middle line
 * in the center of the robot's view. The center is associated with 0 angular degrees.
 * To the right, angular degrees are positive and range from 1-90. Left
 * of the center, the symmetric case with negative values holds.
 * Malfunctioning of the laser is often caused by low voltage. Be sure to use
 * an appropriatly powerful charging device. 13.1 V works.
 * 
 * The underlying data structure is as follows (cf laserServer.h)
 * typedef struct laserSweepType {
 *  int numberLasers; NOTE: bad naming: should be numberOfBeams          
 *  int numLaser;     NOTE: != 0 if and only if multiple lasers used          
 *  float *value;             
 *  laserSweepType; }
 *
 * --------------------------------------------------------*/
int myLaserSweepCallback ( laserSweepType *data ) {
  int *markerAnglePtr = &markerAngle; 
  float *markerDistancePtr = &markerDistance;

  getAngleAndDistance(markerX, data->value, markerAnglePtr, markerDistancePtr);
    if (display) updateLaserDisplay(data->value,markerAngle);
  laserUpdated = 1;

  return 0;
}

/* ---------------------------------------------------------
 *
 * is called every 200ms from the polling routine
 *
 * --------------------------------------------------------*/
void CommandLoop( RaiModule *mod )
{

  mod = mod;

  if ( laserConnected == 0 ) {
    laserConnect( 0 );		/* try connecting  */
    laserSubscribeSweep( 0, 1 ); /* 0 == first laser, 1 = every reading*/

  }

}


/* ---------------------------------------------------------
 *
 * 
 *
 * --------------------------------------------------------*/
void createLaserExampleModule() {
 
  RaiModule *LaserExampleModule;
 
  LaserExampleModule = makeModule( "laserExample", NULL) ;  

  registerLaserSweepCallback( myLaserSweepCallback );
  
  addPolling( LaserExampleModule, CommandLoop, 200 );
  
  fprintf( stderr, "Done.\n" );
  
}









/* ======================================================================
 * Camera-relevant functions
 * ===================================================================*/


/* ---------------------------------------------------------
 *
 * myCameraShmIdCallback() attaches to the shm-segment initialized
 * by the cameraServer. Since for Qcam the server  runs SUID root,
 * the client needs to run SUID as well, because the shm-segment
 * is IPC_PRIVATE.
 *
 * --------------------------------------------------------*/
int myCameraShmIdCallback ( cameraShmIdType *gestureShmId ) {

  int numGrabber;

#if (defined(sun))
  int i;
#endif

  fprintf( stderr, 
	   "%s: Attaching to cameraServer's shmid %d (for grabber %d).\n", 
	   __FILE__, gestureShmId->shmid, gestureShmId->numGrabber );

  numGrabber = gestureShmId->numGrabber;

  if ( numGrabber!=0 && numGrabber!=1 ) {
    MISCERROR;
    return -1;
  }

#if (defined(sun))
  pCameraImage[numGrabber] = (char *) malloc( sizeof(char)*ROWS*COLS*4 );
  i = ROWS*COLS*4 - 1;
  do {
    pCameraImage[numGrabber][i] = 5;
  } while ( --i>=0 );
#else
  /* so that we can access the shared chunk of memory */
  pCameraImage[numGrabber] = (char *) shmat( gestureShmId->shmid, (char *) 0, 0);
  if ( pCameraImage[numGrabber] == (char*)(-1) ) {
    perror( "Attaching failed: shmat's error is " );
    return -1;
  }
#endif

  shmid_attached[numGrabber] = 1;
      
  return 0;
}


/* ---------------------------------------------------------
 *
 * Dumps the ppm image (as rgb). Note, that the source data is
 * four bytes and BGRx and NOT RGBX.
 *
 * --------------------------------------------------------*/
int write4toPPMimage( int num ) {

  FILE *fp;
  char name[50];
  char dump[COLS*ROWS*4];
  char realdump[COLS*ROWS*3];
  int i,j;

  /* dump the current image into a .ppm file */
  sprintf( name, "image-%02d.ppm", num );
  fprintf( stderr, "Writing image into %s for later processing\n", name);
  fp = fopen( name, "w" );
  if ( !fp ) {
    perror("Error opening filename");
    return -1;
  }
  memcpy( dump, pCameraImage[num], (size_t) COLS*ROWS*4 );

  i=0; j=0;
  do {
    realdump[j+0] = dump[i+2];
    realdump[j+1] = dump[i+1];
    realdump[j+2] = dump[i+0];
    i+=4; j+=3;
  } while (i<ROWS*COLS*4);

  fprintf(fp,"P6\n%d %d\n255\n",COLS,ROWS);
  if ( ROWS*COLS*3 != fwrite( realdump,sizeof(char),ROWS*COLS*3,fp ) ) {
    perror("write4toPPMimage");
    exit(-1);
  }
	
  fclose( fp );

  return 0;
}



/* ---------------------------------------------------------
 *
 * writePPM(char* rgbimage, char *filename ) writes rgbImage to ppm
 *  image. note, that the source data is RGB
 *
 * --------------------------------------------------------*/
int writePPM(char* image, char *filename ) {
  FILE *fp;

  /* image the current image into a .ppm file */
  fp = fopen( filename, "wb" );
  if ( !fp ) {
    perror("Error");
    return -1;
  }

  fprintf(fp,"P6\n%d %d\n255\n",COLS,ROWS);
  if ( ROWS*COLS*3 != fwrite( image,sizeof(char),ROWS*COLS*3,fp ) ) {
    perror("writePPMimage");
    exit(-1);
  }
	
  fclose( fp );

  return 0;
}


/* -----------------------------------------------------
 * parsePatternFile()
 * Parser routine for pattern files.
 * We assume a strict order of (laser, image) patterns which
 * is generated through a modified version of learn (io.c).
 * Analogously, getMarkerCoordinates is processed after
 * the image data is available, since it assumed to be last
 * within the (position, laser, image) pattern.
 *
 * See also dat.c, function int mem_load_patterns()
 *
 * -----------------------------------------------------*/

int parsePatternFile(char *filename, int startCount) {
  FILE  *iop;
  int   i, file_ended, error, size_i, size_j, n, m;
  char  name[MAX_STRING_LENGTH];
  char  s[MAX_STRING_LENGTH]; 
  char  systemCommand[MAX_STRING_LENGTH];
  char  text[MAX_STRING_LENGTH];
  char  command[MAX_STRING_LENGTH];  
  int   reading_pattern_set, reading_pattern;
  int   value;
  unsigned char *image;
  float float_value;
  float *lasers;
  int verbose = 0;
  struct timeval time;

  int imageCount = 0;
  int markerY;  
  int *xRes=&markerX, *yRes=&markerY;
  int *mAngle=&markerAngle;
  float *mDistance=&markerDistance;

  image    = (unsigned char *) malloc(sizeof(unsigned char) * ROWS * COLS * 3);
  lasers   = (float *) malloc(sizeof(float) * (NUM_LASER_SENSORS));

  if (image == NULL ||  lasers == NULL){
    fprintf(stderr, "ERROR: Out of memory in mem_load_patterns().\n");
    exit (-1);
  }


  fprintf(stderr, "Loading patterns from file %s...\n", filename);


  if ((iop = fopen(filename, "r")) == 0){
    fprintf(stderr, "ERROR: Could not open pattern file %s.\n", filename);
    return 0;
  }

  reading_pattern_set = 0;
  reading_pattern     = 0;
  n = 0, m = 0;


  for (file_ended = 0, error = 0; !file_ended && !error; ){
    if (fscanf(iop, "%s", command) == EOF)
      file_ended = 1;
    
    else{
      /*fprintf(stderr, "[%s]\n", command);*/


      /*
       * begin(patternset)
       */
      if (reading_pattern_set == 0 &&
	  reading_pattern == 0 &&
	  !strcmp(command, "begin(patternset)")){
	reading_pattern_set = 1;
	if (verbose)
	  fprintf(stderr, "begin(patternset)\n");
      }

      /*
       * name:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 0 &&
	       !strcmp(command, "name:")){
	if (fscanf(iop, "%s", &name[0]) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "set-name: %s\n", name);
	}
      }

      /*
       * type:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 0 &&
	       !strcmp(command, "type:")){
	if (fscanf(iop, "%s", &text[0]) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "type: %s\n", text);
	}
      }
      
      /*
       * begin(pattern)
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 0 &&
	       !strcmp(command, "begin(pattern)")){
	reading_pattern = 1;
	if (verbose)
	  fprintf(stderr, "begin(pattern)\n");
      }

      /*
       * name:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 1 &&
	       !strcmp(command, "name:")){
	if (fscanf(iop, "%s", &name[0]) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else {
	  if (verbose)
	    fprintf(stderr, "pat-name: %s\n", name);

	}
      }

      /*
       * time:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 1 &&
	       !strcmp(command, "time:")){
	if (fscanf(iop, "%ld", &(time.tv_sec)) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (fscanf(iop, "%ld", &(time.tv_usec)) == EOF){
	    fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	    error = 1;
	    file_ended = 1;
	  }
	  else{
	    if (verbose)
	      fprintf(stderr, "time: %ld %ld\n", time.tv_sec, time.tv_usec);
	  }
	}
      }


      /*
       * image:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 1 &&
	       !strcmp(command, "image")){
	if (fscanf(iop, "%d %d : ", &size_i, &size_j) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else {
	  if (verbose)
	    fprintf(stderr, "image: %d %d\n", size_i, size_j);
	  if (size_i != COLS || size_j != ROWS){
	    fprintf(stderr, 
		    "ERROR: image format mismatch (%d %d vs %d %d).\n", 
		    size_i, size_j, COLS, ROWS);
	    error = 1;
	  }
	  for (i = 0; i < size_i * size_j * 3 && !error; i++)
	    if (fscanf(iop, "%2x", &value) == EOF){
	      fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	      error = 1;
	      file_ended = 1;
	    }
	    else image[i] = (unsigned char) value;
	  
	  if ((!error) && imageCount >= startCount) {
	    fprintf(stderr,"Results for pattern %d.\n", imageCount);
	    if (fastGetMarkerCoordinates(image, 3, idecimation, xRes, yRes)) {
	      markerDetected = 1;
	      if (display && showMarker) insertMarker(image, *xRes, *yRes, 3);
	      getAngleAndDistance(*xRes, lasers, mAngle, mDistance);
	      fprintf(stderr, "camera-X: %d. camera-Y: %d\n", *xRes, *yRes);
	      fprintf(stderr, "Laser number: %d. Distance: %5.0f\n", markerAngle, markerDistance);	
	    } else { 
	      markerDetected = 0; 
	      fprintf(stderr, "No marker detected.\n");   
	    }
	    fprintf(stderr, "#position %d %d %d %5.0f\n", imageCount, markerDetected, markerAngle-90, markerDistance);
	    if (display) {
	      sprintf(&s[0], "pattern%03d.ppm", imageCount);
	      writePPM(image, "patternOut.ppm");
	      updateLaserDisplay(lasers,markerAngle);
	      EZX_Flush();	      	    
	      fprintf(stderr,"Press q on xv-window for next frame\n");
      	      system("xv patternOut.ppm -geometry 320x240+100+100");
	    }
	  }
	  imageCount++;
//	  if (imageCount > 70) exit(1);
	}
      }
      
      
      
      
      /*
       * lasers:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 1 &&
	       !strcmp(command, "lasers")){
	if (fscanf(iop, "%d %d : ", &size_i, &size_j) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else {
	  if (verbose)
	    fprintf(stderr, "lasers: %d %d\n", size_i, size_j);
	  if (size_i != NUM_LASER_SENSORS){
	    fprintf(stderr, 
		    "ERROR: infrared format mismatch (%d %d vs %d %d).\n", 
		    size_i, size_j,
		    NUM_LASER_SENSORS, NUM_LASER_SENSORS);
	    error = 1;
	  }
	  for (i = 0; i < size_i + size_j && !error; i++)
	    if (fscanf(iop, "%g", &float_value) == EOF) {
	      fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	      error = 1;
	      file_ended = 1;
	    }
	    else lasers[i] = float_value;
	}
      }
      
      
      
      /*
       * end(pattern)
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 1 &&
	       !strcmp(command, "end(pattern)")){
	reading_pattern = 0;
	n++;
	if (verbose) fprintf(stderr, "end(pattern)\n");
      }

      /*
       * end(patternset)
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 0 &&
	       !strcmp(command, "end(patternset)")){
	reading_pattern_set = 0;
	m++;
	if (verbose)
	  fprintf(stderr, "end(patternset)\n");
      }
      
    }
  }
  

  fclose(iop);
  free(lasers);
  free(image);

  return 1;
  
}







/* ---------------------------------------------------------
 *
 * detectMarker() is an adapted version of getAngleAndDistance
 * of robotDetect.c
 *
 * Since the image is constantly updated, the rectangle to
 * indicate the detection is not inserted into the image  
 * --------------------------------------------------------*/
void detectMarker() {
 int b;
 int *aPtr = &markerX;
 int *bPtr = &b;
 struct timeval TCX_waiting_time = {0, 0};
  
 fprintf(stderr,"Processing current image...\n");
 memcpy(image, pCameraImage[0], (size_t) COLS*ROWS*4 );

 if ((fastGetMarkerCoordinates(image, 4, idecimation, aPtr, bPtr))) {
   markerDetected = 1;
//   insertMarker(image, *aPtr, *bPtr, 4);
   fprintf(stderr, "camera-X: %d. camera-Y: %d\n", *aPtr, *bPtr);
   laserUpdated = 0;
   do {
     TCX_waiting_time.tv_sec  = 0;
     TCX_waiting_time.tv_usec = 0;
     tcxRecvLoop((void *) &TCX_waiting_time);
   } while (!laserUpdated);
   fprintf(stderr, "Laser number: %d. Distance: %5.0f\n", markerAngle, markerDistance);	
 } else { 
   fprintf(stderr, "No marker detected.\n"); 
   markerDetected = 0;
 }
 fprintf(stderr, "#position %d %d %5.0f\n", markerDetected, markerAngle-90, markerDistance);
}





/* -------------------------------------------------------------------------------
 * This helps us retrieve color info
 * ------------------------------------------------------------------------------- 
int displayStats();
int colorAnalysis() {

    if (XCheckMaskEvent(theDisplay, ButtonPressMask, &theEvent)) { 
      XButtonEvent theButtonEvent = theEvent.xbutton;
      int xx = theButtonEvent.x; 
      int yy = theButtonEvent.y; 
      theButtonEvent.button--;
  
      if ( theButtonEvent.window == w_id[window[0]]->w ) {
	if ( theButtonEvent.button == LEFT_BUTTON ) {
	  writePPM(image,"colorAnalysis.ppm")
	} else if ( theButtonEvent.button == RIGHT_BUTTON ) {
           displayStats();
	} else if ( theButtonEvent.button == MIDDLE_BUTTON ) {
	  fprintf( stderr,"Middle clicked at (%d,%d)\n", xx, yy );
	}
      } else if ( shmid_attached[1] && theButtonEvent.window == w_id[window[1]]->w ) {
	if ( theButtonEvent.button == LEFT_BUTTON ) {
	  write4toPPMimage( 1 );
	} else if ( theButtonEvent.button == RIGHT_BUTTON ) {
	  fprintf( stderr,"Right clicked at (%d,%d)\n", xx, yy );
	} else if ( theButtonEvent.button == MIDDLE_BUTTON ) {
	  fprintf( stderr,"Middle clicked at (%d,%d)\n", xx, yy );
	}
      }
    }
  }


 if ((display)) {

******
  convert!!! 3->4 YUCK!!! this really sucks.
*********
      displayImage4ByteColor( window[0], image, COLS, ROWS ); 
      EZX_Flush();
    }

 
  return 0;
}

*/

/* ---------------------------------------------------------
 *
 * DoTheJob() is a very bad name for a function which intercepts
 * tcx-messages to catch the server's reply to our cameraRequestShmId(0)
 * call (see this file's main()). It also updates the display window and
 * handles mouse button events. In particular, it triggers a
 * write4toPPMimage(0) when the left mouse button is pressed within the 
 * display window. And detectMarker() upon right button.
 * 
 * --------------------------------------------------------*/
int callDetectRobot() {

  struct timeval TCX_waiting_time = {0, 0};
  XEvent theEvent;
  
  TCX_waiting_time.tv_sec  = 0;
  TCX_waiting_time.tv_usec = 0;
  tcxRecvLoop((void *) &TCX_waiting_time);
  
  if  ( shmid_attached[0] || shmid_attached[1] ) {
    if (display) {
      displayImage4ByteColor( window[0], pCameraImage[0], COLS, ROWS ); 
      if (XCheckMaskEvent(theDisplay, ButtonPressMask, &theEvent)) { 
	XButtonEvent theButtonEvent = theEvent.xbutton;
	int xx = theButtonEvent.x; 
	int yy = theButtonEvent.y; 
	theButtonEvent.button--;
	/* boy, we really hope to have short circuit boolean evaluation */
	if ( shmid_attached[0] && theButtonEvent.window == w_id[window[0]]->w ) {
	  if ( theButtonEvent.button == LEFT_BUTTON ) {
	    write4toPPMimage( 0 );
	  } else if ( theButtonEvent.button == RIGHT_BUTTON ) {
	    detectMarker();
	  } else if ( theButtonEvent.button == MIDDLE_BUTTON ) {
	    fprintf( stderr,"Middle clicked at (%d,%d)\n", xx, yy );
	  }
	} else if ( shmid_attached[1] && theButtonEvent.window == w_id[window[1]]->w ) {
	  if ( theButtonEvent.button == LEFT_BUTTON ) {
	    write4toPPMimage( 1 );
	  } else if ( theButtonEvent.button == RIGHT_BUTTON ) {
	    fprintf( stderr,"Right clicked at (%d,%d)\n", xx, yy );
	  } else if ( theButtonEvent.button == MIDDLE_BUTTON ) {
	    fprintf( stderr,"Middle clicked at (%d,%d)\n", xx, yy );
	  }
	}
      }      
    } else detectMarker();    
  }
  
  return 0;
}

/* ---------------------------------------------------------
 *
 * displaybParameters() displays the values of various parameters
 * associated with bee/etc/beeSoft.ini.
 *
 * --------------------------------------------------------*/
int displaybParameters() {

  fprintf(stderr, "********  OPTIONS  **********\n");
  fprintf(stderr, "grabberConfig ......... = [%s.framegrabber]\n", grabberConfig );
  fprintf(stderr, "grabberType ........... = %s\n", grabberType );
  fprintf(stderr, "grabberWhichGrabber ... = %s\n", grabberWhichGrabber );
  fprintf(stderr, "grabberDev1 ........... = %s\n", grabberDev0 );
  fprintf(stderr, "grabberDev2 ........... = %s\n", grabberDev1 );
  fprintf(stderr, "decimation ............ = %d\n", idecimation );
  fprintf(stderr, "display... ............ = %d\n", display );
  fprintf(stderr, "*****************************\n");

  return 0;
}


/* -----------------------------------------------------------
 * check_commandline_parameters(int argc, char **argv)
 * --------------------------------------------------------*/
void check_commandline_parameters(int argc, char **argv) {
  int i, j, e, bug = 0;
  char *robotName = NULL;

  for (i = 1; i < argc && !bug; i++){
    for (j = 0, e = 0; j < (int) strlen(argv[i])-1; j++)
      if (argv[i][j] == '=')
	e = 1;
    if (!strcmp(argv[i], "-nodisplay") || !strcmp(argv[i], "-nd"))
      display = 0;
    else if (!strcmp(argv[i], "-file")  || !strcmp(argv[i], "-f")) {
      if (i == argc - 1){
	fprintf(stderr, "Need filename for -file\n");
	bug = 1;
      }
      else patternFileName = argv[++i];
    }
    else if (!strcmp(argv[i], "-start")  || !strcmp(argv[i], "-s")) {
      if (i == argc - 1){
	fprintf(stderr, "Need pattern number for -start\n");
	bug = 1;
      }
      else startPatternNumber = atoi(argv[++i]);
    }
    else if (!strcmp(argv[i], "-robot")) {
      if (i == argc - 1){
	fprintf(stderr, "Need argument for -robot\n");
	bug = 1;
      }
      else
	robotName = argv[++i];
    }
    else if (argv[i][0] != '-' || !e){
      fprintf(stderr, "Cannot parse argument: %s\n", argv[i]);
      bug = 1;
    }
  }

  if (robotName != NULL) {
    tcxSetModuleNameExtension( robotName);
    if (strstr(robotName, "obin")) 
      targetName = "marion";
    fprintf(stderr, "I am %s, looking for my friend %s.\n", robotName, targetName);    
  }
  else fprintf(stderr, "WARNING: please specify -robot {marion,robin} on command-line\n");
  
  if (bug) {
    fprintf(stderr, "Usage: '%s [-robot {marion, robin}] [-nodisplay] [-file <fname>] [-start <#>]\n", 
	    argv[0]);
    exit(1);
  }


}




TCX_REG_HND_TYPE ROBOT_DETECTION_handler_array[] = {
  {"ROBOT_DETECTION_register_auto_update",
   "ROBOT_DETECTION_register_auto_update_handler", /* will be ignored */
   ROBOT_DETECTION_register_auto_update_handler, TCX_RECV_ALL, NULL}
};


void localRegister() {

  int numberOfMessages;
  int numberOfHandlers;

  TCX_REG_MSG_TYPE messages[] = { ROBOT_DETECTION_messages }; 

#if ( defined(TCX_debug) )
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  numberOfMessages = sizeof(messages)/sizeof(TCX_REG_MSG_TYPE); 

  numberOfHandlers = sizeof(ROBOT_DETECTION_handler_array)
    /sizeof(TCX_REG_HND_TYPE); 

  registerInterface( "", 
		     numberOfMessages, 
		     messages,
		     numberOfHandlers, 
		     ROBOT_DETECTION_handler_array );

}






/* ---------------------------------------------------------
 *
 * main() starts with processing of parameters from beeSoft.ini
 * It has been modified to work with Qcam by commenting out Matrox-only
 * code.
 * Note that the the laser-software has an interrupt driven interface while
 * Qcam/SHM requires polling. The Rai-Scheduler has been adopted, it has
 * never been checked, however, if it is actually needed.
 *
 * TODO: fix parameter readings, add cmdLine options (e.g. -color)
 *        include Matrox-support
 * --------------------------------------------------------*/
int main( int argc, char** argv ) {
 struct timeval TCX_waiting_time = {0, 0};
  struct bParamList * params = NULL;
  const char *grabberDecimation;
  char dummy[256];
  
  pCameraImage[0] = NULL;
  pCameraImage[1] = NULL;
  shmid[0] = 0;
  shmid[1] = 0;
  shmid_attached[0] = 0;
  shmid_attached[1] = 0;
  window[0] = 0;
  window[1] = 0;
 

  /* add some parameter files */
  params = bParametersAddFile(params, "etc/beeSoft.ini");
  
  /* add some enviroment variables */
  params = bParametersAddEnv(params, "", "TCXHOST");

  /* add command line arguements */
  params = bParametersAddArray(params, "", argc, argv);

  /* Fill the global parameter structure */
  bParametersFillParams(params);

  /* find out the current grabber config in the robot 
     section should be one of { none, SICK }        */
  grabberConfig = bParametersGetParam( params, "robot", "framegrabber" );

  if ( !strcmp( grabberConfig, "none" ) ) {
    useGrabber = 0;
  } else {
    /* get variables from the section `[<grabberConfig>.framegrabber]' */
    sprintf( dummy, "%s.%s", grabberConfig, "framegrabber" );
    grabberType = bParametersGetParam( params, dummy, "type" );
//    grabberDev0 = bParametersGetParam( params, dummy, "dev1" );
//    grabberDev1 = bParametersGetParam( params, dummy, "dev2" );
    grabberWhichGrabber = bParametersGetParam( params, dummy, "usegrabber" );
//  displaybParameters();

    if ( !grabberType || !grabberWhichGrabber ) {
      fprintf( stderr,"Couldn't parse beeSoft.ini properly.\n");
      commShutdown("",NULL);
    }

    /*
    if ( strcmp( grabberType, "matrox-meteor" ) ) {
      fprintf( stderr,"I cannot handle framegrabbers other than the ");
      fprintf( stderr,"Matrox Meteor at this time.\n");
      commShutdown("",NULL);
    }
    */
    useGrabber = atoi( grabberWhichGrabber );
    if ( useGrabber<0 || useGrabber>3 ) {
      MISCERROR;
      return -1;
    }
  }
      grabberDecimation = bParametersGetParam( params, dummy, "decimation" );
      if(grabberDecimation) {
        idecimation = atoi( grabberDecimation );
        if(idecimation != 1 &&
           idecimation != 2 &&
           idecimation != 4) {
          
          fprintf( stderr,"Bad decimation value in beeSoft.ini: %s,"
                   " should be 1, 2, or 4, using 2.\n", grabberDecimation);
          idecimation = 2;
        }
      }
      else { /* no decimation specified, default to 2. */
        fprintf( stderr,"Warning: no decimation value set in beeSoft.ini."
                 "  Defaulting to 2.\n");
        grabberDecimation = "2";
        idecimation = 1;
      }

      /*
      for (i = 1; i<argc; i++) {
	if (!strcmp(argv[i], "-nodisplay") || !strcmp(argv[i], "-nd"))
	display = 0;
	else if (!strcmp(argv[i], "-start")  || !strcmp(argv[i], "-st")) {
	  if (i == argc - 1) {
	  fprintf(stderr, "Need pattern number [0..n] for -start\n");
	    bug = 1;
	    }
      else
      savingdelay = atof(argv[++i]);
      
      */

      check_commandline_parameters(argc, argv);
      displaybParameters();            
      if (display) 
	ezWindow = EZX_MakeWindow("lasersweep", LASERWINSIZE, LASERWINSIZE, "+600+140");
      
      if ( patternFileName) {
	fprintf(stderr, "Off-line processing from pattern file...\n");
	parsePatternFile(patternFileName, startPatternNumber);
	return 0;
      }   
      

      

      /* We need to connect to both laserServer and cameraServer. */

  localRegister();
  
  laserRegister();
  cameraRegister();
  
  /* close function called if */
  initClient( TCX_ROBOT_DETECTION_MODULE_NAME, commShutdown); /* close function called if server dies */

  cameraConnect( 1 );		/* 1 -> wait until connection has been established */
  laserConnect( 0 );	

  RaiInit();			/* init (but not start) scheduler   */
  catchInterrupts();
  initClientModules();		/* set up Rai modules to do         */
				/* communication for you            */

  
  /* whenever user hits CTRL-C */
  signal( SIGINT, &ctrlcShutdown );
  registerCameraShmIdCallback( myCameraShmIdCallback );
  
  if ( useGrabber&1 || useGrabber == 0 ) {
    cameraRequestShmId( 0 );
   fprintf(stderr, "shmReq\n");
    if (display) {
      window[0] = createDisplay( "color-qcam", COLS, ROWS, 100, 100 ); 
      /* Make private color map. */   
      if (EZX_LoadBestColorMap( w_id[CameraWindow[0]] ) == 0 ) {
	MISCERROR;
	return -1;
      }   
    }
  }

  if ( useGrabber&2 ) {
    cameraRequestShmId( 1 );
    window[1] = createDisplay( "window1", COLS, ROWS, 100+10+COLS, 100 ); 
  }
  fprintf( stderr,
	   "\n\n Left click into the window writes the current image into file\n\n");


  createLaserExampleModule();    /* Register with Rai-Scheduler */

  laserSubscribeSweep( 0, LASER_UPDATEFREQ );	/* 0 == first laser, i = request frequency   */
//  laserRequestSweep(0); 

#if (defined(sun))
  fprintf( stderr, "Sun is defined!");
#endif


  fprintf(stderr, "We enter Dothejob\n");
 
  for (;;) {      
    callDetectRobot();
    if ((markerDetected) && (multiLocalizeModule)) {     
      ROBOT_DETECTION_status_reply_type status;

      fprintf(stderr, "entering tcxSendMsg\n");
      status.robotDetected       = 1;
      status.distance            = markerDistance;
      status.distanceUncertainty = 0.0;
      status.angle               = markerAngle-90;   /* 24..0..-24 */
      status.angleUncertainty    = 0.0;
      status.nameOfDetectedRobot = targetName;
      
      tcxSendMsg( multiLocalizeModule, "ROBOT_DETECTION_status_reply", &status);      
      fprintf(stderr,"sending to module %s\n",tcxModuleName(multiLocalizeModule));
      markerDetected = 0;
    }    
    
    TCX_waiting_time.tv_sec  = 0;
    TCX_waiting_time.tv_usec = 0;
    tcxRecvLoop((void *) &TCX_waiting_time);   
  }
  
  if (display) {
    killDisplay( window[0] );
    EZX_EndWindow(ezWindow);
    killDisplay( window[1] );  
  }
  
  RaiStart();
  return 0;
}

/*
 * $Log: robotDetect.c,v $
 * Revision 1.1  2002/09/14 16:40:52  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1999/08/24 12:59:49  schulz
 * Inserted int before definition of laserUpdated (again)
 *
 * Revision 1.2  1999/04/28 21:31:32  fox
 * Demo version.
 *
 * Revision 1.12  1998/09/18 16:04:18  thrun
 * .
 *
 * Revision 1.11  1998/01/14 16:44:23  swa
 * Changed clientCloseFcn to take two arguments, the name of the module
 * that died and the TCX_MODULE_PTR module. Now one can distinguish in the
 * close function between modules that die.
 *
 * Revision 1.10  1997/10/04 01:06:47  swa
 * Fixed some bugs and inconsistencies wrt to both frame grabbers.
 *
 * Revision 1.9  1997/10/04 00:13:09  swa
 * First working version of the cameraServer that supports two (count'em
 * two!) framegrabbers.  Although the server seems to work just fine, it
 * has not yet been fully tested.
 *
 * Revision 1.8  1997/07/24 16:31:07  swa
 * Forgot to note, that the example program now dies when the cameraServer
 * dies.
 *
 * Revision 1.7  1997/07/24 16:28:36  swa
 * Removed .png stuff, Left click on the window in the example program
 * now saves a .ppm file, which can be read using xv. The order of the
 * bytes is surprisingly BGRX and NOT RGBX; i.e. all other algorithms
 * that access the data have to pay attention to that.
 *
 * Revision 1.6  1997/07/24 00:54:25  swa
 * Fixed some minor bugs, extended the README file and tested the version. The
 * cameraAttachExample does not yet die when the cameraServer dies. Will be
 * fixed in future versions.
 *
 * Revision 1.5  1997/07/22 22:47:17  swa
 * Added file handling (reading/saving). Renamed two (three?) functions to
 * be consistent.
 *
 * Revision 1.4  1997/07/04 17:28:31  swa
 * Left and right mouseclicks have been added to the example-program. Left
 * mouse saves the current cameraimage to a 4 byte .ppm format, right mouse
 * shows the current coordinates.
 *
 * Revision 1.3  1997/06/30 00:14:33  thrun
 * This new version allows saving of images (in .ppm 4-(four!)-byte format), by
 * left-clicking in cameraAttachExample's window . The cameraServer can then
 * later read that image and display it (instead of using the framegrabber). (swa)
 *
 * Revision 1.2  1997/06/25 18:46:50  thrun
 * Added support for tcx messages. It is for mainatt no longer necessary to
 * use argv[1]. It now sends out messages. (swa)
 *
 * Revision 1.1  1997/06/24 16:02:51  thrun
 * shared memory.
 *
 *
 */
