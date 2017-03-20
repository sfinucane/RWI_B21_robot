#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>

#include "evaluate.h"
#include "inline.h"
#include "localize.h"

static int verbose = FALSE;

static void
interpolate( float time,
	     float t1, float x1, float y1, float rot1,
	     float t2, float x2, float y2, float rot2,
	     float* x, float* y, float* rot);
	     
	    
static FILE* 
openScript( char *fileName, int origScript,
	    double* time,
	    float* x, float* y, float* rot,
	    double *nextTime, float* nextX, float* nextY, float* nextRot);

static FILE* 
openCorrectionParameters( char *fileName, float* time,
			  float* corrX, float* corrY,
			  float* corrRot, int* corrType,
			  float* nextTime,
			  float* nextCorrX, float* nextCorrY,
			  float* nextCorrRot, int* nextCorrType);

static int
getNextScriptPosition( FILE* script, int origScript, double* time,
		       float* x, float* y, float* rot);

static int
getNextCorrectionParameters( FILE* correctionFile, float* time,
			     float* corrX, float* corrY, float* corrRot, int* corrType);


#define U_GAMMA 1.645
static float
distance( float x1, float y1, float x2, float y2) {
  return sqrt( fSqr( x1 - x2) + fSqr( y1 - y2));
}


float
tValueForSampleSize( int N)
{
  float tTable[15] = {6.314, 2.920, 2.353, 2.132, 2.015, 1.943, 1.895, 1.860,
		      1.833, 1.812, 1.796, 1.782, 1.771, 1.761};
  float t;
  
  if ( N < 15) {
    t = tTable[N-1];
  }
  else {
    /* Peizer / Pratt approximation. See page 892 in Hartung/Statistik. */
    float c = (N - (5.0/6.0)) / ( fSqr( N - 2.0/3.0 + 1 / (10*N)));
    t = sqrt( N * exp( c * (U_GAMMA * U_GAMMA)) - N);
  }
  return t;
}

int
getNextValues( FILE* fp, float* values, int numberOfColumns,
	       int columnForXValue, float* time)
{
  int column, dataRead = TRUE;
  char c;

  /* Just get the data. */
  for (column = 0; column < numberOfColumns && dataRead; column++) 
    dataRead = ( fscanf(fp, "%f", &values[column]) == 1);

  /*#define DDD */
#ifdef DDD
  if (values[1] < 150.0)
      values[1] = 1.0;
    else
      values[1] = 0.0;
    
  if (values[2] > 0.5)
      values[2] = 1.0;
    else
      values[2] = 0.0;

  if (values[3] < 100.0)
      values[3] = 1.0;
    else
      values[3] = 0.0;
    
#endif
    
  if ( time != NULL)
    *time = values[columnForXValue];

  /* Read till end of line. */
  while (((c = fgetc(fp)) != '\n') && !feof(fp));
  
  return dataRead;
}


int
getInterpolatedValues( FILE* fp,
		       float* values, int numberOfColumns,
		       int columnForXValue, float resolution,
		       float* prevTime, int* useAble)
{
  int column, dataRead = TRUE;
  char c;

  /* Create a file only with the positions. */
  static float prevColumnValue[MAX_NUMBER_OF_COLUMNS];
  static float columnValue[MAX_NUMBER_OF_COLUMNS];
  static float prevScriptTime = 0.0, nextTime = 0.0, scriptTime = 0.0;
  static int firstTime = TRUE;
  float timeInterval;
  
  nextTime = *prevTime + resolution;
  *useAble = TRUE;

  /* Initialize the values. Get the first two lines. */
  if ( firstTime) {
    firstTime = FALSE;
    
    for (column = 0; column < numberOfColumns && dataRead; column++)
      dataRead = ( fscanf(fp, "%f", &prevColumnValue[column]) == 1);
    prevScriptTime = prevColumnValue[columnForXValue];
    while (((c = fgetc(fp)) != '\n') && !feof(fp));
    
    for (column = 0; column < numberOfColumns && dataRead; column++)
      dataRead = ( fscanf(fp, "%f", &columnValue[column]) == 1);
    scriptTime = columnValue[columnForXValue];
    while (((c = fgetc(fp)) != '\n') && !feof(fp));
    
    /* Set the start time of the script. */
    while ( prevScriptTime > nextTime)
      nextTime += resolution;
  }
  
  if (verbose) fprintf(stderr, "scr %f %f %f \n", scriptTime, prevScriptTime, nextTime);

  /* Now search for next timestamp higher than nextTime. */
  while ( scriptTime < nextTime) {
    
    /* Store previous line for interpolation. */
    for (column = 0; column < numberOfColumns && dataRead; column++)
      prevColumnValue[column] = columnValue[column];
    prevScriptTime = prevColumnValue[columnForXValue];
    
    /* Get next line. */
    for (column = 0; column < numberOfColumns && dataRead; column++)
      dataRead = ( fscanf(fp, "%f", &columnValue[column]) == 1);

    if ( ! dataRead) {
      firstTime = TRUE;
      return FALSE;
    }
    else
      scriptTime = columnValue[columnForXValue];
    
    while (((c = fgetc(fp)) != '\n') && !feof(fp));
  }
  
  /* Found a time */
  timeInterval = scriptTime - prevScriptTime;
  
  /* If the time interval in the data is too big don't use it. */
  if ( distance( columnValue[X_COLUMN], columnValue[Y_COLUMN],
		 prevColumnValue[X_COLUMN], prevColumnValue[Y_COLUMN])
       < MAX_INTERPOLATION_DIST)
    
    /* Interpolate between prev values and current values. */
    interpolate( nextTime,
		 prevScriptTime,
		 prevColumnValue[X_COLUMN], prevColumnValue[Y_COLUMN],
		 prevColumnValue[ROT_COLUMN], 
		 scriptTime,
		 columnValue[X_COLUMN], columnValue[Y_COLUMN],
		 columnValue[ROT_COLUMN],
		 &(values[X_COLUMN]), &(values[Y_COLUMN]), &(values[ROT_COLUMN]));
  else
    *useAble = FALSE;
  
  *prevTime = nextTime;
  
  return dataRead;
}


int
getReferenceValues( FILE* fp, float* values, float* stdDevs,
		    int numberOfColumns, float time, int readStdDevs)
{
  int column, dataRead = TRUE;
  int numberOfReferenceColumns;
  
  static float prevColumnValue[MAX_NUMBER_OF_COLUMNS];
  static float columnValue[MAX_NUMBER_OF_COLUMNS];
  static float prevScriptTime = -1.0, scriptTime = 0.0;
  static int firstTime = TRUE;
  float weight, timeInterval;
  int refColumn;
  char c;

  if ( readStdDevs)
    numberOfReferenceColumns = 2 * numberOfColumns+1;
  else
  numberOfReferenceColumns = numberOfColumns;

  /* Initialize the values. Get the first two lines. */
  if ( firstTime) {
    
    for (column = 0; column < numberOfReferenceColumns && dataRead; column++) {
      dataRead = ( fscanf(fp, "%f", &prevColumnValue[column]) == 1);
      if (0) fprintf(stderr, "%d %f\n", column, prevColumnValue[column]);
    }
    
    while (((c = fgetc(fp)) != '\n') && !feof(fp));
    prevScriptTime = prevColumnValue[0];
      
    for (column = 0; column < numberOfReferenceColumns && dataRead; column++) {
      dataRead = ( fscanf(fp, "%f", &columnValue[column]) == 1);
      if (0) fprintf(stderr, "%d %f\n", column, columnValue[column]);
    }
    while (((c = fgetc(fp)) != '\n') && !feof(fp));

    scriptTime = columnValue[0];

    firstTime = FALSE;
  }

  /* Now search for next timestamp higher than time. */
  while ( scriptTime < time) {
    
    /* Store previous line for interpolation. */
    for (column = 0; column < numberOfReferenceColumns; column++)
      prevColumnValue[column] = columnValue[column];
    prevScriptTime = prevColumnValue[0];
    
    /* Get next line. */
    for (column = 0; column < numberOfReferenceColumns && dataRead; column++) {
      dataRead = ( fscanf(fp, "%f", &columnValue[column]) == 1);
    }
    while (((c = fgetc(fp)) != '\n') && !feof(fp));
      
    if ( ! dataRead) {
      return FALSE;
    }
    else
      scriptTime = columnValue[0];
  }
  
  /* Found a time */
  timeInterval = scriptTime - prevScriptTime;

  /* If the time interval in the data is too big don't use it. */
  if ( distance( columnValue[X_COLUMN], columnValue[Y_COLUMN],
		 prevColumnValue[X_COLUMN], prevColumnValue[Y_COLUMN])
       > MAX_INTERPOLATION_DIST)
    return FALSE;
  
  /* Interpolate between prev values and current values. */
  weight = (time - prevScriptTime) / timeInterval;
  
  if ( weight < 0.0)
    return FALSE;
  
  /* Get the interpolated values and standard deviations. */
  for (column = 0; column < numberOfColumns; column++) {
    
    values[column] = prevColumnValue[column] + weight *
      (columnValue[column] - prevColumnValue[column]);
    
    if ( stdDevs != NULL) {
      if ( column > 0) {
	
	refColumn = column + numberOfColumns - 1;
	
	stdDevs[column] = prevColumnValue[refColumn] + weight *
	  (columnValue[refColumn] - prevColumnValue[refColumn]);
      }
    }
  }
  
  /* Compute the combined standard deviation of x and y value and store it in
     * the zero field. */
  if ( stdDevs != NULL) {
    stdDevs[COMBINED_STDDEV_COLUMN] =
      prevColumnValue[numberOfReferenceColumns-1] + weight *
      (columnValue[numberOfReferenceColumns-1] -
       prevColumnValue[numberOfReferenceColumns-1]);
  }

/* printf( "%f %f %f %f #ref\n", time, values[X_COLUMN], values[Y_COLUMN], values[ROT_COLUMN]); */
  return TRUE;
}


int
getCorrectedReferenceValues( char* scriptFileName, int origScript,
			     char* correctionFileName,
			     float* values, float time)
{
  static double scriptStartTime, scriptTime, nextScriptTime;

  static float corrX, corrY, corrRot;
  static float nextCorrX, nextCorrY, nextCorrRot;
  static float correctionTime, nextCorrectionTime;

  static int corrType, nextCorrType;
  static float x, y, rot;
  static float nextX, nextY, nextRot;
  static FILE *sFile, *cFile;
  static int noMoreCorrectionParameters = FALSE;
  
  static int firstTime = TRUE;

  if ( 1 && noMoreCorrectionParameters) {
    fprintf( stderr, "Last correction parameters reached. Stop!\n");
    exit(0);
    firstTime = TRUE;
    noMoreCorrectionParameters = FALSE;
    return FALSE;
  }
  
  /* Initialize the values. Get the first two lines. */
  if ( firstTime) {

    sFile = openScript( scriptFileName, origScript, 
			&scriptStartTime, &x, &y, &rot,
			&nextScriptTime, &nextX, &nextY, &nextRot);
    
    scriptTime = 0.0;
    nextScriptTime -= scriptStartTime;
    
    cFile = openCorrectionParameters( correctionFileName, &correctionTime,
				      &corrX, &corrY, &corrRot, &corrType,
				      &nextCorrectionTime,
				      &nextCorrX, &nextCorrY, &nextCorrRot, &nextCorrType);
    firstTime = FALSE;
  }

  /* Search for the next interval in the script. */
  while( time > nextScriptTime) {

    /* Copy the old values. */
    scriptTime = nextScriptTime;
    x = nextX; y = nextY; rot = nextRot;
    
    /* Get new ones. */
    if ( ! getNextScriptPosition( sFile, origScript, &nextScriptTime,
				  &nextX, &nextY, &nextRot)) {
      fprintf( stderr, "Script ended %f %f.\n", time, scriptTime);
      return FALSE;
    }
    nextScriptTime -= scriptStartTime;
  }

  /* Search for the next correction parameters. */
  while( ! noMoreCorrectionParameters &&
	 (time > nextCorrectionTime)) {
    
    /* Copy the old values. */
    correctionTime = nextCorrectionTime;
    corrX = nextCorrX; corrY = nextCorrY;
    corrRot = nextCorrRot; corrType = nextCorrType;
    
    /* Get new ones. */
    if ( ! getNextCorrectionParameters( cFile, &nextCorrectionTime,
					&nextCorrX, &nextCorrY,
					&nextCorrRot, &nextCorrType)) {
      fprintf( stderr, "Final correction parameters %f.\n", correctionTime);
      noMoreCorrectionParameters = TRUE;
    }
  }
  

  /* Now we got the position interval and the correction parameters. */
  values[0] = scriptTime;
  
  if (0)
    fprintf(stderr, "%f [%f %f]\n", time, scriptTime, nextScriptTime); 
  
  robotCoordinates2MapCoordinates( x, y, rot,
				   corrX, corrY, corrRot, corrType,
				   &(values[X_COLUMN]), &(values[Y_COLUMN]), &(values[ROT_COLUMN]));
  
  /*    printf( "%f %f %f %f #ref\n", time, values[X_COLUMN], values[Y_COLUMN], values[ROT_COLUMN]); */

  return TRUE;
}


static FILE* 
openScript( char *fileName, int origScript, double* time,
	    float* x, float* y, float* rot,
	    double* nextTime, float* nextX, float* nextY, float* nextRot)
{
  FILE* fp;

#define BUFFLEN 255
#define ORIG_SCRIPTPOSMARK "#ROBOT"
#define ORIG_TIMEMARK "@SENS"

#define NEW_SCRIPTPOSMARK "position:"
#define NEW_TIMEMARK "time:"

  
  /* try to open file */
  if ( strncmp( fileName, "STDIN", 5) != 0) {
    if ((fp = fopen( fileName,"rt")) == NULL) {
      fprintf(stderr,"ERROR: Could not open file '%s'!\n",fileName);
      exit(0);
    }
  }
  else
    fp=stdin;
  
  /* check file, load start data */
  if ( ! getNextScriptPosition( fp, origScript, time, x, y, rot)) {
    fprintf(stderr, "Error %s is no script file.\n", fileName);
    fclose(fp);
    exit(0);
  }

  if ( ! getNextScriptPosition( fp, origScript,
				nextTime, 
				nextX, nextY, nextRot)) {
    fprintf(stderr, "Only one line in %s.\n", fileName);
    fclose(fp);
    exit(0);
  }

  fprintf( stderr, "# Script starts with X:%.2f  Y:%.2f  Rot:%.2f\n", *x,  *y, *rot);
  return(fp);
}


static FILE* 
openCorrectionParameters( char *fileName, float* time,
			  float* corrX, float* corrY,
			  float* corrRot, int* corrType,
			  float* nextTime,
			  float* nextCorrX, float* nextCorrY,
			  float* nextCorrRot, int* nextCorrType)
{
  FILE* fp;

  /* try to open file */
  if ((fp = fopen( fileName,"rt")) == NULL) {
    fprintf(stderr,"ERROR: Could not open file '%s'!\n",fileName);
    exit(0);
  }

  /* check file, load start data */
  if ( ! getNextCorrectionParameters( fp, time, corrX, corrY, corrRot, corrType)) {
    fprintf(stderr, "Error %s is no correction file.\n", fileName);
    fclose(fp);
    exit(0);
  }
    
  if ( ! getNextCorrectionParameters( fp, nextTime, nextCorrX, nextCorrY,
				      nextCorrRot, nextCorrType)) {
    fprintf(stderr, "Oops. Only one correction parameter.\n");
    *nextTime = 1e20;
  }
  fprintf(stderr, "Correction starts at %f with %f %f %f %d.\n", *time,
	  *corrX, *corrY, *corrRot, *corrType);
  
  return fp;
}


static int
getNextScriptPosition( FILE* fp, int origScript, double* scriptTime,
		       float* x, float* y, float* rot)
{
  int timeRead = FALSE;
  int positionRead = FALSE;
  int hour, minute;
  float second;
  int eof = FALSE;
  char line[BUFFLEN];

  /* check file, load start data */
  while (!eof && !(timeRead && positionRead)){

    if (!(eof = (fgets(line,BUFFLEN,fp) == NULL))){

      if ( origScript) {
	if (( strncmp(line,ORIG_SCRIPTPOSMARK,
		      strlen(ORIG_SCRIPTPOSMARK)) == 0)) {
	  
	  if (sscanf(&line[strlen(ORIG_SCRIPTPOSMARK)],"%f %f %f", x, y, rot) == 3)
	    positionRead = TRUE;
	}
	if (strncmp(line, ORIG_TIMEMARK, strlen(ORIG_TIMEMARK)) == 0) {
	  if (sscanf(&line[strlen(ORIG_TIMEMARK)+9], "%d:%d:%f",
		     &hour, &minute, &second) == 3) {
	    timeRead = TRUE;
	    *scriptTime = hour * 3600 + minute * 60 + second;
	  }
	}
      }
      else {
	if (( strncmp(line,NEW_SCRIPTPOSMARK,
		      strlen(NEW_SCRIPTPOSMARK)) == 0)) {
	  
	  if (sscanf(&line[strlen(NEW_SCRIPTPOSMARK)],"%f %f %f", x, y, rot) == 3)
	    positionRead = TRUE;
	}
	if (strncmp(line, NEW_TIMEMARK, strlen(NEW_TIMEMARK)) == 0) {
	  long secs, usecs;
	  if (sscanf(&line[strlen(NEW_TIMEMARK)], "%ld %ld",
		     &secs, &usecs) == 2){
	    timeRead = TRUE;
	    *scriptTime = secs + usecs/1000000.0;
	  }
	}
      }
    }
  }

  /* #define CORRECTION */
#ifdef CORRECTION
#define CORRECTION_FACTOR 1.024609
  *x *= CORRECTION_FACTOR;  
  *y *= CORRECTION_FACTOR;  
#endif
  *rot = 90.0 - *rot; 
 
  return timeRead && positionRead;
}

static int
getNextCorrectionParameters( FILE* fp, float* time,
			     float* corrX, float* corrY, float* corrRot, int* corrType)
{
  char line[BUFFLEN];
  
  while ( TRUE) {
    if ( fgets(line,BUFFLEN,fp) != NULL){
      if ( sscanf( line, "%f %f %f %f %d", time, corrX, corrY, corrRot, corrType) < 5) {
	fprintf(stderr, "Error no correction file.\n");
	fclose(fp);
	return FALSE;
      }
      else 
	return TRUE;
    }
    else
      return FALSE;
  }
}


static void
interpolate( float time,
	     float t1, float x1, float y1, float rot1,
	     float t2, float x2, float y2, float rot2,
	     float* x, float* y, float* rot)
{
  float weight = (time - t1) / (t2 - t1);
  float rotDist = rot2 - rot1;

  *x = x1 + weight * ( x2 - x1);
  *y = y1 + weight * ( y2 - y1);
  
  if ( rotDist > 180.0)
    *rot = rot1 + weight * (rot2 - 360.0 - rot1);
  else if ( rotDist < -180.0)
    *rot = rot1 + weight * ( rot2 + 360.0 - rot1);
  else
    *rot = rot1 + weight * ( rot2 - rot1);
}
