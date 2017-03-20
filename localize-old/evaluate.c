
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>

#include "evaluate.h"
#include "inline.h"

int numberOfBumpsToBeCounted = 0;

int verbose = FALSE;

#define BUFFLEN 256

#define MIN_RECOVER_TIME 30.0

#define MAX_TIME_FROM_BUMP 120.0

static float
rad2Deg(float x)
{
  return x * 57.29578;
}


static int
checkIfPositionLost( float time,
		     float xyDist, float rotDist,
		     float xyDistForLost, float rotDistForLost, float timeForLost,
		     int bumpOccured, 
		     float* recoverTime, float* bumpRecoverTime,
		     char* gnuFileName, float lostHeight, int endReached)
{
  static float intervalStart = 0.0, lastBumpEvent = -1000.0;
  static float intervalEnd = 0.0;
  static int currentlyLost = FALSE, recoverFromBump = FALSE;
  static int numberOfLostReports = 0;
    
  static FILE *fp = NULL;
  static int firstTime = TRUE;
  static int bumpCnt = 0, numberOfBumpToBeRecovered;
  
  int bumpShouldBeCounted = FALSE;

  if ( firstTime) {
    firstTime = FALSE;
    if ( gnuFileName != NULL) {
      if ((fp = fopen( gnuFileName,"w")) == NULL) {
	fprintf(stderr,"ERROR: Could not open gnu file '%s'!\n",
		gnuFileName);
	exit(0);
      }
    }
  }

  /* ---------------------------------------------------------------------
   * If we reached the end of the script and the position is not recovered.
   * --------------------------------------------------------------------- */
  if ( endReached) {

    if ( currentlyLost) {
      
      if ( fp) {
	fprintf( fp, "%f %f\n", intervalStart - 1, 0.0);
	fprintf( fp, "%f %f\n", intervalStart + 1, lostHeight);
	fprintf( fp, "%f %f\n", time - 1, lostHeight);
	fprintf( fp, "%f %f\n", time + 1, 0.0);
      }
      
      if ( recoverFromBump) {
	*bumpRecoverTime += time - lastBumpEvent;
	printf( "\n %d %.1f #from bump %.1f %.1f %.1f %.1f\n",
		numberOfBumpToBeRecovered,
		time - lastBumpEvent,
		time, lastBumpEvent, *bumpRecoverTime,
		intervalStart - lastBumpEvent);
      }
      else {
	*recoverTime += time - intervalStart;
	printf( "\n#Was lost %.1f since %.1f for %.1f secs.\n",
		time, intervalStart, time - intervalStart);
      }
    }
  }
  
  /* Normalize the rotation distance. */
  if ( rotDist > 180.0)
    rotDist = 360.0 - rotDist;
  
  
  /* ---------------------------------------------------------------------
   * If a bump occured we have to check whether the robot is already lost
   * due to another event. 
   * --------------------------------------------------------------------- */
  if ( bumpOccured) {
    bumpCnt++;
    bumpShouldBeCounted = TRUE;
    numberOfBumpsToBeCounted++;
    
    printf( "#Bump %d %.1f: ", bumpCnt, time);
    
    /* If the robot is already lost due to an earlier bump we reset the
     * corresponding times. */
    //#define OLD
#ifdef OLD
    if ( recoverFromBump) {
      printf( "#Bump %d %.1f  but already lost due to earlier bump %f.\n",
	      bumpCnt, time, lastBumpEvent);
      bumpShouldBeCounted = FALSE;
      /* We don't account for overlayed bumps. */
      recoverFromBump = FALSE;
    }
#else
    if (0) {
    }
#endif
    
    /* The robot is lost due to something else. Don't add this time to the
     * recover time. */
    else if ( currentlyLost) {
      if (1 || time - intervalStart < 60) {
	printf( "Close (%.1f secs) to position loss. ", time - intervalStart);
	if ( recoverFromBump) {
	  printf("#Previous bump should not be counted.\n");
	  numberOfBumpsToBeCounted--;
	}
	intervalStart = time;
	recoverFromBump = TRUE;
	numberOfBumpToBeRecovered = bumpCnt;
	lastBumpEvent = intervalStart;
	currentlyLost = FALSE;
      }
      else
	printf( "#Bump %d %.1f  but already lost since %f secs.\n",
		bumpCnt, time, time - intervalStart);
      numberOfBumpToBeRecovered = bumpCnt;
      lastBumpEvent = intervalStart;
#ifndef OLD
      lastBumpEvent = time;
#endif
    }
    
    else {
      printf( "\n#Bump %d %.1f.\n", bumpCnt, time);
      lastBumpEvent = time;
      numberOfBumpToBeRecovered = bumpCnt;
    }
  }

  if ( currentlyLost) {

    /* ---------------------------------------------------------------------
     * If the robot has recovered the position we dump the corresponding
     * information.
     * --------------------------------------------------------------------- */
    if ( xyDist < xyDistForLost && rotDist < rotDistForLost) {

      /* Check whether the robot has recovered for at least MIN_RECOVERED_TIME
       * secs. */
      if ( intervalEnd > intervalStart) {
	
	if ( ((time - intervalEnd) > MIN_RECOVER_TIME)) {
	  
	  /* OK! The robot has recovered but we don't account for failures
	   * shorter than MIN_INTERVAL_SIZE. */
	  if ( ((intervalEnd - intervalStart) > timeForLost)
	       && numberOfLostReports > 2) {
	    
	    if ( fp) {
	      fprintf( fp, "%f %f\n", intervalStart - 1, 0.0);
	      fprintf( fp, "%f %f\n", intervalStart + 1, lostHeight);
	      fprintf( fp, "%f %f\n", intervalEnd - 1, lostHeight);
	      fprintf( fp, "%f %f\n", intervalEnd + 1, 0.0);
	    }

	    if ( recoverFromBump) {
	      *bumpRecoverTime += intervalEnd - lastBumpEvent;
	      printf( "\n%d %f #from bump %.1f %.1f %.1f %.1f\n",
		      numberOfBumpToBeRecovered,
		      intervalEnd - lastBumpEvent,
		      intervalEnd, lastBumpEvent, *bumpRecoverTime,
		      intervalStart - lastBumpEvent);
	    }
	    else {
	      *recoverTime += intervalEnd - intervalStart;
	      printf( "#Was lost %.1f since %.1f for %.1f secs.\n",
		      intervalEnd, intervalStart, intervalEnd - intervalStart);
	    }
	  }
	  currentlyLost = FALSE;
	  recoverFromBump = FALSE;
	}
      }
      else
	intervalEnd = time;
    }
    else {
      numberOfLostReports++;
      intervalEnd = 0.0;
    }
  }

  /* The robot is lost for the first time. */
  else if ( xyDist > xyDistForLost || rotDist > rotDistForLost) {
    currentlyLost = TRUE;
    intervalStart = time;
    numberOfLostReports = 1;

    /* Is it caused by a bump? */
    if ( time - lastBumpEvent < 60.0)
      recoverFromBump = TRUE;
  }

  return bumpShouldBeCounted;
}


static int
checkIfBumpOccured( float time, char* bumpFileName)
{
  static int checkPossible = TRUE;

  if ( checkPossible) {

    char line[BUFFLEN];
    static float bumpTime;
    static int lastBumpExpired = TRUE;
    static FILE *bumpFp;
    static int firstTime = TRUE;

    if ( firstTime) {

      firstTime = FALSE;
      
      if ( bumpFileName != NULL) {
	if ((bumpFp = fopen( bumpFileName,"r")) == NULL) {
	  fprintf(stderr,"ERROR: Could not open bump file '%s'!\n",
		  bumpFileName);
	  checkPossible = FALSE;
	}
	else
	  fprintf(stderr, "Extracting bump events from %s.\n", bumpFileName);
      }
      else
	checkPossible = FALSE;
      
      if ( ! checkPossible)
	return FALSE;
    }
    
    /* Read the next line if last bump is expired. */
    if ( lastBumpExpired) {
      static int first = TRUE;

      if ( 0 && first) {
	first = FALSE;
	lastBumpExpired = FALSE;
	return TRUE;
      }
      
      if ( fgets( line, BUFFLEN, bumpFp) != NULL) {
	
	/* Extract information from line. */
	sscanf( line, "%f", &bumpTime);
	if (1 || verbose)
	  fprintf( stderr, "Next bump after %f secs.\n", bumpTime);
      }
      else {
	checkPossible = FALSE;
      }
      
      lastBumpExpired = FALSE;
    }
    else if ( time >= bumpTime) {
      lastBumpExpired = TRUE;
      if (1 || verbose)
	fprintf( stderr, "Found bump after %f secs.\n", time);
      return TRUE;
    }
  }
  return FALSE;
}


//#define READ_DIST
#ifndef READ_DIST

int
main(argc, argv)
     int argc;
     char *argv[];
{

  FILE *fixedFp,*interpolateFp;
  char *fixedFileName=NULL, *interpolateFileName=NULL;
  char *bumpFileName=NULL, *gnuFileName=NULL;
  char c;
  
  float time = 0.0, xyDist, xyDistFactor, lostHeight = 1000.0;
  int   argCount;
  int numberOfColumns = 4;
  int column;
  int considerBumps = FALSE, bumpOccured = FALSE;
  int numberOfBumps = 0; //, numberOfBumpsToBeCounted = 0;
  int useMarker = TRUE, useCorrection = FALSE;
  int dontReadFixedAtAll = FALSE, origScript = TRUE;
  
  float recoverTime = 0.0, bumpRecoverTime = 0.0;
  
  float values[MAX_NUMBER_OF_COLUMNS];
  float refValues[MAX_NUMBER_OF_COLUMNS];
  float refStdDevs[MAX_NUMBER_OF_COLUMNS];
  float distances[MAX_NUMBER_OF_COLUMNS];

  float xyDistForLost = 50.0;
  float rotDistForLost = 5.0;
  float timeForLost = 30.0;
  
  if (argc < 3) {
    fprintf(stderr, "usage -- for MARKER, SCRIPT, and CORR file:\n\t");
    fprintf( stderr, "cat SCRIPT | %s -int CORR -corr -fix MARKER\n\n",
	     argv[0]);

    fprintf(stderr, "usage -- for SCRIPT, and CORR file:\n\t");
    fprintf( stderr, "cat SCRIPT | %s -int CORR -corr -nomarker\n\n",
	     argv[0]);

    fprintf( stderr, "usage -- for MARKER and MAX file:\n\t");
    fprintf( stderr, "%s -int MAX -nomarker -fix MARKER\n\n", argv[0]);

    fprintf(stderr, "usage -- for REFERENCE and MAX file:\n\t");
    fprintf( stderr, "%s -int REFERENCE -fix MAX\n\t\t[-b bumpFile -g gnuFile -l lostHeight -xy xyDistForLost -rot rotDistForLost -t timeForLost]\n", argv[0]);

    fprintf( stderr, "Use option -new for new script type.\n");
    exit(0);
  }      

  for (argCount=1; argCount<argc; argCount++) {

    if ((strcmp(argv[argCount],"-int")==0)) {
      if ( argCount < argc - 1) {
	interpolateFileName =  argv[++argCount];
	fprintf( stderr, "Interpolate file: %s.\n", interpolateFileName);
      }
      else {
	fprintf( stderr, "ERROR: name must follow keyword -int.\n");
	exit;
      }
    }
    else if ((strcmp(argv[argCount],"-fix")==0)) {
      if ( argCount < argc - 1)
	fixedFileName = argv[++argCount];
      else {
	fprintf( stderr, "ERROR: name must follow keyword -fix.\n");
	exit;
      }
    }
    else if ((strcmp(argv[argCount],"-g")==0)) {
      if ( argCount < argc - 1) {
	gnuFileName = argv[++argCount];
      }
      else {
	fprintf( stderr, "ERROR: name must follow keyword -g.\n");
	exit;
      }
    }
    else if ((strcmp(argv[argCount],"-b")==0)) {
      if ( argCount < argc - 1) {
	bumpFileName = argv[++argCount];
	considerBumps = TRUE;
      }
      else {
	fprintf( stderr, "ERROR: name must follow keyword -b.\n");
	exit;
      }
    }
    else if ((strcmp(argv[argCount],"-l")==0)) {
      if ( argCount < argc - 1) {
	lostHeight = atof(argv[++argCount]);
      }
      else {
	fprintf( stderr, "ERROR: float must follow keyword -l.\n");
	exit;
      }
    }
    else if ((strcmp(argv[argCount],"-n")==0)) {
      if ( argCount < argc - 1) {
	numberOfColumns =  atoi(argv[++argCount]);
	if ( 2 * numberOfColumns - 1 > MAX_NUMBER_OF_COLUMNS) {
	  fprintf( stderr, "ERROR: too many columns.\n");
	  exit(0);
	}
      }
      else {
	fprintf( stderr, "ERROR: number must follow keyword -n.\n");
	exit(0);
      }
    }
    else if ((strcmp(argv[argCount],"-xy")==0)) {
      if ( argCount < argc - 1) {
	xyDistForLost = atof(argv[++argCount]);
      }
      else {
	fprintf( stderr, "ERROR: distance must follow keyword -xy.\n");
	exit;
      }
    }
    else if ((strcmp(argv[argCount],"-t")==0)) {
      if ( argCount < argc - 1) {
	timeForLost = atof(argv[++argCount]);
      }
      else {
	fprintf( stderr, "ERROR: time must follow keyword -t.\n");
	exit;
      }
    }
    else if ((strcmp(argv[argCount],"-nomarker")==0)) {
      useMarker = FALSE;
    }
    else if ((strcmp(argv[argCount],"-corr")==0)) {
      useCorrection = TRUE;
    }
    else if ((strcmp(argv[argCount],"-new")==0)) {
      origScript = FALSE;
    }
    else if ((strcmp(argv[argCount],"-rot")==0)) {
      if ( argCount < argc - 1) {
	rotDistForLost = atof(argv[++argCount]);
      }
      else {
	fprintf( stderr, "ERROR: distance must follow keyword -rot.\n");
	exit;
      }
    }
    else {
      fprintf(stderr, "Unknown parameter: %s\n", argv[argCount]);
      getchar();
    }
  }
  
  if (numberOfColumns > MAX_NUMBER_OF_COLUMNS - 2) {
    fprintf(stderr, "Too many columns. Adjust MAX_NUMBER_OF_COLUMNS.\n");
    exit(0);
  }

  /* Open file and reference file. */
  if ( fixedFileName == NULL) {
    if ( ! useMarker && useCorrection) {
      dontReadFixedAtAll = TRUE;
      fixedFp = NULL;
    }
    else
      fixedFp = stdin;
  }
  else if ((fixedFp = fopen( fixedFileName,"rt")) == NULL) {
    fprintf(stderr,"ERROR: Could not open fixed file '%s'!\n",
	    fixedFileName);
    exit(0);
  }
  
  if ( interpolateFileName == NULL) {
    fprintf(stderr,"ERROR: No reference file name given. Use -f !\n");
    exit(0);
  }
  else if ((interpolateFp = fopen( interpolateFileName,"rt")) == NULL) {
    fprintf(stderr,"ERROR: Could not open interpolate file '%s'!\n",
	    interpolateFileName);
    exit(0);
  }
  
  if ( useMarker) {
    if ( useCorrection) {
      fprintf(stderr, "#Use markers from %s and correction from %s.\n",
	      fixedFileName, interpolateFileName);
      printf( "#Use markers from %s and correction from %s.\n",
	      fixedFileName, interpolateFileName);
    }
    else {
      fprintf(stderr, "#Use markers from %s.\n", fixedFileName);
      printf( "#Use markers from %s.\n", fixedFileName);
    }
  }
  else if ( useCorrection) {
    fprintf(stderr, "#Convert script into corrected script using %s.\n",
	    interpolateFileName);
    printf( "#Convert script into corrected script using %s.\n",
	    interpolateFileName);
  }
  
  fprintf(stderr, "#Parameters for lost: xy %f, rot %f, time %f.\n",
	  xyDistForLost, rotDistForLost, timeForLost);
  
  printf( "#Parameters for lost: xy %f, rot %f, time %f.\n",
	  xyDistForLost, rotDistForLost, timeForLost);
  
  /* skip first line */
  if ( ! dontReadFixedAtAll)
    while (((c = fgetc(fixedFp)) != '\n') && !feof(fixedFp));

  while (dontReadFixedAtAll || !feof(fixedFp)) {
    
    int ok = TRUE;
    
    if ( ! dontReadFixedAtAll)
      ok = getNextValues( fixedFp, values, numberOfColumns, TIME_COLUMN, &time);

    if ( ok){
      
      int success;

#define TIME_SKIP 0.3
      if ( dontReadFixedAtAll)
	time += TIME_SKIP;
      
      if ( useCorrection) { 
	success = getCorrectedReferenceValues( "STDIN", origScript,
					       interpolateFileName,
					       refValues, time);

	refValues[ROT_COLUMN] = rad2Deg( refValues[ROT_COLUMN]);
      }
      else
	success = getReferenceValues( interpolateFp, refValues, refStdDevs,
				      numberOfColumns, time, ! useMarker);


      if ( dontReadFixedAtAll) {
	if ( success)
	  printf( "%f %f %f %f %f\n", time , refValues[0], refValues[1], refValues[2], refValues[3]);
	else
	  exit(0);
      }
      else if ( success) {

	if ( !considerBumps) 
	  printf( "%f", time);
	  
	/* Print the distances. */
	for ( column = 1; column < numberOfColumns; column++) 
	  distances[column] = values[column] - refValues[column];
	
	while (distances[ROT_COLUMN] > 180)
	  distances[ROT_COLUMN] -= 360;
	while (distances[ROT_COLUMN] < -180)
	  distances[ROT_COLUMN] += 360;
	
	if ( ! considerBumps) {

	  for ( column = 1; column < numberOfColumns; column++) 
	    printf( " %f", distances[column]);
	  
	  for ( column = 1; column < numberOfColumns; column++) 
	    printf( " %f", refValues[column]);
	  
	  if ( ! useMarker) {
	    for ( column = 1; column < numberOfColumns; column++) 
	      if ( refStdDevs[column] != 0.0) 
		printf( " %f", distances[column] / refStdDevs[column]);
	      else
		printf( " 0.0");
	  }
	}
	
	/* Print combined distance of x and y value. */
	xyDist = sqrt( fSqr( distances[X_COLUMN]) + fSqr( distances[Y_COLUMN]));
	
	if ( ! useMarker && refStdDevs[COMBINED_STDDEV_COLUMN] != 0.0) 
	  xyDistFactor = xyDist / refStdDevs[COMBINED_STDDEV_COLUMN];
	else
	  xyDistFactor = 0.0;
	
	if ( !considerBumps) {
	  
	  printf( " %f %f", xyDist, xyDistFactor);
	  
	  printf("\n");
	}
	
	bumpOccured = checkIfBumpOccured( time, bumpFileName);
	
	if ( bumpOccured)
	  numberOfBumps++;
	
	/* This function checks whether the position is lost. It considers if
	 * a bump occured for computing the recoverTime and the bumpRecoverTime.
	 * It returns whether the bump should be countet or not. */
	if ( checkIfPositionLost( time, xyDist, fabs(distances[ROT_COLUMN]),
				  xyDistForLost, rotDistForLost,
				  timeForLost, bumpOccured, 
				  &recoverTime, &bumpRecoverTime,
				  gnuFileName, lostHeight, FALSE))
	  ; //	  numberOfBumpsToBeCounted++;
      }
    }
  }
  
  /* Dump the final information. */
  checkIfPositionLost( time, 0.0, 0.0,
		       xyDistForLost, rotDistForLost,
		       timeForLost, bumpOccured, 
		       &recoverTime, &bumpRecoverTime,
		       gnuFileName, lostHeight, TRUE);
  
  if ( considerBumps)
    if ( numberOfBumpsToBeCounted > 0)
      printf( "%f %f %f %f %d %d #RECOVER %s\n",
	      bumpRecoverTime, recoverTime, 
	      bumpRecoverTime + recoverTime, (bumpRecoverTime + recoverTime) / time,
	      numberOfBumps, numberOfBumpsToBeCounted, fixedFileName);
    else
      printf( "%f %f %f %f %d %d #RECOVER\n",
	      bumpRecoverTime, recoverTime,
	      bumpRecoverTime + recoverTime, (bumpRecoverTime + recoverTime) / time,
	      numberOfBumps, numberOfBumpsToBeCounted);
  exit(0);
}

  
#else  


  

int
main(argc, argv)
     int argc;
     char *argv[];
{

  char *distFileName=NULL;
  char *bumpFileName=NULL;
  char *gnuFileName=NULL;
  char c;

  FILE* distFp;
  
  float time = 0.0, xyDist, xyDistFactor, lostHeight = 1000.0;
  int   argCount;
  int numberOfColumns = 4;
  int column;
  int considerBumps = TRUE, bumpOccured = FALSE;
  int numberOfBumps = 0;
  int distCol = 1;
  
  float recoverTime = 0.0, bumpRecoverTime = 0.0;
  
  float distances[MAX_NUMBER_OF_COLUMNS];

  float xyDistForLost = 50.0;
  float rotDistForLost = 5.0;
  float timeForLost = 30.0;

  if (argc < 3) {
    fprintf(stderr, "usage -- for MARKER, SCRIPT, and CORR file:\n\t");
    fprintf( stderr, "cat SCRIPT | %s -int CORR -corr -fix MARKER\n\n",
	     argv[0]);

    fprintf(stderr, "usage -- for SCRIPT, and CORR file:\n\t");
    fprintf( stderr, "cat SCRIPT | %s -int CORR -corr -nomarker\n\n",
	     argv[0]);

    fprintf( stderr, "usage -- for MARKER and MAX file:\n\t");
    fprintf( stderr, "%s -int MAX -nomarker -fix MARKER\n\n", argv[0]);
    
    fprintf(stderr, "usage -- for REFERENCE and MAX file:\n\t");
    fprintf( stderr, "%s -int REFERENCE -fix MAX\n\t\t[-b bumpFile -g gnuFile -l lostHeight -xy xyDistForLost -rot rotDistForLost -t timeForLost]\n", argv[0]);

    fprintf( stderr, "Use option -new for new script type.\n");
    exit(0);
  }      

  for (argCount=1; argCount<argc; argCount++) {

    if ((strcmp(argv[argCount],"-dist")==0)) {
      if ( argCount < argc - 1)
	distFileName = argv[++argCount];
      else {
	fprintf( stderr, "ERROR: file name must follow keyword -dist.\n");
	exit;
      }
    }
    else if ((strcmp(argv[argCount],"-b")==0)) {
      if ( argCount < argc - 1) {
	bumpFileName = argv[++argCount];
	considerBumps = TRUE;
      }
      else {
	fprintf( stderr, "ERROR: name must follow keyword -b.\n");
	exit;
      }
    }
    else if ((strcmp(argv[argCount],"-c")==0)) {
      if ( argCount < argc - 1) {
	distCol = atoi(argv[++argCount]);
      }
      else {
	fprintf( stderr, "ERROR: column must follow keyword -c.\n");
	exit;
      }
    }
    else if ((strcmp(argv[argCount],"-n")==0)) {
      if ( argCount < argc - 1) {
	numberOfColumns =  atoi(argv[++argCount]);
	if ( 2 * numberOfColumns - 1 > MAX_NUMBER_OF_COLUMNS) {
	  fprintf( stderr, "ERROR: too many columns.\n");
	  exit(0);
	}
      }
      else {
	fprintf( stderr, "ERROR: number must follow keyword -n.\n");
	exit(0);
      }
    }
    else if ((strcmp(argv[argCount],"-g")==0)) {
      if ( argCount < argc - 1) {
	gnuFileName = argv[++argCount];
      }
      else {
	fprintf( stderr, "ERROR: name must follow keyword -g.\n");
	exit;
      }
    }
    else if ((strcmp(argv[argCount],"-xy")==0)) {
      if ( argCount < argc - 1) {
	xyDistForLost = atof(argv[++argCount]);
      }
      else {
	fprintf( stderr, "ERROR: distance must follow keyword -xy.\n");
	exit;
      }
    }
    else if ((strcmp(argv[argCount],"-t")==0)) {
      if ( argCount < argc - 1) {
	timeForLost = atof(argv[++argCount]);
      }
      else {
	fprintf( stderr, "ERROR: time must follow keyword -t.\n");
	exit;
      }
    }
    else {
      fprintf(stderr, "Unknown parameter: %s\n", argv[argCount]);
      getchar();
    }
  }
  
  if (numberOfColumns > MAX_NUMBER_OF_COLUMNS - 2) {
    fprintf(stderr, "Too many columns. Adjust MAX_NUMBER_OF_COLUMNS.\n");
    exit(0);
  }

  /* Open file and reference file. */
  if ((distFp = fopen( distFileName,"rt")) == NULL) {
    fprintf(stderr,"ERROR: Could not open dist file '%s'!\n",
	    distFileName);
    exit(0);
  }
  
  fprintf(stderr, "#Use dist file %s.\n", distFileName);
  printf(  "#Use dist file %s.\n", distFileName);
  
  fprintf(stderr, "#Parameters for lost: xy %f, rot %f, time %f.\n",
	  xyDistForLost, rotDistForLost, timeForLost);
  
  printf( "#Parameters for lost: xy %f, rot %f, time %f.\n",
	  xyDistForLost, rotDistForLost, timeForLost);
  
  while ( !feof(distFp)) {

    int ok = getNextValues( distFp, distances, numberOfColumns, TIME_COLUMN, &time);
    
    if ( ok) {

      xyDist = distances[distCol];
      
      bumpOccured = checkIfBumpOccured( time, bumpFileName);
      
      if ( bumpOccured)
	numberOfBumps++;
      
      /* This function checks whether the position is lost. It considers if
       * a bump occured for computing the recoverTime and the bumpRecoverTime.
       * It returns whether the bump should be countet or not. */
      if ( checkIfPositionLost( time, xyDist, 0.0,
				xyDistForLost, rotDistForLost,
				timeForLost, bumpOccured, 
				&recoverTime, &bumpRecoverTime,
				gnuFileName, lostHeight, FALSE))
	; //numberOfBumpsToBeCounted++;
    }
  }
  
  /* Dump the final information. */
  checkIfPositionLost( time, 0.0, 0.0,
		       xyDistForLost, 0.0,
		       timeForLost, bumpOccured, 
		       &recoverTime, &bumpRecoverTime,
		       gnuFileName, lostHeight, TRUE);
  
  if ( considerBumps)
    if ( numberOfBumpsToBeCounted > 0)
      printf( "%f %f %f %f %d %d #RECOVER\n",
	      bumpRecoverTime, recoverTime, 
	      bumpRecoverTime + recoverTime, (bumpRecoverTime + recoverTime) / time,
	      numberOfBumps, numberOfBumpsToBeCounted);
    else
      printf( "%f %f %f %f %d %d #RECOVER\n",
	      bumpRecoverTime, recoverTime,
	      bumpRecoverTime + recoverTime, (bumpRecoverTime + recoverTime) / time,
	      numberOfBumps, numberOfBumpsToBeCounted);
  exit(0);
}

  
#endif  


  








