#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>
#include <values.h>

#include "evaluate.h"
#include "inline.h"

#define DEG_180 M_PI
#define DEG_360 (M_PI + M_PI)

#define MAX_NUMBER_OF_ROWS 50000
#define NUMBER_OF_FILES 2

int verbose = FALSE;

    
static float
standardDeviation( int N, float sum, float squaredSum)
{
  float avg = sum / N;
  return sqrt( (1.0 / ( N - 1)) * (squaredSum - N * fSqr( avg)));
}


static float
computeTValue( int N1, float s1, float avg1,
	       int N2, float s2, float avg2)
{
  
  float pooledStd =
    ((N1 - 1) * fSqr( s1) + (N2 - 1) * fSqr( s2))
    / 
    (N1 + N2 - 2);
  
  float estimatedPooledStd =
    sqrt( pooledStd * (1.0 / N1 + 1.0 / N2));
  
  float tValue =
    (avg1 - avg2) / estimatedPooledStd;
  
  fprintf( stderr, "tv  %f %f\n",
	   tValue, tValueForSampleSize(N1+N2-2));

  return tValue;
}

static void
computeErrorBars( int N,
		  float val[MAX_NUMBER_OF_ROWS][MAX_NUMBER_OF_COLUMNS],
		  int firstColumn, int lastColumn,
		  float errorBar[MAX_NUMBER_OF_COLUMNS], float threshold,
		  char* idString)
{
  int column, row;
  int aboveThreshold = 0;
  
  if ( N < 2) {
    fprintf( stderr, "%d: not enough values.\n", N);
    return;
  }

  for ( column = firstColumn; column < lastColumn; column++) {
    
    float minVal = MAXFLOAT;
    float maxVal = MINFLOAT;
    
    float sum = 0.0, squaredSum = 0.0;
    float std, sampleStd;
    
    for ( row = 0; row < N; row++) {
      if( val[row][column] < threshold) {
	sum += val[row][column];
	squaredSum += fSqr(val[row][column]);
      }
      else {
	aboveThreshold++;
      }

      if ( val[row][column] < minVal)
	minVal = val[row][column];
      if ( val[row][column] > maxVal)
	maxVal = val[row][column];
    }

    N -= aboveThreshold;

    std = standardDeviation( N, sum, squaredSum);

    sampleStd = std / sqrt( N);
    errorBar[column] = sampleStd * tValueForSampleSize(N-1);

    if ( idString != NULL)
      printf( "%d %f %f %f %d %f %f %f %s\n",
	      column, sum / N, errorBar[column], std, N, minVal, maxVal, 100.0 * (float) aboveThreshold / (float) (N + aboveThreshold), idString);
    else
      printf( "%d %f %f %f %d %f %f %f\n",
	      column, sum / N, errorBar[column], std, N, minVal, maxVal, 100.0 * (float) aboveThreshold / (float) (N + aboveThreshold));
      
  }
}
			 

static void
performPairedSampleTest( int N1, int N2, 
			 float val1[MAX_NUMBER_OF_ROWS][MAX_NUMBER_OF_COLUMNS],
			 float val2[MAX_NUMBER_OF_ROWS][MAX_NUMBER_OF_COLUMNS],
			 int firstColumn, int lastColumn,
			 float tValue[MAX_NUMBER_OF_COLUMNS],
			 char* idString)
{
  int column, row, N = N1;
  float addLevel, increment = 0.01;
  
  if ( N1 != N2) {
    fprintf( stderr, "Error: number of rows must be the same %d %d.\n",
	       N1, N2);
    exit(0);
  }
  else if ( N1 < 2) {
    fprintf( stderr, "%d: not enough values.\n", N);
    return;
  }

  /* Check different levels to be added to the first data. */
  for ( addLevel = 1.0; addLevel < 2.5; addLevel += increment) {
    
    for ( column = firstColumn; column < lastColumn; column++) {
      
      float sum = 0.0, squaredSum = 0.0;
      float std, sampleStd;
      
      for ( row = 0; row < N; row++) {
	
	float delta = val1[row][column] - addLevel * val2[row][column];
	
	sum += delta;
	squaredSum += fSqr(delta);
      }
      
      std = standardDeviation( N, sum, squaredSum);
      
      sampleStd = std / sqrt( N);
      tValue[column] = (sum / N) / sampleStd;
      
      if ( tValue[column] > tValueForSampleSize( N))
	
	fprintf( stderr, "%f %f %d:  %f %f --> %f\n",
		 tValueForSampleSize( N), (addLevel - 1) * 100.0,
		 column, std, sampleStd, tValue[column]);
    }
  }
}
			 

static void
performTwoSampleTest( int N1, int N2,
		      float val1[MAX_NUMBER_OF_ROWS][MAX_NUMBER_OF_COLUMNS],
		      float val2[MAX_NUMBER_OF_ROWS][MAX_NUMBER_OF_COLUMNS],
		      int firstColumn, int lastColumn,
		      float tValue[MAX_NUMBER_OF_COLUMNS],
		      char* idString)
{
  int column, row;
  int numberOfRows = iMax( N1, N2);
  float addLevel, increment = 0.01;
  
  /* Check different levels to be added to the first data. */
  for ( addLevel = 1.0; addLevel < 2.3; addLevel += increment) {
    
    for ( column = firstColumn; column < lastColumn; column++) {
      
      float sum1=0.0, squaredSum1=0.0, sum2=0.0, squaredSum2=0.0;
      float std1, std2;
      
      for ( row = 0; row < numberOfRows; row++) {
	
	if ( row < N1) {	
	  sum1 += val1[row][column];
	  squaredSum1 += fSqr(val1[row][column]);
	}
	
	if ( row < N2) {
	  sum2 += addLevel * val2[row][column];
	  squaredSum2 += fSqr(addLevel * val2[row][column]);
	}
      }
      
      std1 = standardDeviation( N1, sum1, squaredSum1);
      std2 = standardDeviation( N2, sum2, squaredSum2);
      
      tValue[column] = computeTValue( N1, std1, sum1 / N1,
				      N2, std2, sum2 / N2);

      if ( fabs( tValue[column]) > tValueForSampleSize( N2))
	
	fprintf( stderr, "%f %f %d:  %f %f --> %f\n",
		 tValueForSampleSize( N2), (addLevel - 1) * 100.0,
		 column, std1, std2, tValue[column]);

    }
  }
}


int
main(argc, argv)
     int argc;
     char *argv[];
{

  FILE *fp;
  int argCount, fileNumber;
  int column, row;
  int valueColumn = 0;
  float threshold = MAXFLOAT;
  int numberOfColumns = 1, numberOfReadColumns;
  int allColumns = TRUE;
  int pairedEvaluation = FALSE, errorBars = FALSE;
  int numberOfLinesToBeSkipped = 0;
  char* fileName[NUMBER_OF_FILES] = {NULL, NULL};
  
  /* For an explanation of the variables see p. 127 in Empirical AI. */
  int cnt[NUMBER_OF_FILES];
  float std[NUMBER_OF_FILES][MAX_NUMBER_OF_COLUMNS];
  float avg[NUMBER_OF_FILES][MAX_NUMBER_OF_COLUMNS];
  float sum[NUMBER_OF_FILES][MAX_NUMBER_OF_COLUMNS];
  float squaredSum[NUMBER_OF_FILES][MAX_NUMBER_OF_COLUMNS];
  float pooledStd[MAX_NUMBER_OF_COLUMNS];
  float estimatedPooledStd[MAX_NUMBER_OF_COLUMNS];
  float tValue[MAX_NUMBER_OF_COLUMNS];
  char* idString = NULL;
  
  int start, end;
  
  float values[NUMBER_OF_FILES][MAX_NUMBER_OF_ROWS][MAX_NUMBER_OF_COLUMNS];
  
  if (argc < 2) {
    fprintf(stderr, "usage: %s file1 -t threshold [-lower file2] [-c column] -e -p [-n numberOfColumns] [-s skip lines] [-id string]\n", argv[0]);
    exit(0);
  }      

  fileName[0] =  argv[1];

  for (argCount=2; argCount<argc; argCount++) {

    if ((strcmp(argv[argCount],"-lower")==0)) {
      if ( argCount < argc - 1) {
	fileName[1] =  argv[++argCount];
      }
      else {
	fprintf( stderr, "ERROR: TWO file must follow keyword -lower.\n");
	exit;
      }
    }
    else if ((strcmp(argv[argCount],"-c")==0)) {

      if ( argCount < argc - 1) {
	
	valueColumn =  atoi(argv[++argCount]) - 1;
	allColumns = FALSE;
	if ( valueColumn > MAX_NUMBER_OF_COLUMNS-1) {
	  fprintf(stderr, "Not enough columns (%d).\n", MAX_NUMBER_OF_COLUMNS);
	  exit(0);
	}
      }
      else {
	fprintf( stderr, "ERROR: column number must follow keyword -c.\n");
	exit;
      }
    }
    else if ((strcmp(argv[argCount],"-s")==0)) {

      if ( argCount < argc - 1) {
	
	numberOfLinesToBeSkipped =  atoi(argv[++argCount]);
      }
      else {
	fprintf( stderr, "ERROR: number of lines to be skipped must follow keyword -s.\n");
	exit;
      }
    }
    else if ((strcmp(argv[argCount],"-id")==0)) {

      if ( argCount < argc - 1) {
	
	idString = argv[++argCount];
      }
      else {
	fprintf( stderr, "ERROR: string skipped must follow keyword -id.\n");
	exit;
      }
    }
    else if ((strcmp(argv[argCount],"-t")==0)) {
      if ( argCount < argc - 1) {	
	threshold = atof(argv[++argCount]);
      }
    }
    else if ((strcmp(argv[argCount],"-n")==0)) {
      if ( argCount < argc - 1) {
	allColumns = TRUE;
	numberOfColumns = atoi(argv[++argCount]);
	if ( numberOfColumns > MAX_NUMBER_OF_COLUMNS) {
	  fprintf(stderr, "Not enough columns (%d).\n", MAX_NUMBER_OF_COLUMNS);
	  exit(0);
	}
      }
      else {
	fprintf( stderr, "ERROR: number of columns must follow keyword -c.\n");
	exit;
      }
    }
    else if ((strcmp(argv[argCount],"-p")==0)) {
      pairedEvaluation = TRUE;
    }
    else if ((strcmp(argv[argCount],"-e")==0)) {
      errorBars = TRUE;
    }
    else {
      fprintf(stderr, "Unknown parameter: %s\n", argv[argCount]);
      getchar();
    }
  }
  
  if ( fileName[0] == NULL) {
    fprintf( stderr, "ERROR: file -upper must be given.\n");
    exit(0);
  }
  if ( fileName[1] == NULL && ! errorBars) {
    fprintf( stderr, "ERROR: file -lower must be given.\n");
    exit(0);
  }
    
  /*----------------------------------------------------------------------
   * Initialize the fields.
   *----------------------------------------------------------------------*/
  for ( column = 0; column < MAX_NUMBER_OF_COLUMNS; column++) {
    for ( fileNumber = 0; fileNumber < NUMBER_OF_FILES; fileNumber++) {
      std[fileNumber][column] = 0.0;
      avg[fileNumber][column] = 0.0;
      sum[fileNumber][column] = 0.0;
      squaredSum[fileNumber][column] = 0.0;
      for ( row = 0; row < MAX_NUMBER_OF_ROWS; row++)
	values[fileNumber][row][column] = 0.0;
    }
    pooledStd[column] = 0.0;
    estimatedPooledStd[column] = 0.0;
    tValue[column] = 0.0;
  }
  for ( fileNumber = 0; fileNumber < NUMBER_OF_FILES; fileNumber++) {
    cnt[fileNumber] = 0;
    if ( ! allColumns) {
      start = valueColumn;
      end = valueColumn + 1;
    }
    else {
      start = 0;
      end = numberOfColumns;
    }
  }
  
  if ( allColumns)
    fprintf( stderr, "Analyzing all %d columns.\n", numberOfColumns);
  else
    fprintf( stderr, "Analyzing at column %d.\n",  valueColumn);
  

  /* First read all the data. */
  for (fileNumber = 0; fileNumber < NUMBER_OF_FILES; fileNumber++){

    if ( fileName[fileNumber] != NULL) {
      char* stdinName = "STDIN";

      if ( strncmp( fileName[fileNumber], stdinName, strlen(stdinName)) == 0) {
	fprintf( stderr, "Read data from stdin.\n");
	fp = stdin;
      }
      else if ((fp = fopen(fileName[fileNumber], "rt")) == NULL) {
	fprintf(stderr,"ERROR: Could not open file '%s'! Skipped\n",
		argv[fileNumber]);
	exit(0);
      }
      else
	fprintf( stderr, "Scanning file %s ...\n", fileName[fileNumber]);

      
      row = 0;
      
      while (!feof(fp)) {
	
	/* Skip the first lines. */
	while ( numberOfLinesToBeSkipped > 0) {
	  char line[1024];
	  fgets(line, 1024, fp);
	  printf( "#SKIP: %s", line);
	  if ( line[0] != '#') numberOfLinesToBeSkipped--;
	}
	
	/*----------------------------------------------------------------------
v	 * Read the next line and update the values.
	 *----------------------------------------------------------------------*/
	if ( row < MAX_NUMBER_OF_ROWS) {	    
	  if ( getNextValues( fp, values[fileNumber][row], end, 0, NULL)) {
	    cnt[fileNumber]++;
	    row++;
	  }
	}
      }
    }
  }

  fprintf( stderr, "Read %d %d rows.\n", cnt[0], cnt[1]);
  
  if ( pairedEvaluation) {
    performPairedSampleTest( cnt[0], cnt[1], values[0], values[1],
			     start, end, tValue, idString);
  }
  else if ( errorBars) {
    computeErrorBars( cnt[0], values[0],
		      start, end, tValue, threshold, idString);
  }
  else
    performTwoSampleTest( cnt[0], cnt[1], values[0], values[1],
			  start, end, tValue, idString);

  exit(0);
}
  
  


  

