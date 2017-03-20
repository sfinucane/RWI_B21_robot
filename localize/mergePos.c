#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>

#include "evaluate.h"
#include "inline.h"

#define DEG_180 M_PI
#define DEG_360 (M_PI + M_PI)

#define NUMBER_OF_POSITION_COLUMNS 4

#define MIN_POS_STDDEV 0.0

int verbose = FALSE;


static void
convolve( float* value[MAX_NUMBER_OF_INDICES][MAX_NUMBER_OF_COLUMNS],
	  int columnForXValue, int maxIndex, int numberOfColumns)
{

#define NUMBER_OF_CONVOLUTIONS 10000
  int column, x;
  float tmpArray[MAX_NUMBER_OF_INDICES];
  float kernel[6] = {252.0, 210.0, 120.0, 45.0, 10.0, 1.0};
  int kernelSize = 6, convolveCnt, kernCnt;
  float kernelSum = kernel[0];

  /* Normalize the kernel. */
  for ( kernCnt = 1; kernCnt < kernelSize; kernCnt++) 
    kernelSum += 2.0 * kernel[kernCnt];
  for ( kernCnt = 0; kernCnt < kernelSize; kernCnt++) 
    kernel[kernCnt] /= kernelSum;
    
  for ( convolveCnt = 0; convolveCnt < NUMBER_OF_CONVOLUTIONS; convolveCnt++) {
    
    for ( column = 0; column < numberOfColumns; column++) {
      if ( column != columnForXValue) {
	
	int copyCnt, maxX;
	float kernelElement;
	
	for (x = 0; x < maxIndex; x++){
	  
	  /* In this loop we deal with the borders of the vector. In these
	   * areas not all values are defined so that we have to check for
	   * range errors and consider this in the norm factor <kernelSum>. */
	  
	  float weight = kernel[0];
	  
	  /* Jump to the end of the data. */
	  if ( x == (kernelSize - 1))
	    x = maxIndex - kernelSize + 1;
	  
	  /* The middle of the kernel. */
	  tmpArray[x] = kernel[0] * value[x][column][0];
	  
	  /* Store the weighted sum in the tmpArray. */
	  for ( kernCnt = 1; kernCnt < kernelSize; kernCnt++) {
	    
	    kernelElement = kernel[kernCnt];
	    
	    if ( x + kernCnt < maxIndex) {
	      weight += kernelElement;
	      tmpArray[x] += kernelElement * value[x + kernCnt][column][0];
	    }
	    
	    if (x - kernCnt >= 0) {
	      weight += kernelElement;
	      tmpArray[x] += kernelElement * value[x - kernCnt][column][0];
	    }
	  }
	  if ( weight != 0.0)
	    tmpArray[x] /= weight;
	}
	
	/* Now we can compute the weighted sums without range checks. */
	maxX = maxIndex-kernelSize+1;
	for ( x = kernelSize - 1; x < maxX; x++) {
	  
	  /* The middle of the kernel. */
	  tmpArray[x] = kernel[0] * value[x][column][0];
	  
	  /* Store the weighted kernelSum in the tmpArray. */
	  for ( kernCnt = 1; kernCnt < kernelSize; kernCnt++) 
	    tmpArray[x] += kernel[kernCnt]
	  * (value[x+kernCnt][column][0] + value[x-kernCnt][column][0]);
	}
	
	/* Now copy the values from the tmpArray back into the vector. */
	for ( copyCnt = 0; copyCnt < maxIndex; copyCnt++) 
	  value[copyCnt][column][0] = tmpArray[copyCnt];
      }
    }
  }
  
  /* Now dump the values. */
  for (x = 0; x < maxIndex; x++){
    for ( column = 0; column < numberOfColumns; column++) {
      printf( "%f ", value[x][column][0]);
    }
    printf( "\n");
  }    
}



int
main(argc, argv)
     int argc;
     char *argv[];
{

  FILE *fp;
  
  float *value[MAX_NUMBER_OF_INDICES][MAX_NUMBER_OF_COLUMNS];
  int    valueCnt[MAX_NUMBER_OF_INDICES];
  int    index=-1, maxIndex = 0, argCount;
  int    mergePositions = FALSE, useAble;
  int interpolate = FALSE;
  int convolveValues = FALSE;
  
  int column, columnForXValue = 0, fileNumber, numberOfColumns = 5;
  float resolution = 1.0, prevPlotted;
  float columnValue[MAX_NUMBER_OF_COLUMNS];
  float stdDev[MAX_NUMBER_OF_COLUMNS], mean[MAX_NUMBER_OF_COLUMNS];
  char c;

  if (argc < 2) {
    fprintf(stderr, "usage: %s -xcol columnForXValue -convolve -r resolution -n numberCols -pos -f file1 ...\n", argv[0]);
    exit(0);
  }      

  for (column = 0; column < MAX_NUMBER_OF_INDICES; column++) {
    valueCnt[column] = 0;
  }

  for (argCount=1; argCount<argc; argCount++) {

    if ((strcmp(argv[argCount],"-f")==0)) {
      argCount++;
      break;
    }
    else if ((strcmp(argv[argCount],"-r")==0)) {
      if ( argCount < argc - 1)
	resolution =  atof(argv[++argCount]);
      else {
	fprintf( stderr, "ERROR: resolution must follow keyword -r.\n");
	exit;
      }
    }
    else if ((strcmp(argv[argCount],"-xcol")==0)) {
      if ( argCount < argc - 1)
	columnForXValue =  atoi(argv[++argCount]) - 1;
      else {
	fprintf( stderr, "ERROR: column must follow keyword -c.\n");
	exit;
      }
    }
    else if ((strcmp(argv[argCount],"-n")==0)) {
      if ( argCount < argc - 1)
	numberOfColumns =  atoi(argv[++argCount]);
      else {
	fprintf( stderr, "ERROR: number must follow keyword -n.\n");
	exit;
      }
    }
    else if ((strcmp(argv[argCount],"-pos")==0)) {
      mergePositions = TRUE;
    }
    else if ((strcmp(argv[argCount],"-convolve")==0)) {
      convolveValues = TRUE;
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
  
  if ( mergePositions) {
    numberOfColumns = NUMBER_OF_POSITION_COLUMNS;
    columnForXValue = TIME_COLUMN;
    interpolate = TRUE;
    fprintf( stderr, "Merge Positions with resolution %f.\n", resolution);
  }
  else
    fprintf( stderr, "Settings: res %f, x-column %d, columns %d\n",
	     resolution, columnForXValue, numberOfColumns);
  
  for (fileNumber = argCount; fileNumber < argc; fileNumber++){
    if ((fp = fopen(argv[fileNumber],"rt")) == NULL) {
      fprintf(stderr,"ERROR: Could not open file '%s'! Skipped\n",
	      argv[fileNumber]);
    }
    else {
      
      float time = 0.0;
      
      fprintf( stderr, "Scanning file %s ...\n", argv[fileNumber]);
      
      /* skip first line */
      while (((c = fgetc(fp)) != '\n') && !feof(fp));
      
      while (!feof(fp)){
	int success; 
	if ( mergePositions) {
	  success = getInterpolatedValues( fp,
					   columnValue, numberOfColumns,
					   columnForXValue, resolution,
					   &time, &useAble);
	}
	else {
	  useAble = TRUE;
	  success = getNextValues( fp, columnValue, numberOfColumns,
				   columnForXValue, &time);
	}
	  
	if ( success && useAble) {
	    
	    if ( mergePositions) 
	      index = (int) (time / resolution);
	    else if ( ! convolveValues)
	      index = (int) ((time / resolution) + 0.5);
	    else
	      index++;
	    
	    if (index < MAX_NUMBER_OF_INDICES &&
		valueCnt[index] < MAX_NUMBER_OF_VALUES){

	      if (index > maxIndex)
		maxIndex = index;
	      
	      for ( column = 0; column < numberOfColumns; column++) {
		
		if ( ! convolveValues) {
		  if (valueCnt[index] == 0) {
		    value[index][column] = (float *) malloc(MAX_NUMBER_OF_VALUES * sizeof(float));

		    if ( value[index][column] == NULL) {
		      fprintf( stderr, "Not enough memory. Resize defines.\n");
		      exit(0);
		    }
		  }
		  if (0) fprintf( stderr, "%d %d\n", valueCnt[index], column);
		  value[index][column][valueCnt[index]] = columnValue[column];
		}
		/* Every new value is stored in a new row. */
		else {
		  value[index][column] = (float *) malloc(sizeof(float));
		  value[index][column][0] = columnValue[column];
		}
	      }
	      valueCnt[index]++;
	      
	      if ( valueCnt[index] > MAX_NUMBER_OF_VALUES) {
		fprintf(stderr, "Too many values per index (%d %d).\n",
			valueCnt[index], MAX_NUMBER_OF_VALUES);
		exit(0);
	      }
	      if (0) fprintf( stderr, "%d %f\n", valueCnt[index], columnValue[column]);
	    }
	    else {
	      fprintf( stderr, "Too many indices %d %d or values %d %d.\n",
		       index, MAX_NUMBER_OF_INDICES, valueCnt[index], MAX_NUMBER_OF_VALUES);
	      exit(0);
	    }
	}
      }
      fclose(fp);
      fprintf( stderr, "done.\n");
    }
  }

  if ( convolveValues) {
    convolve( value, columnForXValue, maxIndex, numberOfColumns);
    exit(0);
  }

  for (index = 0; index < maxIndex; index++){
    
    int j;
    float positionStdDev=0.0, positionErrorBar=0.0;
    
    if (valueCnt[index] > 0) {
      
      /* Compute all means and stdDevs for this index. */
      for ( column = 0; column < numberOfColumns; column++) {
	
	mean[column] = stdDev[column] = 0.0;
	
	/* Compute mean. */
	for (j = 0; j < valueCnt[index]; j++)
	  mean[column] += value[index][column][j];
	mean[column] /= valueCnt[index];

	/* Compute stdDev. */
	if ( valueCnt[index] > 1) {
	  float sampleStd;
	  for (j = 0; j < valueCnt[index]; j++)
	    stdDev[column] +=
	      (value[index][column][j] - mean[column]) *
	      (value[index][column][j] - mean[column]);
	  
	  stdDev[column] /= (valueCnt[index] - 1);
	  stdDev[column] = sqrt(stdDev[column]);

	  sampleStd = stdDev[column] / sqrt(valueCnt[index]);
	  stdDev[column] = sampleStd * tValueForSampleSize(valueCnt[index]);

/* 	  sampleStd = stdDev[column] / 3; */
/* 	  stdDev[column] = sampleStd * tValueForSampleSize(3); */
	}
	else
	  stdDev[column] = 0.0;
      }

      /* Compute the stdDev of rotation and combined x and y values. */
      if ( mergePositions) {
	
	if ( valueCnt[index] > 1) {
	  
	  /* Mean and stdDev of rotation has to be measured differently. */
	  mean[ROT_COLUMN] = value[index][ROT_COLUMN][0];
	  stdDev[ROT_COLUMN] = 0.0;
	  
	  for (j = 1; j < valueCnt[index]; j++) {
	    
	    /* If the distance to the mean is bigger than 180 degree
	     * we have to shift the value accordingly. */
	    if ( fabs( value[index][ROT_COLUMN][j] - mean[ROT_COLUMN]/j) > 180.0)
	      if ( value[index][ROT_COLUMN][j] > mean[ROT_COLUMN])
		mean[ROT_COLUMN] += value[index][ROT_COLUMN][j] - 360.0;
	      else
		mean[ROT_COLUMN] += value[index][ROT_COLUMN][j] + 360.0;
	    else
	      mean[ROT_COLUMN] += value[index][ROT_COLUMN][j];
	  }

	  mean[ROT_COLUMN] /= valueCnt[index];
	  
	  /* Normalize the mean. */
	  if ( mean[ROT_COLUMN] > 360.0)
	    mean[ROT_COLUMN] -= 360.0;
	  else if ( mean[ROT_COLUMN] < 0.0)
	    mean[ROT_COLUMN] += 360.0;
	  
	  /* Compute stdDev. */
	  for (j = 0; j < valueCnt[index]; j++) {
	    float dist = fabs( value[index][ROT_COLUMN][j] - mean[ROT_COLUMN]);
	    if ( dist > 180.0)
	      dist = 360.0 - dist;
	    stdDev[ROT_COLUMN] += fSqr( dist);
	  }
	  stdDev[ROT_COLUMN] /= (valueCnt[index] - 1);
	  stdDev[ROT_COLUMN] = sqrt(stdDev[ROT_COLUMN]);
	  
	  /* Combined xy-dist. */ 
	  /* Here we also  compute the error bars. */
	  {
	    float sum = 0.0, sampleStdDev;

	    for (j = 0; j < valueCnt[index]; j++)
	      sum +=
		sqrt(
		     fSqr((value[index][X_COLUMN][j] - mean[X_COLUMN]) *
			  (value[index][X_COLUMN][j] - mean[X_COLUMN])) +
		     fSqr((value[index][Y_COLUMN][j] - mean[Y_COLUMN]) *
			  (value[index][Y_COLUMN][j] - mean[Y_COLUMN])));

	    positionStdDev = sqrt(sum/(valueCnt[index] - 1));
	    sampleStdDev = positionStdDev / sqrt( valueCnt[index]);
	    positionErrorBar = sampleStdDev * tValueForSampleSize(valueCnt[index]);
	  }
	}
	else {
	  stdDev[ROT_COLUMN] = 0.0;
	  positionStdDev = 0.0;
	  positionErrorBar = 0.0;
	}
      }
      if ( mergePositions)
	printf( "%f", index * resolution);
      else
	printf( "%.2f", (index+0.5) * resolution);
      
      /* Print the means. */
      for ( column = 0; column < numberOfColumns; column++) 
	if ( column != columnForXValue)
	  if ( mergePositions && column == ROT_COLUMN) {
	    static int firstTime = TRUE;
	    if (firstTime) {
	      firstTime = FALSE;
	      prevPlotted = mean[column];
	    }
	    else {
	      float rotDistToPrevious = mean[column] - prevPlotted;
	      if ( rotDistToPrevious > 180.0)
		prevPlotted = mean[column] - 360;
	      else if ( rotDistToPrevious < -180.0)
		prevPlotted = mean[column] + 360;
	      else
		prevPlotted = mean[column];
	    }
	    printf( " %f", prevPlotted);
	  }
	  else
	    printf( " %f", mean[column]);

      /* Print the stdDevs. */
      for ( column = 0; column < numberOfColumns; column++) 
	if ( column != columnForXValue)
	  printf( " %f", stdDev[column]);
      
      if ( mergePositions) 
	printf( " %f %f", fMax( positionStdDev, MIN_POS_STDDEV), positionErrorBar);
      printf("\n");
    }
  }
  exit(0);
}

  
  


  

