#define MAX_NUMBER_OF_DISTANCES 256
#define BUFFLEN 80
#define UNKNOWN 0.0
#define SQRT2PI 2.5066283
#define DELTA 30
#define NUMBER_OF_LINS 1

#include <math.h>
#include <stdio.h>
#include <values.h>
#include <stdlib.h>

typedef float probability;

typedef struct {
  int numberOfMeasuredDistances;
  int numberOfExpectedDistances;
  probability prob[MAX_NUMBER_OF_DISTANCES][MAX_NUMBER_OF_DISTANCES];
  int count[MAX_NUMBER_OF_DISTANCES][MAX_NUMBER_OF_DISTANCES];
} distProbTable;

typedef struct{
  int numberOfLins;
  float probShort[NUMBER_OF_LINS];
  float probFar[NUMBER_OF_LINS];
} multiLinFunction;

typedef struct {
  float sigmaShort;
  float sigmaFar;
  float minProbShort;
  float minProbFar;
  multiLinFunction maxRange;
  float maxRangeMinLevel;
} probFunctionParameters;


typedef struct{
  float value[MAX_NUMBER_OF_DISTANCES];
  int count[MAX_NUMBER_OF_DISTANCES];
  int numberOfValues;
} vector;


float
fMin( float x, float y)
{
 return ( x > y ? y : x);
}


float
fMax(float x, float y)
{
 if (x > y)
   return x;
 else
   return y;
}
     

float
fSqr(float x){
  return x*x;
}
     

float
gauss( float x, float sigma, float mean)
{
  float tmp;
    
  if (sigma != 0.0) {
    tmp = exp( -0.5 * fSqr((mean-x) / sigma));
    return( tmp / (SQRT2PI * sigma));
  }
  else {
    if (x == mean)
      return 1.0;
    else
      return 0.0;
  }
    
}


float
linFunction( float x, float x1, float y1, float x2, float y2)
{
  return y1 + x * ((y2 - y1) / (x2 - x1 + 1));
  
}
   


/**************************************************************************
**************************************************************************
* Functions for the probabilities of distance readings.
**************************************************************************
**************************************************************************/
float
maxRangeValue( probFunctionParameters params,
	       distProbTable tab,
	       int x)
{
  int numberOfValues = tab.numberOfExpectedDistances
    / params.maxRange.numberOfLins;
  int lin = x / numberOfValues;
  int pos = x % numberOfValues;
  float maxRangeProbM = (params.maxRange.probFar[lin]
			 - params.maxRange.probShort[lin])
    / numberOfValues;

  return params.maxRange.probShort[lin] + pos * maxRangeProbM;
}


void
computeDistProbFunction( probFunctionParameters params,
			 distProbTable *tab,
			 int numberOfExpectedDistances,
			 int numberOfMeasuredDistances,
			 int delta)
{

  register int x,y;
  float sum;
  float sigma;
  float sigmaM =
    (params.sigmaFar - params.sigmaShort) / numberOfExpectedDistances;

  tab->numberOfExpectedDistances = numberOfExpectedDistances;
  tab->numberOfMeasuredDistances = numberOfMeasuredDistances;

  for (x = 0; x < numberOfExpectedDistances; x++)
     for (y = 0; y < numberOfMeasuredDistances; y++)
	tab->count[x][y] = 1.0;
  
  for (x = 0; x < numberOfExpectedDistances; x++){
    sigma = params.sigmaShort + sigmaM * x;
    for ( y = 0; y < x; y++)
      tab->prob[x][y] =
	fMax(params.minProbShort, gauss( y, sigma, x));
    for ( y = x; y < numberOfMeasuredDistances; y++)
      tab->prob[x][y] =
	fMax(params.minProbFar, gauss( y, sigma, x));
    }

  params.maxRange.probFar[0] += 0.1;
  for (x = 0; x < numberOfExpectedDistances; x++){
    tab->prob[x][numberOfMeasuredDistances - 1] +=
      maxRangeValue(params, *tab, x);
  }

  for (x = numberOfExpectedDistances - delta;
       x < numberOfExpectedDistances; x++){
     sum = 0;
     for (y = x - delta; y < x + x - numberOfExpectedDistances; y++){
	sum += tab->prob[x][y] - params.minProbShort;
     }
     tab->prob[x][numberOfMeasuredDistances - 1] += sum;
  }

  for (x = 0; x < numberOfExpectedDistances; x++){
    tab->prob[x][numberOfMeasuredDistances - 1] *= 1;
  }

  tab->prob[numberOfExpectedDistances-1][numberOfMeasuredDistances - 1] *= 4;

  /*  for (y = 0; y < tab->numberOfMeasuredDistances - 1; y++)
      tab->prob[tab->numberOfExpectedDistances -1][y] = params.maxRangeMinLevel;
      */
}



/*****************************************************************************
 * Computes the probabilities of sensor measurements for given expected
 *  measurements.
 *****************************************************************************/
float
probFunctionDistance( distProbTable table1,
		      distProbTable table2)
{
  register int measuredCnt, expectedCnt;
  register double error = 0.0;

  
  /***********************************************************************
   * Fill in the probabilities.
   ***********************************************************************/
  for ( expectedCnt = 0;
	expectedCnt < table1.numberOfExpectedDistances; expectedCnt++) {
    for ( measuredCnt = 0;
	  measuredCnt < table1.numberOfMeasuredDistances - 1; measuredCnt++){
      if (table1.count[expectedCnt][measuredCnt] > 0)
	error += fSqr( table1.prob[expectedCnt][measuredCnt] -
		       table2.prob[expectedCnt][measuredCnt])
	  * table1.count[expectedCnt][measuredCnt];
    }
  }
  return error;
}


void
readSampleData( char* fileName, int resolution, distProbTable *table){
  FILE *fp;
  char line[BUFFLEN];
  float xFloat, yFloat;
  int x, y, max, maxX, maxY, count;
  float prob;

  if ((fp = fopen(fileName, "rt")) == NULL){
    fprintf(stderr, "Error: coult not open %s\n", fileName);
    exit(0);
  }

  for (x = 0; x < MAX_NUMBER_OF_DISTANCES; x++)
    for (y = 0; y < MAX_NUMBER_OF_DISTANCES; y++)
      table->prob[x][y] = UNKNOWN;


  
  maxX = maxY = 0;
  while (!feof(fp)){
    if (fgets(line, BUFFLEN, fp) != NULL
	&& sscanf(line, "%f %f %e %d", &xFloat, &yFloat, &prob, &count) == 4){
      x = ((int) xFloat + 0.01) / resolution;
      y = ((int) yFloat + 0.01) / resolution;
      if (x >= MAX_NUMBER_OF_DISTANCES || y >= MAX_NUMBER_OF_DISTANCES){
	fprintf(stderr, "Error: to large distances, fix that first\n");
	exit(0);
      }
      if (maxX < x) maxX = x;
      if (maxY < y) maxY = y;
      table->prob[x][y] = prob;
      table->count[x][y] = count;
    }
  }
  fclose(fp);
  if (maxX > maxY) max = maxX; else max = maxY;
  table->numberOfExpectedDistances = max + 1;
  table->numberOfMeasuredDistances = max + 1;
}



void
dumpDistProbFunction(char *fileName,
		     float maxDist, float zScale,
		     distProbTable probTab)
{
  int x, y;
  float step = 1.0;
  FILE *fp = fopen(fileName, "wt");

  if ( maxDist > 1.0)
    step = maxDist / probTab.numberOfExpectedDistances;
  
  fprintf(fp, "# %d %d\n",
	  probTab.numberOfExpectedDistances,
	  probTab.numberOfMeasuredDistances);
  
  for (x = 0; x < probTab.numberOfExpectedDistances; x++){
    for ( y = 0; y < probTab.numberOfMeasuredDistances; y++){
      fprintf(fp, "%f %f %.3e\n", x * step, y * step,
	      zScale * probTab.prob[x][y]);
    }
    fprintf(fp, "\n");
  }
}


void
computeLinearRegression( vector values, float *from, float *to)
{
  register int x;
  long int count = 0;
  double averageX = 0.0, tmp = 0.0, xDiffSum = 0.0, averageY = 0.0;
  

/*   for (x = 0; x < values.numberOfValues; x++){ */
/*     printf("%d %d: \n", x, values.count[x]); */
/*   } */
  
  for (x = 0; x < values.numberOfValues; x++){
    averageX += values.count[x]*x;
    count += values.count[x];
  }

  printf("Count: %d\n", (int) count);
  if (count < 2)
    averageX = 0;
  else
    averageX /= count;

  for ( x = 0; x < values.numberOfValues; x++){
    xDiffSum += values.count[x] * fSqr(x - averageX);
    averageY += values.count[x] * values.value[x];
  }

  if (count < 2)
    averageY = 0;
  else
    averageY /= count;

  printf("AverageX: %f, averageY: %f\n", (float) averageX, (float) averageY);
  
  
  tmp = 0.0;
  for ( x = 0; x < values.numberOfValues; x++){
    tmp += values.count[x] * (x - averageX)*(values.value[x] - averageY);
  }

  if (xDiffSum != 0.0)
    tmp /= xDiffSum;
  else
    tmp = 0.0;

  *from = averageY - tmp * averageX;
  *to = *from + tmp * (values.numberOfValues - 1);
}


void
computeMaxRangeParamsMultiLin( distProbTable probTab,
		       float minProbFar,
		       multiLinFunction *maxRange,
		       float *maxRangeMinLevel)
{
  register int i, x;
  int z = probTab.numberOfMeasuredDistances-1;
  int numberOfValues;
  float sum;
  int count;

  maxRange->numberOfLins =
    fMin(probTab.numberOfExpectedDistances / 4, NUMBER_OF_LINS);
  numberOfValues = probTab.numberOfExpectedDistances / NUMBER_OF_LINS;
  /* compute start value */

  for (i = 0; i < maxRange->numberOfLins; i++){
    sum = count = 0;
    for (x = i * numberOfValues - 2 ; x <= i * numberOfValues +2; x++){
      if (x > 0 && x < probTab.numberOfExpectedDistances){
	sum += (probTab.prob[x][z] - minProbFar) * probTab.count[x][z];
	count += probTab.count[x][z];
      }
    }
    
    if (count > 0)
      sum /= count;
    if (sum < 0)
      sum = 0;
    fprintf(stderr, "%d, %f\n", i, sum);

    if (i > 0)
      maxRange->probFar[i-1] = sum;

    maxRange->probShort[i] = sum;
  
  }

  if (probTab.count[probTab.numberOfExpectedDistances - 1][z] > 0)
    maxRange->probFar[maxRange->numberOfLins - 1] =
      probTab.prob[probTab.numberOfExpectedDistances - 1][z];
  
  sum = 0.0;
  count = 0;
  for (x = 0; x < probTab.numberOfMeasuredDistances - 1; x++){
    sum += probTab.prob[probTab.numberOfExpectedDistances - 1][x] *
      probTab.count[probTab.numberOfExpectedDistances - 1][x];
    count += probTab.count[probTab.numberOfExpectedDistances - 1][x];
  }
  if (count > 0)
    sum /= count;
  else
    sum = 0;

  *maxRangeMinLevel = sum;
}
  

void
computeMaxRangeParamsLin( distProbTable probTab,
			  float minProbFar,
			  int maxRangeDelta,
			  multiLinFunction *maxRange)
{
  register int i;
  vector values;
  int maxRangeIndex = probTab.numberOfMeasuredDistances - 1;


  maxRange->numberOfLins = 1;

  values.numberOfValues = probTab.numberOfExpectedDistances;
  
  for (i = 0; i < maxRangeDelta; i++){
    values.value[i] = 0;
    values.count[i] = 0;
  }
  
  for (i = maxRangeDelta; i < probTab.numberOfExpectedDistances - maxRangeDelta; i++){
    values.value[i] = probTab.prob[i][maxRangeIndex];
    values.count[i] = probTab.count[i][maxRangeIndex];
  }
  
  for (i = probTab.numberOfExpectedDistances - maxRangeDelta;
       i < probTab.numberOfExpectedDistances; i++){
    values.value[i] = 0;
    values.count[i] = 0;
  }

  computeLinearRegression( values, &(maxRange->probShort[0]),
			   &(maxRange->probFar[0]));

  fprintf(stderr, "%f %f\n", maxRange->probShort[0], maxRange->probFar[0]);
  
}




void
oldnormalize( distProbTable *tab, int delta)
{
  int x, y;
  float sum, sum1;
  float factor;
  int startY, endY;
  
  for (x = 0; x < tab->numberOfExpectedDistances; x++){
    sum = sum1 = 0.0;
    if (x - delta < 0) startY = 0; else startY = x - delta;
    if (x + delta >= tab->numberOfMeasuredDistances)
      endY = tab->numberOfMeasuredDistances-1;
    else
      endY = x + delta;
    for ( y = 0; y < startY; y++)
      if (tab->count[x][y] > 0)
	sum += tab->prob[x][y];
    
    for ( y = startY ; y < endY; y++)
      if (tab->count[x][y] > 0)
	sum1 += tab->prob[x][y];
    
    for ( y = endY; y < tab->numberOfMeasuredDistances; y++)
      if (tab->count[x][y] > 0)
	sum += tab->prob[x][y];

    if (sum1 != 0.0)
      factor = sum1 / (1 - sum);
    else
      factor = 1.0;
    for ( y = startY ; y < endY; y++)
      tab->prob[x][y] /= factor;
  }
}

void
normalize( distProbTable *tab, int delta)
{
  int x, y;
  float sum;
  
  for (x = 0; x < tab->numberOfExpectedDistances; x++){
    sum = 0.0;
    for ( y = 0; y < tab->numberOfMeasuredDistances; y++)
      if (tab->count[x][y] > 0)
        sum += tab->prob[x][y];
    if (sum > 0.0)
      for ( y = 0; y < tab->numberOfMeasuredDistances; y++)
        tab->prob[x][y] /= sum;
  }
}


void
dumpDistProbFunctionDifference(char *fileName, distProbTable tab1,
			       distProbTable tab2)
{
  int x, y;
  FILE *fp = fopen(fileName, "wt");
  
  for ( y = 0; y < tab2.numberOfMeasuredDistances; y++){
    for (x = 0; x < tab1.numberOfExpectedDistances; x++){
      if (tab1.count[x][y] > 0)
	fprintf(fp, "%f\n", tab1.prob[x][y]-tab2.prob[x][y]);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
}



void
dumpDaniel( char *fileName,
	    distProbTable tab,
	    int resolution)
{
  int x, y;
  FILE *fp = fopen(fileName, "wt");
  
  fprintf(fp, "# resolution of expected values: %f\n", (float) resolution);
  
  for (x = 0; x < tab.numberOfExpectedDistances; x++){
    for ( y = 0; y < tab.numberOfMeasuredDistances; y++)
      fprintf(fp, "%f %f %f\n",
	      (float) resolution*x,
	      (float) resolution*y,
	      (float) tab.prob[x][y]);
    fprintf(fp, "\n");
  }
  fclose(fp);
}



void
printSingleRow(char *fileName, int row,
	       float maxDist, float yScale,
	       distProbTable tab,  distProbTable estimated)
{
  int x;
  float step = 1.0;
  FILE *fp = fopen(fileName, "wt");
  
  if ( maxDist > 0.0)
    step = maxDist / estimated.numberOfMeasuredDistances;
  
  fprintf(fp, "# row number: %d\n", row);
  
  for ( x = 0; x < estimated.numberOfMeasuredDistances; x++)
    if (tab.count[row][x] > 0)
      fprintf(fp, "%f %f %f\n",
	      x * step,
	      yScale * (float) tab.prob[row][x], yScale * estimated.prob[row][x]);
  
  fclose(fp);
}



void
estimateMinProbs(distProbTable tab, int delta, float *minProbShort,
		 float *minProbFar)
{
  int x, y;
  int countShort = 0, countFar=0;

  *minProbShort = 0.0;
  *minProbFar = 0.0;
  
  for (x = 0; x < tab.numberOfExpectedDistances-1; x++){
    for (y = 0; y < x - delta; y++){
      *minProbShort += tab.prob[x][y]*tab.count[x][y];
      countShort += tab.count[x][y];
    }
    for (y = x + delta; y < tab.numberOfMeasuredDistances - 1; y++){
      *minProbFar += tab.prob[x][y]*tab.count[x][y];
      countFar += tab.count[x][y];
    }
  }
  printf("********* %f %d %f %d\n", *minProbShort, countShort,
	 *minProbFar, countFar);

  if (countShort == 0)
    *minProbShort = 0.0;
  else
    *minProbShort /= countShort;
  if (countFar == 0)
    *minProbFar = 0.0;
  else
    *minProbFar /= countFar;
}



void
estimateStdDevs( distProbTable tab,
		 int delta,
		 float minProbShort,
		 float minProbFar,
		 float factor,
		 float *sigmaShort,
		 float *sigmaFar)
{
  int x;
  vector stdDevs;
  
  *sigmaShort = 0.0;
  *sigmaFar = 0.0;
  
  for (x = 0; x < tab.numberOfExpectedDistances; x++)
     stdDevs.value[x]=stdDevs.count[x]=0;
  
  for (x = delta; x < tab.numberOfExpectedDistances-2*delta; x++){
     if (tab.count[x][x-1] > 0 && tab.count[x][x] > 0 &&
	 tab.count[x][x+1] > 0){
	float value = factor *
	   fMax(fMax(tab.prob[x][x-1],tab.prob[x][x]),tab.prob[x][x+1]);
	stdDevs.value[x] = 1 / (SQRT2PI * value);
     }
     else
	stdDevs.value[x] = 0;
/*     value = tab.prob[x][x];*/
     stdDevs.count[x] = tab.count[x][x];
     /* fprintf(stderr, "val %d: %d, %f\n", x, stdDevs.count[x],stdDevs.value[x]); */
  }
  
  stdDevs.numberOfValues = tab.numberOfExpectedDistances;
  
  computeLinearRegression( stdDevs, sigmaShort,  sigmaFar);
  if (*sigmaShort < 0)
     *sigmaShort = *sigmaFar / 2;
  fprintf(stderr, "Regression: %f -> %f\n", *sigmaShort, *sigmaFar);

{
   FILE *fp;
   fp = fopen("stddev", "wt");
   for (x = 0; x < tab.numberOfExpectedDistances; x++){
      float stddev = linFunction(x, 0,
			  *sigmaShort,
			  tab.numberOfExpectedDistances - 1, *sigmaFar);
      
      fprintf(fp, "%d %f %f %f %d\n", x, stdDevs.value[x],
	      stddev,
	      gauss(x, stddev, x),
	      stdDevs.count[x]);
   }
   fclose(fp);
}
/*  *sigmaShort = 1 / (SQRT2PI * *sigmaShort);
  *sigmaFar = 1/ (SQRT2PI * *sigmaFar); */

}


void
initializeProbTable(distProbTable tab1, distProbTable *tab2)
{
  int x, y;
  
  tab2->numberOfMeasuredDistances = tab1.numberOfMeasuredDistances;
  tab2->numberOfExpectedDistances = tab1.numberOfExpectedDistances;
  
  for (x = 0; x < tab2->numberOfExpectedDistances; x++)
    for (y = 0; y < tab2->numberOfMeasuredDistances; y++){
      tab2->prob[x][y] = UNKNOWN;
      tab2->count[x][y] = tab1.count[x][y];
    }
}

void
setCountsProbTable(distProbTable *tab2)
{
  int x, y;
  
  for (x = 0; x < tab2->numberOfExpectedDistances; x++)
    for (y = 0; y < tab2->numberOfMeasuredDistances; y++){
      tab2->count[x][y] = 1;
    }
}



int
main(argc, argv)
     int argc;
     char *argv[];
{

  int resolution;
  int delta = DELTA;
  float factor = 1.0;
  float maxDist = 0.0, scale = 1.0;
  int maxRangeDelta = 4;
  
  distProbTable probTab, estimatedProbTab;
  probFunctionParameters params;
  int x;
  
  if (argc < 6){
    fprintf(stderr, "usage: %s dataFile resolution delta factor maxdelta [maxDist] [scale]\n", argv[0]);
    exit(1);
  }

  resolution = atoi(argv[2]);
  if (resolution < 1 || resolution > 100){
    fprintf(stderr, "invalid resolution: %d\n", resolution);
    exit(-1);
  }

  delta = atoi(argv[3]);
  if (delta < 1 || delta > 100){
    fprintf(stderr, "invalid delta: %d\n", delta);
    exit(-1);
  }

  factor = atof(argv[4]);
  if (factor <= 0 || factor > 100){
    fprintf(stderr, "invalid factor: %f\n", factor);
    exit(-1);
  }

  maxRangeDelta = atoi(argv[5]);
  if (maxRangeDelta <= 0 || maxRangeDelta > 100){
    fprintf(stderr, "invalid maxRangeDelta: %d\n", maxRangeDelta);
    exit(-1);
  }

  if ( argc > 6)
    maxDist = atof( argv[6]);

  if ( argc > 7)
    scale = atof( argv[7]);

  readSampleData( argv[1], resolution, &probTab);

  initializeProbTable(probTab, &estimatedProbTab);

  setCountsProbTable(&estimatedProbTab);
  
  estimateMinProbs( probTab, delta, &params.minProbShort,
		    &params.minProbFar);
  estimateStdDevs( probTab, delta,
		   params.minProbShort,
		   params.minProbFar,
		   factor,
		   &params.sigmaShort,
		   &params.sigmaFar);
  computeMaxRangeParamsLin( probTab, params.minProbFar, maxRangeDelta,
			    &(params.maxRange));

  
  printf("sigma: (%f->%f) minProb: (%f->%f)\n",
	 params.sigmaShort, params.sigmaFar,
	 params.minProbShort, params.minProbFar);
  for (x = 0; x < params.maxRange.numberOfLins; x++){
    printf("MaxRange: %d, (%f->%f)\n",x, 
	   params.maxRange.probShort[x], params.maxRange.probFar[x]);
  }
  
  computeDistProbFunction( params, &estimatedProbTab,
			   probTab.numberOfExpectedDistances,
			   probTab.numberOfExpectedDistances,
			   maxRangeDelta);

  
  dumpDistProbFunction("estimated.gnu",
		       maxDist, scale, estimatedProbTab);

  if (0) {
    int x;
    char file[30];
    
    for (x = 0; x<probTab.numberOfExpectedDistances; x++){
      sprintf(file, "estrow%03d", x);
      printSingleRow(file, x, maxDist, scale, probTab, estimatedProbTab);
    }
  }
  dumpDistProbFunction("measured.gnu", maxDist, scale, probTab);
  dumpDistProbFunctionDifference("difference.gnu", probTab, estimatedProbTab);
  
  normalize(&estimatedProbTab, delta);
  {
    int x;
    char file[30];
    
    for (x = 0; x<probTab.numberOfExpectedDistances; x++){
      sprintf(file, "row%03d", x);
      printSingleRow(file, x, maxDist, scale, probTab, estimatedProbTab);
    }
  }
  
  dumpDistProbFunction("normalized.gnu", maxDist, scale, estimatedProbTab);
  dumpDistProbFunctionDifference("normeddiff.gnu",probTab, estimatedProbTab);
  dumpDaniel("danielfloat",estimatedProbTab,resolution);

  exit(0);
}




