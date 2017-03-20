#include <math.h>
#include <stdio.h>
#include <values.h>
#include <stdlib.h>

#define NUMBER_OF_INFINITY_DISTANCES 1000
#define MAX_DISTANCE 500.0
#define BUFFLEN 80
#define SQRT2PI 2.5066283

#define SURROUND_SIZE NUMBER_OF_INFINITY_DISTANCES
/* #define MAP_STDDEV 10.0 */
#define MAP_STDDEV 4.2

#define probOfDetectHuman 0.99
/* #define probOfDetectHuman params.probOfDetectObstacle */

double sensorGaussTab[2*SURROUND_SIZE+1];
double mapGaussTab[2*SURROUND_SIZE+1];
double summedGaussTab[2*SURROUND_SIZE+1];

typedef double probability;

typedef struct {
  int numberOfMeasuredDistances;
  int numberOfExpectedDistances;
  probability** prob;
} distProbTable;

typedef struct {
  double sigma;
  double probOfMiss;
  double probOfDetectObstacle;
} probFunctionParameters;


double
fMin( double x, double y)
{
 return ( x > y ? y : x);
}


double
fMax(double x, double y)
{
 if (x > y)
   return x;
 else
   return y;
}
     

int
iMin( int x, int y)
{
 return ( x > y ? y : x);
}


int
iMax(int x, int y)
{
 if (x > y)
   return x;
 else
   return y;
}
     
int iAbs( int x)
{
  return ( x > 0) ? x : -x;
}

double
fSqr(double x){
  return x*x;
}
     

double
gauss( double x, double sigma, double mean)
{
  double tmp;
    
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

int
checkIfPerform( char* question)
{
  char inputString[2];

  fprintf(stderr, "%s", question);
  gets( inputString);

  return (inputString[0] == 'y');
}


double
linFunction( double x, double x1, double y1, double x2, double y2)
{
  return y1 + x * ((y2 - y1) / (x2 - x1 + 1));
  
}

probability***
allocate3D( int numberOfDistances)
{
  probability*** probs;
  
  int sensorVal, dh;
  
  probs = (probability***)
    malloc( numberOfDistances * sizeof( probability**));
  
  for ( sensorVal = 0; sensorVal < numberOfDistances; sensorVal++) {
    probs[sensorVal] =
      (probability**) malloc( numberOfDistances * sizeof(probability*));
  }
  for ( sensorVal = 0; sensorVal < numberOfDistances; sensorVal++) {
    for ( dh = 0; dh < numberOfDistances; dh++) 
      probs[sensorVal][dh] = (probability*)
	malloc( numberOfDistances * sizeof(probability));
  }
  return probs;
}

probability** 
allocate2D( int numberOfDistances)
{
  probability** probs;
  int sensorVal;
    
  probs = (probability**)
    malloc( numberOfDistances * sizeof( probability*));
  
  for ( sensorVal = 0; sensorVal < numberOfDistances; sensorVal++) {
    probs[sensorVal] = (probability*)
      malloc( numberOfDistances * sizeof(probability));
  }
  return probs;
}

/**************************************************************************
**************************************************************************
* Functions for the probabilities of distance readings.
**************************************************************************
**************************************************************************/

void
computeDistProbFunction( probFunctionParameters params,
			 int numberOfDistances,
			 distProbTable *tab,
			 int minDist)
{
  register int x,y;
  double probOfHit = 1.0 - params.probOfMiss;
  double infinityBeam[NUMBER_OF_INFINITY_DISTANCES];
  
  double combinedSigma = sqrt( (fSqr( params.sigma) + fSqr(MAP_STDDEV)));

  tab->numberOfExpectedDistances = numberOfDistances;
  tab->numberOfMeasuredDistances = numberOfDistances;
  tab->prob = NULL;
  
  tab->prob = allocate2D( numberOfDistances);

  for ( y = 0; y < SURROUND_SIZE + 1; y++) {
    sensorGaussTab[SURROUND_SIZE+y] = sensorGaussTab[SURROUND_SIZE-y]
      = gauss( y, params.sigma, 0);
    mapGaussTab[SURROUND_SIZE+y] = mapGaussTab[SURROUND_SIZE-y]
      = gauss( y, MAP_STDDEV, 0);
    summedGaussTab[SURROUND_SIZE+y] = summedGaussTab[SURROUND_SIZE-y]
      = gauss( y, combinedSigma, 0);
  }

  
  for (x = 0; x < tab->numberOfExpectedDistances; x++){
    
    double geometrySum = 0.0, probOfHit = 1.0 - params.probOfMiss;
    double pObstacle, pGeometry;
    double infinitySum = 0.0;
    double sum = 0.0;

    for (y = 0; y < NUMBER_OF_INFINITY_DISTANCES; y++) {
      
      pGeometry = (1.0 - infinitySum) * probOfHit;
      
      pObstacle = (1.0 - geometrySum) *
	params.probOfDetectObstacle * summedGaussTab[SURROUND_SIZE + x - y];

      geometrySum += pGeometry;
      
      infinityBeam[y] =	1.0 - (1.0 - pGeometry) * (1.0 - pObstacle);

      infinitySum += infinityBeam[y];
    }
    
    for ( y = 0; y < tab->numberOfMeasuredDistances - 1; y++) {
      tab->prob[y][x] = infinityBeam[y] / infinitySum;
      sum += tab->prob[y][x];
    }
    if (1) {
      tab->prob[y][x] = 1.0 - sum;
      fprintf( stderr, "inf %f --> %f\n", sum, tab->prob[y][x]);
    }
    else {
      tab->prob[y][x] = infinityBeam[y] / infinitySum;
      fprintf( stderr, "inf %f --> %f\n", sum, tab->prob[y][x]);
    }
  }
}

void
computeUnexpectedProbFunction( distProbTable* tab,
			       distProbTable distTab)
{

  register int x,y;
  int numberOfDistances;
  
  *tab = distTab;
  numberOfDistances = tab->numberOfExpectedDistances;

  tab->prob = allocate2D( numberOfDistances);

  for (x = 0; x < tab->numberOfExpectedDistances; x++){
      
    double sum = 0.0;
   
    for ( y = 0; y < tab->numberOfMeasuredDistances; y++) {
      sum += distTab.prob[y][x];

      tab->prob[y][x] = 1.0 - sum;
    }
  }
}

void
computeDistProbFunctionGivenHuman( probFunctionParameters params,
				   int numberOfDistances,
				   double ***probs,
				   int human)
{
  register int x,y;
  double probOfHit = 1.0 - params.probOfMiss;
  double infinityBeam[NUMBER_OF_INFINITY_DISTANCES];
  
  for (x = 0; x < numberOfDistances; x++){
      
    double infinitySum = 0.0;
    double sum = 0.0;

    double geometryObstacleSum = 0.0, geometryHumanSum = 0.0;
    
    double pHuman, pObstacle, pGeometry;

    for ( y = 0; y < NUMBER_OF_INFINITY_DISTANCES; y++) {
      
      pGeometry = (1.0 - infinitySum) * probOfHit;
      
      pHuman    = ( 1.0 - geometryObstacleSum) *
	probOfDetectHuman * sensorGaussTab[SURROUND_SIZE + human - y];
      
      pObstacle = (1.0 - geometryHumanSum) *
	params.probOfDetectObstacle * summedGaussTab[SURROUND_SIZE + x - y];

      infinityBeam[y] = 1.0 - ( (1.0 - pGeometry) * (1.0 - pObstacle) * (1.0 - pHuman)); 
      
      geometryObstacleSum += 1.0 - ( 1.0 - pGeometry) * ( 1.0 - pObstacle); 
      geometryHumanSum += 1.0 - ( 1.0 - pGeometry) * ( 1.0 - pHuman); 
	
      infinitySum += infinityBeam[y];
    }
    

    for ( y = 0; y < numberOfDistances - 1; y++) {
      probs[y][human][x] = infinityBeam[y] / infinitySum;
      sum += probs[y][human][x];
    }

    probs[y][human][x] = 1.0 - sum;
  }
}

void
dumpHumanProbs( probFunctionParameters params,
		double*** pOfSGivenDhO,
		double*** pOfDhGivenSO,
		int numberOfDistances,
		int obstacle)
{
  int human, distance, y;
  float deltaDist = MAX_DISTANCE / numberOfDistances;

  char fileName[255];
  FILE *fp;

  if ( obstacle < 0) {
     
    int obstacle;

    /* Just dump the human prob function. */
    sprintf( fileName, "laser-human-detection");
    if ((fp = fopen(fileName, "wt")) == NULL) {
      fprintf(stderr, "Cannot open %s.\n", fileName);
      exit(-1);
    }
    else
      fprintf(stderr, "Dump  human probabilities into %s.\n", fileName);

    fprintf(fp, "pOfDhGivenSO %d %f\n", numberOfDistances, deltaDist);

    for ( human = 0; human < numberOfDistances; human++)
      for ( distance = 0; distance < numberOfDistances; distance ++) 
	for ( obstacle = 0; obstacle < numberOfDistances; obstacle++)
	  fprintf(fp, "%f ", (float) fMax(0.0, pOfDhGivenSO[human][distance][obstacle]));
    fclose(fp);
    return;
  }

  /* Probs of distances given human and obstacle. */
  for ( human = 0; human < numberOfDistances; human += 10) {
    sprintf( fileName, "sGDhO%d-%d", human, obstacle);
    fp = fopen(fileName, "wt");
    
    fprintf(fp, "# human and obstacle number: %d %d\n", human, obstacle);
    
    for ( distance = 0; distance < numberOfDistances -1; distance++)
      fprintf(fp, "%f %f\n", distance * deltaDist, (float) pOfSGivenDhO[distance][human][obstacle]);
    
    fclose(fp);
  }

  /* Probs of human given distance and obstacle. */
  for ( distance = 0; distance < numberOfDistances; distance += 10) {
    sprintf( fileName, "dhGSO%d-%d", distance, obstacle);
    fp = fopen(fileName, "wt");
    
    fprintf(fp, "# human and obstacle number: %d %d\n", human, obstacle);
    
    for ( human = 0; human < numberOfDistances -1; human++)
      fprintf(fp, "%f %f\n", human * deltaDist, (float) pOfDhGivenSO[human][distance][obstacle]);
    
    fclose(fp);
  }
  
  
  /* Apriori probs of human. */
  for ( human = 0; human < numberOfDistances; human += 10) {
    sprintf( fileName, "hProbs%d", human);
    fp = fopen( fileName, "wt");
    for ( y = 0; y < numberOfDistances -1; y++)
      fprintf(fp, "%f %d %f\n", y * deltaDist, y == human,
	      (float) probOfDetectHuman * sensorGaussTab[SURROUND_SIZE + human - y]);
    
    fclose(fp);
  }

  /* Apriori probs of obstacle. */
  sprintf( fileName, "oProbs%d", obstacle);
  fp = fopen( fileName, "wt");
  for ( y = 0; y < numberOfDistances -1; y++)
    fprintf(fp, "%f %f %f\n", y * deltaDist, (float) mapGaussTab[SURROUND_SIZE + obstacle - y],
	    (float) probOfDetectHuman * summedGaussTab[SURROUND_SIZE + obstacle - y]);
  
  fclose(fp);

  /* Apriori probs of unknown. */
  sprintf( fileName, "uProbs");
  fp = fopen( fileName, "wt");
  for ( y = 0; y < numberOfDistances -1; y++)
    fprintf(fp, "%f %f\n", y * deltaDist, (float) 1.0 - params.probOfMiss);
  
  fclose(fp);
}
     

void
computeHumanProbFunction( probFunctionParameters params,
			  distProbTable* tab,
			  distProbTable distTab,
			  int printRow)
{
  static int firstTime = 1;
  register int sensorVal,dh;
  int numberOfDistances;
  double pOfHuman;
  double probOfHumanNorm = 1.0 / mapGaussTab[SURROUND_SIZE];
  int obstacleDistance;
  static probability*** pOfDhGivenSO = NULL;
  static probability*** pOfSGivenDhO = NULL;
  static probability** pOfBlockedGivenSO = NULL;
  static probability** test = NULL;

  *tab = distTab;
  numberOfDistances = tab->numberOfExpectedDistances;
  tab->prob = allocate2D( numberOfDistances); 

  if ( firstTime) {
    
    pOfSGivenDhO = allocate3D( numberOfDistances);
    
    pOfDhGivenSO = allocate3D( numberOfDistances);
    
    pOfBlockedGivenSO = allocate2D( numberOfDistances); 
    
    test = allocate2D( numberOfDistances); 
    
    firstTime = 0;
  }
    
  pOfHuman = 1.0 - params.probOfMiss;

  /* First compute p(s|dh,dobs). */
  for ( dh = 0; dh < tab->numberOfMeasuredDistances; dh++) 
    computeDistProbFunctionGivenHuman( params, numberOfDistances,
				       pOfSGivenDhO, dh);


  if (0) {
    double sum;
    for (sensorVal = 0; sensorVal < numberOfDistances; sensorVal++){
      for ( obstacleDistance = 0; obstacleDistance < numberOfDistances; obstacleDistance++) {
	 sum = 0.0;
	for ( dh = 0; dh < numberOfDistances; dh++) {
	  sum += pOfSGivenDhO[sensorVal][dh][obstacleDistance] * pOfHuman * (1.0 - probOfHumanNorm * mapGaussTab[SURROUND_SIZE + obstacleDistance - dh]);
	}
	test[sensorVal][obstacleDistance] = sum;
      }
    }
    for ( obstacleDistance = 0; obstacleDistance < numberOfDistances; obstacleDistance++) {
      sum = 0.0;
      for (sensorVal = 0; sensorVal < numberOfDistances; sensorVal++) {
	sum += test[sensorVal][obstacleDistance];
      }
      for (sensorVal = 0; sensorVal < numberOfDistances; sensorVal++) {
	if (0) test[sensorVal][obstacleDistance] /= sum;
	if (obstacleDistance == printRow) printf( "%f %f  %f\n", test[sensorVal][printRow], (float) distTab.prob[sensorVal][printRow], test[sensorVal][printRow] / distTab.prob[sensorVal][printRow]);
	distTab.prob[sensorVal][obstacleDistance] = test[sensorVal][obstacleDistance];
      }
    }
  }

  /* All obstacle distances. */
  for ( obstacleDistance = 0; obstacleDistance < numberOfDistances; obstacleDistance++) {

    /* All measured distances. */
    for (sensorVal = 0; sensorVal < numberOfDistances; sensorVal++){
      
      /* All human distances. */
      for ( dh = 0; dh < numberOfDistances; dh++) {
	
	pOfDhGivenSO[dh][sensorVal][obstacleDistance] =
	  pOfSGivenDhO[sensorVal][dh][obstacleDistance] *
	  pOfHuman * (1.0 - probOfHumanNorm * mapGaussTab[SURROUND_SIZE + obstacleDistance - dh]) /
	  distTab.prob[sensorVal][obstacleDistance];
      }
    }

    /* All measured distances. */
    for (sensorVal = 0; sensorVal < numberOfDistances; sensorVal++){
      
      double notBlocked = 1.0;
      
      /* All human distances. */
      for ( dh = 0; dh <= obstacleDistance; dh++) {
	notBlocked *= ( 1.0 - pOfDhGivenSO[dh][sensorVal][obstacleDistance]);
      }
      pOfBlockedGivenSO[sensorVal][obstacleDistance] = 1.0 - notBlocked;
    }
  }
  
  dumpHumanProbs( params,
		  pOfSGivenDhO, pOfDhGivenSO,
		  numberOfDistances,
		  printRow);


  
  /* Copy the values. */
  for ( obstacleDistance = 0; obstacleDistance < numberOfDistances; obstacleDistance++) 
    for (sensorVal = 0; sensorVal < numberOfDistances; sensorVal++) 
      tab->prob[sensorVal][obstacleDistance] = pOfBlockedGivenSO[sensorVal][obstacleDistance];

  /* Set the boarders for max range. */
/* #define MAX_RANGE 127 */
/*   for (sensorVal = 0; sensorVal < numberOfDistances; sensorVal++)  */
/*     tab->prob[sensorVal][MAX_RANGE] = 1.0; */

}

void
dumpDistProbFunction( char *fileName, probFunctionParameters params,
		      distProbTable probTab, int skip)
{
  
  int x, y;
  double deltaDist = MAX_DISTANCE / probTab.numberOfExpectedDistances;
  FILE *fp = fopen(fileName, "wt");

  fprintf( fp, "# %d %d pOfMiss %f std %f pOfDetect %f\n",
	   probTab.numberOfMeasuredDistances, probTab.numberOfExpectedDistances,
	   params.probOfMiss, params.sigma, params.probOfDetectObstacle);

  for ( y = 0; y < probTab.numberOfMeasuredDistances; y++){
    if ( y % skip == 0 || y == probTab.numberOfMeasuredDistances - 1) {
      for (x = 0; x < probTab.numberOfExpectedDistances; x++){
	if ( x % skip == 0 || x == probTab.numberOfExpectedDistances - 1)
/*  	  fprintf(fp, "%f %f %.3e\n", x * deltaDist, y * deltaDist, probTab.prob[y][x]);  */
 	  fprintf(fp, "%f\n", probTab.prob[y][x]); 
      }
      fprintf(fp, "\n");
    }
  }
}



void
normalize( distProbTable *tab)
{
  int x, y;
  double sum;
  
  for (x = 0; x < tab->numberOfExpectedDistances; x++){
    sum = 0.0;
    for ( y = 0; y < tab->numberOfMeasuredDistances; y++)
      sum += tab->prob[y][x];
    for ( y = 0; y < tab->numberOfMeasuredDistances; y++)
      tab->prob[y][x] /= sum;
  }
}


void
convolveWithAverage( distProbTable *tab)
{
  
  int measuredCnt, expectedCnt;

#define TAB_WEIGHT 0.02

  double average = 1.0 / tab->numberOfMeasuredDistances;

  for ( expectedCnt = 0;
	expectedCnt < tab->numberOfExpectedDistances; expectedCnt++){    
    for ( measuredCnt = 0;
	  measuredCnt < tab->numberOfMeasuredDistances; measuredCnt++) 
      tab->prob[measuredCnt][expectedCnt] =
	TAB_WEIGHT * tab->prob[measuredCnt][expectedCnt] + (1.0 - TAB_WEIGHT) * average;
  }    
}



void
divideByPOfFeature( distProbTable *tab)
{
  int measuredCnt, expectedCnt;

  double pOfMeasured;
  
  /* Compute probability of measuring no max range. */
  for ( measuredCnt = 0;
	measuredCnt < tab->numberOfMeasuredDistances; measuredCnt++) {

    pOfMeasured = 0.0;

    for ( expectedCnt = 0;
	  expectedCnt < tab->numberOfExpectedDistances; expectedCnt++){
      pOfMeasured += tab->prob[measuredCnt][expectedCnt];
    }

    pOfMeasured /= tab->numberOfExpectedDistances;
      
    for ( expectedCnt = 0;
	  expectedCnt < tab->numberOfExpectedDistances; expectedCnt++){
      tab->prob[measuredCnt][expectedCnt] /= pOfMeasured;
    }
  }
}




void
printSingleRows( int row, char* prefix, distProbTable tab)
{
  int start, end;
  
  if ( row < 0) {
    start = 0; end = tab.numberOfExpectedDistances;
  }
  else {
    start = row; end = row+1;
  }

  for ( row = start; row < end; row++) {
    int x;
    char fileName[255];
    FILE *fp;
    double deltaDist = MAX_DISTANCE / tab.numberOfExpectedDistances;
    
    sprintf( fileName, "%s%d", prefix, row);
    fp = fopen(fileName, "wt");
    
    fprintf(fp, "# row number: %d\n", row);
    
    for ( x = 0; x < tab.numberOfMeasuredDistances - 1; x++)
      fprintf(fp, "%f %f\n", x * deltaDist, (float) tab.prob[x][row]);
    
    fprintf(fp, "%f %f\n", MAX_DISTANCE, (float) tab.prob[x][row]);
    
    fclose(fp);
  }
}

int
main(argc, argv)
     int argc;
     char *argv[];
{  
  distProbTable probTab;
  distProbTable unexpectedTab;
  distProbTable humanProbTab;
  probFunctionParameters params;
  int numberOfDistances = 128;
  int printRow = -1, skip = 1, minDist = -1;

  if (argc < 5){
    fprintf(stderr, "usage: %s pOfMiss stdDev pOfDetect skip numberDist minDist.\n", argv[0]);
    fprintf(stderr, "Laser values: 0.989 4.5 0.59 2 128 10\n");
    fprintf(stderr, "Sonar values: 0.982 5.5 0.5 2 128 5\n");
    /* Next values if weight for geometry is fixed to one. */
    fprintf(stderr, "Laser values: 0.995 1.2 0.77 2 128 10\n");
    fprintf(stderr, "Sonar values: 0.992 4.0 0.75 2 128 5\n");
    exit(1);
  }
  
  params.probOfMiss = atof(argv[1]);
  params.sigma = atof(argv[2]);
  params.probOfDetectObstacle = atof(argv[3]);
  
  if ( argc > 4)
    skip = atoi( argv[4]);
  
  if ( argc > 5)
    numberOfDistances = atoi( argv[5]);

  if ( argc > 6)
    minDist = atoi( argv[6]);

  if ( argc > 7)
    printRow = atoi( argv[7]);

  computeDistProbFunction( params, numberOfDistances, &probTab, minDist);
  printSingleRows( printRow, "norm-", probTab);
  dumpDistProbFunction("2D-norm", params, probTab, skip);
  
  if ( checkIfPerform("Convolved?")) {
    convolveWithAverage(&probTab);
    printSingleRows( printRow, "conv-", probTab);
  }
  
  if ( checkIfPerform("Merge?")) {
    computeHumanProbFunction( params, &humanProbTab, probTab, printRow);
    printSingleRows( printRow, "merge-", humanProbTab);
    dumpDistProbFunction("2D-human", params, humanProbTab, skip);
  }
  
  if ( checkIfPerform("Divided?")) {
    divideByPOfFeature(&probTab);
    dumpDistProbFunction("2D-div", params, probTab, skip);
    printSingleRows( printRow, "div-", probTab);
  }
  
  if ( checkIfPerform("Geometric?")) {
    double save = params.probOfDetectObstacle;
    params.probOfDetectObstacle = 0.0;
    computeDistProbFunction( params, numberOfDistances, &probTab, minDist);
    printSingleRows( printRow, "geom-", probTab);
    dumpDistProbFunction("2D-geom", params, probTab, skip);
    params.probOfDetectObstacle = save;
  }  

  if ( checkIfPerform("Gaussian?")) {
    double save = params.probOfMiss;
    params.probOfMiss = 1.0;
    computeDistProbFunction( params, numberOfDistances, &probTab, minDist);
    printSingleRows( printRow, "gauss-", probTab);
    dumpDistProbFunction("2D-gauss", params, probTab, skip);
    params.probOfMiss = save;
  }  

  if ( checkIfPerform("Unexpected?")) {
    computeUnexpectedProbFunction( &unexpectedTab, probTab);   
    printSingleRows( printRow, "unexp-", unexpectedTab);
  }
  
  putc(7, stderr);
  exit(0);
}


