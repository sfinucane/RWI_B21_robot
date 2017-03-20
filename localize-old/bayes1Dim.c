#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <sys/time.h>

#include <EZX11.h>

#include "inline.h"

#define NUMBER 81
#define SCALEX 10
#define SCALEY 30
#define VERBOSE 0
#define IMPOSSIBLE 0.0001
#define NUMBER_OF_FEATURES 10

typedef float (*probOfFeatureFunction) ( int feature, int position, float map[]);

typedef struct sensorType {
  int numberOfFeatures;
  probOfFeatureFunction probOfFeature;
} sensorType;
  
float
probOfDist( int dist, int pos, float map[])
{
  int i;

#define HIT_PROB 0.999

  float missProb = (1.0 - HIT_PROB) / (float) (NUMBER_OF_FEATURES - 1);

  for ( i = 0; i < dist; i++)
    if ( pos + i >= NUMBER)
      return missProb;
    else if ( map[pos+i] > 0.2)
      return missProb;
  
  if ( pos + dist == NUMBER)
    return HIT_PROB;
  else if (map[pos+dist] > 0.2)
    return HIT_PROB;
  else if ( dist == NUMBER_OF_FEATURES - 1)
    return HIT_PROB;
  else
    return missProb;
}


void
show( EZXW_p win, float probs[], int number)
{
  int i;
  float colDist = 100;
  float min = 1e20, max = 0;
  float probDist;
  
  for (i = 0; i < number; i++) {
    if ( probs[i] < min)
      min = probs[i];
    if ( probs[i] > max)
      max = probs[i];
  }
  probDist = max - min;
  
  EZX_NoMotionEvents();

#define COL_WHITE 154
  
  for ( i = 0; i < number; i++) {
    if ( (max <= 1.0 && min >= 0.0) || probDist == 0.0) {
      EZX_SetColor( COL_WHITE - probs[i] * colDist);
    }
    else {
      int normedCol = COL_WHITE - ( probs[i] - min) / probDist * colDist;
      if (VERBOSE) fprintf(stderr, "%f %f %f ---> %d\n", min, max, probs[i], normedCol);
      EZX_SetColor( normedCol);
    }
    EZX_FillRectangle( win, i * SCALEX, 0, SCALEX, SCALEY);
  }
  EZX_Flush();
}


void
normalize( float probs[NUMBER])
{
  int i;
  float sum = 0.0;

  for ( i = 0; i < NUMBER; i++) 
    sum += probs[i];
  
  for ( i = 0; i < NUMBER; i++) 
    if ( sum != 0.0)
      probs[i] /= sum;
    else
      probs[i] = 1.0 / NUMBER;
}

void
copyVector( float source[NUMBER], float dest[NUMBER])
{
  int i;

  for ( i = 0; i < NUMBER; i++)
    dest[i] = source[i];
}


void
applySmoothedSingleStep( int right,
			 float probs[NUMBER],
			 float probsAfterJump[NUMBER])
{
  int i;

  float weight1 = 0.98;
  float weight2 = 0.01;

  int step = right ? 1 : -1;
  
  for ( i = 0; i < NUMBER; i++) {

    int startPoint = i - step;

    if ( (startPoint >= 0) &&  (startPoint < NUMBER)) {
      if ( probs[startPoint] > 0.001)
	probsAfterJump[i] = 0.97 * probs[startPoint];
      else
	probsAfterJump[i] = probs[startPoint];
    }
    else
      probsAfterJump[i] = IMPOSSIBLE;
  }
  return;
  
  for ( i = 0; i < NUMBER; i++) {

    int startPoint = i - step;

    if ( (startPoint >= 0) &&  (startPoint < NUMBER)) 
      probsAfterJump[i] = weight1 * probs[startPoint];
    else
      probsAfterJump[i] = weight1 * IMPOSSIBLE;
    
     startPoint -= 1;
    if ( (startPoint >= 0) &&  (startPoint < NUMBER)) 
      probsAfterJump[i] += weight2 * probs[startPoint];
    else
      probsAfterJump[i] += weight2 * IMPOSSIBLE;

    startPoint += 2;
    if ( (startPoint >= 0) &&  (startPoint < NUMBER)) 
      probsAfterJump[i] += weight2 * probs[startPoint];
    else
      probsAfterJump[i] += weight2 * IMPOSSIBLE;
  }
}


void
applySmoothedJump( int jump,
		   float probs[NUMBER],
		   float probsAfterJump[NUMBER])
{
  int i;
  float tmp[NUMBER];

  int right = (jump > 0) ? 1 : 0;
  
  copyVector( probs, tmp);
  
  if ( jump == 0)
    for ( i = 0; i < NUMBER; i++)
      probsAfterJump[i] = probs[i];
  else
    for ( i = 0; i < jump; i++) {
      applySmoothedSingleStep( right, tmp, probsAfterJump);
      copyVector( probsAfterJump, tmp);
    }
  
  normalize( probsAfterJump);
}


void
applyJump( int jump,
	   float probs[NUMBER],
	   float probsAfterJump[NUMBER])
{
  int i;

  for ( i = 0; i < NUMBER; i++)
    probsAfterJump[i] = IMPOSSIBLE;
  
  for ( i = 0; i < NUMBER; i++) {
    int endPoint = i + jump;
    if ( (endPoint >= 0) &&  (endPoint < NUMBER)) 
      probsAfterJump[endPoint] = probs[i];
  }

  normalize( probsAfterJump);
}


float
computeAPosterioriErrorDist( float probs[NUMBER],
			     float map[NUMBER],
			     sensorType sensor)
{
  float aPosterioriError = 0.0;
  int i, j;

  int feature;

  float pOfFeature[NUMBER_OF_FEATURES];
  float pOfFeatureGivenI[NUMBER_OF_FEATURES];
  float pOfFeatureGivenJ[NUMBER_OF_FEATURES];
    
  /* First compute the likelihood of features. */
  for ( feature = 0; feature < NUMBER_OF_FEATURES; feature++) {

    pOfFeature[feature] = 0.0;

    for ( i = 0; i < NUMBER; i++) 
      pOfFeature[feature] += sensor.probOfFeature( feature, i, map) * probs[i];
    if (0) fprintf( stderr, "%d %f\n", feature, pOfFeature[feature]);
  }
  
  for ( i = 0; i < NUMBER; i++) {

    /* Now get the conditional probabilities of the different features */
    for ( feature = 0; feature < NUMBER_OF_FEATURES; feature++)
      pOfFeatureGivenI[feature] = sensor.probOfFeature( feature, i, map);

    for ( j = 0; j < NUMBER; j++)
      
      if ( i != j) {
	
	float sum = 0.0;
	float factor = fSqr( i - j) * probs[i] * probs[j];
	if (1) factor = probs[i] * probs[j];

	/* Now get the conditional probabilities of the different features */
	for ( feature = 0; feature < NUMBER_OF_FEATURES; feature++) 
	  pOfFeatureGivenJ[feature] = sensor.probOfFeature( feature, j, map);

	for ( feature = 0; feature < NUMBER_OF_FEATURES; feature++) 
	  sum += pOfFeatureGivenI[feature] * pOfFeatureGivenJ[feature]
	    / pOfFeature[feature];

	aPosterioriError += factor * sum;
      }
  }
  return aPosterioriError;
}


float
computeAPosterioriErrorEntropy( float probs[NUMBER],
				float map[NUMBER],
				sensorType sensor)
{
  float aPosterioriError = 0.0;
  int i;
  float factor;

  int feature;

  float pOfFeature[NUMBER_OF_FEATURES];
  float pOfFeatureGivenI[NUMBER_OF_FEATURES];
    
  /* First compute the likelihood of features. */
  for ( feature = 0; feature < NUMBER_OF_FEATURES; feature++) {

    pOfFeature[feature] = 0.0;

    for ( i = 0; i < NUMBER; i++) 
      pOfFeature[feature] += sensor.probOfFeature( feature, i, map) * probs[i];
  }
  
  for ( i = 0; i < NUMBER; i++) {
    
    /* Now get the conditional probabilities of the different features */
    for ( feature = 0; feature < NUMBER_OF_FEATURES; feature++) {
      
      pOfFeatureGivenI[feature] = sensor.probOfFeature( feature, i, map);
      
      factor = pOfFeatureGivenI[feature] * probs[i];

      aPosterioriError += factor * log ( pOfFeature[feature] / factor);
    }
  }
  return aPosterioriError;
}



int
main( int argc, char** argv)
{
  int i;
  
  float map[NUMBER];
  float probs[NUMBER];
  float probsAfterJump[NUMBER];
  float aPosterioriErrors[NUMBER];
  int jump;

  sensorType sensor = { NUMBER_OF_FEATURES, &(probOfDist)};
  
  EZXW_p mapWin = EZX_MakeWindow("map window",
				NUMBER * SCALEX,
				SCALEY,
				"+0+0");
  
  EZXW_p probsWin = EZX_MakeWindow("prob window",
				NUMBER * SCALEX,
				SCALEY,
				"+0+50");
  
  EZXW_p probsAfterWin = EZX_MakeWindow("after window",
					NUMBER * SCALEX,
					SCALEY,
					"+0+100");
  
  EZXW_p aPosterioriWin = EZX_MakeWindow("error window",
					 (NUMBER) * SCALEX,
					 SCALEY,
					 "+0+150");
  
  
  for ( i = 0; i < NUMBER; i++) {
    map[i] = IMPOSSIBLE;
    probs[i] = IMPOSSIBLE;
  }

  if (1) map[20] = 0.99;  
  if (0) probs[12] = probs[17] = 0.49; 
  probs[1] = probs[4] = 0.49; 

  normalize( probs);
  normalize( map);

  for ( jump = 0; jump < NUMBER; jump++) 
    aPosterioriErrors[jump] = 0.0;
    
  show( probsWin, probs, NUMBER);
  show( mapWin, map, NUMBER);
  
  for ( jump = 0; jump < NUMBER; jump++) {

    if (0)
      applySmoothedJump( jump, probs, probsAfterJump);
    else
      applyJump( jump, probs, probsAfterJump);
    show( probsAfterWin, probsAfterJump, NUMBER);
    
    if ( 1 || argc > 1)
      aPosterioriErrors[jump] =
	computeAPosterioriErrorEntropy( probsAfterJump, map, sensor);
    else
      aPosterioriErrors[jump] =
	computeAPosterioriErrorDist( probsAfterJump, map, sensor);

    show( aPosterioriWin, aPosterioriErrors, NUMBER);
    
    fprintf( stderr, "#jump: %d\n", jump);
    fprintf( stderr, "%f\n", aPosterioriErrors[jump]);
    if (0) getchar();
  }

  getchar();

  exit(0);
}



