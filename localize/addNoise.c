#include <stdio.h>
#include <strings.h>
#include <stdlib.h>
#include <math.h>

#include "general.h"
#include "graphic.h"
#include "probGrid.h"
#include "function.h"
#include "map.h"
#include "script.h"
#include "proximityTools.h"
#include "file.h"
#include "allocate.h"
#include "fileNames.h"
#include "EZX11.h"

#define ORIG_SCRIPTROBOTMARK "#ROBOT"
#define NEW_SCRIPTROBOTMARK "position:"
#define BUFFER_LENGTH 10000


int
eventOccurs( float bumpProbability)
{
  static int firstTime = TRUE;
  static int threshold;
  
  if ( firstTime) {
    firstTime = FALSE;
    threshold = bumpProbability * RAND_MAX;
  }
  return (threshold > rand());
}


static float
randomGauss()
{
  static int iset = 0;
  static float gset;
  float fac, rsq, v1, v2;
  if(iset == 0) {
    do {
      v1 = 2.0 * rand()/(RAND_MAX+1.0) - 1.0;
      v2 = 2.0 * rand()/(RAND_MAX+1.0) - 1.0;         
      rsq = v1*v1 + v2*v2;
    } while(rsq >= 1.0 || rsq == 0.0);
    fac = sqrt(-2.0*log(rsq)/rsq);
    gset = v1*fac;
    iset = 1;
    return v2*fac;
  }
  else {
    iset = 0;
    return gset;
  }
}

/* to convert stddev in 1-d into 2-d. */
#define TWO_D_NORM_VALUE 0.707107

static void
addNoise( movement* move, float distance,
	  float distNoise, float rotNoise,
	  float rotNoisePerDist)
{
  float distStdDev = TWO_D_NORM_VALUE * distance * distNoise;
  float rotMoveStdDev = rotNoisePerDist * distance;
  float rotStdDev =  move->rotation * rotNoise;

  move->forward  += distStdDev * randomGauss();
  move->sideward += distStdDev * randomGauss();
  move->rotation += rotStdDev * randomGauss(); 
  move->rotation += rotMoveStdDev * randomGauss(); 
}


static void
addBumpNoise( movement* move, 
	      float distNoise, float rotNoise)
{
  float distStdDev = TWO_D_NORM_VALUE * distNoise;

  float fd = distStdDev * randomGauss();
  float sd = distStdDev * randomGauss();
  float rd = 0.7 * DEG_180 - rotNoise * randomGauss();

  /* deviation between -180 and +180 deg. */
  rd = normalizedAngle(rd);
  if ( rd > DEG_180) rd -= DEG_360;
  
  fprintf(stderr, "%f #deg\n", rad2Deg(rd));
  fprintf(stderr, "deviation: (%f, %f) %f\n", fd, sd, rad2Deg(rd));

  printf( "#BUMP %f %f %f \n", fd, sd, rd);
	  
  move->forward +=  fd;
  move->sideward += sd;
  move->rotation += rd;
}


int
main( int argc, char** argv)
{
  int origMarkLength = strlen(ORIG_SCRIPTROBOTMARK);
  int newMarkLength = strlen(NEW_SCRIPTROBOTMARK);
  int firstTime = TRUE;
  char line[BUFFER_LENGTH];
  char *positionP;
  realPosition noisyPos, prevNoisyPos;
  realPosition pos, prevPos;

  int posHasChanged, positionLine;
  int seed = 1000, skip = 1;
  int onlyPositions = FALSE, origScript = TRUE;
  float distNoise = 0.0;
  float rotNoisePerDist = 0.0;
  float rotNoise = 0.0;
  int posCnt = 1;
  int i, noiseDesired = FALSE;
  int bumpDesired = FALSE;
  float bumpDiscretization = 10.0, bumpProbability = 0.0;
  float bumpDistNoise = 0.0, bumpRotNoise = 0.0;
  float distanceSinceLastEventCheck = 0.0;
  float scriptDist = 0.0;
  int nomove = 0;
  int bumpCnt = 0;
  
  int setStartPos = 0;
  float startX=0, startY=0, startRot=0;

  for (i=1; i<argc; i++) {

    if (strcmp(argv[i],"-d")==0) {
      if ( i < argc - 1) {
	distNoise = atof( argv[++i]);
      }
      else {
	fprintf( stderr, "ERROR: value must follow keyword -d (distNoise).\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-r")==0) {
      if ( i < argc - 1) {
	rotNoise = atof( argv[++i]);
      }
      else {
	fprintf( stderr, "ERROR: value must follow keyword -r (rotNoise).\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-rd")==0) {
      if ( i < argc - 1) {
	rotNoisePerDist = deg2Rad(atof( argv[++i]));
      }
      else {
	fprintf( stderr, "ERROR: value must follow keyword -rd (rotNoisePerDist).\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-seed")==0) {
      if ( i < argc - 1) {
	seed = atoi( argv[++i]);
	fprintf(stderr, "Set seed to %d.\n", seed);
      }
      else {
	fprintf( stderr, "ERROR: value must follow keyword -seed.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-bump")==0) {
      if ( i < argc - 4) {
	bumpDiscretization = atof( argv[++i]);
	bumpProbability = atof( argv[++i]);
	bumpDistNoise = atof( argv[++i]);
	bumpRotNoise = atof( argv[++i]);
	bumpDesired = TRUE;
	fprintf(stderr,
		"Bump all %f cm with prob %f. Bump will cause (%f cm, %f deg) deviation.\n",
		bumpDiscretization, bumpProbability, bumpDistNoise, bumpRotNoise);
	bumpRotNoise = deg2Rad(bumpRotNoise);
      }
      else {
	fprintf( stderr,
		 "ERROR: four values must follow keyword -bump (disc. prob cm. deg).\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-skip")==0) {
      if ( i < argc - 1) {
	skip = atoi( argv[++i]);
      }
      else {
	fprintf( stderr, "ERROR: value must follow keyword -skip.\n");
	exit;
      }
    }
    else if (strcmp(argv[i],"-p")==0) {
      onlyPositions = TRUE;
    }
    else if (strcmp(argv[i],"-new")==0) {
      origScript = FALSE;
    }
    else if (strcmp(argv[i],"-start")==0) {
      if ( i < argc - 3) {
	setStartPos = TRUE;
	startX   = atof( argv[++i]);
	startY   = atof( argv[++i]);
	startRot = deg2Rad(atof( argv[++i]));
	fprintf(stderr, "Set start pos: %f %f %f\n", startX, startY, rad2Deg(startRot));
      }
      else {
	fprintf( stderr, "ERROR: three coordinates must follow keyword -start.\n");
	exit;
      }
    }
    else {
      fprintf( stderr, "Unknown parameter: %s\n", argv[i]);
      fprintf( stderr, "Usage: addNoise -d distNoise -r rotNoise -rd rotNoisePerDist [-seed seed] [-skip skip] -p -new [-start x y rot]\n");
      exit(-1);
    }
  }
  fprintf( stderr, "Settings: %f %f %f\n", distNoise, rotNoise, rotNoisePerDist);

  /* Set the seed. */
  srand( seed);

  if ( (distNoise != 0.0) || (rotNoise != 0.0) || (rotNoisePerDist != 0.0))
    noiseDesired = TRUE;

#define START_DIST -110146.0859

  while (fgets(line,BUFFER_LENGTH,stdin) != NULL){

    positionLine = origScript ?
      strncmp(line,ORIG_SCRIPTROBOTMARK,  origMarkLength) == 0
      :
      strncmp(line,NEW_SCRIPTROBOTMARK,  newMarkLength) == 0;
    
    if ( positionLine) {
      
      positionP = origScript ? &line[origMarkLength] : &line[newMarkLength];
      
      if ( sscanf(positionP, "%f %f %f", &pos.x, &pos.y, &pos.rot) == 3) {
	
	posHasChanged = FALSE;
	
	/* Convert rotation. */
	pos.rot = normalizedAngle(deg2Rad( pos.rot));
	
	if (firstTime) {
	  firstTime = FALSE;
	  if ( setStartPos) {
	    noisyPos.x = startX;
	    noisyPos.y = startY;
	    noisyPos.rot = startRot;
	  }
	  else
	    noisyPos = pos;
	  posHasChanged = TRUE;
	}

	/* The robot has moved. */
	else if ( pos.x != prevPos.x || pos.y != prevPos.y
		  || pos.rot != prevPos.rot) {

	  movement move;
	  float currentDist = distanceBetweenPoints( prevPos, pos);	  
	  int bumpOccured = FALSE;
	  
	  if (currentDist > 1000.0) fprintf(stderr, "jump %f after %f meters\n%s",
					   currentDist, scriptDist / 100.0,
					   line);
	  
	  scriptDist += currentDist;
	  
	  if ( scriptDist > 110000.0) {
	    fprintf( stderr, "Stop after 1000 meters.\n");
	    exit(1);
	  }
	  
	  posHasChanged = TRUE;
	  
	  /*---------------------------------------------------------------
	   * Model the occurence of a bump causing a large deviation.
	   *---------------------------------------------------------------*/
	  if ( bumpDesired) {
	    
	    distanceSinceLastEventCheck += currentDist;
	    
	    move = movementBetweenPoints( prevPos, pos);
		
	    while ( distanceSinceLastEventCheck > bumpDiscretization) {
	      
	      /* Check whether the bump occurs. Only one bump is considered. */
	      if ( eventOccurs( bumpProbability) && ! bumpOccured) {

		bumpOccured = TRUE;
		bumpCnt++;
		
		fprintf(stderr, "Bump at (%f, %f) after %f meters.\n",
			prevNoisyPos.x, prevNoisyPos.y, scriptDist / 100.0);
		
		if ( move.rotation > DEG_180)
		  move.rotation -= DEG_360;
		
		addBumpNoise( &move, 
			      bumpDistNoise, bumpRotNoise);
		
		noisyPos = endPoint( prevNoisyPos, move);
	      }
	      distanceSinceLastEventCheck -= bumpDiscretization;
	    }
	  }

	  /*---------------------------------------------------------------
	   * If no bump has occured model dead reckoning noise.
	   *---------------------------------------------------------------*/
	  if ( noiseDesired && ! bumpOccured) {
	    
	    move = movementBetweenPoints( prevPos, pos);
	    
	    if ( move.rotation > DEG_180)
	      move.rotation -= DEG_360;
	    
	    addNoise( &move, currentDist,
		      distNoise, rotNoise, rotNoisePerDist);
	    
	    noisyPos = endPoint( prevNoisyPos, move);
	  }
	  
	  else if ( bumpDesired)
	    noisyPos = endPoint( prevNoisyPos, move);
	  /* Just concatenate the positions. Extract movement in order to be
	   * able to set a start point. */
	  else {
	    move = movementBetweenPoints( prevPos, pos);
	    noisyPos = endPoint( prevNoisyPos, move);
	  }
	  nomove = 0;
	}
	
	/* Dump the line. */
 	if ( scriptDist >= START_DIST) {
	  if (onlyPositions) {
	    if ( posHasChanged && ( posCnt % skip == 0))
	      printf( "%f %f %f\n", noisyPos.x, noisyPos.y, rad2Deg(noisyPos.rot));
	  }
	  else
	    {
	      if (origScript)
		printf( "%s %f %f %f\n", ORIG_SCRIPTROBOTMARK,
			noisyPos.x, noisyPos.y, rad2Deg(noisyPos.rot));
	      else
		printf( "%s %f %f %f\n", NEW_SCRIPTROBOTMARK,
			noisyPos.x, noisyPos.y, rad2Deg(noisyPos.rot));
	      
	    }
	}
	
	prevPos = pos;
	prevNoisyPos = noisyPos;
	posCnt++;
      }
    }
    else if ( ! onlyPositions)
      if ( scriptDist >= START_DIST) {
	printf("%s", line);
      }
  }
  fprintf( stderr, "Script is %f meters long (%d bumps).\n", scriptDist / 100.0, bumpCnt);
  return 0;
}

