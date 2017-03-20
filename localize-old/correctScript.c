#include <stdio.h>
#include <string.h>
#include <math.h>
#include <values.h>
#include "corr.h"
#include "general.h"


#define CORRFILENAME "corr.log"
#define STRINGLENGTH 4096

#define MAXCORRECTIONPARAMS 10000

#define NEW_POSMARK "position:"
#define NEW_TIMEMARK "time:"
#define OLD_POSMARK "#ROBOT"
#define OLD_TIMEMARK "@SENS"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_360 (M_PI + M_PI)



float
rad2Deg(float x)
{
  return x * 57.29578;
}

float
deg2Rad(float x)
{
  return x * 0.017453293;
}



/**************************************************************************
 * Norms the angle between [0:2pi].
 **************************************************************************/

float 
normalizedAngle(float angle)
{
  while (angle < 0.0)
    (angle) += DEG_360;
  
  while (angle >= DEG_360)
    (angle) -= DEG_360;

  return(angle);
}


/**************************************************************************
 * Converts a position in the robot's coordinates into a
 * correct position.
 **************************************************************************/
realPosition
robotPosition2CorrectPosition( realPosition robPos)
{
  realPosition pos;

  pos.x   = robPos.x;
  pos.y   = robPos.y;
  pos.rot = normalizedAngle( deg2Rad( - robPos.rot + 90.0));

  return pos;
}



/**************************************************************************
 * Computes the movement between two points in the correct coordinate system.
 **************************************************************************/
movement
movementBetweenPoints( realPosition start, realPosition end)
{
  movement delta;

  /* compute forward and sideward sensing_MOVEMENT */
  delta.forward =
    + (end.y - start.y) * sin(start.rot)
    + (end.x - start.x) * cos(start.rot);
  
  delta.sideward =
    - (end.y - start.y) * cos(start.rot)
    + (end.x - start.x) * sin(start.rot);
  
  delta.rotation = end.rot - start.rot;

  return delta;
}


/**************************************************************************
 * Computes the movement between two points in the robot's coordinate system.
 **************************************************************************/
movement
movementBetweenRobotPoints( realPosition start, realPosition end)
{
  return( movementBetweenPoints( robotPosition2CorrectPosition( start),
				 robotPosition2CorrectPosition( end)));
}


realPosition
endPoint( realPosition start, movement move)
{
  realPosition end;
  float cosRot;
  float sinRot;

  cosRot =  cos(start.rot);
  sinRot =  sin(start.rot);
  if ( move.forward == 0.0 && move.sideward == 0.0 && move.rotation == 0.0)
    return start;
  
  /* This is the correct formula. */
  /*   end.x = start.x + */
  /*     cos( start.rot) * move.forward + cos( start.rot - DEG_90) * move.sideward; */
  
  /*   end.y = start.y + */
  /*     sin( start.rot) * move.forward + sin( start.rot - DEG_90) * move.sideward; */
  
  
  /* Replaced cos( r - 90) by sin( r) and sin( r - 90) by -cos( r) */
  end.x = start.x + cosRot * move.forward + sinRot * move.sideward;
  end.y = start.y + sinRot * move.forward - cosRot * move.sideward;
  end.rot = normalizedAngle( start.rot + move.rotation);

  return end;
}


realPosition
scriptPosition2RobotPosition( realPosition scriptPos)
{
  realPosition pos;

  pos.x   = scriptPos.x;
  pos.y   = scriptPos.y;
  pos.rot = 90.0 - scriptPos.rot;

  return pos;
}



int
main(int argc, char *argv[]){

  FILE *fp;
  char *corrFileName = CORRFILENAME;
  char line[STRINGLENGTH];
  int numberOfCorrectionParams = 0;
  float time[MAXCORRECTIONPARAMS];
  realPosition corr[MAXCORRECTIONPARAMS];
  int  corrType[MAXCORRECTIONPARAMS];
  int firstPosition = 1;
  int startUp = 1;
  long seconds = 0;
  long startSeconds = 0;
  long corrLogTime = 0;
  unsigned char old = 0;
  int i;
  char *posMark, *timeMark;
  
  if (argc >= 2){
    corrFileName = argv[1];
  }

  if ((fp = fopen(corrFileName, "r")) == NULL){
    fprintf(stderr, "Error: could not open %s\n", corrFileName);
    exit(1);
  }

  for (i = 0; i < argc; i++)
    if (strncmp("-old", argv[i], 4) == 0){
      old = 1;
    }
  
  while (!feof(fp) && numberOfCorrectionParams <= MAXCORRECTIONPARAMS){
    if (fgets(line, STRINGLENGTH, fp) != NULL){
      if (sscanf(line, "%f %f %f %f %d",
		 &time[numberOfCorrectionParams],
		 &corr[numberOfCorrectionParams].x,
		 &corr[numberOfCorrectionParams].y,
		 &corr[numberOfCorrectionParams].rot,
		 &corrType[numberOfCorrectionParams]) == 5)
	numberOfCorrectionParams++;
    }
  }

  fprintf(stderr, "CorrectionParams: %d\n", numberOfCorrectionParams);

  fclose(fp);
  if (numberOfCorrectionParams == 0){
    fprintf(stderr, "No correction parameters found. Nothing has to be done!\n");
    exit(0);
  }
  
  fp = stdin;


  if (old){
    posMark = OLD_POSMARK;
    timeMark = OLD_TIMEMARK;
  }
  else{
    posMark = NEW_POSMARK;
    timeMark = NEW_TIMEMARK;
  }
  while (!feof(stdin) && (1 || (corrLogTime < time[numberOfCorrectionParams-1] + 20))){
    FILE *fp1;
    if (fgets(line, STRINGLENGTH, fp) != NULL){
      if (strncmp(line, timeMark, strlen(timeMark)) == 0){
	long tmp;
	if (old){
	  int hh,mm;
	  float ss;
	  if ( sscanf(&line[strlen(timeMark) + 10],
		      "%d:%d:%f", &hh, &mm, &ss) == 3){
	    seconds = hh * (long) 3600 + mm *60 + ss;
	    if (startUp){
	      startSeconds = seconds;
	      startUp = 0;
	    }
	  }
	}
	else {
	  if ( sscanf(&line[strlen(timeMark)], "%ld", &tmp) == 1){
	    seconds = tmp;
	    if (startUp){
	      startSeconds = seconds;
	      startUp = 0;
	    }
	  }
	}
      }
      if (strncmp(line, posMark, strlen(posMark)) == 0){
	realPosition pos;
	if (startUp){
	  fprintf(stderr, "Error: found position before first time\n");
	  exit(1);
	}
	if ( sscanf(&line[strlen(posMark)], "%f %f %f",
		    &pos.x, &pos.y, &pos.rot) == 3){
	  /* compute the new positions... */
	  realPosition newPos, previousNewPos, previousPos, accPos;
	  int n;
	  n = 0;
	  corrLogTime = seconds - startSeconds;

	  while (n < numberOfCorrectionParams &&
		 corrLogTime > time[n]){
	    n++;
	  }
	  if (1) fprintf(stderr, "n: %d %ld %ld\r", n, corrLogTime, seconds);

	  if (n > 0)
	    n--;

	  if (0 ) corr[n].x = corr[n].y = corr[n].rot = n = 0; 

	  pos = scriptPosition2RobotPosition(pos);
	  robotCoordinates2MapCoordinates( (float) pos.x, (float) pos.y, (float)
					   pos.rot,
					   corr[n].x, corr[n].y, corr[n].rot,
					   corrType[n],
					   &newPos.x, &newPos.y, &newPos.rot);

	  /* newPos.rot = rad2Deg(normalizedAngle(M_PI * 0.5 - newPos.rot)); */
	    
	  /* fprintf(stderr, "%d\n", (int) (seconds - startSeconds)); */

	  newPos.rot = rad2Deg(newPos.rot);
	  if (0) fprintf(stdout, "Diff %f\n", newPos.rot - pos.rot);
	  if (1) printf("%s %f %f %f\n", posMark,
			newPos.x,
			newPos.y,
			newPos.rot);

	  if (0) printf("%s %f %f %f\n", posMark, pos.x, pos.y, pos.rot);

	  if (0){
	    if (!firstPosition){

	    movement move;
	    move = movementBetweenRobotPoints(previousPos, pos);
	    accPos.rot = deg2Rad(90 -  accPos.rot);
	    accPos = endPoint(accPos, move);
	    accPos.rot = 90 - rad2Deg(accPos.rot);
	    fprintf(fp1, "#Parameter: %d\n", n);
	    fprintf(fp1, "#OldMovement: %f %f %f\n", move.forward,
		    move.sideward, move.rotation);


	    move = movementBetweenRobotPoints(previousNewPos, newPos);


	    fprintf(fp1, "#NewMovement: %f %f %f \n", move.forward,
		    move.sideward, move.rotation);

	    fprintf(fp1, "%f %f %f %f %f %f\n", accPos.x, accPos.y, accPos.rot,
		   newPos.x, newPos.y, newPos.rot);
	    
	  }
	  else{
	    firstPosition = 0;
	    accPos = newPos;
	    fp1 = fopen("mmmm", "w");
	    fprintf(fp1, "%f %f %f %f %f %f \n", 
		   accPos.x, accPos.y, accPos.rot,
		   newPos.x, newPos.y, newPos.rot);
	  }
	  }
	  previousNewPos = newPos;
	  previousPos = pos;
	    
	}
      }
      else
	if (1) printf("%s", line);
    }
      
      

  }
  exit(0);
}
    


