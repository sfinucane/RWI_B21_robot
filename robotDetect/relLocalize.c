#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "qcamspecs.h"

#ifdef i386
#include <getopt.h>
#endif

#include <string.h>
#include <sys/time.h>

#define MAX_STRING_LENGTH 80
#define MAXPOSITIONS 5000
#define SCALING 2.37
#define MAXTIMEDIFF  1000  // (1250)milliseconds
// #define MAXDISTANCE (1250) 1600


typedef struct {
  float x, y, orientation;
  struct timeval time;  
} robot_position_type;


double maxDistance = 1220;


/* ------------------------------------------------------------ 
 * deg2Rad(int x)
 * converts degrees into radians
 * Taken from bee/src/localize/function.c
 * -----------------------------------------------------------*/
float deg2Rad(int x) {
  return x * 0.017453293;
}

float
rad2Deg(float x)
{
  return x * 57.29578;
}



/* ---------------------------------------------------------
 * getMax() returns the maximum value of array data.
 * --------------------------------------------------------*/
float getMax(float* data, int count) {
  float max = data[0];
  int i;
  for (i = 0; i < count; i++) {
    if (data[i] > max) max = data[i];
  }
  return max;
}





/* -----------------------------------------------------
 * int readPositionsFromFile(char *filename, robot_position_type *positions)
 * Positions are parsed from the input file and stored into
 * memory at (*positions)
 * returns number of processed patterns;
 * -----------------------------------------------------*/

int readPositionsFromFile(char *filename, robot_position_type** positions) {
  FILE  *iop;
  int   file_ended, error, n, m;
  char  name[MAX_STRING_LENGTH];
  char  text[MAX_STRING_LENGTH];
  char  command[MAX_STRING_LENGTH];  
  int   reading_pattern_set, reading_pattern;
  int   patternCount =0;
  robot_position_type position;
  float float_value;
  struct timeval time;
  int verbose = 0;

  fprintf(stderr, "Loading position info from file %s...\n", filename);


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
	       !strcmp(command, "time:") && !strcmp(name, "control")){
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
       * position:
       */

      else if (reading_pattern_set == 1 &&
	       reading_pattern == 1 &&
	       !strcmp(command, "position:") && !strcmp(name, "control")){
	if (verbose)
	  fprintf(stderr, "position:\n");

	if (fscanf(iop, "%f", &float_value) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  position.x = float_value;
	  if (fscanf(iop, "%f", &float_value) == EOF){
	    fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	    error = 1;
	    file_ended = 1;
	  }
	  else{
	    position.y = float_value;
	    if (fscanf(iop, "%f", &float_value) == EOF){
	      fprintf(stderr, "ERROR: surprising end of file %s.\n",
		      filename);
	      error = 1;
	      file_ended = 1;
	    }
	    else{
	      position.orientation = float_value;
	      position.time = time;
	      positions[patternCount] = (robot_position_type *) malloc(sizeof(robot_position_type));
	      if (positions[patternCount] == NULL){
		fprintf(stderr, "ERROR: Out of memory, allocating space for one position.\n");
		exit (-1);
	      }	    
	      bcopy(&position, positions[patternCount], sizeof(robot_position_type));
	      patternCount++;
	    }
	  }
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

  return patternCount;
  
}





/* ------------------------------------------------------------------------
 * visible <->  winkel between blickrichtung and marker position < VIEW/2
 * relAngle < 0: look right, else look left
 * NOTE: atan2: x->[-180;180]
 * ----------------------------------------------------------------------- */
void detectMarker(robot_position_type* detectorPos, robot_position_type* markerPos, int patternNr) {
  double absAngle;
  double relAngle;
  double relDistance;
  int markerDetected;



  relDistance =  (detectorPos->y - markerPos->y) * (detectorPos->y - markerPos->y);
  relDistance += (detectorPos->x - markerPos->x) * (detectorPos->x - markerPos->x);
  relDistance = sqrt(relDistance);
 // relDistance *= 61.0/29.0;

 
    fprintf(stderr, "detPosition  %5.0f  %5.0f  %5.0f\n",detectorPos->x ,detectorPos->y , detectorPos->orientation);
    fprintf(stderr, "marPosition  %5.0f  %5.0f  %5.0f\n",markerPos->x ,markerPos->y , markerPos->orientation);

  absAngle = atan2((detectorPos->y - markerPos->y), (detectorPos->x - markerPos->x));
  absAngle = rad2Deg(absAngle);   
  fprintf(stderr,"absAngle: %5.0f\n", absAngle);

  if (detectorPos->x > markerPos->x) {
    if (detectorPos->y < markerPos->y) 
      absAngle = 180+absAngle;
    else if (detectorPos->y > markerPos->y) 
      absAngle =-180+absAngle;
  }

  relAngle = absAngle - detectorPos->orientation;
  if (relAngle > 180) relAngle -= 360;
  else if (relAngle < -180) relAngle += 360;
   
  fprintf(stderr, "Calculated relative angle: %5.0f. Distance: %5.0f\n", relAngle, relDistance);   
  markerDetected = 0;
  if ((relDistance < maxDistance) && (abs(relAngle) < VIEW/2))  {
    markerDetected = 1;
    fprintf(stderr, "Marker visible\n");
  }
  else fprintf(stderr, "No marker detected.\n"); 
  fprintf(stderr, "#position  %d %d  %5.0f  %5.0f\n", patternNr, markerDetected, relAngle, relDistance);
}


/*
 * This is our main routine.
 */

int main( int argc, char** argv ) {

  int i, j, minNumberOfPositions;
  int timeDiff, initialTimeDiff;
  /*
  robot_position_type* detectorPositions = (robot_position_type *) malloc(MAXPOSITIONS));
  robot_position_type* markerPositions= (robot_position_type *) malloc(MAXPOSITIONS));
  */
  robot_position_type* detectorPositions[MAXPOSITIONS];
  robot_position_type* markerPositions[MAXPOSITIONS];

  
  fprintf(stderr, "Arguments: %d\n", argc);
  minNumberOfPositions = readPositionsFromFile(argv[1], detectorPositions);
  fprintf(stderr, "# detector positions read %d\n", minNumberOfPositions);
  i = readPositionsFromFile(argv[2], markerPositions);
  fprintf(stderr, "# marker positions read %d\n", i);
  if (i < minNumberOfPositions) minNumberOfPositions = i;
  maxDistance = atof(argv[3]);

 
  initialTimeDiff = (detectorPositions[0]->time.tv_sec-markerPositions[0]->time.tv_sec)*1000+
    (detectorPositions[0]->time.tv_usec-markerPositions[0]->time.tv_usec)/1000;
  fprintf(stderr, "inital time difference: %.1f sec\n", (float)initialTimeDiff/1000.0);

 i=0;
 j=0;
 while ((i<minNumberOfPositions)&& (j<minNumberOfPositions)) {
   timeDiff = (detectorPositions[i]->time.tv_sec-markerPositions[j]->time.tv_sec)*1000+
     (detectorPositions[i]->time.tv_usec-markerPositions[j]->time.tv_usec)/1000 
     - initialTimeDiff;
   while ((abs(timeDiff)>MAXTIMEDIFF) && (i<minNumberOfPositions) && (j<minNumberOfPositions)){
     fprintf(stderr,"Big timeDiff: %.1f sec\n", (float)timeDiff/1000.0);
     if (timeDiff>0) j++;
     else i++;
     timeDiff = (detectorPositions[i]->time.tv_sec-markerPositions[j]->time.tv_sec)*1000+
       (detectorPositions[i]->time.tv_usec-markerPositions[j]->time.tv_usec)/1000 
       - initialTimeDiff;
   } 
   fprintf(stderr,"Results of combining patterns %d, %d with time difference %.1f sec\n", i,j,(float)timeDiff/1000.0);
   detectMarker(detectorPositions[i], markerPositions[j],i);
   i++; j++;
 }
  return 0;

}
  

