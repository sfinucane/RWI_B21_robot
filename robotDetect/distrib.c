#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "qcamspecs.h"

#ifdef i386
#include <getopt.h>
#endif

#include <string.h>

#define MAX_STRING_LENGTH 80
#define MAXPOSITIONS 5000
#define ANGLECALIBRATION 1.42592610
#define DISTANCECALIBRATION 32.35185210
#define PI 3.14159265358979323846
#define PLOTRANGE 50
#define STEP 0.5

typedef struct {
  int patternNr;
  int visible;
  float angle, distance;
} robot_relative_position_type;

/* ------------------------------------------------------------ 
 * deg2Rad(int x)
 * converts degrees into radians
 * Taken from bee/src/localize/function.c
 * -----------------------------------------------------------*/
float deg2Rad(int x) {
  return x * 0.017453293;
}

float rad2Deg(float x) {
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


float
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



/* -----------------------------------------------------
 * int readFromFile(char *filename, robot_position_type *positions)
 * Positions are parsed from the input file and stored into
 * memory at (*positions)
 * returns number of processed patterns;
 * -----------------------------------------------------*/

int readFromFile(char *filename, robot_relative_position_type** positions) {
  FILE  *iop;
  int   file_ended, error, n, m;
  char  command[MAX_STRING_LENGTH];  
  int   patternCount =0;
  robot_relative_position_type position;
  float float_value;
  int int_value;
  int verbose = 0;

  fprintf(stderr, "Loading position info from file %s...\n", filename);


  if ((iop = fopen(filename, "r")) == 0){
    fprintf(stderr, "ERROR: Could not open pattern file %s.\n", filename);
    return 0;
  }


  n = 0, m = 0;


  for (file_ended = 0, error = 0; !file_ended && !error; ){
    if (fscanf(iop, "%s", command) == EOF)
      file_ended = 1;
    
    else{
     // fprintf(stderr, "[%s]\n", command);


      /*
       * position:
       */

     if (!strcmp(command, "#position")) {
       if (verbose)
	 fprintf(stderr, "#position\n");
       
       if (fscanf(iop, "%d", &int_value) == EOF){
	 fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	 error = 1;
	 file_ended = 1;
       }
       else{
	 position.patternNr= int_value;
	 if (fscanf(iop, "%d", &int_value) == EOF){
	   fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	   error = 1;
	   file_ended = 1;
	 } 
	 else {
	   position.visible= int_value;
	   if (fscanf(iop, "%f", &float_value) == EOF){
	     fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	     error = 1;
	     file_ended = 1;
	   }	   
	   else{
	     position.angle = float_value;
	     if (fscanf(iop, "%f", &float_value) == EOF){
	       fprintf(stderr, "ERROR: surprising end of file %s.\n",
		       filename);
	       error = 1;
	       file_ended = 1;
	     }
	     else{
	       position.distance = float_value;
	       positions[patternCount] = (robot_relative_position_type *) malloc(sizeof(robot_relative_position_type));
	       if (positions[patternCount] == NULL){
		 fprintf(stderr, "ERROR: Out of memory, allocating space for one position.\n");
		 exit (-1);
	       }	    
	       bcopy(&position, positions[patternCount], sizeof(robot_relative_position_type));
	       patternCount++;
	     }
	   }	 
	 }             
       }
     }
    }
  }

  fclose(iop);

  return patternCount;
  
}



/* ------------------------------------------------------------------------
 * 
 * for Mathematica
 * ----------------------------------------------------------------------- */
void printList(robot_relative_position_type** estimatedPositions, robot_relative_position_type** realPositions, int numberOfPositions) {
 int i;

 fprintf(stderr, "\nangles={");
 for (i=0; i<numberOfPositions; i++) {
   if (estimatedPositions[i]->visible && realPositions[i]->visible) 
     fprintf(stderr, "{%.0f,%.0f},", realPositions[i]->angle, (estimatedPositions[i]->angle- ANGLECALIBRATION));
 }
 fprintf(stderr, "}\n");

 fprintf(stderr, "\ndistances={");
 for (i=0; i<numberOfPositions; i++) {
   if (estimatedPositions[i]->visible && realPositions[i]->visible) 
     fprintf(stderr, "{%.0f,%.0f},", realPositions[i]->distance, (estimatedPositions[i]->distance-DISTANCECALIBRATION));
 }
 fprintf(stderr, "}\n");
}



/* ------------------------------------------------------------------------
 * 
 * Gauss func
 * ----------------------------------------------------------------------- */
double gaussXY(double x,double y, double norm, double coeff, double aavg, double avar, double davg, double dvar, double roh) {

return norm*exp(coeff*(((x-aavg)*(x-aavg))/avar - 2*roh*(((x-aavg)*(y-davg))/(sqrt(avar)*sqrt(dvar))) + ((y-davg)* (y-davg))/dvar));

}


void printGauss(double x,double y, double norm, double coeff, double aavg, double avar, double davg, double dvar, double roh) {

 for (x=-PLOTRANGE; x<=PLOTRANGE;x+=STEP) {
   for (y=-PLOTRANGE; y<=PLOTRANGE;y+=STEP) 
     fprintf(stderr,"%f10 %f10 %f10\n", x, y, gaussXY(x,y,  norm,  coeff,  aavg,  avar,  davg,  dvar,  roh));
   fprintf(stderr,"\n");
 }
}


/* ------------------------------------------------------------------------
 * 
 * Doerfler: Mathematik fuer Informatiker (Carl Hansen Verlag), p.344, 3.76
 * ----------------------------------------------------------------------- */
void angleDistanceStats(robot_relative_position_type** estimatedPositions, robot_relative_position_type** realPositions, int numberOfPositions) {
 int i,n=0;
 double adelta, aavg, asum=0.0, asqrsum=0.0;
 double maxadelta=0, maxddelta=0;
 int maxadeltaPatternNr=0, maxddeltaPatternNr=0;
 double ddelta, davg, dsum=0.0, dsqrsum=0.0;
 double dvar, avar, covar=0.0;
 double otherdvar, otheravar;
 double roh,norm,coeff;

 for (i=0; i<numberOfPositions; i++) {
   if (estimatedPositions[i]->visible && realPositions[i]->visible) {
     n++;
     adelta = (realPositions[i]->angle - (estimatedPositions[i]->angle - ANGLECALIBRATION));
     if (abs(adelta)>abs(maxadelta)) {
       maxadelta = adelta;
       maxadeltaPatternNr = realPositions[i]->patternNr;
     }
     asum += adelta;
     asqrsum += adelta*adelta;
     ddelta = (realPositions[i]->distance - (estimatedPositions[i]->distance - DISTANCECALIBRATION));
//     if (abs(ddelta) > 50.0) 
//       ddelta = 50.0 * ((ddelta < 0) ? 1 : -1);
     if (abs(ddelta)>abs(maxddelta)) {
       maxddelta = ddelta;
       maxddeltaPatternNr = realPositions[i]->patternNr;
     }
     dsum += ddelta;
     dsqrsum += ddelta*ddelta;          
   }
 }

 aavg = asum/n; 
 davg = dsum/n;
 
 dvar = dsqrsum/n - davg*davg;
 avar = asqrsum/n - aavg*aavg;
 otherdvar = (1.0/(n-1)) * (dsqrsum - n * dsum * dsum); 
 otheravar = (1.0/(n-1)) * (asqrsum - n * asum * asum); 

 for (i=0; i<numberOfPositions; i++) {
   if (estimatedPositions[i]->visible && realPositions[i]->visible) {
     adelta = (realPositions[i]->angle - (estimatedPositions[i]->angle - ANGLECALIBRATION));
     ddelta = (realPositions[i]->distance - (estimatedPositions[i]->distance - DISTANCECALIBRATION));
     covar +=(adelta - aavg) *(ddelta-davg);     
   }
 }
 

 covar = covar/n;

 roh = covar/((sqrt(avar) * sqrt(dvar)));
fprintf(stderr, "# patterns with visible and detected marker: %d\n", n);
fprintf(stderr, "Angle mean and std %f10 %f10\n", aavg, sqrt(avar));
fprintf(stderr, "Angle variance  %f10\n", avar);
fprintf(stderr, "Dieter's variance %f, std %f\n", otheravar,sqrt(otheravar) );
fprintf(stderr, "Max angle delta %f\n\n", maxadelta);

fprintf(stderr, "Distance mean and std %f10 %f10\n", davg, sqrt(dvar));
fprintf(stderr, "Distance variance  %f10\n", dvar);
fprintf(stderr, "Dieter's variance %f, std %f\n", otherdvar,sqrt(otherdvar) );
fprintf(stderr, "Max distance delta %f\n\n", maxddelta);

fprintf(stderr, "Covariance of angle and distance %f10\n", covar);
fprintf(stderr, "Correlation coefficient roh: %f10\n", roh);
norm =  1.0/((sqrt(2*PI) * sqrt(avar) * sqrt(dvar) * sqrt(1-(roh*roh))));
fprintf(stderr, "Gauss Normalizer: %f10\n", norm);
coeff =  -1.0/(2*(1-(roh*roh)));
fprintf(stderr, "Gauss coeff: %f10\n",coeff);
fprintf(stderr, "g[x_,y_]:=%f10*Exp[%f10*((((x-(%f10))^2)/%f10) - 2*%f10*((x-(%f10))*(y-(%f10)))/(%f10*%f10)+((y-(%f10))^2)/%f10)]", norm, coeff, aavg, avar, roh, aavg, davg,sqrt(avar), sqrt(dvar), davg, dvar);
}




/* ------------------------------------------------------------------------
 * 
 * Gauss Test
 * ----------------------------------------------------------------------- */
void gaussfit(int numberOfPositions) {
 int i,n=0;
 double adelta, aavg, asum=0.0, asqrsum=0.0;
 double maxadelta=0, maxddelta=0;
 double x,y;

 double ddelta, davg, dsum=0.0, dsqrsum=0.0;
 double dvar, avar, covar=0.0;
 double otherdvar, otheravar;
 double roh, norm, coeff;
 int xvalues[2000];
 int yvalues[2000];


 for (i=0; i<numberOfPositions; i++) {
     n++;
     xvalues[i] = randomGauss();
     adelta = xvalues[i];
     if (abs(adelta)>abs(maxadelta)) {
       maxadelta = adelta;
     }
     asum += adelta;
     asqrsum += adelta*adelta;
     yvalues[i] = randomGauss();
     ddelta = yvalues[i];
     if (abs(ddelta)>abs(maxddelta)) {
       maxddelta = ddelta;

     }
     dsum += ddelta;
     dsqrsum += ddelta*ddelta;             
 }

 aavg = asum/n; 
 davg = dsum/n;
 
 dvar = dsqrsum/n - davg*davg;
 avar = asqrsum/n - aavg*aavg;
 otherdvar = (1.0/(n-1)) * (dsqrsum - n * dsum * dsum); 
 otheravar = (1.0/(n-1)) * (asqrsum - n * asum * asum); 

 for (i=0; i<numberOfPositions; i++) {
      adelta = xvalues[i]; 
     ddelta = yvalues[i];
     covar +=(adelta - aavg) *(ddelta-davg);     
 }
 covar = covar/n;

 roh = covar/((sqrt(avar) * sqrt(dvar)));
fprintf(stderr, "# patterns with visible and detected marker: %d\n", n);
fprintf(stderr, "Angle mean and std %f10 %f10\n", aavg, sqrt(avar));
fprintf(stderr, "Angle variance  %f10\n", avar);
fprintf(stderr, "Dieter's variance %f, std %f\n", otheravar,sqrt(otheravar) );
fprintf(stderr, "Max angle delta %f\n\n", maxadelta);

fprintf(stderr, "Distance mean and std %f10 %f10\n", davg, sqrt(dvar));
fprintf(stderr, "Distance variance  %f10\n", dvar);
fprintf(stderr, "Dieter's variance %f, std %f\n", otherdvar,sqrt(otherdvar) );
fprintf(stderr, "Max distance delta %f\n\n", maxddelta);

fprintf(stderr, "Covariance of angle and distance %f10\n", covar);
fprintf(stderr, "Correlation coefficient roh: %f10\n", roh);
norm =  1.0/((sqrt(2*PI) * sqrt(avar) * sqrt(dvar) * sqrt(1-(roh*roh))));
fprintf(stderr, "Gauss Normalizer: %f10\n", norm);
coeff =  -1.0/(2*(1-(roh*roh)));
fprintf(stderr, "Gauss coeff: %f10\n",coeff);

 for (x=-PLOTRANGE; x<=PLOTRANGE;x+=STEP) {
   for (y=-PLOTRANGE; y<=PLOTRANGE;y+=STEP) 
     fprintf(stderr,"%f10 %f10 %f10\n", x, y, gaussXY(x,y, norm, coeff, aavg, avar, davg, dvar, roh));
   fprintf(stderr,"\n");
 }

}

/* ------------------------------------------------------------------------
 * calcs recognition stats
 * ----------------------------------------------------------------------- */
void recognitionStats(robot_relative_position_type** estimatedPositions, robot_relative_position_type** realPositions, int numberOfPositions) {
  
  double prob;
  int i,n,k;
  
  n=0;
  k=0;
  for (i=0; i< numberOfPositions; i++) {
    if (realPositions[i]->visible == 1) {
      n++;
      if (estimatedPositions[i]->visible ==1)
	k++;
    }
  }
  prob = (double)k / (double)n; 
  fprintf(stderr, "P(detected,visible) = %f (%d/%d)\n", prob, k ,n);  
  fprintf(stderr,"P(not detected,visible) = %f\n", 1-prob);  
  
  
  n=0;
  k=0;
  for (i=0; i< numberOfPositions; i++) {
    if (estimatedPositions[i]->visible == 1) {
      n++;      
      if (realPositions[i]->visible ==1)
	k++;
    }
  }
  prob = (double)k / (double)n;  
  fprintf(stderr,"P(visible,detected) = %f(%d/%d)\n", prob, k, n);  
  fprintf(stderr,"P(not visible,detected) = %f\n", 1-prob);  


}


/*
 * This matches entries of estimated and real positions.
 */
void compressPositions(robot_relative_position_type** estimatedPositions, robot_relative_position_type** realPositions, int numberOfPositions) {
 int i,j,k=0;
 for (i=0; i< numberOfPositions; i++) {
   j = realPositions[i]->patternNr;
   estimatedPositions[k++]=estimatedPositions[j];
 }
}





/*
 * This is our main routine.
 */

int main( int argc, char** argv ) {

  int i, minNumberOfPositions;
  /*
  robot_relative_position_type* estimatedPositions = (robot_relative_position_type *) malloc(MAXPOSITIONS));
  robot_relative_position_type* realPositions= (robot_relative_position_type *) malloc(MAXPOSITIONS));
  */
  robot_relative_position_type* estimatedPositions[MAXPOSITIONS];
  robot_relative_position_type* realPositions[MAXPOSITIONS];

  
  fprintf(stderr, "Arguments: %d\n", argc);
  minNumberOfPositions = readFromFile(argv[1], estimatedPositions);
  fprintf(stderr, "# estimated positions read %d\n", minNumberOfPositions);
  i = readFromFile(argv[2], realPositions);
  fprintf(stderr, "# real positions read %d\n", i);
  if (i < minNumberOfPositions) minNumberOfPositions = i;
  
 
//  gaussfit(500);
//  exit(1);
  compressPositions(estimatedPositions, realPositions,minNumberOfPositions );
  printList(estimatedPositions, realPositions,minNumberOfPositions );

  recognitionStats(estimatedPositions, realPositions,minNumberOfPositions );
  angleDistanceStats(estimatedPositions, realPositions,minNumberOfPositions );

  return 0;

}
  

