#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <math.h>
#include <string.h>

/*
  
******************************************
     OLD SCRIPT FORMAT
******************************************

--- ROBOT COORDS ---
@SENS 20-02-98 15:48:17.156567 
#ROBOT -0.065646 -0.065646 89.647887

--- LASER VALUES ---
@SENS 20-02-98 15:48:17.155578
#LASER 180 0: 500 500 500 ...

--- SONAR VALUES ---
@SENS 20-02-98 15:48:17.156918
#SONAR 24: 3248.000000 288.399994 ...

******************************************
      NEW SCRIPT FORMAT
******************************************

begin(patternset)
name: recorded
type: unknown

--- LASER VALUES ---
begin(pattern)
name: laser
time: 902803430 930960
lasers 180 180 : 291 289 276 ...
position: 3996.899902 4628.590820 178.340729
end(pattern)

--- SONAR VALUES ---
begin(pattern)
name: sonar
time: 902803430 600609
sonars 24 : 137.5 3481.2 ...
position: 3996.899902 4628.590820 178.305649
end(pattern)

end(patternset)

*/
   

void save_laser( FILE *fout, long sticks, long uticks, 
		 int numflaser, int numrlaser, int *laserval,
		 float posx, float posy, float poso );

void save_sonar( FILE *fout, long sticks, long uticks, 
		 int numsonar, float *sonarval,
		 float posx, float posy, float poso );

void write_header( FILE *fout );

void write_footer( FILE *fout );

long comp_ticks( int year, int month, int day, int hour, int min, int sec );

float float_rest( float zahl );



int
main( int argc, char *argv[] ) 
{
  struct timeval stime  = {0, 0};
  struct timeval ctime  = {0, 0};
  struct tm ts;
  
  FILE *fpin;
  FILE *fpout;
  
  char infile[256];
  char outfile[256];
  char line[4096];
  char command[4096];
  char dummy[4096];
  char sformat[4096];
  int day, month, year, hour, min;
  float sec;
  float cur_posx =0;
  float cur_posy =0;
  float cur_poso =0;
  int rline=0;
  int flaser, rlaser, nsonar, i;
  int laser[720];
  float sonar[256];
  int numpos=0;
  int numsonar=0;
  int numlaser=0;
  char *running;
  char *rtoken;
  
  if (argc==3) {
    strncpy(infile,argv[1],256);
    strncpy(outfile,argv[2],256);
    if ((fpin=fopen(infile,"r"))==0) {
      fprintf( stderr, "Can't open input file %s !\n", infile );
      return 0;
    }
    if ((fpout=fopen(outfile,"w"))==0) {
      fprintf( stderr, "Can't open output file %s !\n", outfile );
      return 0;
    }

    write_header(fpout);

    while (fgets(line,4096,fpin) != NULL) {
      rline++;
      if (rline%100==0)
	fprintf( stderr, "\r-> read line: %d", rline );
      if ((strlen(line)>1) && (sscanf(line,"%s",command)!=0)) {
	if (!strcmp( command, "@OPEN")) {
	  sscanf(line, "%s %d-%d-%d %d:%d:%f",
		 &dummy, &day, &month, &year,
		 &hour, &min, &sec );
	  ts.tm_year    = year;
	  ts.tm_mon     = month;
	  ts.tm_mday    = day;
	  ts.tm_hour    = hour;
	  ts.tm_min     = min;
	  ts.tm_sec     = sec;
	  ts.tm_isdst   = 0;
	  stime.tv_sec  = mktime(&ts);
	  stime.tv_usec = (long) ( float_rest( sec ) * 1000000 );
	} else if (!strcmp( command, "@SENS")) {
	  sscanf(line, "%s %d-%d-%d %d:%d:%f",
		 &dummy, &day, &month, &year,
		 &hour, &min, &sec );
	  ts.tm_year    = year;
	  ts.tm_mon     = month;
	  ts.tm_mday    = day;
	  ts.tm_hour    = hour;
	  ts.tm_min     = min;
	  ts.tm_sec     = sec;
	  ts.tm_isdst   = 0;
	  ctime.tv_sec  = mktime(&ts);
	  ctime.tv_usec = (long) ( float_rest( sec ) * 1000000 );
	} else if (!strcmp( command, "#LASER")) {
	  sscanf(line, "%s %d %d:", &dummy, &flaser, &rlaser );
	  running = line;
	  strtok( running, " ");
	  strtok( NULL, " ");
	  strtok( NULL, " ");
	  for (i=0; i<flaser+rlaser; i++) {
	    laser[i] = atoi( strtok( NULL, " ") );
	  }
	  save_laser( fpout, ctime.tv_sec, ctime.tv_usec, 
		      flaser, rlaser, laser, 
		      cur_posx, cur_posy, cur_poso );
	  numlaser++;
	} else if (!strcmp( command, "#SONAR")) {
	  sscanf( line, "%s %d:", &dummy, &nsonar );
	  running = line;
	  strtok( running, " "); 
	  strtok( NULL, " ");
	  for (i=0; i<nsonar; i++){
	    sonar[i] = atof( strtok( NULL, " ") );
	  }
	  save_sonar( fpout, ctime.tv_sec, ctime.tv_usec, 
		      nsonar, sonar, 
		      cur_posx, cur_posy, cur_poso );
	  numsonar++;
	} else if (!strcmp( command, "#ROBOT")) {
	  sscanf(line, "%s %f %f %f",
		 &dummy, &cur_posx, &cur_posy, &cur_poso );
	  numpos++;
	}
      }
    }

    fclose(fpin);

    write_footer(fpout);
    fclose(fpout);
    fprintf( stderr, "\r --- READY !!! ---         \n" );
    fprintf( stderr, " -> %d lines\n", rline );
    fprintf( stderr, " -> %d position scans\n", numpos );
    fprintf( stderr, " -> %d sonar scans\n", numsonar );
    fprintf( stderr, " -> %d laser scans\n", numlaser ); 
    return 0;
  } else {
    fprintf( stderr, "usage: %s <infile> <outfile>\n", argv[0] );
    return 0;
  }
}

void
write_header( FILE *fout )
{
    /* write header */
    fprintf( fout, "begin(patternset)\n" );
    fprintf( fout, "name: recorded\n" );
    fprintf( fout, "type: unknown\n\n" );
}

void
write_footer( FILE *fout )
{
    /* write footer */
    fprintf( fout, "end(patternset)\n" );
}

void
save_laser( FILE *fout, long sticks, long uticks, 
	    int numflaser, int numrlaser, int *laserval,
	    float posx, float posy, float poso )
{
  int i;
  fprintf( fout, "begin(pattern)\n" );
  fprintf( fout, "name: laser\n" );
  fprintf( fout, "time: %d %d\n", (int) sticks, (int) uticks );
  fprintf( fout, "lasers %d %d: ", numflaser, numrlaser );
  for (i=0; i<numflaser+numrlaser; i++) {
    fprintf( fout, "%d ", laserval[i] );
  }
  fprintf( fout, "\n" );
  fprintf( fout, "position: %f %f %f\n", posx, posy, poso );
  fprintf( fout, "end(pattern)\n\n" );
}

void
save_sonar( FILE *fout, long sticks, long uticks, 
	    int numsonar, float *sonarval,
	    float posx, float posy, float poso )
{
  int i;
  fprintf( fout, "begin(pattern)\n" );
  fprintf( fout, "name: sonar\n" );
  fprintf( fout, "time: %d %d\n", (int) sticks, (int) uticks );
  fprintf( fout, "sonars %d: ", numsonar );
  for (i=0; i<numsonar; i++) {
    fprintf( fout, "%.1f ", sonarval[i] );
  }
  fprintf( fout, "\n" );
  fprintf( fout, "position: %f %f %f\n", posx, posy, poso );
  fprintf( fout, "end(pattern)\n\n" );
}

float
float_rest( float zahl )
{
  return( zahl-((int) zahl) );
}

