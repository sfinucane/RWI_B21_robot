
#include <stdio.h>
#include <math.h>
#include "fakeSensors.hh"

main(int argc, char **argv)
{
  float xrot, yrot;

  float distance;
  int times;
  int i;
  FILE *mapfd = fopen(argv[1], "r");
  /* FILE* fd= fopen("/tmp/plotfile", "w"); */
  installSimMap(mapfd);

  for(times = 0; times < 500; times++) {
    for(i = 0; i < 360; i++) {
      getDistance(900.0, 900.0, 20.0,
		  cos(M_PI/180 * (double)i), sin(M_PI/180 * (double)i),0.0,
		  500.0, &distance);
      /* fprintf(fd,"%g %g\n", distance*cos((double)i*M_PI/180), distance*sin((double)i*M_PI/180)); */
    }
    /*fclose(fd); */
  }
  exit(0);
}
