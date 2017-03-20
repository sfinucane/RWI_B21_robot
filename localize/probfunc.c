#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>
#include <values.h>

#define MAX_NUMBER_OF_DISTANCES 255
#define BUFFLEN 1000





int
main(argc, argv)
     int argc;
     char *argv[];
{

  FILE *fp;
  
  int distFuncCount[MAX_NUMBER_OF_DISTANCES][MAX_NUMBER_OF_DISTANCES];
  int i, j;
  int maxI = 0;
  int maxJ = 0;

  int sum;
  
  if (argc > 2)
  {
    fprintf(stderr, "usage: %s file\n", argv[0]);
    exit(0);
  }      

  for (i = 0; i < MAX_NUMBER_OF_DISTANCES; i++)
    for (j = 0; j < MAX_NUMBER_OF_DISTANCES; j++)
      distFuncCount[i][j] = 0;

  if (argc == 2){
     if ((fp = fopen(argv[1],"rt")) == NULL) {
	fprintf(stderr,"ERROR: Could not open file '%s'! Skipped\n",
		argv[1]);
	exit(0);
     }
  }
  else
     fp = stdin;

  while (!feof(fp)){
     char line[BUFFLEN];
     if (fgets(line,BUFFLEN,fp) != NULL)
	if (sscanf(line, "%d %d", &i, &j) == 2)
	   if (i >= MAX_NUMBER_OF_DISTANCES || j >= MAX_NUMBER_OF_DISTANCES){
	      fprintf(stderr, "More than %d distances, fix that first!\n",
		      MAX_NUMBER_OF_DISTANCES);
	      exit(0);
	  }
	   else{
	      if (i >= 0 && j >= 0)
		 distFuncCount[i][j]++;
	      if (i > maxI) maxI = i;
	      if (j > maxJ) maxJ = j;
	   }
  }
  
  if (fp != stdin) fclose(fp);
  
  for (i = 0; i <= maxI; i++){
     sum = 0;
     for (j = 0; j <= maxJ; j++)
	sum += distFuncCount[i][j];
     
     if (sum > 0.0)
	for (j = 0; j <= maxJ; j++)
	   if (distFuncCount[i][j] != 0)
	      printf("%d %d %e %d\n", i, j, (float) distFuncCount[i][j] / sum,
		     distFuncCount[i][j]);
     
     printf("\n");
  }
  
  
  
  exit(0);
  
}


