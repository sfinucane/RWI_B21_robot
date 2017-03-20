

#define MAX_STRING_LENGTH 80
#define NUM_LASER_SENSORS 180


/* ---------------------------------------------------------
 *
 * writePPMimage(char* rgbimage, char *filename ) writes rgbImage to ppm image. note, that the source data is RGB
 *
 * --------------------------------------------------------*/
int writePPMimage( char *filename ) {
  FILE *fp;

  /* rgbImage the current image into a .ppm file */
  fp = fopen( filename, "wb" );
  if ( !fp ) {
    perror("Error");
    return -1;
  }

  fprintf(fp,"P6\n%d %d\n255\n",COLS,ROWS);
  if ( ROWS*COLS*3 != fwrite( rgbImage,sizeof(char),ROWS*COLS*3,fp ) ) {
    perror("writePPMimage");
    exit(-1);
  }
	
  fclose( fp );

  return 0;
}


/* -----------------------------------------------------
 * parsePatternFile()
 * Parser routine for pattern files.
 * We assume a strict order of (laser, image) patterns which
 * is generated through a modified version of learn (io.c).
 * Analogously, getMarkerCoordinates is processed after
 * the image data is available, since it assumed to be last
 * within the (position, laser, image) pattern.
 *
 * See also dat.c, function int mem_load_patterns()
 *
 * -----------------------------------------------------*/

int parsePatternFile(char *filename) {
  FILE  *iop;
  int   i, file_ended, error, size_i, size_j, n, m;
  char  name[MAX_STRING_LENGTH];
  char  text[MAX_STRING_LENGTH];
  char  command[MAX_STRING_LENGTH];  
  int   reading_pattern_set, reading_pattern;
  int   value;
  unsigned char *image;
  float float_value;
  float *lasers;
  int verbose = 1;
  struct timeval time;

  int *xRes, *yRes;
  int *mAngle;
  float *mDistance;

  image    = (unsigned char *) malloc(sizeof(unsigned char) * ROWS * COLS * 3);
  lasers   = (float *) malloc(sizeof(float) * (NUM_LASER_SENSORS));

  if (image == NULL ||  lasers == NULL){
    fprintf(stderr, "ERROR: Out of memory in mem_load_patterns().\n");
    exit (-1);
  }


  fprintf(stderr, "Loading patterns from file %s...", filename);


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
	       !strcmp(command, "time:")){
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
       * image:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 1 &&
	       !strcmp(command, "image")){
	if (fscanf(iop, "%d %d : ", &size_i, &size_j) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else {
	  if (verbose)
	    fprintf(stderr, "image: %d %d\n", size_i, size_j);
	  if (size_i != COLS || size_j != ROWS){
	    fprintf(stderr, 
		    "ERROR: image format mismatch (%d %d vs %d %d).\n", 
		    size_i, size_j, COLS, ROWS);
	    error = 1;
	  }
	  for (i = 0; i < size_i * size_j * 3 && !error; i++)
	    if (fscanf(iop, "%2x", &value) == EOF){
	      fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	      error = 1;
	      file_ended = 1;
	    }
	    else image[i] = (unsigned char) value;
	  
	  if (!error) {

	    if (fastGetMarkerCoordinates(image, 3, xRes, yRes)) {
	      markerDetected = 1;
	      getAngleAndDistance(*xRes, lasers, mAngle, mDistance);
	      insertMarker(image, *xRes, *yRes, 3);
	      fprintf(stderr, "camera-X: %d. camera-Y:: %d\n", *xRes, *yRes);
	      fprintf(stderr, "Laser number: %d. Distance: %5.0f\n", mAngle, mDistance);	
	    } else { 
	      markerDetected = 0; 
	      fprintf(stderr, "No marker detected.\n");   
	    }
	    writePPM(image, "patternOut.ppm");
	    system("xv patternOut.ppm -drift 0 0 -geometry 150x120+100+100");  	  
	    updateLaserDisplay(lasers,mAngle);
	    EZX_Flush();	      	    
	    fprintf(stderr,"Press a letter for next frame");
	    fflush(NULL);
	    int key = getchar();        
	  }
	}
      }
      
      
      
      
      /*
       * lasers:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 1 &&
	       !strcmp(command, "lasers")){
	if (fscanf(iop, "%d %d : ", &size_i, &size_j) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else {
	  if (verbose)
	    fprintf(stderr, "lasers: %d %d\n", size_i, size_j);
	  if (size_i != NUM_LASER_SENSORS){
	    fprintf(stderr, 
		    "ERROR: infrared format mismatch (%d %d vs %d %d).\n", 
		    size_i, size_j,
		    NUM_LASER_SENSORS, NUM_LASER_SENSORS);
	    error = 1;
	  }
	  for (i = 0; i < size_i + size_j && !error; i++)
	    if (fscanf(iop, "%g", &float_value) == EOF) {
	      fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	      error = 1;
	      file_ended = 1;
	    }
	    else lasers[i] = float_value;
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
  free(lasers);
  free(image);

  return 1;
  
}




