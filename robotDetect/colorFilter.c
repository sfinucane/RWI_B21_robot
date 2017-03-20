/*
 * This file contains all functions relevant to robot Detection.
 * The filter is purely ad-hoc very much relies on the robust detection of
 *  the marker colors (red/green). 
 *
 *  
 *  readPPM, writePPM
 *  rgbFilter(rawImage, lr,lg,lb, ur,ug,ub)  
 *  disjunctImg()
 *  imageAND(rawImage1, rawImage2)
 *
 * $Id: colorFilter.c,v 1.1 2002/09/14 16:40:52 rstone Exp $
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <imagesize.h>
#include <math.h>
#ifdef i386                        // as opposed to SUN
#include <getopt.h>
#endif
#include <string.h>

#include "qcamspecs.h"
#include "functions.h"
#include "filterConfig.h"

#define IMAGESHIFT 5
#define BPP 3                  // bytes per pixel
#define IMAGESIZE ROWS*COLS*BPP
#define BLACK 0
#define WHITE 255
#define ROWSKIP 100             // lower rows which are not considered for marker detection
#define BOXWIDTH 5
#define BOXLENGTH 10
#define LASERYETANOTHER 180
#define GRADDECNTDEPTH 10        // 1 == disabled???





#ifdef i386
/* ---------------------------------------------------------
 *
 *  Assume PPM file is 320x240 RGB
 * --------------------------------------------------------
int openAndReadPPM(char* rgbImage, char *filename) {
  FILE *fp;
  long int ppmHeaderLength;

  // open the file and read the frames 
  fp = fopen( filename, "rb" );
  if ( !fp ) {
    perror("\nError");
    return -1;
  }

  fseek(fp, 0, SEEK_END);              // skip header      
  ppmHeaderLength = ftell(fp) - ROWS*COLS*3;
  fseek(fp,ppmHeaderLength, SEEK_SET);


  if ( fread(rgbImage,1,ROWS*COLS*3,fp)!=ROWS*COLS*3 ) {
    fprintf( stderr, "\nError reading frame from file, eof?\n");
    fclose( fp );
    return -1;
  }

  fclose( fp );

  return 0;
}

*/


/* ---------------------------------------------------------
 *
 * writePPMimage(char* rgbImage, char* filename ) writes rgbImage to ppm image.
 * note that the source data is three-bytes-per-pixel RGB
 *
 * --------------------------------------------------------*/
int writePPMimage(char* rgbImage, char *filename ) {
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
#endif

/*
 * colorStats returns some measurements about the type of colors used in the image
 */


/* -------------------------------------------------------------------------------
 * rgbFilter(char* rgbImage, int lred, int ured, int lgrn, int ugrn, int lblu, int ublu) blackens all pixels within the specified 
 * lower (lr lg lb)and upper RGB intervals;
 * all other pixels are set to white.
 * -------------------------------------------------------------------------------*/
void rgbFilter(char* rgbImage, int lred, int ured, int lgrn, int ugrn, int lblu, int ublu) {
  int x,y,i;

 for (y=0; y < ROWS; y++)
   for (x=0; x < COLS; x++) {
     i = (y*COLS + x) * 3;                                             /* i denotes the current pixel */
     if ((rgbImage[i] <= ured) && (rgbImage[i] >= lred)
	 && (rgbImage[i+1] <= ugrn) && (rgbImage[i+1] >= lgrn)
	 && (rgbImage[i+2] <= ublu) && (rgbImage[i+2] >= lblu)) {
       rgbImage[i] = 0; rgbImage[i+1] = 0; rgbImage[i+2] = 0;
     } else {
       rgbImage[i] = WHITE; rgbImage[i+1] = WHITE; rgbImage[i+2] = WHITE; 
     }
   }
}

/* ----------------------------------------------------------------------------
 * whitens all pixels within the specified RG-ratio and RB-ratio intervals;
 * all other pixels are set to black.
 * NOTE: rgbImage[i] keeps being misinterpreted as an int. Why? The 
 * type enforcements and maskings solve this problem, but it looks ugly..
 * ---------------------------------------------------------------------------- */
void rgbRatioFilter(char* rgbImage, double rgLow, double rgHigh, double rbLow, double rbHigh) {
  int x,y,i;
  double rg, rb;

 for (y=0; y < ROWS; y++)
   for (x=0; x < COLS; x++) {
     i = (y*COLS + x) * 3;                                             /* i denotes the current pixel */
     rg = 0;
     rb = 0;
     
     if (rgbImage[i+1]) 
       rg = (double) (rgbImage[i]& 0xff) / (double) (rgbImage[i+1] & 0xff);
     if (rgbImage[i+2])
       rb = (double)(rgbImage[i]& 0xff) / (double) (rgbImage[i+2] & 0xff);
     if ((rgLow <= rg) && (rg <= rgHigh)
	 && (rbLow <= rb) && (rb <= rbHigh)) {
       rgbImage[i] = WHITE; rgbImage[i+1] = WHITE; rgbImage[i+2] = WHITE;
     } else {
       rgbImage[i] = BLACK; rgbImage[i+1] = BLACK; rgbImage[i+2] = BLACK; 
     }
   }
}

/* ------------------------------------------------------------------------------------ 
 * disjunctImage(char* rgbImage, char *otherImage) makes both images disjunct by whitening pixels in rgbImage
 * that are already set (=black) in otherImage. 
 * Assumes rgbImage and otherImage are BW.
 * -------------------------------------------------------------------------------------*/
void disjunctImage(char* rgbImage, char *otherImage) {
  int x,y,i;

 for (y=0; y < ROWS; y++)
   for (x=0; x < COLS; x++) {
     i = (y*COLS + x) * 3;                                             /* i denotes the current pixel */
     if ((rgbImage[i] == 0) && (*otherImage == 0)) {                 /* pixels black in both images */
       rgbImage[i] = WHITE; rgbImage[i+1] = WHITE; rgbImage[i+2] = WHITE; 
     }
     otherImage += 3;
   }
} 


/* ------------------------------------------------------------------------------------
 * imageAND(char* rgbImage char* otherImage) ANDs rgbImage with specified otherImage.
 * NOTE: rgbImage is shifted IMAGESHIFT pixel rows up for processing but not changed permanently
 * One should load the green-filtered image into rgbImage, as green is the bottom part of the marker, i.e.
 * shifting the image up does not hurt.
 * Assumes both images are BW.
 * -------------------------------------------------------------------------------------*/
void imageAND(char* rgbImage, char *otherImage) {
  int x,y,i;
  

 for (y=IMAGESHIFT; y < ROWS; y++)
   for (x=0; x < COLS; x++) {
     i = (y*COLS + x) * 3;                                             /* i denotes the current pixel */
     rgbImage[i  ] &= ((unsigned char)*otherImage++) & 0xff;
     rgbImage[i+1] &= ((unsigned char)*otherImage++) & 0xff;
     rgbImage[i+2] &= ((unsigned char)*otherImage++) & 0xff;
   }
}


/* ------------------------------------------------------------------------------
 * getRobotCoordinates returns the median XY coordinates of non-black pixels, that are assumed
 * to be evidence of a detected robot.
 * For X and Y, it first projects all non-black pixels onto a line (i.e single column or row)
 * After counting the number of non-black pixels it returns the median X and Y values
 * This function has sideeffects: *resultX and *resultY are modified
 * -----------------------------------------------------------------------------*/
void getRobotMedianCoordinates(char* rgbImage, int* resultX, int* resultY)
{
 int x,y,i,n,k;
 char row[COLS];  // 320
 char column[ROWS]; // 240
 bzero(row, COLS);
 bzero(column, ROWS);

 for (y=0; y < ROWS; y++)
   for (x=0; x < COLS; x++) {
     i = (y*COLS + x) * 3; 
     if (rgbImage[i]) {
       row[x] = 1; column[y] = 1;
     }
   }      

 n = 0;
 for (y=0; y < ROWS; y++)
   if (column[y]) n++;
 y = 0; k = 0;
 do {
   if (column[y++]) k++;
 } while (k < n/2); 
 *resultY = y;
 
 n = 0;
 for (x=0; x < COLS; x++)
   if (row[x]) n++;

 x = 0; k = 0;
 do {
  if (row[x++]) k++;
 } while (k < n/2);
 *resultX = x;

}


/*
 * This version takes the average instead of the median
 */
void getRobotCoordinates(char* rgbImage, int* resultX, int* resultY)
{
 int n,x,y,i;
 int avgX =0;
 int avgY =0;
 
 n=0;
 for (y=0; y < ROWS; y++)
   for (x=0; x < COLS; x++) {
     i = (y*COLS + x) * 3; 
     if (rgbImage[i]) {
       avgX += x; avgY += y;
       n++;
     }
   }      
 
 if (n==0) return;
 *resultX = avgX/n;
 *resultY = (avgY/n)+IMAGESHIFT;

}


/* ---------------------------------------------------------------------------------------
 *  getRelativeAngle(int x) returns the angle of a pixel relative to a virtual vertical
 * line located in the middle of the image. Note that this assumes that the camera and
 * the marker are equidistant to the ground, which is not really the case.
 * ---------------------------------------------------------------------------------------*/
double getRelativeAngle(int x)
{
  int a;
  a  = (COLS/2) - x ;
  return (double)(-a * (VIEW/2)/(COLS/2));
}


/*
 * rgbx2rgb eliminates the fourth (x-)byte . Note
 * that raw32_to_24 from libcqcam does NOT do the same.
 */
void rgbx2rgb(char* rgbDstImage, char* rgbxSrcImage, int rows, int cols)
{
  int i=0; 
  int k=0;
  int x;
  for (x = 0 ; x < rows*cols; x++) {
    rgbDstImage[i++] = rgbxSrcImage[k++];
    rgbDstImage[i++] = rgbxSrcImage[k++];
    rgbDstImage[i++] = rgbxSrcImage[k++];
    k++;
  }
}


/*
 * 3x3 Median Filter for 3-byte per pixel B/W images
 */
void IPL_3median(char *imageIn, char *imageOut, int height, int width)
{
    int i;

    memset(imageOut, 0, width*BPP);

    /*
     *    loop over the image, starting at the second row, second column
     */

    for (i = width; i < width * (height - 1); i++)
    {
        int x, y;
        int k = 0;

        for (y = -1; y <= 1; y++) {       
	  for (x = -1; x <= 1; x++) {            
	    char val = imageIn[(i + y * width + x) * BPP];
	    if (val) k++; 		
	  }
	}
	if (k>4) {
	imageOut[i*BPP] = WHITE; imageOut[i*BPP+1] = WHITE; imageOut[i*BPP+2] = WHITE;       
	} else {
	imageOut[i*BPP] = BLACK; imageOut[i*BPP+1] = BLACK; imageOut[i*BPP+2] = BLACK;       
	}
    }

    memset(imageOut + i*BPP, 0, width*BPP);
}



/* -------------------------------------------------------------------
 * Fast version image->robot coordinates. 1 if successful, else return
 * value is 0.
 * Only one pass of the image is needed and one pass of the resulting
 * matrix for despeckling. BPP should be 3 (rgb) or 4 (rgbx) but is NOT
 * checked within the function.
 * We use 3x3 despeckling in order to lower the probability of false alarms.
 * We check IMAGESHIFT rows for red only. Green is important if and only if
 * IMAGESHIFT*COLS pixels before red was found, hence it is ANDed.
 * The last IMAGESHIFT rows are omitted completely for speedup. In practice,
 * more rows could probably be skipped.
 * Upon despeckling the matrix, we calculate the mean value of X and Y
 * coordinates. If we have no evidence, we conclude the marker was not detected
 * and 0 is returned.
 * The decimation process is efficient but complicated: bwMatrix is downscaled
 * by the decimation factor and the results are scaled back.
 * NOTICE: imgDecimation has been disabled.
 * -------------------------------------------------------------------*/
int fastGetMarkerCoordinates(char* image, int bpp, int imgDecimation, int* resultX, int* resultY) {
 unsigned char bwMatrix[ROWS*COLS];
 double rg, rb;
 int i,x,y,n;
 int avgX, avgY;
 int p;

imgDecimation = 1;
 rg = 0.0;
 rb = 0.0;
 
 /* we skip the first row completely, since the matrix will be despeckled later */
 p = COLS/imgDecimation;
 for (y=1; y<IMAGESHIFT; y+=imgDecimation)
   for (x=0; x<COLS; x+=imgDecimation) {
     i = y*COLS+x;    
     if (image[i*bpp+1]) 
       rg = (double) (image[i*bpp]& 0xff) / (double) (image[i*bpp+1] & 0xff);
     if (image[i*bpp+2])
       rb = (double)(image[i*bpp]& 0xff) / (double) (image[i*bpp+2] & 0xff);
     
     if ((RED_RGLOW <= rg) && (rg <= RED_RGHIGH)
	 && (RED_RBLOW <= rb) && (rb <= RED_RBHIGH)) bwMatrix[p] = 1;
     else bwMatrix[p] = 0;
     p++;
   }


 
 for (y=IMAGESHIFT; y<ROWS-ROWSKIP; y+=imgDecimation)
   for (x=0; x<COLS; x+=imgDecimation) {
     i = y*COLS+x;    
     if (image[i*bpp+1]) 
       rg = (double) (image[i*bpp]& 0xff) / (double) (image[i*bpp+1] & 0xff);
     if (image[i*bpp+2])
       rb = (double)(image[i*bpp]& 0xff) / (double) (image[i*bpp+2] & 0xff);
     
     /* we observe that the green interval is below the red interval */
     
     if ((rg < GRN_RGLOW) || (rb < GRN_RBLOW)
	 || (rg > RED_RGHIGH) || (rb > RED_RBHIGH)) bwMatrix[p] = 0;
     else if ((rg >= RED_RGLOW) && (rb >= RED_RBLOW)) bwMatrix[p] = 1; 
     else if ((rg <= GRN_RGHIGH) && (rb <= GRN_RBHIGH)) {
       bwMatrix[p] = 0;
       bwMatrix[p-(IMAGESHIFT/imgDecimation)*(COLS/imgDecimation)]++;      
     }
     else bwMatrix[p] = 0;
     p++;
}


/* now bwMatrix has 0 for non-green/non-red pixels, 1 for red-pixels and 2 for those involved in
   a red-green top-down transition. Only the latter are considered for despeckling */
 
 bzero(bwMatrix, COLS/imgDecimation);
 n = 0;
 avgX = 0;
 avgY = 0;

    /*
     *  Despeckling:  loop over the matrix, starting at the second row, second column
     * and skipping the lower ROWSKIP rows for speed.
     */

for (y = 1; y < (ROWS-ROWSKIP) / imgDecimation; y++)
     for (x = 1; x < (COLS/imgDecimation); x++) {
       int xOfs, yOfs;
       int k = 0;
       
       for (yOfs = -1; yOfs <= 1; yOfs++) {       
	 for (xOfs = -1; xOfs <= 1; xOfs++) {            
	   if (bwMatrix[(y+yOfs)*(COLS/imgDecimation)+ x + xOfs] == 2) k++;
	 }	 
       }       
       if (k>4) {
	 n++;
	 avgX += x; avgY += y;
       }
     }


 if (n!=0) {
   *resultX = (avgX/n) * imgDecimation;
   *resultY = ((avgY/n)+(IMAGESHIFT/imgDecimation)) * imgDecimation;   
   return 1; 
 }
 return 0;
}






/* -----------------------------------------------------------------
 * insertMarker() inserts a white solid box in image.
 * -----------------------------------------------------------------*/
void insertMarker(char* image, int xPos, int yPos, int bpp) {
 int x,y;
 char color = WHITE;

 for (y=-BOXWIDTH; y<=BOXWIDTH; y++) 
   for (x=-BOXLENGTH; x<=BOXLENGTH; x++) {
     image[((yPos+y)*COLS+xPos+x)*bpp+0] = color;
     image[((yPos+y)*COLS+xPos+x)*bpp+1] = color;
     image[((yPos+y)*COLS+xPos+x)*bpp+2] = color;
   }
}


/* -----------------------------------------------------------------
 * insertGrid() inserts a white solid box in image.
 * -----------------------------------------------------------------*/
void insertGrid(char* image, int bpp) {
 int x,y;
 char color = WHITE;

 for (x=32;x<COLS;x+=32)
   for (y=0;y<ROWS;y++)  {
     image[(y*COLS+x)*bpp] = 0;
     image[(y*COLS+x)*bpp] = 0;
     image[(y*COLS+x)*bpp] = 255;
   }
}


/* -----------------------------------------------------------------
 * preProcessing
 * Postfiltering by shrinking/enlarging? FFT? mild despeckle?
 * -----------------------------------------------------------------*/
int preProcessImage(char *rgbImage) {
  static unsigned char *otherImage = NULL;
  otherImage = (unsigned char *) malloc(sizeof(unsigned char) * ROWS * COLS * 3);

  memcpy(otherImage, rgbImage, (size_t) ROWS*COLS*3);

  rgbRatioFilter(otherImage, RED_RGLOW, RED_RGHIGH, RED_RBLOW, RED_RBHIGH);
 // IPL_median(otherImage, otherImage, ROWS, COLS);


 
  if ( writePPMimage(otherImage, "redFiltered.PPM" ) != 0 ) {
    fprintf( stderr, "RED: writePPMimage\n");
    return -1;
  }

  
  rgbRatioFilter(rgbImage, GRN_RGLOW, GRN_RGHIGH, GRN_RBLOW, GRN_RBHIGH);
//  IPL_median(rgbImage, rgbImage, ROWS, COLS);

 
  if ( writePPMimage(rgbImage, "grnFiltered.PPM" ) != 0 ) {
    fprintf( stderr, "GRN: writePPMimage\n");
    return -1;
  }

//  disjunctImage(otherImage);
  imageAND(rgbImage, otherImage);
  free(otherImage);

  IPL_3median(rgbImage, rgbImage, ROWS, COLS);
//  IPL_median(rgbImage, rgbImage, ROWS, COLS);
 return 1;
}


/* ----------------------------------------------------------- 
 * gradientDescent(int index, float* value)
 * If the robot stands for itself, it's very helpful to look
 * at laserValues around the estimated value[index]
 * the gradientDescent searches maximum GRADDECNTDEPTH left/right
 *
 * Note: two problems, you could be two beams away from the
 * right spot, also you will get misleading results if the
 * robot does not really stand for itself.
 *-----------------------------------------------------------*/
int gradientDescent(int index, float* value) {

  int minIndex = index;
  if ((minIndex > 0) && (minIndex < LASERYETANOTHER -1)) {
    while ((minIndex > 0) && (value[minIndex-1] < value[minIndex])
	   && (abs(index - minIndex) < GRADDECNTDEPTH))
      minIndex--;
    
    while ((minIndex < LASERYETANOTHER-1) && (value[minIndex+1] < value[minIndex])
	   && (abs(index - minIndex) < GRADDECNTDEPTH))
      minIndex++;
  }
  
  return minIndex;
}


/* ----------------------------------------------------------- 
 * getAngleAndDistance(x, laserSweep) maps an image-plane [0..COLS] x value
 * on a laserBeam number and extracts the associated distance.
 * In: camera-x, laserSweep[180]
 * Out: angle, distance
 *-----------------------------------------------------------*/
int getAngleAndDistance(int x, float* laserSweep,  
			 int* mAngle, float* mDistance) {
double temp;
int i;

  temp = 90+(VIEW/2-1) - (float)x * (VIEW-1)/(COLS-1) + ALIGNMT_CORRECTION;
//  fprintf(stderr, "Linear: %f %d\n", temp, (int)temp);

  temp = tan(deg2Rad(VIEW/2));
  temp = temp*(COLS/2-x)/(COLS/2);
  temp = rad2Deg(atan(temp));
//  fprintf(stderr, "Tangent: %f %d\n", temp, (int)temp);

  *mAngle = (int)temp + LASERYETANOTHER/2;
//  fprintf(stderr, "LaserNo before %d",*mAngle);
  i = gradientDescent(*mAngle, laserSweep);
  //  fprintf(stderr, "  . After gd %d\n", i);
  *mDistance= laserSweep[*mAngle]; 
  return i;
}
 


/* ----------------------------------------------------------- 
 * printAngleAndDistance() prints relative angle and distance
 * of marker, if detected.
 *-----------------------------------------------------------*/
void printAngleAndDistance(char* rgbImage, int bpp, int imgDecimation, float* laserSweep) {
  
  int a,x,y;
  float d;
  int *xRes=&x, *yRes=&y;
  float *mDistance=&d; 
  int *mAngle=&a;
  
  if (fastGetMarkerCoordinates(rgbImage, bpp, imgDecimation, xRes, yRes)) {
    getAngleAndDistance(*xRes, laserSweep, mAngle, mDistance);
    fprintf(stderr, "Relative angle: %d. Distance: %5.0f\n", *mAngle, *mDistance);	
  } else fprintf(stderr, "No marker detected.\n");
  
}		




/* ----------------------------------------------------------- 
 * getMarkerCoordinates() non-destructively filters rgbxImage (4Bytes) and returns
 * the x and y coordinates where the marker was found in the
 * image. If the marker was detected, 1 is returned, if not 0 and
 * -1 upon disk-IO errors.
 * NOTE: rgbxImage is 4 Bytes per pixel
 * ATTN: malloc'ed memory MUST be freed.
 *-----------------------------------------------------------*/
int getMarkerCoordinates(char* rgbxImage, int* xPtr, int* yPtr) {
    static unsigned char *rgbImage = NULL;  
#ifdef i386

  rgbImage = (unsigned char *) malloc(sizeof(unsigned char) * ROWS * COLS * 3);
  rgbx2rgb(rgbImage,rgbxImage, ROWS, COLS);

   if ( writePPMimage(rgbImage, "rgbImage.PPM" ) != 0 ) {
    fprintf( stderr, "RGB: writePPMimage\n");
    return -1;
  }
   preProcessImage(rgbImage);   
   getRobotCoordinates(rgbImage, xPtr,yPtr); 
   if ( writePPMimage(rgbImage, "filtered.PPM" ) != 0 ) {
     fprintf( stderr, "AND: writePPMimage\n");
     return -1;
   }
   free(rgbImage);
   return 1;

#else   
   fprintf( stderr, "This program requires Linux.\n");  
#endif
}
  

/*
 * $Log: colorFilter.c,v $
 * Revision 1.1  2002/09/14 16:40:52  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1999/04/28 21:31:32  fox
 * Demo version.
 *
 * Revision 1.3  1997/07/25 16:34:32  swa
 * extractframe now compiles under SUN as well.
 *
 * Revision 1.2  1997/07/25 02:41:42  swa
 * Added support for mpeg. Changed README accordingly.
 *
 * Revision 1.1  1997/07/24 21:21:00  swa
 * Added extractFrame, which extracts a particular frame from a raw imagefile,
 * and writes that frame to a ppm file.
 *
 *
 */
