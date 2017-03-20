
//*****************************************************************************
// stack.c: code to read in a histogram stack and calculate a probability from it
//
// Copyright (c) 1997-98 Frank Dellaert 
// All rights reserved
//*****************************************************************************

#include "stack.h"

#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//*****************************************************************************
// HistogramStack ADT
//*****************************************************************************

typedef unsigned char uchar;
typedef float probability;

struct HistogramStackStruct
  {
  probability **Pzx;
  int width, height;
  int cmPerCell;
  };

//*****************************************************************************
// error handler
//*****************************************************************************

void error(const char *msg)
  {
  fprintf(stderr,msg);
  exit(1);
  }

//*****************************************************************************
// Read a PGM file
//*****************************************************************************

uchar *ReadPGM(const char *filename, int *width, int *height)
{
  FILE *fp; 
  char buf[200];
  uchar *data;
  size_t size;

  fp = fopen(filename, "rb");
  if (fp==0)
    {
      printf("ReadPGM: pgm file %s not found\n",filename);
      exit(1);
    }

  /*--- the header ---------------*/

  fgets(buf, 200, fp); // P5
  fgets(buf, 200, fp); // possible comment line
  while(buf[0]=='#')
    fgets(buf, 200, fp);
	sscanf(buf,"%03d %03d\n",width,height);
  fgets(buf, 200, fp); // get 255 or comment line
  while(buf[0]=='#')
    fgets(buf, 200, fp);

  /*--- the i ---------------*/

  size = (*width)*(*height);
  data = (uchar *)malloc(size);
  if (data==NULL) error("ReadPGM malloc failed");
  fread(data, 1, size, fp);

  fclose(fp);

  return data;
}

//*****************************************************************************
// Save a pgm image
//*****************************************************************************

void WritePGM(uchar *data, int width, int height, const char *filename)
{
  FILE *fp; 
  fp = fopen(filename, "wb");
  if (fp==0)
    {
      printf("WritePGM: could not open %s for writing\n",filename);
      exit(1);
    }

  /*--- the header ---------------*/
	fprintf(fp,"P5\n");
	fprintf(fp,"# Created by Frank Dellaert pgm.c(pp)\n");
	fprintf(fp,"%03d %03d\n",width, height);
	fprintf(fp,"255\n");

  /*--- write image out to file ---------------*/
  fwrite(data, 1, width*height, fp);

  fclose(fp);
}

//*****************************************************************************
// create ADT
//*****************************************************************************

HistogramStack *HistogramStack_New(int width, int height, int cmPerCell)
  {
  HistogramStack *stack = malloc(sizeof(HistogramStack));
  if (stack==NULL)
    error("HistogramStack_New malloc failed");

  // initialize fields
  stack->Pzx = NULL;
  stack->width = width;
  stack->height=height;
  stack->cmPerCell= cmPerCell;
  return stack;
  }

//*****************************************************************************
// delete ADT
//*****************************************************************************

void HistogramStack_Delete(HistogramStack *stack)
  {
  if(stack->Pzx != NULL)
    {
    int i;
    for(i=0;i<256;i++)
      free((stack->Pzx)[i]);
    free(stack->Pzx);
    }
  free(stack);
  }

//*****************************************************************************
// Load one image and convert it to cumulative probability function
//*****************************************************************************

static float oneOver255 = (float)(1.0/255.0);

static void HistogramStack_LoadOneImage
(HistogramStack *stack, const char *path, int verbose, int i)
  {
  int width, height;
  uchar *data;
  probability *cdf;
  int cell;
  char filename[1000];
  int size = stack->width*stack->height;
  
  // create the filename
  sprintf(filename,"%s/bin%03d.pgm",path,i);
  if (verbose)
    fprintf(stderr,"loading %s\n",filename);
  
  // load the unsigned char image
  data = ReadPGM(filename, &width, &height);
  if (width!=stack->width || height!=stack->height)
    error("HistogramStack_Load size mismatch");

  // allocate the cumulative probability array for this value (i)
  cdf = malloc((size_t)(size*sizeof(probability)));
  if (cdf==NULL)
    error("HistogramStack_Load malloc failed");

  // copy the values from unsigned char to the cdf
  for(cell=0;cell<size;cell++)
    cdf[cell] = oneOver255*(float)(data[cell]);

  // free the unsigned char image
  free(data);

  // store this pointer in the ADT
  (stack->Pzx)[i] = cdf;
  }

//*****************************************************************************
// convert cumulative probabilities into a probability mass function
//*****************************************************************************

static void HistogramStack_ConvertFromCumulative(HistogramStack *stack)
  {
  int i,cell;
  int size = stack->width*stack->height;

  for(i=255;i>=0;i--)
    {
    probability *cdf = (stack->Pzx)[i];
    probability *pmf = (stack->Pzx)[i];
    if (i==0)
      {
      for(cell=0;cell<size;cell++)
        pmf[cell] = cdf[cell];
      }
    else
      {
      probability *cdfbelow = (stack->Pzx)[i-1];
      for(cell=0;cell<size;cell++)
        pmf[cell] = cdf[cell] - cdfbelow[cell];
      }
    }
  }

//*****************************************************************************
// Load a stack of histograms
// The histograms are stored as 256 PGM images
// Each pixel stores the CUMULATIVE probability value for that location
// up to the value to which the images corresponds.
// Thus, bin000.pgm is probably black and bin255.pgm should be entirely white !
// This is done to get rid of truncation errors that screw up normalization
//*****************************************************************************

void HistogramStack_Load(HistogramStack *stack, const char *path, int verbose)
  {
  int i;

  if(stack->Pzx != NULL)
    error("HistogramStack_Load called with non-empty stack");
  
  // allocate space for the probabilities
  stack->Pzx = malloc(256*sizeof(probability*));
  if (stack->Pzx==NULL)
    error("HistogramStack_Load malloc failed");

  // first simply load the cdf values into the stack struct
  for(i=0;i<256;i++)
    HistogramStack_LoadOneImage(stack, path, verbose, i);

  // now convert cumulative probabilities into a probability mass function
  HistogramStack_ConvertFromCumulative(stack);
  }

//*****************************************************************************
// Calculate P(measurement|location)
//*****************************************************************************

double HistogramStack_GetProbability
(HistogramStack *stack, unsigned char measurement, int cellIndex)
  {
  return (double)((stack->Pzx)[measurement][cellIndex]);
  }

//*****************************************************************************
// Calculate cdf for a given location
//*****************************************************************************

void HistogramStack_GetProbabilities
(HistogramStack *stack, int cellIndex, double *cdf)
  {
  int i;
  for(i=0;i<256;i++)
    cdf[i] = HistogramStack_GetProbability
                         (stack,(unsigned char)i,cellIndex);
  }

//*****************************************************************************
// Print cdf for a given location
//*****************************************************************************

void HistogramStack_PrintProbabilities
(HistogramStack *stack, int cellIndex, FILE* fp)
  {
  int i;
  double cdf[256];
  HistogramStack_GetProbabilities(stack,cellIndex,cdf);
  for(i=0;i<256;i++)
    fprintf(fp,"%d %e\n",i,cdf[i]);
  }

//*****************************************************************************
// calculate index for a location given in integer x,y cell coordinates
//*****************************************************************************

int HistogramStack_CalculateCellIndex(HistogramStack *stack, int x, int y)
  {
  return y*(stack->width) + x;
  }

//*****************************************************************************
// create a mean image
//*****************************************************************************

void HistogramStack_CreateMeanImage(HistogramStack *stack, const char
*filename)
  {
  int w = stack->width;
  int h = stack->height;
  int cell;
  int size = w*h;
  uchar *data = (uchar *)malloc((size_t)size);
  if (data==NULL) error("HistogramStack_CreateMeanImage malloc failed");

  for(cell=0;cell<size;cell++)
    {
    int i;
    double mean = 0;
    for(i=0;i<256;i++)
      mean += (double)i*HistogramStack_GetProbability(stack,(unsigned
char)i,cell);
    data[cell] = (unsigned char)mean;
    }
  WritePGM(data, w, h, filename);
  }

//*****************************************************************************
// convolve histogram probabilities with zero-mean measurement noise
//*****************************************************************************

#define MARGIN 100
#define CONVOLVED_SIZE MARGIN+256+MARGIN

void HistogramStack_Convolve(HistogramStack *stack, unsigned char
standardDeviation)
  {
  float convolved[CONVOLVED_SIZE];
  int i,j,cell;
  int size = stack->width*stack->height;
  int filterCenter = standardDeviation*3;
  int filterSize = 1+filterCenter*2;
  float sum = 0, variance = (float)(standardDeviation*standardDeviation);
  float *filter = (float *)malloc(filterSize*sizeof(float));
  if (filter==NULL) error("HistogramStack_Convolve malloc failed");

  // calculate the filter
  for(j=0;j<filterSize;j++)
    {
    double e = j-filterCenter;
    float weight = (float)exp(-0.5*e*e/variance);
    filter[j] = weight;
    sum += weight;
    }

  // normalize the filter
  for(j=0;j<filterSize;j++)
    filter[j] /= sum;

  // for each cell in the grid
  for(cell=0;cell<size;cell++)
    {
    // zero out temporary storage
    for(i=0;i<CONVOLVED_SIZE;i++)
      convolved[i] = 0;

    // convolve the histogram
    for(i=0;i<256;i++)
      {
      float original = (stack->Pzx)[i][cell];
      for(j=0;j<filterSize;j++)
        convolved[MARGIN+i+j-filterCenter] += filter[j] * original;
      }
    
    // and copy it back into the images
    for(i=0;i<256;i++)
      (stack->Pzx)[i][cell] = convolved[MARGIN+i];
    
    // making sure to fold overlap into extreme bins
    for(i=0;i<MARGIN;i++)
      {
      (stack->Pzx)[  0][cell] += convolved[i];
      (stack->Pzx)[255][cell] += convolved[MARGIN+256+i];
      }
    }

  free(filter);
  }

//*****************************************************************************
// add obstruction density
//*****************************************************************************

static float oneOver256 = (float)(1.0/256.0);

void HistogramStack_AddObstructionDensity(HistogramStack *stack, double
obstructionProbability)
  {
  int i,cell;
  int size = stack->width*stack->height;
  float PO = (float)obstructionProbability, PU=(float)(1.0-PO);

  // just do P(z|x) = P(not O)*P(z|x,not O) + P(O)*P(z|x,O)
  // where P(z|x,O) = 1/256, by lack of anything better

  for(i=0;i<256;i++)
    {
    probability *pmf = (stack->Pzx)[i];
    for(cell=0;cell<size;cell++)
      pmf[cell] = PU*pmf[cell] + PO*oneOver256;
    }
  }

//*****************************************************************************
// convert probability mass function into cumulative probabilities  
//*****************************************************************************

static void HistogramStack_ConvertToCumulative(HistogramStack *stack)
  {
  int i,cell;
  int size = stack->width*stack->height;

  for(i=0;i<256;i++)
    {
    probability *cdf = (stack->Pzx)[i];
    probability *pmf = (stack->Pzx)[i];
    if (i==0)
      {
      for(cell=0;cell<size;cell++)
        cdf[cell] = pmf[cell];
      }
    else
      {
      probability *cdfbelow = (stack->Pzx)[i-1];
      for(cell=0;cell<size;cell++)
        cdf[cell] = cdfbelow[cell] + pmf[cell];
      }
    }
  }

//*****************************************************************************
// Load one image and convert it to cumulative probability function
//*****************************************************************************

static void HistogramStack_SaveOneImage
(HistogramStack *stack, const char *path, int verbose, int i)
  {
  uchar *data;
  probability *cdf = (stack->Pzx)[i];
  int cell;
  char filename[1000];
  int size = stack->width*stack->height;
  
  // create the filename
  sprintf(filename,"%s/bin%03d.pgm",path,i);
  if (verbose)
    fprintf(stderr,"saving %s\n",filename);
  
  // allocate an image
  data = malloc((size_t)(size*sizeof(uchar)));
  if (data==NULL)
    error("HistogramStack_Save malloc failed");

  // copy the values from the cdf
  for(cell=0;cell<size;cell++)
    data[cell] = (unsigned char)(255.0*cdf[cell]);

  // save the unsigned char image
  WritePGM(data, stack->width, stack->height, filename);

  // free the unsigned char image
  free(data);
  }

//*****************************************************************************
// Save a stack of histograms
// see load for format comments
//*****************************************************************************

void HistogramStack_Save(HistogramStack *stack, const char *path, int verbose)
  {
  int i;

  // first convert back to a cumulative distribution function
  HistogramStack_ConvertToCumulative(stack);

  // now save each plane to a pgm image
  for(i=0;i<256;i++)
    HistogramStack_SaveOneImage(stack, path, verbose, i);
  }

//*****************************************************************************


