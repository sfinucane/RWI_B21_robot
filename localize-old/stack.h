//*****************************************************************************
// stack.h: code to read in a histogram stack and calculate a probability from it
//
// Copyright (c) 1997-98 Frank Dellaert 
// All rights reserved
//*****************************************************************************

#ifndef __STACK__
#define __STACK__

#include <stdio.h> // for FILE :-(

#ifdef  __cplusplus
extern "C" {
#endif

//*****************************************************************************
// HistogramStack ADT
//*****************************************************************************

struct HistogramStackStruct;
typedef struct HistogramStackStruct HistogramStack;

// create ADT
HistogramStack *HistogramStack_New
  (int width, int height, int cmPerCell);

// delete ADT
void HistogramStack_Delete
  (HistogramStack *stack);

// Load a stack of histograms
void HistogramStack_Load
  (HistogramStack *stack, const char *path, int verbose);

// Calculate P(measurement|location)
double HistogramStack_GetProbability
  (HistogramStack *stack, unsigned char measurement, int cellIndex);

// Calculate probabilities for a given location
void HistogramStack_GetProbabilities
  (HistogramStack *stack, int cellIndex, double *probabilities);

// Print probabilities for a given location
void HistogramStack_PrintProbabilities
  (HistogramStack *stack, int cellIndex, FILE* fp);

// calculate index for a location given in integer x,y cell coordinates
int HistogramStack_CalculateCellIndex
  (HistogramStack *stack, int x, int y);

// create a mean image
void HistogramStack_CreateMeanImage
  (HistogramStack *stack, const char *filename);

// convolve histogram probabilities with zero-mean measurement noise
void HistogramStack_Convolve
  (HistogramStack *stack, unsigned char standardDeviation);

// add obstruction density
void HistogramStack_AddObstructionDensity
  (HistogramStack *stack, double obstructionProbability);

// save a stack of histograms
void HistogramStack_Save
  (HistogramStack *stack, const char *path, int verbose);

//*****************************************************************************

#ifdef  __cplusplus
  }
#endif

#endif // __STACK__


