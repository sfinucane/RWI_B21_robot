
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** Welcome!
 *****
 ***** This file is part of the robot control software provided
 ***** by Real World Interface Inc.
 *****
 ***** All copyrights are by Real World Interface Inc., Carnegie
 ***** Mellon University, and the University of Bonn, Germany.
 ***** Removing the copyright information is illegal. Please read
 ***** and make sure you understand the disclaimer below.
 *****
 ***** Contact tyson@rwii.com if you have questions or would
 ***** like to obtain further information.
 *****
 ***** We hope you enjoy working with this software package.
 *****
 *****                      Tyson D. Sawyer and Sebastian Thrun
 *****
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** 
 ***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED
 ***** BY APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING
 ***** THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM
 ***** "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR
 ***** IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 ***** OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE
 ***** ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM
 ***** IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME
 ***** THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
 ***** 
 ***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
 ***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
 ***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
 ***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
 ***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
 ***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO
 ***** LOSS OF DATA OR DATA BEING RENDERED INACCURATE OR LOSSES
 ***** SUSTAINED BY YOU OR THIRD PARTIES OR A FAILURE OF THE PROGRAM
 ***** TO OPERATE WITH ANY OTHER PROGRAMS OR FAILURE TO CONTROL A
 ***** PHYSICAL DEVICE OF ANY TYPE), EVEN IF SUCH HOLDER OR OTHER
 ***** PARTY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/allocate.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:39 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: allocate.c,v $
 * Revision 1.1  2002/09/14 20:45:39  rstone
 * *** empty log message ***
 *
 * Revision 1.9  2000/01/10 19:04:19  fox
 * DON'T USE!
 *
 * Revision 1.8  1999/01/11 19:47:45  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.7  1998/08/20 00:22:55  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.6  1996/12/13 09:45:24  wolfram
 * Fixed a bug in free1D and free2D
 *
 * Revision 1.5  1996/12/02 18:46:24  fox
 * First version with the new expected distances.
 *
 * Revision 1.4  1996/12/02 10:31:59  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.3  1996/11/18 09:58:28  ws
 * Added support for selecting the optimal sensor
 * Changed some variables to improve compatibility with C++
 *
 * Revision 1.2  1996/11/15 17:44:04  ws
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:31  rhino
 * General reorganization of the directories/repository, fusion with the
 * RWI software.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "allocate.h"


void
free1D( void* field, int type)
{
  switch (type){
  case INT:
    free( (int*) field);
    break;
  case FLOAT:
        free( (float*) field);
    break;
  case DOUBLE:
        free( (double*) field);
    break;
  case BOOL:
        free( (bool*) field);
    break;
  case PROBABILITY:
        free( (probability*) field);
    break;
  case MAP_PROBABILITY:
        free( (mapProbability*) field);
    break;
  case REAL_POSITION:
         free( (realPosition*) field);
    break;
  case POSITION:
         free( (position*) field);
    break;
  }
}

void
free2D(void **field, int dim, int type)
{
  int i;
  switch (type){
  case INT:
    {
      int **p = (int **) field;
      for (i = 0; i < dim; i++)
	free(p[i]);
      free(p);
    }
    break;
  case FLOAT:
    {
      float **p = (float **)field;
      for (i = 0; i < dim; i++)
	free(p[i]);
      free(p);
    }
    break;
  case DOUBLE:
    {
      double **p = (double **) field;
      for (i = 0; i < dim; i++)
	free(p[i]);
      free(p);
    }
    break;
  case BOOL:
    {
      bool **p = (bool **) field;
      for (i = 0; i < dim; i++)
	free(p[i]);
      free(p);
    }
    break;
  case PROBABILITY:
    {
      probability **p = (probability **) field;
      for (i = 0; i < dim; i++)
	free(p[i]);
      free(p);
    }
    break;
  case MAP_PROBABILITY:
    {
      mapProbability **p = (mapProbability **) field;
      for (i = 0; i < dim; i++)
	free(p[i]);
      free(p);
    }
    break;
  case REAL_POSITION:
    {
      realPosition **p = (realPosition **) field;
      for (i = 0; i < dim; i++)
	free(p[i]);
      free(p);
    }
    break;
  case POSITION:
    {
      position **p = (position **) field;
      for (i = 0; i < dim; i++)
	free(p[i]);
      free(p);
    }
    break;
  case EXPECTED_DISTANCE:
    {
      expectedDistance **p = (expectedDistance **) field;
      for (i = 0; i < dim; i++)
	free(p[i]);
      free(p);
    }
    break;
  case PIXEL:
    {
      pixel **p = (pixel **) field;
      for (i = 0; i < dim; i++)
	free(p[i]);
      free(p);
    }
    break;
  }
}

void*
allocate1D( int dim, int type)
{
  switch (type) {
  case INT:
    {
      int* pt;
      if ( (pt = (int*) malloc( dim * sizeof(int))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate1D>.\n",
		 type);
	fprintf( stderr, "Desired dimension: %d\n", dim);
      }
      return (void*) pt;
    }
  case FLOAT:
    {
      float* pt;
      if ( (pt = (float*) malloc( dim * sizeof(float))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate1D>.\n",
		 type);
	fprintf( stderr, "Desired dimension: %d\n", dim);
      }
      return (void*) pt;
    }
  case DOUBLE:
    {
      double* pt;
      if ( (pt = (double*) malloc( dim * sizeof(double))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate1D>.\n",
		 type);
	fprintf( stderr, "Desired dimension: %d\n", dim);
      }
      return (void*) pt;
    }
  case BOOL:
    {
      bool* pt;
      if ( (pt = (bool*) malloc( dim * sizeof(bool))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate1D>.\n",
		 type);
	fprintf( stderr, "Desired dimension: %d\n", dim);
      }
      return (void*) pt;
    }
  case PROBABILITY:
    {
      probability* pt;
      if ( (pt = (probability*) malloc( dim * sizeof(probability))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate1D>.\n",
		 type);
	fprintf( stderr, "Desired dimension: %d\n", dim);
      }
      return (void*) pt;
    }
  case MAP_PROBABILITY:
    {
      mapProbability* pt;
      if ( (pt = (mapProbability*) malloc( dim * sizeof(mapProbability))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate1D>.\n",
		 type);
	fprintf( stderr, "Desired dimension: %d\n", dim);
      }
      return (void*) pt;
    }
  case EXPECTED_DISTANCE:
    {
      expectedDistance* pt;
      if ( (pt = (expectedDistance*) malloc( dim * sizeof(expectedDistance))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate1D>.\n",
		 type);
	fprintf( stderr, "Desired dimension: %d\n", dim);
      }
      return (void*) pt;
    }
  case REAL_POSITION:
    {
      realPosition* pt;
      if ( (pt = (realPosition*) malloc( dim * sizeof(realPosition))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate1D>.\n",
		 type);
	fprintf( stderr, "Desired dimension: %d\n", dim);
      }
      return (void*) pt;
    }
  case POSITION:
    {
      position* pt;
      if ( (pt = (position*) malloc( dim * sizeof(position))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate1D>.\n",
		 type);
	fprintf( stderr, "Desired dimension: %d\n", dim);
      }
      return (void*) pt;
    }
  default:
    fprintf( stderr, "Type %d not implemented.\n", type);
    return NULL;
  }
}

void**
allocate2D( int dim1, int dim2, int type)
{
  switch (type) {
  case INT:
    {
      int x;
      int** pt;
      if ( (pt = (int**) malloc( dim1 * sizeof( int*))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		 type);
	fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	return (void**) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (int*) malloc( dim2 * sizeof(int))) == NULL) {
	  fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		   type);
	  fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	  return (void**) NULL;
	}
      return (void**) pt;
    }
  case MAP_PROBABILITY:
    {
      int x;
      mapProbability** pt;
      if ( (pt = (mapProbability**)
	    malloc( dim1 * sizeof( mapProbability*))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		 type);
	fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	return (void**) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (mapProbability*)
	malloc( dim2 * sizeof(mapProbability))) == NULL) {
	  fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		   type);
	  fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	  return (void**) NULL;
	}
      return (void**) pt;
    }
  case PROBABILITY:
    {
      int x;
      probability** pt;
      if ( (pt = (probability**)
	    malloc( dim1 * sizeof( probability*))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		 type);
	fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	return (void**) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (probability*)
	malloc( dim2 * sizeof(probability))) == NULL) {
	  fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		   type);
	  fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	  return (void**) NULL;
	}
      return (void**) pt;
    }
  case FLOAT:
    {
      int x;
      float** pt;
      if ( (pt = (float**)
	    malloc( dim1 * sizeof( float*))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		 type);
	fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	return (void**) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (float*)
	malloc( dim2 * sizeof(float))) == NULL) {
	  fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		   type);
	  fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	  return (void**) NULL;
	}
      return (void**) pt;
    }
  case DOUBLE:
    {
      int x;
      double** pt;
      if ( (pt = (double**)
	    malloc( dim1 * sizeof( double*))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		 type);
	fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	return (void**) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (double*)
	malloc( dim2 * sizeof(double))) == NULL) {
	  fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		   type);
	  fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	  return (void**) NULL;
	}
      return (void**) pt;
    }
  case POSITION:
    {
      int x;
      position** pt;
      if ( (pt = (position**)
	    malloc( dim1 * sizeof( position*))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		 type);
	fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	return (void**) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (position*)
	malloc( dim2 * sizeof(position))) == NULL) {
	  fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		   type);
	  fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	  return (void**) NULL;
	}
      return (void**) pt;
    }
  case REAL_POSITION:
    {
      int x;
      realPosition** pt;
      if ( (pt = (realPosition**)
	    malloc( dim1 * sizeof( realPosition*))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		 type);
	fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	return (void**) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (realPosition*)
	malloc( dim2 * sizeof(realPosition))) == NULL) {
	  fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		   type);
	  fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	  return (void**) NULL;
	}
      return (void**) pt;
    }
  case PIXEL:
    {
      int x;
      pixel** pt;
      if ( (pt = (pixel**)
	    malloc( dim1 * sizeof( pixel*))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		 type);
	fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	return (void**) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (pixel*)
	malloc( dim2 * sizeof( pixel))) == NULL) {
	  fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate2D>.\n",
		   type);
	  fprintf( stderr, "Desired dimensions: %d %d.\n", dim1, dim2);
	  return (void**) NULL;
	}
      return (void**) pt;
    }
  default:
    fprintf( stderr, "Type %d not implemented.\n", type);
    return NULL;
  }
}


void***
allocate3D( int dim1, int dim2, int dim3, int type)
{
  switch (type) {
  case INT:
    {
      int x, y;
      int*** pt;
      if ( (pt = (int***) malloc( dim1 * sizeof( int**))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate3D>.\n",
		 type);
	fprintf( stderr, "Size: %d X %d X %d\n", dim1, dim2, dim3);
	return (void***) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (int**) malloc( dim2 * sizeof( int*))) == NULL) {
	  fprintf( stderr,
		   "Error! Not enough memory for allocating type %d in <allocate3D>.\n",
		   type);
	  fprintf( stderr, "Size: %d X %d X %d\n", dim1, dim2, dim3);
	  return (void***) NULL;
	}
      for ( x = 0; x < dim1; x++)
	for ( y = 0; y < dim2; y++)
	  if ( (pt[x][y] = (int*) malloc( dim3 * sizeof(int))) == NULL) {
	  fprintf( stderr,
		   "Error! Not enough memory for allocating type %d in <allocate3D>.\n",
		   type);
	  fprintf( stderr, "Size: %d X %d X %d\n", dim1, dim2, dim3);
	  return (void***) NULL;
	  }
      return (void***) pt;
    }
  case FLOAT:
    {
      int x, y;
      float*** pt;
      if ( (pt = (float***) malloc( dim1 * sizeof( float**))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate3D>.\n",
		 type);
	fprintf( stderr, "Size: %d X %d X %d\n", dim1, dim2, dim3);
	return (void***) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (float**) malloc( dim2 * sizeof( float*))) == NULL) {
	  fprintf( stderr,
		   "Error! Not enough memory for allocating type %d in <allocate3D>.\n",
		   type);
	  fprintf( stderr, "Size: %d X %d X %d\n", dim1, dim2, dim3);
	  return (void***) NULL;
	}
      for ( x = 0; x < dim1; x++)
	for ( y = 0; y < dim2; y++)
	  if ( (pt[x][y] = (float*) malloc( dim3 * sizeof(float))) == NULL) {
	    fprintf( stderr,
		   "Error! Not enough memory for allocating type %d in <allocate3D>.\n",
		   type);
	  fprintf( stderr, "Size: %d X %d X %d\n", dim1, dim2, dim3);
	  return (void***) NULL;
	  }
      return (void***) pt;
    }
  case DOUBLE:
    {
      int x, y;
      double*** pt;
      if ( (pt = (double***) malloc( dim1 * sizeof( double**))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate3D>.\n",
		 type);
	fprintf( stderr, "Size: %d X %d X %d\n", dim1, dim2, dim3);
	return (void***) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (double**) malloc( dim2 * sizeof( double*))) == NULL) {
	  fprintf( stderr,
		   "Error! Not enough memory for allocating type %d in <allocate3D>.\n",
		   type);
	  fprintf( stderr, "Size: %d X %d X %d\n", dim1, dim2, dim3);
	  return (void***) NULL;
	}
      for ( x = 0; x < dim1; x++)
	for ( y = 0; y < dim2; y++)
	  if ( (pt[x][y] = (double*) malloc( dim3 * sizeof(double))) == NULL) {
	  fprintf( stderr,
		   "Error! Not enough memory for allocating type %d in <allocate3D>.\n",
		   type);
	  fprintf( stderr, "Size: %d X %d X %d\n", dim1, dim2, dim3);
	  return (void***) NULL;
	  }
      return (void***) pt;
    }
  case PROBABILITY:
    {
      int x, y;
      probability*** pt;
      if ( (pt = (probability***)
	    malloc( dim1 * sizeof( probability**))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate3D>.\n",
		 type);
	fprintf( stderr, "Size: %d X %d X %d\n", dim1, dim2, dim3);
	return (void***) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (probability**)
	malloc( dim2 * sizeof( probability*))) == NULL) {
	  fprintf( stderr,
		   "Error! Not enough memory for allocating type %d in <allocate3D>.\n",
		   type);
	  fprintf( stderr, "Size: %d X %d X %d\n", dim1, dim2, dim3);
	  return (void***) NULL;
	}
      for ( x = 0; x < dim1; x++)
	for ( y = 0; y < dim2; y++)
	  if ( (pt[x][y] = (probability*)
	  malloc( dim3 * sizeof(probability))) == NULL) {
	  fprintf( stderr,
		   "Error! Not enough memory for allocating type %d in <allocate3D>.\n",
		   type);
	  fprintf( stderr, "Size: %d X %d X %d\n", dim1, dim2, dim3);
	  return (void***) NULL;
	  }
      return (void***) pt;
    }
  case MAP_PROBABILITY:
    {
      int x, y;
      mapProbability*** pt;
      if ( (pt = (mapProbability***)
	    malloc( dim1 * sizeof( mapProbability**))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate3D>.\n",
		 type);
	fprintf( stderr, "Size: %d X %d X %d\n", dim1, dim2, dim3);
	return (void***) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (mapProbability**)
	malloc( dim2 * sizeof( mapProbability*))) == NULL) {
	  fprintf( stderr,
		   "Error! Not enough memory for allocating type %d in <allocate3D>.\n",
		   type);
	  fprintf( stderr, "Size: %d X %d X %d\n", dim1, dim2, dim3);
	  return (void***) NULL;
	}
      for ( x = 0; x < dim1; x++)
	for ( y = 0; y < dim2; y++)
	  if ( (pt[x][y] = (mapProbability*)
	  malloc( dim3 * sizeof(mapProbability))) == NULL) {
	  fprintf( stderr,
		   "Error! Not enough memory for allocating type %d in <allocate3D>.\n",
		   type);
	  fprintf( stderr, "Size: %d X %d X %d\n", dim1, dim2, dim3);
	  return (void***) NULL;
	  }
      return (void***) pt;
    }
  case EXPECTED_DISTANCE:
    {
      int x, y;
      expectedDistance*** pt;
      if ( (pt = (expectedDistance***)
	    malloc( dim1 * sizeof( expectedDistance**))) == NULL) {
	fprintf( stderr,
		 "Error! Not enough memory for allocating type %d in <allocate3D>.\n",
		 type);
	fprintf( stderr, "Size: %d X %d X %d\n", dim1, dim2, dim3);
	return (void***) NULL;
      }
      for ( x = 0; x < dim1; x++)
	if ( (pt[x] = (expectedDistance**)
	malloc( dim2 * sizeof( expectedDistance*))) == NULL) {
	  fprintf( stderr,
		   "Error! Not enough memory for allocating type %d in <allocate3D>.\n",
		   type);
	  fprintf( stderr, "Size: %d X %d X %d\n", dim1, dim2, dim3);
	  return (void***) NULL;
	}
      for ( x = 0; x < dim1; x++)
	for ( y = 0; y < dim2; y++)
	  if ( (pt[x][y] = (expectedDistance*)
	  malloc( dim3 * sizeof(expectedDistance))) == NULL) {
	  fprintf( stderr,
		   "Error! Not enough memory for allocating type %d in <allocate3D>.\n",
		   type);
	  fprintf( stderr, "Size: %d X %d X %d\n", dim1, dim2, dim3);
	  return (void***) NULL;
	  }
      return (void***) pt;
    }
  default:
    fprintf( stderr, "Type %d not implemented.\n", type);
    return NULL;
  }
}














