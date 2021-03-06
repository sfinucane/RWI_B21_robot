
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/tcx/sampleTest.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:48:16 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: sampleTest.h,v $
 * Revision 1.1  2002/09/14 15:48:16  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:02  rhino
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







/**********************************************************************
 *
 * PROJECT: TCX
 *
 * PRELIMINARY RELEASE. COMMENTS AND QUESTIONS CAN BE SENT TO fedor@cs.cmu.edu 
 * (c) Copyright 1993 Christopher Fedor. All rights reserved.
 * 
 * MODULE: sample - a,b example
 *
 * FILE: sampleTest.h
 *
 * ABSTRACT: example include file 
 *
 * EXPORTS:
 *
 * HISTORY:
 *  $Log: sampleTest.h,v $
 *  Revision 1.1  2002/09/14 15:48:16  rstone
 *  *** empty log message ***
 *
 *  Revision 1.1.1.1  1996/09/22 16:46:02  rhino
 *  General reorganization of the directories/repository, fusion with the
 *  RWI software.
 *
 *  Revision 1.4  1994/10/23 08:50:27  tyson
 *  fixed (I hope) i386 structure indexing.
 *
 * Revision 1.3  1994/10/22  18:47:09  tyson
 * VMS version. Fixed structure indexing (computation of the offsets
 * in a struct). Added signal handlers to a1, b1 tcxServer.
 *
 * Revision 1.2  1994/05/31  21:00:56  rhino
 * General reorganization. New header file. New makefiles.
 *
 * Revision 1.1.1.1  1994/03/31  08:35:03  rhino
 *
 * Revision 1.1.1.1  1994/03/17  14:22:27  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:27  rhino
 * test
 *
 * Revision 1.3  1993/03/12  20:59:10  fedor
 * Updated test cases and hid global variables for vxworks
 *
 *
 *********************************************************************/
          
#ifndef INCsampleTest
#define INCsampleTest

#include "tcaMatrix.h"

extern void regInit();

typedef char *StringType;

typedef struct {
  int x;
  StringType s;
} SampleType, *SamplePtr;
                                                                                                                       
typedef struct {
  int dum;
  int *x;
} SelfType, *SelfPtr;

typedef struct {
  int n;
  double *x;
  double *y;
  int silly;
} PolyType, *PolyPtr;
     
typedef int ReplyType, *ReplyPtr;

static StringType SampleDataForm = "{4, string}";

static StringType PolyDataForm = "{4,polygon}";

static StringType ReplyDataForm = "int";

/****************************************/

/* link example */

typedef struct _s2 {
  int a;
  int b;
  StringType s;
  struct _s2 *next;
} s2Type, *s2Ptr;

static StringType s2TypeForm = "{int, int, string, *!}";

#define FIXED_ARRAY_DIM1 2
#define FIXED_ARRAY_DIM2 3

typedef int two_d_array[FIXED_ARRAY_DIM1][FIXED_ARRAY_DIM2];

static StringType two_d_array_form = "[int : 2, 3]";

#define VAR_ARRAY_DIM1 3
#define VAR_ARRAY_DIM2 2

typedef struct { int         dim1;
		 int         dim2;
/* variable length (2D) array of strings */
		 StringType *elements; 
	       } var_two_d_array;

static StringType var_two_d_array_form = "{int, int, <string : 1, 2>}";


/* For Structured Formatters */

typedef struct {
  float x, y, z;
} point;

typedef struct {
  int   leg_no;
  StringType name;
  point pt1, pt2;
} two_points;

typedef struct {
  point       points[6];
  int         length;
  two_points *more_points;
  point      *pt_ptr;
} complex_points;

typedef struct {
  int x;
  char a;
  double y;
} STRUCT_TEST1_TYPE, *STRUCT_TEST1_PTR;

typedef struct {
  char A;
  int x;
  char B;
  short n;
  char C;
  double y;
  char a;
  char b;
} STRUCT_TEST2_TYPE, *STRUCT_TEST2_PTR;
  
typedef struct {
  int x;
  char a;
} STRUCT_TEST3_TYPE, *STRUCT_TEST3_PTR;
  
typedef struct {
  int w;
  STRUCT_TEST3_TYPE t;
  float z;
} STRUCT_TEST4_TYPE, *STRUCT_TEST4_PTR;

static char *point_format = "{float, float, float}";
static char *two_points_format = "{int, string, point, point}";
static char *complex_points_format = "{[point:6], int, <two_points:2>, *point}";

static char *TEST1_FORM = "{int, char, double}";
static char *TEST2_FORM = "{char, int, char, short, char, double, char, char}";
static char *TEST3_FORM = "{int, char}";
static char *TEST4_FORM = "{int, {int, char}, float}";

TCX_REG2_MSG_TYPE mArray[] = {
    {"charMsg", "char", 2}
};

TCX_REG_MSG_TYPE messageArray[] = {
  {"FloatMsg", "float"},
  {"DoubleMsg", "double"},
  {"SelfPtrMsg", "{int, *int}"}, 
  {"LinkTestMsg", "{int, int, string, *!}"},
  {"SelfQueryMsg", "int"},
  {"GoalMsg", "{4, string}"},
  {"GoalMsg2", "{4, string}"},
  {"PolyMsg", "{int, <double:1>, <double:1>, int}"},
  {"charMsg", "char"},
  {"ucmatMsg", "cmat"},
  {"cmatMsg", "cmat"},
  {"smatMsg", "smat"},
  {"imatMsg", "imat"},
  {"lmatMsg", "lmat"},
  {"fmatMsg", "fmat"},
  {"dmatMsg", "dmat"},
  {"FixedArrayMsg", "[int : 2, 3]"},
  {"VarArrayMsg", "{int, int, <string : 1, 2>}"},
  {"StructTest1Msg", "{int, char, double}"},
  {"StructTest2Msg", "{char, int, char, short, char, double, char, char}"},
  {"StructTest3Msg", "{int, char}"},
  {"StructTest4Msg", "{int, {int, char}, float}"},
  {"queryChar", "char"},
  {"replyInt", "int"},
  {"nullTestMsg", NULL},
  {"stringMsg", "string"}
};

/*
  {500,  "StructuredFormatters", "{[point:6], int, <two_points:2>, 
  {float, float, float}", NULL},
  */
 
#endif /* INCsampleTest */
