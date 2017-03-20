
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/tcx/formatters.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:48:15 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: formatters.h,v $
 * Revision 1.1  2002/09/14 15:48:15  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1998/05/19 16:36:07  arbuckle
 * #endif stateements should not have trailing non-commented text.
 * The function prototype for malloc is given in <stdlib.h>. Fixed
 * several files to agree with the standard definition.
 * Fixed some silly casts that were casting function pointers to void*.
 * Compilers other than gcc do not support the __FUNCTION__ construct.
 * Used conditional compilation on __GNUC__ to provide alternative versions.
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







/*****************************************************************************
* PROJECT: Carnegie Mellon Planetary Rover Project
*          Task Control Architecture
*
* (c) Copyright 1991 Christopher Fedor and Reid Simmons.  All rights reserved.
* 
* MODULE: formatters
*
* FILE: formatters.h
*
* ABSTRACT:
*
* Data Format Routines. Include File.
*
*****************************************************************************/

#ifndef INCformatters
#define INCformatters

typedef enum {Encode, Decode, ELength,
		ALength, RLength, SimpleType, DPrint, DFree} TRANS_OP_TYPE;

/* note: fedor: 31-Aug-89 should this go away??? */
typedef char *DATA_PTR;

typedef enum {primitiveFMT, lengthFMT, structFMT, pointerFMT, fixedArrayFMT,
	      varArrayFMT} FORMAT_CLASS_TYPE;

typedef union { 
  int i;
  struct _FORMAT_TYPE *f;
} FORMAT_ARRAY_TYPE, *FORMAT_ARRAY_PTR;

typedef union {
int i;
struct _FORMAT_TYPE *f;
FORMAT_ARRAY_PTR     a;
} FMT_ELEMENT_TYPE;

typedef struct _FORMAT_TYPE {
  FORMAT_CLASS_TYPE type;
  FMT_ELEMENT_TYPE formatter;
} FORMAT_TYPE, *FORMAT_PTR;

typedef struct {
  int bstart;
  char *buffer;
} BUFFER_TYPE, *BUFFER_PTR;

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#define MAXFORMATTERS 30

#define INT_FMT     1
#define BOOLEAN_FMT 2
#define FLOAT_FMT   3
#define DOUBLE_FMT  4
#define BYTE_FMT    5
#define TWOBYTE_FMT 6
#define STR_FMT     7
#define FORMAT_FMT  8
#define UBYTE_FMT   9
#define CMAT_FMT    10
#define SMAT_FMT    11
#define IMAT_FMT    12
#define LMAT_FMT    13
#define FMAT_FMT    14
#define DMAT_FMT    15
#define CHAR_FMT    16
#define SHORT_FMT   17
#define LONG_FMT    18
#define UCMAT_FMT   19
#define TCA_REF_PTR_FMT 20

#define SIUCMAT_FMT 21
#define SICMAT_FMT  22
#define SISMAT_FMT  23
#define SIIMAT_FMT  24
#define SILMAT_FMT  25
#define SIFMAT_FMT  26
#define SIDMAT_FMT  27

#define REF(type, datastruct, dstart) *(type *)(datastruct+dstart)

#define TO_BUFFER_AND_ADVANCE(data, buffer, bstart, length) \
  {bcopy(data, (buffer)+bstart, length); bstart += length;}

#define FROM_BUFFER_AND_ADVANCE(data, buffer, bstart, length) \
  {bcopy((buffer)+bstart, data, length); bstart += length;}

#define NEW_FORMATTER() (FORMAT_PTR)tcaMalloc(sizeof(FORMAT_TYPE))

#define NEW_FORMAT_ARRAY(size) \
  (FORMAT_ARRAY_PTR)tcaMalloc(size * sizeof(FORMAT_ARRAY_TYPE))

#endif /* INCformatters */
