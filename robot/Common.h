
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/robot/Common.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:55:27 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: Common.h,v $
 * Revision 1.1  2002/09/14 15:55:27  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:12  rhino
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



#ifndef COMMON_LOADED
#define COMMON_LOADED

#ifdef __GNUC__
#define INLINE __inline__
#else
#define INLINE inline
#endif

#define EXTERN extern
#define PRIVATE static
#define PUBLIC

#ifdef  NULL
#undef  NULL
#endif
#define NULL  (void*) 0

#define BOOLEAN int
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#define CBOOLEAN char
#define CBOOL	 char
#define CTRUE (char)1
#define CFALSE (char)0
  
#define UCHAR unsigned char
#define USINT unsigned short int
#define SINT short int
  
#define NILPTR (char *)0

typedef void  *Pointer;

typedef float DEGREES;
typedef float RADIANS;
typedef float METERS;
typedef float CMS;
typedef float FEET;

typedef void (*VOID_FN1)(double);
typedef void (*VOID_FN2)(double,double);
typedef void (*VOID_FN3)(double,double,double);
typedef void (*VOID_FN4)(double,double,double,double);

#define DEGREES_FORMAT "float" /* for TCA usage */
#define RADIANS_FORMAT "float" /* for TCA usage */
#define FEET_FORMAT    "float" /* for TCA usage */
#define METERS_FORMAT  "float" /* for TCA usage */
#define CMS_FORMAT     "float" /* for TCA usage */

#define Abs(x)	  ((x) >= 0 ? (x) : -(x))
#define Max(x,y)  ((x) > (y) ? (x) : (y))
#define Min(x,y)  ((x) > (y) ? (y) : (x))
#define Sqr(x)    ((x) * (x))

#define ABS(x)	  ((x) >= 0 ? (x) : -(x))
#define MAX(x,y)  ((x) > (y) ? (x) : (y))
#define MIN(x,y)  ((x) > (y) ? (y) : (x))
#define SQR(x)    ((x) * (x))

#define NEAR(x1,x2,eps) (ABS((x1)-(x2))<=(eps))


#define IRINT(x)  ((int) rint(x))

#ifndef PI
#define PI 3.1415926535897932384626433
#endif
#define SQRT2 1.4142135
#define TWO_PI (2 * PI)
#define Rad_to_Deg(r)   ((r) * 180.0 / PI)
#define Deg_to_Rad(d)   ((d) * PI / 180.0)

#define RAD_TO_DEG(r)   ((r) * 57.29578)
#define DEG_TO_RAD(d)   ((d) * 0.017453293)

#define FT2M (12.0/39.37)
#define FEET_TO_METERS(ft) ((ft)*FT2M)
#define METERS_TO_FEET(ms) ((ms)/FT2M)

#define INCHES_TO_CMS(in) (100.0*FEET_TO_METERS((in)/12.0))
#define CMS_TO_INCHES(cm) (12.0*METERS_TO_FEET((cm)/100.0))

#define DEFAULT_LINE_LENGTH 80

#endif






