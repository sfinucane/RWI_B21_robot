
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/evaluate.h,v $
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
 * $Log: evaluate.h,v $
 * Revision 1.1  2002/09/14 20:45:39  rstone
 * *** empty log message ***
 *
 * Revision 1.6  1999/09/09 15:27:59  fox
 * Nothing special.
 *
 * Revision 1.5  1998/03/04 14:46:43  fox
 * This version should run.
 *
 * Revision 1.4  1998/02/13 14:12:19  fox
 * Minor changes.
 *
 * Revision 1.3  1998/01/27 15:25:23  fox
 * Minor changes.
 *
 * Revision 1.2  1998/01/22 13:06:13  fox
 * First version after selection-submission.
 *
 * Revision 1.1  1998/01/06 15:11:20  fox
 * Added evaluation tools.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#ifndef EVALUATE_INCLUDE
#define EVALUATE_INCLUDE

#define MAX_NUMBER_OF_INDICES 60000
#define MAX_NUMBER_OF_VALUES 1200
#define MAX_NUMBER_OF_COLUMNS 20

#define TIME_COLUMN 0
#define X_COLUMN 1
#define Y_COLUMN 2
#define ROT_COLUMN 3
#define COMBINED_STDDEV_COLUMN 0
      
#define TRUE 1
#define FALSE 0

#define MAX_INTERPOLATION_DIST 60.0
#define MAX_INTERVAL_SIZE 30.0

float
tValueForSampleSize( int N);

int
getNextValues( FILE* fp, float* values, int numberOfColumns,
	       int columnForXValue, float* time);

int
getInterpolatedValues( FILE* fp,
		       float* values, int numberOfColumns,
		       int columnForXValue, float resolution,
		       float* prevTime, int* useAble);

int
getReferenceValues( FILE* fp, float* values, float* stdDevs,
		    int numberOfColumns, float time, int readStdDevs);

int
getCorrectedReferenceValues( char* script, int origScript,
			     char* correctionFile,
			     float* values, float time);


#endif
