
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/tcx/tcaMatrix.h,v $
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
 * $Log: tcaMatrix.h,v $
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







/*****************************************************************************
* PROJECT: Carnegie Mellon Planetary Rover Project
*          Task Control Architecture
*
* PRELIMINARY RELEASE. COMMENTS AND QUESTIONS CAN BE SENT TO fedor@cs.cmu.edu 
* (c) Copyright 1993 Christopher Fedor. All rights reserved.
*
* MODULE: matrix
*
* FILE: tcaMatrix.h
*
* ABSTRACT:
*
* typedefs for matrix package used in GIL
*
* REVISION HISTORY:
*
* 17-Apr-90 Christopher Fedor at School of Computer Science, CMU
* Added ucmat. Seems that the gil vision version of matrix.h is different.
*
*  3-Apr-90 Christopher Fedor at School of Computer Science, CMU
* Created from matrix.h to avoid perception link conflicts.
*
*****************************************************************************/

#ifndef INCtcaMatrix
#define INCtcaMatrix

/* matrix.h -- define types for matrices using Iliffe vectors
 *************************************************************
 * HISTORY
 * 25-Nov-80  David Smith (drs) at Carnegie-Mellon University
 * Changed virtual base address name to "el" for all data
 * types (Previously vali, vald, ...)  This was possible due to the
 * compiler enhancement which keeps different structure declarations
 * separate.
 *
 * 30-Oct-80  David Smith (drs) at Carnegie-Mellon University
 *	Rewritten for record-style matrices
 *
 */

typedef struct {
	int	lb1, ub1, lb2, ub2;
	char	*mat_sto;
	unsigned char	**el;
	}
	ucmat;

typedef struct {
	int	lb1, ub1, lb2, ub2;
	char	*mat_sto;
	char	**el;
	}
	cmat;

typedef struct {
	int	lb1, ub1, lb2, ub2;
	char	*mat_sto;
	short	**el;
	}
	smat;

typedef struct {
	int	lb1, ub1, lb2, ub2;
	char	*mat_sto;
	int	**el;
	}
	imat;

typedef struct {
	int	lb1, ub1, lb2, ub2;
	char	*mat_sto;
	long	**el;
	}
	lmat;

typedef struct {
	int	lb1, ub1, lb2, ub2;
	char	*mat_sto;
	float	**el;
	}
	fmat;

typedef struct {
	int	lb1, ub1, lb2, ub2;
	char	*mat_sto;
	double	**el;
	}
	dmat;

extern ucmat newucmat2();
extern cmat newcmat2();
extern smat newsmat2();
extern imat newimat2();
extern lmat newlmat2();
extern fmat newfmat2();
extern dmat newdmat2();

#define FREEMAT(m) free((m).mat_sto)

#endif /* INCtcaMatrix */
