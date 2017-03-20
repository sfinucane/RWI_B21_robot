
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/tcx/tcaMatrix.c,v $
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
 * $Log: tcaMatrix.c,v $
 * Revision 1.1  2002/09/14 15:48:16  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1998/05/20 10:05:42  arbuckle
 * Changed tcaMalloc to use size_t and return void* like the real malloc.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:01  rhino
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
* MODULE: matrix
*
* FILE: tcaMatrix.c
*
* ABSTRACT:
*
* matrix creation routines for matrix package used in GIL
* Modified from existing matrix package.
*
* REVISION HISTORY:
*
* 17-Apr-90 Christopher Fedor at School of Computer Science, CMU
* Added newucmat2.
*
*  3-Apr-90 Christopher Fedor at School of Computer Science, CMU
* Created from matrix.c to avoid perception link conflicts.
*
*****************************************************************************/

/* matrix.c -- library routines for constructing dynamic matrices
 * with arbitrary bounds using Iliffe vectors
 ****************************************************************
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
 * 28-Oct-80  David Smith (drs) at Carnegie-Mellon University
 *	Written.
 *
 */

#ifdef VMS
#include "vms.h"                      
#endif                

#include <stdlib.h>
#include "tcaMatrix.h"
extern void *tcaMalloc(size_t size);

ucmat newucmat2(rs, re, cs, ce, error)
	int rs, re, cs, ce, *error;
	{
	register unsigned char *p, **b;
	register int r, rows, cols;
	ucmat matrix;

	*error=0;
	rows = re-rs+1;
	cols = ce-cs+1;

	if (rows<=0 || cols<=0) {*error=1; return matrix;}

	matrix.lb1 = rs;
	matrix.ub1 = re;
	matrix.lb2 = cs;
	matrix.ub2 = ce;
	b = (unsigned char **)
	    tcaMalloc(rows*sizeof(unsigned char *) +
	    rows*cols*sizeof(unsigned char));
	if (b==0) {*error=1; return(matrix);}
	matrix.mat_sto = (char *) b;
	p = ((unsigned char *) &b[rows]) - cs;
	matrix.el = b -= rs;
	for (r=rs; r<=re; r++)
	    {
	    b[r] = p;
	    p += cols;
	    }
	return matrix;
      }

cmat newcmat2(rs, re, cs, ce, error)
	int rs, re, cs, ce, *error;
	{
	register char *p, **b;
	register int r, rows, cols;
	cmat matrix;

	rows = re-rs+1;
	cols = ce-cs+1;
	if (rows<=0 || cols<=0) {*error=1; return matrix;}
	matrix.lb1 = rs;
	matrix.ub1 = re;
	matrix.lb2 = cs;
	matrix.ub2 = ce;
	b = (char **)
	    tcaMalloc(rows*sizeof(char *) + rows*cols*sizeof(char));
	if (b==0) {*error=1; return(matrix);}
	matrix.mat_sto = (char *) b;
	*error=0;
	p = ((char *) &b[rows]) - cs;
	matrix.el = b -= rs;
	for (r=rs; r<=re; r++)
	    {
	    b[r] = p;
	    p += cols;
	    }
	return matrix;
	}

smat newsmat2(rs, re, cs, ce, error)
	int rs, re, cs, ce, *error;
	{
	register short *p, **b;
	register int r, rows, cols;
	smat matrix;

	rows = re-rs+1;
	cols = ce-cs+1;
	if (rows<=0 || cols<=0) {*error=1; return matrix;}
	matrix.lb1 = rs;
	matrix.ub1 = re;
	matrix.lb2 = cs;
	matrix.ub2 = ce;
	b = (short **)
	    tcaMalloc(rows*sizeof(short *) + rows*cols*sizeof(short));
	if (b==0) {*error=1; return(matrix);}
	matrix.mat_sto = (char *) b;
	*error=0;
	p = ((short *) &b[rows]) - cs;
	matrix.el = b -= rs;
	for (r=rs; r<=re; r++)
	    {
	    b[r] = p;
	    p += cols;
	    }
	return matrix;
	}

imat newimat2(rs, re, cs, ce, error)
	int rs, re, cs, ce, *error;
	{
	register int *p, **b;
	register int r, rows, cols;
	imat matrix;

	rows = re-rs+1;
	cols = ce-cs+1;
	if (rows<=0 || cols<=0) {*error=1; return matrix;}
	matrix.lb1 = rs;
	matrix.ub1 = re;
	matrix.lb2 = cs;
	matrix.ub2 = ce;
	b = (int **)
	    tcaMalloc(rows*sizeof(int *) + rows*cols*sizeof(int));
	if (b==0) {*error=1; return(matrix);}
	matrix.mat_sto = (char *) b;
	*error=0;
	p = ((int *) &b[rows]) - cs;
	matrix.el = b -= rs;
	for (r=rs; r<=re; r++)
	    {
	    b[r] = p;
	    p += cols;
	    }
	return matrix;
	}

lmat newlmat2(rs, re, cs, ce, error)
	int rs, re, cs, ce, *error;
	{
	register long *p, **b;
	register int r, rows, cols;
	lmat matrix;

	rows = re-rs+1;
	cols = ce-cs+1;
	if (rows<=0 || cols<=0) {*error=1; return matrix;}
	matrix.lb1 = rs;
	matrix.ub1 = re;
	matrix.lb2 = cs;
	matrix.ub2 = ce;
	b = (long **)
	    tcaMalloc(rows*sizeof(long *) + rows*cols*sizeof(long));
	if (b==0) {*error=1; return(matrix);}
	matrix.mat_sto = (char *) b;
	*error=0;
	p = ((long *) &b[rows]) - cs;
	matrix.el = b -= rs;
	for (r=rs; r<=re; r++)
	    {
	    b[r] = p;
	    p += cols;
	    }
	return matrix;
	}

fmat newfmat2(rs, re, cs, ce, error)
	int rs, re, cs, ce, *error;
	{
	register float *p, **b;
	register int r, rows, cols;
	fmat matrix;

	rows = re-rs+1;
	cols = ce-cs+1;
	if (rows<=0 || cols<=0) {*error=1; return matrix;}
	matrix.lb1 = rs;
	matrix.ub1 = re;
	matrix.lb2 = cs;
	matrix.ub2 = ce;
	b = (float **)
	    tcaMalloc(rows*sizeof(float *) + rows*cols*sizeof(float));
	if (b==0) {*error=1; return(matrix);}
	matrix.mat_sto = (char *) b;
	*error=0;
	p = ((float *) &b[rows]) - cs;
	matrix.el = b -= rs;
	for (r=rs; r<=re; r++)
	    {
	    b[r] = p;
	    p += cols;
	    }
	return matrix;
	}

dmat newdmat2(rs, re, cs, ce, error)
	int rs, re, cs, ce, *error;
	{
	register double *p, **b;
	register int r, rows, cols;
	dmat matrix;

	rows = re-rs+1;
	cols = ce-cs+1;
	if (rows<=0 || cols<=0) {*error=1; return matrix;}
	matrix.lb1 = rs;
	matrix.ub1 = re;
	matrix.lb2 = cs;
	matrix.ub2 = ce;
	b = (double **)
	    tcaMalloc(rows*sizeof(double *) + rows*cols*sizeof(double));
	if (b==0) {*error=1; return(matrix);}
	matrix.mat_sto = (char *) b;
	*error=0;
	p = ((double *) &b[rows]) - cs;
	matrix.el = b -= rs;
	for (r=rs; r<=re; r++)
	    {
	    b[r] = p;
	    p += cols;
	    }
	return matrix;
	}

