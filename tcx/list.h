
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/tcx/list.h,v $
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
 * $Log: list.h,v $
 * Revision 1.1  2002/09/14 15:48:15  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1998/05/19 16:36:08  arbuckle
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







/**************************************************************************
* 
* PROJECT: Carnegie Mellon Planetary Rover Project
*
* PRELIMINARY RELEASE. COMMENTS AND QUESTIONS CAN BE SENT TO fedor@cs.cmu.edu 
* (c) Copyright 1993 Christopher Fedor. All rights reserved.
*
* MODULE: list
*
* FILE: list.h
*
* ABSTRACT: The list module provides basic list creation and manipulation
* routines and serves as the base abstract data type for the tca.
* The include file list.h provides the top level routines for other modules.
*
* EXPORTS:
*
* REVISION HISTORY:
*
* See list.c for history.
*
**************************************************************************/

#ifndef INClist
#define INClist

typedef struct _LIST_ELEM {
  char *item;
  struct _LIST_ELEM *next, *previous;
} LIST_ELEM_TYPE, *LIST_ELEM_PTR;

typedef struct _LIST {
  int length;
  struct _LIST *freeList;
  LIST_ELEM_PTR first, last, next;
} LIST_TYPE, *LIST_PTR;

extern LIST_PTR listCreate();
extern void listFree();
extern void listInsertItem();
extern void listDeleteItem();
extern void listFreeAllItems();
extern void listTestDeleteItem();
extern void listInsertItemAfter();
extern void listInsertUnique();

extern int listEqual();
extern int listLength();
extern int listIterate();
extern int listMemberItem();

extern LIST_PTR listCopy();

extern LIST_PTR listMake1();
extern LIST_PTR listMake2();

extern char *listFirst();
extern char *listLast();
extern char *listNext();

#endif /* INClist */
