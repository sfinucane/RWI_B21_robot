
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/tcx/tcaMem.c,v $
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
 * $Log: tcaMem.c,v $
 * Revision 1.1  2002/09/14 15:48:16  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1998/05/20 10:05:42  arbuckle
 * Changed tcaMalloc to use size_t and return void* like the real malloc.
 *
 * Revision 1.2  1998/05/19 16:36:09  arbuckle
 * #endif stateements should not have trailing non-commented text.
 * The function prototype for malloc is given in <stdlib.h>. Fixed
 * several files to agree with the standard definition.
 * Fixed some silly casts that were casting function pointers to void*.
 * Compilers other than gcc do not support the __FUNCTION__ construct.
 * Used conditional compilation on __GNUC__ to provide alternative versions.
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


/******************************************************************************
*
* PROJECT: Carnegie Mellon Planetary Rover Project
*          Task Control Architecture 
* 
* (c) Copyright 1991 Christopher Fedor and Reid Simmons.  All rights reserved.
* 
* MODULE: memory
*
* FILE: tcaMem.c
*
* ABSTRACT:
* 
* Interface to memory management routines.
*
* REVISION HISTORY
*
*  4-Jul-91 Christopher Fedor, School of Computer Science, CMU
* Added tcaRegisterMallocHnd to complete the routines so that modules
* can handle all memory allocation.
*
*  4-Jun-91 Christopher Fedor, School of Computer Science, CMU
* Added tcaRegisterFreeMemHnd so that modules can be called to free
* memory to satisfy a malloc request.
*
* 25-Oct-90 Christopher Fedor, School of Computer Science, CMU
* Created.
*
******************************************************************************/

#ifdef VMS
#include "vms.h"
#endif

#ifdef VXWORKS
#include "/usr/vxworks/vx5.0.2b/h/vxWorks.h"
#include "/usr/vxworks/vx5.0.2b/h/stdioLib.h"
#else
#include "stdio.h" 
#endif

#include "global.h"

#if 0
extern void dataMsgDisplayStats();
#endif


/******************************************************************************
*
* FUNCTION: void tcaRegisterFreeMemHnd(func, retry)
*
* DESCRIPTION: 
* Sets a function for freeing memory when malloc returns NULL.
* The function is passed an unsigned int amount of the memory requested.
* The number of times this routine is called is set by retry.
* The default is 1.
*
* INPUTS: 
* void (*func)();
* int retry;
*
* OUTPUTS: void.
*
******************************************************************************/

void tcaRegisterFreeMemHnd(func, retry)
void (*func)();
int retry;
{
  Global->freeMemRetryAmount = retry;

  if (Global->freeMemRetryAmount < 1)
    Global->freeMemRetryAmount = 1;

  Global->tcaFreeMemoryHnd = func;
}


/******************************************************************************
*
* FUNCTION: void tcaRegisterMallocHnd(func, retry)
*
* DESCRIPTION: 
* Registers a function to call in place of malloc.
* The routine will be passed an unsigned int of the amount of storage needed.
* The routine will be called a max of retry times if NULL is returned.
* The default retry amount is 1.
*
* INPUTS: 
* char *(*func)();
* int retry;
*
* OUTPUTS:
*
******************************************************************************/

void tcaRegisterMallocHnd(func, retry)
void *(*func)(size_t size);
int retry;
{
  Global->mallocMemRetryAmount = retry;

  if (Global->mallocMemRetryAmount < 1)
    Global->mallocMemRetryAmount = 1;

  if (func)
    Global->tcaMallocMemHnd = func;
}


/******************************************************************************
*
* FUNCTION: void tcaFree(item)
*
* DESCRIPTION: 
* An interface to free - should use tcaFreeData or tcaFreeReply in most cases.
*
* INPUTS: char *item;
*
* OUTPUTS: void.
*
******************************************************************************/

void tcaFree(item)
char *item;
{
  /* printf("tcaFree:\n"); */
  free(item);
}


/******************************************************************************
*
* FUNCTION: void *tcaMalloc(amount)
*
* DESCRIPTION: Interface to malloc requests from tca.
*
* INPUTS: size_t amount;
*
* OUTPUTS: void * - generic pointer to the memory
*
* NOTES: 
* Stops everything if we have run out of memory. 
* Note there may not be enough memory to print that we have run out.
*
******************************************************************************/

void *tcaMalloc(size_t amount)
{
  int i, j;
  char *mem;
/*printf("=[%d]=",amount);fflush(stdout);(S.Thrun 93-5-1) */
  mem = NULL;

  if (Global->tcaMallocMemHnd) 
    for(i=0;!mem && i < Global->mallocMemRetryAmount;i++) {
      mem = (*(Global->tcaMallocMemHnd))(amount);
    }

  if (mem) {
    Global->totalMemRequest += amount;
    return(mem);
  }

  if (Global->tcaFreeMemoryHnd) 
    for(j=0;!mem && j < Global->freeMemRetryAmount;j++) {
      (*(Global->tcaFreeMemoryHnd))(amount);

      if (Global->tcaMallocMemHnd) 
	for(i=0;!mem && i < Global->mallocMemRetryAmount;i++) {
	  mem = (*(Global->tcaMallocMemHnd))(amount);
	}
    }

  if (mem) {
    Global->totalMemRequest += amount;
    return(mem);
  }
  
  fprintf(stderr, 
	  "tcaMalloc: NULL returned from malloc for request: %d\n", amount);
#ifdef LISP
  fflush(stderr);
#endif
  tcaModError(NULL);
  return NULL;
}


/******************************************************************************
*
* FUNCTION: void tcaStats()
*
* DESCRIPTION: Quick hack to display some memory stats.
*
* INPUTS: none.
*
* OUTPUTS: void.
*
******************************************************************************/

void tcaStats()
{
  fprintf(stderr, "Total Memory Requests Filled: %d\n", 
	  Global->totalMemRequest);
  fprintf(stderr, "\n");
#if 0
  dataMsgDisplayStats();
#endif
  fprintf(stderr, "\n");
#ifdef LISP
  fflush(stderr);
#endif
}



void tcxLock()
{
;
}


void tcxUnlock()
{
;
}
