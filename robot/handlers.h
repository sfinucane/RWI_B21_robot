
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/robot/handlers.h,v $
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
 * $Log: handlers.h,v $
 * Revision 1.1  2002/09/14 15:55:27  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:14  rhino
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
 * PROJECT: Rhino
 *
 * FILE: handlers.h
 *
 * ABSTRACT:
 *
 *****************************************************************************/

#ifndef HANDLERS_H
#define HANDLERS_H


/***********************************************************************
 *
 *  A handler is a function that expect two parameters: callback_data
 *  and client_data. The callback_data is the new data from the device,
 *  the client_data is a data given by the client when it installed the
 *  the handler (may be NULL).
 *  The type of the callback_data can be a generic pointer (which can be
 *  used to store a char, int, long or whatever pointer, but NOT a double
 *  neither a float) or a double.
 *  In case the device have to pass more than one parameter, we'll have
 *  to define an structure in which store those paramaters, and the 
 *  callback_data will be a pointer to that structure.
 *  The type of the client_data is always a generic pointer.
 *
 *  Examples of correct definitions of handlers:
 *
 *  void NewLaserStream(Pointer callback_data, Pointer client_data)
 *  void ChangeInVelocity(double new_velocity, Pointer client_data)
 *
 *
 *  
 *  See the test at the end of handlers.c
 ***********************************************************************/


#include <stdio.h>
#include "Common.h"

#define MAX_HANDLERS_PER_EVENT 10

typedef void (*Handler)(Pointer, Pointer);

typedef struct _HandlerData {
  Handler handler[MAX_HANDLERS_PER_EVENT];
  Pointer client_data[MAX_HANDLERS_PER_EVENT];
} _HandlerData;

typedef _HandlerData *HandlerList;



HandlerList CreateHandlerList(int number_handlers);
void        InstallHandler(HandlerList handler_list, Handler handler, 
			   int position, void *client_data);
BOOLEAN     IsHandlerInstalled(HandlerList handler_list, Handler handler, int position);
void        RemoveHandler(HandlerList handler_list, Handler handler, int position);
void        RemoveAllHandlers(HandlerList handler_list, int position);
void        FireHandler(HandlerList handler_list, int position, void *callback_data);

#endif
