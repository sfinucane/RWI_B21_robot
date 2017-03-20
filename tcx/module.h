
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/tcx/module.h,v $
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
 * $Log: module.h,v $
 * Revision 1.1  2002/09/14 15:48:16  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1998/05/19 16:36:09  arbuckle
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







/**********************************************************************
 *
 *  PROJECT:  TCX
 *
 * PRELIMINARY RELEASE. COMMENTS AND QUESTIONS CAN BE SENT TO fedor@cs.cmu.edu 
 * (c) Copyright 1993 Christopher Fedor. All rights reserved.
 * 
 *  MODULE: Module
 *
 *  FILE: module.h
 *
 *  ABSTRACT: Module Include File.
 *
 *  EXPORTS:
 *
 *  HISTORY:
 *  $Log: module.h,v $
 *  Revision 1.1  2002/09/14 15:48:16  rstone
 *  *** empty log message ***
 *
 *  Revision 1.2  1998/05/19 16:36:09  arbuckle
 *  #endif stateements should not have trailing non-commented text.
 *  The function prototype for malloc is given in <stdlib.h>. Fixed
 *  several files to agree with the standard definition.
 *  Fixed some silly casts that were casting function pointers to void*.
 *  Compilers other than gcc do not support the __FUNCTION__ construct.
 *  Used conditional compilation on __GNUC__ to provide alternative versions.
 *
 *  Revision 1.1.1.1  1996/09/22 16:46:02  rhino
 *  General reorganization of the directories/repository, fusion with the
 *  RWI software.
 *
 *  Revision 1.2  1994/05/31 21:00:53  rhino
 *  General reorganization. New header file. New makefiles.
 *
 * Revision 1.1.1.1  1994/03/31  08:35:03  rhino
 *
 * Revision 1.1.1.1  1994/03/17  14:22:26  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:27  rhino
 * test
 *
 * Revision 1.7  1993/03/12  20:58:54  fedor
 * Updated test cases and hid global variables for vxworks
 *
 * Revision 1.6  1992/11/19  03:02:06  fedor
 * Remove module from a pending sets if it crashes before a confirming connection.
 *
 * Revision 1.5  1992/11/16  15:51:06  fedor
 * Rewrote initializing connections and auto reconnect.
 *
 * Revision 1.4  1992/11/13  14:47:17  fedor
 * Update archive with a working version and partially working server route
 *
 * Revision 1.3  1992/10/23  20:15:28  fedor
 * Redid method of connecting to modules. Added the notion of a connection.
 * Added some auto-reconnect for those modules that are probably listening.
 * See detail notes.
 *
 * Revision 1.2  1992/07/03  10:13:22  fedor
 * New module definition and routines
 *
 * Revision 1.1.1.1  1992/06/16  17:22:06  fedor
 * Import TCX 2.0 16-Jun-92
 *
 * Revision 1.2  1992/04/20  10:32:36  fedor
 * *** empty log message ***
 *
 * Revision 1.1  1992/04/20  09:19:35  fedor
 * Initial revision
 *
 * Revision 1.1  1992/04/20  09:19:35  fedor
 * Initial revision
 *
 *  
 *  $Revision: 1.1 $
 *  $Date: 2002/09/14 15:48:16 $
 *  $Author: rstone $
 *
 *********************************************************************/

#ifndef INCmodule
#define INCmodule

#define MODULE_IS_ACTIVE(module) (module->status & STATUS_ACTIVE)

extern MODULE_PTR moduleCreate(int sd, int port, char *name, char *host,
			       int status);

extern MODULE_PTR moduleCreate2(int sd, MODULE_INFO_PTR moduleInfo,
			       int status);

extern void moduleOpen(int sd);
extern void moduleClose(MODULE_PTR module);
extern void reconnectModule2(MODULE_PTR module);

#endif /* INCmodule */
