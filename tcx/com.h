
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/tcx/com.h,v $
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
 * $Log: com.h,v $
 * Revision 1.1  2002/09/14 15:48:15  rstone
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
 * MODULE: Communications 
 *
 * FILE: com.h
 *
 * ABSTRACT: Communication Include File 
 *
 * EXPORTS:
 *
 * HISTORY:
 *  $Log: com.h,v $
 *  Revision 1.1  2002/09/14 15:48:15  rstone
 *  *** empty log message ***
 *
 *  Revision 1.1.1.1  1996/09/22 16:46:02  rhino
 *  General reorganization of the directories/repository, fusion with the
 *  RWI software.
 *
 *  Revision 1.3  1994/09/30 13:08:22  thrun
 *  Changed header declaration.
 *
 * Revision 1.2  1994/05/31  21:00:45  rhino
 * General reorganization. New header file. New makefiles.
 *
 * Revision 1.1.1.1  1994/03/31  08:35:03  rhino
 *
 * Revision 1.1.1.1  1994/03/17  14:22:25  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:27  rhino
 * test
 *
 * Revision 1.3  1993/03/12  20:58:12  fedor
 * Updated test cases and hid global variables for vxworks
 *
 * Revision 1.2  1992/07/10  16:04:46  fedor
 * Changes to compile for VxWorks. Also some changes to avoid compiler warnings.
 *
 * Revision 1.1.1.1  1992/06/16  17:22:03  fedor
 * Import TCX 2.0 16-Jun-92
 *
 * Revision 1.3  1992/04/20  10:37:56  fedor
 * *** empty log message ***
 *
 * Revision 1.2  1992/04/20  10:30:26  fedor
 * test file
 *
 * Revision 1.2  1992/04/20  10:30:26  fedor
 * test file
 *
 * Revision 1.1  1992/04/20  07:05:53  fedor
 * Initial revision
 *
 *  
 *  $Revision: 1.1 $
 *  $Date: 2002/09/14 15:48:15 $
 *  $Author: rstone $
 *
 *********************************************************************/

extern int readAll(int sd, char *buf, int nbytes);
extern int writeAll(int sd, char *buf, int nbytes);
extern int connectAtPort(char *machine, int port, int *sd);
extern int listenAtPort(int *port, int *sd);

extern int connectToServer(char *serverHost);
extern int singlePassActiveModules(void *timeout);
