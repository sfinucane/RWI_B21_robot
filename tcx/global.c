
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/tcx/global.c,v $
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
 * $Log: global.c,v $
 * Revision 1.1  2002/09/14 15:48:15  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1998/05/19 16:36:07  arbuckle
 * #endif stateements should not have trailing non-commented text.
 * The function prototype for malloc is given in <stdlib.h>. Fixed
 * several files to agree with the standard definition.
 * Fixed some silly casts that were casting function pointers to void*.
 * Compilers other than gcc do not support the __FUNCTION__ construct.
 * Used conditional compilation on __GNUC__ to provide alternative versions.
 *
 * Revision 1.2  1997/02/22 15:43:01  thrun
 * Fixed some problems that caused compiler warnings.
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
* PROJECT: TCX
*
* PRELIMINARY RELEASE. COMMENTS AND QUESTIONS CAN BE SENT TO fedor@cs.cmu.edu 
* (c) Copyright 1993 Christopher Fedor. All rights reserved.
* 
* MODULE: globals
*
* FILE: global.c
*
* ABSTRACT:
* 
* A Temporary Global Solution
*
* REVISION HISTORY
*
* $Log: global.c,v $
* Revision 1.1  2002/09/14 15:48:15  rstone
* *** empty log message ***
*
* Revision 1.3  1998/05/19 16:36:07  arbuckle
* #endif stateements should not have trailing non-commented text.
* The function prototype for malloc is given in <stdlib.h>. Fixed
* several files to agree with the standard definition.
* Fixed some silly casts that were casting function pointers to void*.
* Compilers other than gcc do not support the __FUNCTION__ construct.
* Used conditional compilation on __GNUC__ to provide alternative versions.
*
* Revision 1.2  1997/02/22 15:43:01  thrun
* Fixed some problems that caused compiler warnings.
*
* Revision 1.1.1.1  1996/09/22 16:46:01  rhino
* General reorganization of the directories/repository, fusion with the
* RWI software.
*
* Revision 1.2  1994/10/22 18:47:30  tyson
* VMS version. Fixed structure indexing (computation of the offsets
* in a struct). Added signal handlers to a1, b1 tcxServer.
*
 * Revision 1.1.1.1  1994/03/31  08:35:04  rhino
 *
 * Revision 1.1.1.1  1994/03/17  14:22:29  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:28  rhino
 * test
 *
 * Revision 1.1  1993/03/12  20:58:29  fedor
 * Updated test cases and hid global variables for vxworks
 *
*
******************************************************************************/

#ifdef VMS
#include "vms.h"
#endif

#include "global.h"
#include <stdlib.h>

G_PTR Global = NULL;

void globalInit()
{
  /* support some random calls for handler connects/disconnects 
   before tcxInitialize */
  /* 12-Mar-93: fedor: this may be a very bad idea - blah1 */

  if (Global) {
    return;
  }

  Global = (G_TYPE *)malloc(sizeof(G_TYPE));

#ifdef VXWORKS
  if (taskVarAdd(0, (int *)&Global) != OK) {
    printErr("taskVarAdd failed\n");
  }
#endif  

  /* formatters.c */
  Global->formatNamesTable = NULL;

  /* lex.c */
  Global->currentLocationGlobal = 0;
  Global->currentStringGlobal = NULL;
  Global->ungetTokenGlobal = NULL;

  /* list.c */
  Global->listFreeListGlobal = NULL;
  Global->listCellFreeListGlobal = NULL;

  /* tcaMem.c */
  Global->totalMemRequest = 0;
  Global->freeMemRetryAmount = 0;
  Global->tcaFreeMemoryHnd = NULL;
  Global->mallocMemRetryAmount = 1;
  Global->tcaMallocMemHnd = malloc;

  /* parseFmttrs.c */
  Global->parseString = NULL;
  Global->searchFormat = NULL;
  Global->foundKey = NULL;

  /* com.c */
  /* Global->tcxCreateFMT = NULL; ** special */
  Global->closeModGlobal = 0;
  Global->tcxCloseHndG = NULL;
  Global->hndIdTable = NULL;
  Global->hndConnectTable = NULL;
  Global->tcxInitFlagG = 0;
  Global->sigFlagGlobal = 0;
  Global->sigerrorGlobal = 0;
  Global->tcxServerGlobal = NULL;
  Global->tcxModuleGlobal = NULL;
  Global->dataBufGlobal.len = 0;
  Global->dataBufGlobal.buf = NULL;
  Global->dataListGlobal = NULL;
  FD_ZERO(&(Global->tcxConnectionListGlobal));

  /* data.c */
  Global->recvUseHndGlobal = 1;

  /* module.c */
  Global->moduleListGlobal = NULL; 

  /* msg.c */
  Global->tcxRegMsgFlagG = 0;
  Global->msgRefGlobal = 1;
  Global->msgInitFlagGlobal = 0;
  Global->msgQGlobal = NULL;
  Global->msgFreeRefS = NULL;
  Global->msgIdTableGlobal = NULL;
  Global->msgNameTableGlobal = NULL;

  /* reg.c */
  Global->tcxExitHndGlobal = NULL;

  /* tcxServer.c */
  Global->msgIdS = 1000; /* 0 not used start user messages at 1000 */
}

