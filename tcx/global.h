
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/tcx/global.h,v $
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
 * $Log: global.h,v $
 * Revision 1.1  2002/09/14 15:48:15  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1998/10/30 18:49:20  fox
 * Added support for multiple robots.
 *
 * Revision 1.2  1998/05/19 16:36:07  arbuckle
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







/******************************************************************************
*
* PROJECT: TCX
*
* PRELIMINARY RELEASE. COMMENTS AND QUESTIONS CAN BE SENT TO fedor@cs.cmu.edu 
* (c) Copyright 1993 Christopher Fedor. All rights reserved.
* 
* MODULE: globals
*
* FILE: global.h
*
* ABSTRACT:
* 
* A Temporary Global Solution
*
* REVISION HISTORY
*
* $Log: global.h,v $
* Revision 1.1  2002/09/14 15:48:15  rstone
* *** empty log message ***
*
* Revision 1.3  1998/10/30 18:49:20  fox
* Added support for multiple robots.
*
* Revision 1.2  1998/05/19 16:36:07  arbuckle
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
* Revision 1.3  1994/10/22 18:47:07  tyson
* VMS version. Fixed structure indexing (computation of the offsets
* in a struct). Added signal handlers to a1, b1 tcxServer.
*
 * Revision 1.2  1994/05/31  21:00:47  rhino
 * General reorganization. New header file. New makefiles.
 *
 * Revision 1.1.1.1  1994/03/31  08:35:03  rhino
 *
 * Revision 1.1.1.1  1994/03/17  14:22:26  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:27  rhino
 * test
 *
 * Revision 1.1  1993/03/12  20:58:32  fedor
 * Updated test cases and hid global variables for vxworks
 *
*
******************************************************************************/

#ifdef VXWORKS
#include "/usr/vxworks/vx5.0.2b/h/vxWorks.h"
#include "/usr/vxworks/vx5.0.2b/h/stdioLib.h"
#include "/usr/vxworks/vx5.0.2b/h/ctype.h"
#include "/usr/vxworks/vx5.0.2b/h/socket.h"
#include "/usr/vxworks/vx5.0.2b/h/in.h"
#include "/usr/vxworks/vx5.0.2b/h/sigLib.h"
#include "/usr/vxworks/vx5.0.2b/h/errno.h"
#else
#include "stdio.h"
#include "ctype.h"
#include "signal.h"
#include "sys/time.h"
#include "sys/timeb.h"
#include "sys/types.h"
#include "sys/socket.h"
#include "netinet/in.h"
#include "netinet/tcp.h"
#include "netdb.h"
#include "errno.h"

extern int errno;

#endif  /* !VXWORKS */

#ifdef i386
#include <malloc.h>
#endif

#include <stdlib.h>

#include "tcx.h"
#include "tcxP.h"

#include "hash.h"
#include "lex.h"
#include "list.h"
#include "formatters.h"

/***********************************************/

typedef struct {
  /* formatters.c */
  HASH_TABLE_PTR formatNamesTable;

  /* lex.c */
  int currentLocationGlobal;
  char *currentStringGlobal;
  TOKEN_PTR ungetTokenGlobal;

  /* list.c */
  LIST_PTR listFreeListGlobal;
  LIST_ELEM_PTR listCellFreeListGlobal;

  /* tcaMem.c */
  unsigned totalMemRequest;
  int freeMemRetryAmount;
  void (*tcaFreeMemoryHnd)();
  int mallocMemRetryAmount;
  void *(*tcaMallocMemHnd)(size_t size);

  /* parseFmttrs.c */
  char *parseString;
  FORMAT_PTR searchFormat;
  char *foundKey;

  /* com.c */
  /** TCX_FMT_PTR tcxCreateFMT; ** special value */
  void (*tcxCloseHndG)();
  HASH_TABLE_PTR hndIdTable;
  HASH_TABLE_PTR hndConnectTable;
  int sigFlagGlobal;
  int sigerrorGlobal;
  int closeModGlobal;
  TCX_MODULE_PTR tcxServerGlobal;
  TCX_MODULE_PTR tcxModuleGlobal;
  DATA_BUF_TYPE	dataBufGlobal;
  LIST_PTR dataListGlobal;
  fd_set tcxConnectionListGlobal;
  int tcxInitFlagG;

  /* module.c */
  int recvUseHndGlobal;

  /* data.c */
  LIST_PTR moduleListGlobal;
  int tcxRegMsgFlagG;
  int msgRefGlobal;
  int msgInitFlagGlobal;
  LIST_PTR msgQGlobal;
  LIST_PTR msgFreeRefS;
  HASH_TABLE_PTR msgIdTableGlobal;
  HASH_TABLE_PTR msgNameTableGlobal;
  
  /* reg.c */
  void (*tcxExitHndGlobal)();

  /* tcxServer.c */
  int msgIdS;

} G_TYPE, *G_PTR;

extern G_PTR Global;
extern void globalInit();

extern int extendedModuleNameSet;
extern char moduleNameExtension[];
