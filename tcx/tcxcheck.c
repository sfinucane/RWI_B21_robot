
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/tcx/tcxcheck.c,v $
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
 * $Log: tcxcheck.c,v $
 * Revision 1.1  2002/09/14 15:48:16  rstone
 * *** empty log message ***
 *
 * Revision 1.4  1998/10/30 18:49:20  fox
 * Added support for multiple robots.
 *
 * Revision 1.3  1998/04/07 18:32:12  wolfram
 * Removed Multi calls from tcx. Version runs. Major rework has to be
 * done in order to support multiple robots.
 *
 * Revision 1.2  1998/04/04 03:05:39  wolfram
 * Intermediate version supporting multiple robots
 *
 * Revision 1.1  1997/02/21 22:41:23  thrun
 * Changed tcxInitialize so that it checks if a module with the same name
 * is already up and running. If this is the case, it prints out an error
 * message and exists.
 * This way we should now have an effective mechanisms that prevents from
 * running multiple copies of the same module - without changing a line
 * of code outside tcx.
 * I also fixed a bug introduced by bUtils, that made tcxServer crash on
 * my SUN.
 *
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/



#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <signal.h>


#include "tcx.h"
#include "tcxP.h"
#include "global.h"

/*
 * Routine connects to TCX, and checks if a module with the name
 * moduleName is already running. If so it prints an error message
 * and returns 0 - otherwise 1.
 *
 * The purpose of this command is to detect + prevent having multiple
 * modules of the same type running at the same time. Uff.
 */


void
tcxInitialize(char *moduleName, char *tcxHost)
{
  char tcxName[128], hostname[128], pidname[128];
  static char* extendedName = NULL;
  if ( ! extendedName)
    extendedName = (char*) malloc( 255 * sizeof(char));

  strcpy( extendedName, moduleName);
  if ( extendedModuleNameSet) {
    strcat( extendedName, moduleNameExtension);
#ifdef MULTI_DEBUG
    fprintf( stderr, "-3-: %s --> %s\n", moduleName, extendedName);
#endif
  }

  /*
   * create a unique name (using the PID and the hostname)
   */

  /* Fox: New support for multi robots. */  
  strcpy(tcxName, extendedName);
  sprintf(pidname, "_%d_", getpid());
  strcat(tcxName, pidname);
  gethostname(hostname, 32);
  strcat(tcxName, hostname); 
  
#ifdef MULTI_DEBUG
  fprintf(stderr, "MODULE %s\n", tcxName);
#endif
   /*
   * initialize TCX
   */

  tcxInitializeInternal(tcxName, tcxHost);
  
  /*
   * and check if the module is already running
   */

  if ( tcxConnectOptional(moduleName) == NULL){
    tcxCloseAll();
    Global->msgInitFlagGlobal = 0;
    tcxInitializeInternal(extendedName, tcxHost);
  }
  else{
    fprintf(stderr, "ERROR: You cannot run multiple copies of the same module.\n");
    exit(-1);
  }
}
