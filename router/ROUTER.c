
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/router/ROUTER.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 16:01:03 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: ROUTER.c,v $
 * Revision 1.1  2002/09/14 16:01:03  rstone
 * *** empty log message ***
 *
 * Revision 1.4  1998/05/13 07:57:25  fox
 * Adapted init_tcx to new version.
 *
 * Revision 1.3  1997/05/13 15:31:16  fox
 * Added broadcast of laser messages.
 *
 * Revision 1.2  1997/05/06 20:14:21  tyson
 * minor stuff
 *
 * Revision 1.1.1.1  1996/09/22 16:46:34  rhino
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






#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <math.h>

#include "tcx.h"
#include "tcxP.h"
#include "Common.h"
#include "libc.h"
#include "LIB-math.h"
#include "ROBOT.h"
#include <bUtils.h>

#if defined(__STDC__) && defined(__GCC_NEW_VARARGS__)
typedef void (*VOID_FN)(void);
#elif  defined(__STDC__)
typedef void (*VOID_FN)(void);
#else /* __STDC__ */
typedef void (*VOID_FN)();
#endif

typedef struct command {
  char *name;
  VOID_FN func;
  int  n_parameters;
} command;


BOOLEAN use_collision;
BOOLEAN use_simulator = FALSE;


extern int sonar_on;
extern int status_on;
extern int colli_on;
extern int laser_on;
extern int verbose_on;

struct bParamList * bParamList = NULL;

/***************************************************************************
 * main                                                                    *
 *                                                                         *
 * Parameters : argc, *argv[]                                              *
 * Returns    : Nothing                                                    *
 * Does       : Checking options, starts up initialization , starts router *
 * Called     : Well it's - main.                                          *
 ***************************************************************************/
void main(int argc, char *argv[])
{
  char    machine[20];
  char    log_file[20];
  int     i;
  char *robotName = NULL;


  /* add some parameter files */
  bParamList = bParametersAddFile(bParamList, "etc/beeSoft.ini");
  
  /* add some enviroment variables */
  bParamList = bParametersAddEnv(bParamList, "", "TCXHOST");
  
  /* add command line arguements */
  bParamList = bParametersAddArray(bParamList, "", argc, argv);
  
  /* Fill the global parameter structure */
  bParametersFillParams(bParamList);
  
  for (i=1; i<argc; i++) {
    if ((strcmp(argv[i],"-nostatus")==0))
      status_on = 0;
    else if ((strcmp(argv[i],"-nosonar")==0))
      sonar_on = 0;
    else if ((strcmp(argv[i],"-usecolli")==0))
      colli_on = 1;
    else if ((strcmp(argv[i],"-uselaser")==0))
      laser_on = 1;
    else if ((strcmp(argv[i],"-verbose")==0))
      verbose_on = 1;
    else {
      fprintf (stderr, "\nusage: ROUTER [-nostatus (no status autoreply)]");
      fprintf (stderr, "\n              [-nosonar  (no sonar autoreply)]");
      fprintf (stderr, "\n              [-usecolli (starts colli autoreply)]");
      fprintf (stderr, "\n              [-uselaser (starts laser autoreply)]");
      fprintf (stderr, "\n              [-verbose  (displays some information)]\n");
 
#if 0
      exit(-1);
#endif
    }
  }

  init_tcx( robotName);

  if (bRobot.fork) {
    bDaemonize("router.log");
  }
  
  MainControlLoop();
  /*  signal(SIGINT,  (void *) Quit); */
  /*  signal(SIGBUS,  (void *) Quit);*/
  /*  signal(SIGSEGV, (void *) Quit);*/


  exit(0);
}
