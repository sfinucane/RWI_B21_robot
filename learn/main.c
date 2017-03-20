
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
 ***** Please read and make sure you understand the disclaimer below.
 *****
 ***** Contact thrun@cs.cmu.edu if you have questions or would
 ***** like to obtain further information.
 *****
 ***** We hope you enjoy working with this software package.
 *****
 *****                                     (c) Sebastian Thrun, 1997
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/learn/main.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 16:36:05 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: main.c,v $
 * Revision 1.1  2002/09/14 16:36:05  rstone
 * *** empty log message ***
 *
 * Revision 1.33  1999/07/11 18:48:33  thrun
 * slight reorganization
 *
 * Revision 1.32  1998/08/22 03:08:37  thrun
 * .
 *
 * Revision 1.31  1998/07/04 15:21:38  thrun
 * variable logging.
 *
 * Revision 1.30  1998/07/03 23:59:59  thrun
 * .
 *
 * Revision 1.29  1998/06/20 21:05:32  thrun
 * Reorganization (created: fa.{c,h}, xtypes.h)
 *
 * Revision 1.28  1998/05/05 04:00:34  thrun
 * Direct control mode (and button). Other stuff I don't fully remember.
 * Intermediate version.
 *
 * Revision 1.27  1998/04/28 23:30:46  thrun
 * autosave of weights, we can now switch off learning in selected
 * networks and variables, which is important for incremental
 * programming/learning
 *
 * Revision 1.26  1998/04/26 15:03:08  thrun
 * subroutines appear to work fine. also changed the initialiation.
 *
 * Revision 1.25  1998/04/20 03:12:11  thrun
 * Intermediate version with subroutines.
 *
 * Revision 1.24  1998/04/18 20:42:25  thrun
 * 1. neuro net can now be called with the same variable as input
 * and output
 * 2. direct control modus for the robot. controls are
 * now generated exclusively through the display (left upper corner)
 *
 * Revision 1.23  1998/04/14 02:35:56  thrun
 * .
 *
 * Revision 1.22  1997/10/23 02:28:23  thrun
 * .
 *
 * Revision 1.21  1997/10/05 18:11:18  thrun
 * new data library "libdat.a"
 *
 * Revision 1.20  1997/08/05 04:15:22  thrun
 * (1) avg_value are saved for each stoch variable, (2) enhanced replay
 * functions.
 *
 * Revision 1.19  1997/07/29 22:44:48  thrun
 * .
 *
 * Revision 1.18  1997/07/16 17:09:43  thrun
 * Neat, working version. Simpliefied the application, too.
 *
 * Revision 1.17  1997/07/14 22:17:13  thrun
 * Fixed the bug (I believe). Now comes testing.
 *
 * Revision 1.16  1997/07/06 22:51:49  thrun
 * Autosaving of the parameters (every 2 minutes)
 *
 * Revision 1.15  1997/07/06 20:48:47  thrun
 * .
 *
 * Revision 1.14  1997/07/06 18:42:04  thrun
 * Radial basis networks implemented.
 *
 * Revision 1.13  1997/07/04 19:51:12  thrun
 * intermediate eversion
 *
 * Revision 1.12  1997/07/04 18:09:33  thrun
 * intermediate version - do not use
 *
 * Revision 1.11  1997/07/04 00:28:55  thrun
 * Intermediate verson of the learning stuff. What works is:
 * - training of deterministic variables (incl. chaining)
 * - training of multiple determinstic with a stochastic variables (incl chaining)
 * What doesn't yet work includes
 * - temporal chaining of stochastic variables, using Bayes rule
 * - convolving: gradients
 * - stochatic input
 *
 * Revision 1.10  1997/06/30 05:53:58  thrun
 * intermediate version - new neural network (which seems to work!).
 *
 * Revision 1.9  1997/06/29 04:04:55  thrun
 * intrmediate version - not really good for anything
 *
 * Revision 1.8  1997/06/28 13:41:42  thrun
 * This is the fully functional recorder/display/analysis tool.
 * Check out this version if you'd like to have it (without any
 * of the learning stuff). What's missing is the ability to feed back
 * the data intp the baseServer, colli and camera. Also missing is a link
 * to the pantilt unit.
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
#include <sys/time.h>
#include <math.h>
#include <signal.h>


#include "EZX11.h"
#include "o-graphics.h"


#include "rai.h"
#include "devUtils.h"
#include "raiClient.h"
#include "baseClient.h"
#include "sonarClient.h"
#include "tactileClient.h"
#include "rai.h"
#include "raiClient.h"
#include "cameraClient.h"
#include "buttonClient.h"
#include "pantiltClient.h"

#include "dat.h"
#include "global.h"
#include "mem.h"
#include "xtypes.h"
#include "vars.h"
#include "fa.h"



/************************************************************************
 *
 *   NAME:         main()
 *                 
 *   FUNCTION:     main program - initialization and devMainLoop
 *                 
 *   PARAMETERS:   command lines only
 *                 
 *                 
 ************************************************************************/


int
main(int argc, char *argv[])
{
  init_all(argc, argv);


  for (;;){
    fprintf(stderr, "### %d\n",
	    global_modus_recording);
    main_loop();
    fprintf(stderr, "### %d\n",
	    global_modus_recording);
    if (!global_modus_movie){
      struct timeval block_waiting_time;
      block_waiting_time.tv_sec  = 1;
      block_waiting_time.tv_usec = 0;
      (void) block_wait(&block_waiting_time, global_use_tcx, global_use_X);
    }
  }  
}



