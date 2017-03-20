
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/tcx/version.c,v $
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
 * $Log: version.c,v $
 * Revision 1.1  2002/09/14 15:48:16  rstone
 * *** empty log message ***
 *
 * Revision 1.4  1997/08/02 18:37:03  swa
 * The string `*** BeeSoft Version Control Report ***' will now only be printed
 * if there is a mismatch.
 *
 * Revision 1.3  1997/02/26 10:09:08  fox
 * No version control vor UNIBONN.
 *
 * Revision 1.2  1997/02/22 05:16:59  thrun
 * Version numbers are now also checked for libraries.
 *
 * Revision 1.1  1997/02/22 00:59:16  thrun
 * Introduced version number support
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


/*****************************************************************************
 * INCLUDES
 *****************************************************************************/

#include "stdio.h"
#include "ctype.h"
#include "signal.h"
#include "sys/time.h"
#include "sys/types.h"
#include "sys/socket.h"
#include "errno.h"

#include "tcx.h"
#include "tcxP.h"
#include "global.h"

#include "beeSoftVersion.h"


extern int beeSoftMaj;
extern int beeSoftMin;
extern int beeSoftRobotType;

static int version_major_error = 0;
static int version_warning = 0;
static int first_check = 1;

static int  local_major = -1;
static int  local_minor = -1;
static int  local_robot_type = -1;
static char local_date[80];

void
check_version_number(int major, int minor, int robot_type, char *date, 
		     char *reference_text, 
		     int final_check)
{
  static int external = 1;
  static int local_printed = 0;
  int header_printed = 0;

  /*
   * First check: verify the external version number
   */

#ifdef UNIBONN
  fprintf( stderr, "No version control wanted!\n");
  return;
#endif
  
  if (first_check){

    /*
     * First check: let's copy the parameters: local version
     */

    local_major = major;
    local_minor = minor;
    local_robot_type = robot_type;
    strcpy(local_date, date);

    /*
     * First check: now let's check the tcxServer version number
     */

    if (beeSoftMaj == -1 || beeSoftMin == -1 || beeSoftRobotType == -1){
      fprintf(stderr, "\n*** BeeSoft Version Control Report ***\n");
      header_printed=1;
      fprintf(stderr, "WARNING: TCX not yet initialized. Unable to verify external version number.\n");
      external = 0;
      version_warning = 1;
    }
    
    if (external && beeSoftMaj != local_major){
      if ( !header_printed )
	fprintf(stderr, "\n*** BeeSoft Version Control Report ***\n");
      header_printed=1;
      fprintf(stderr, "ERROR: Major version number mismatch (tcxServer).\n");
      version_major_error = 1;
    }
    
    if (external && beeSoftRobotType != local_robot_type){
      if ( !header_printed )
	fprintf(stderr, "\n*** BeeSoft Version Control Report ***\n");
      header_printed=1;
      fprintf(stderr, "ERROR: Inconsistent robot type (tcxServer).\n");
      version_major_error = 1;
    }
    
    if (external && beeSoftMin != local_minor){
      if ( !header_printed )
	fprintf(stderr, "\n*** BeeSoft Version Control Report ***\n");
      header_printed=1;
      fprintf(stderr, "WARNING: Minor version number mismatch (tcxServer).\n");
      version_warning = 1;
    }

    /*
     * First check: now let's check the libtcx.a version number
     */
    
    if (BEESOFT_VERSION_MAJOR != local_major){
      if ( !header_printed )
	fprintf(stderr, "\n*** BeeSoft Version Control Report ***\n");
      header_printed=1;
      fprintf(stderr, "ERROR: Major version number mismatch (tcxServer).\n");
      version_major_error = 1;
    }
    
    if (BEESOFT_VERSION_ROBOT_TYPE != local_robot_type){
      if ( !header_printed )
	fprintf(stderr, "\n*** BeeSoft Version Control Report ***\n");
      header_printed=1;
      fprintf(stderr, "ERROR: Inconsistent robot type (tcxServer).\n");
      version_major_error = 1;
    }
    
    if (BEESOFT_VERSION_MINOR != local_minor){
      if ( !header_printed )
	fprintf(stderr, "\n*** BeeSoft Version Control Report ***\n");
      header_printed=1;
      fprintf(stderr, "WARNING: Minor version number mismatch (tcxServer).\n");
      version_warning = 1;
    }
    
    

  }
  /*
   * not the first check
   */

  else{
    /*
     * All other checks: now let's check the corresponding version number
     */
    
    if (major != local_major){
      if ( !header_printed )
	fprintf(stderr, "\n*** BeeSoft Version Control Report ***\n");
      header_printed=1;
      fprintf(stderr, "ERROR: Major version number mismatch (%s).\n",
	      reference_text);
      version_major_error = 1;
    }
    
    if (robot_type != local_robot_type){
      if ( !header_printed )
	fprintf(stderr, "\n*** BeeSoft Version Control Report ***\n");
      header_printed=1;
      fprintf(stderr, "ERROR: Inconsistent robot type (%s).\n",
	      reference_text);
      version_major_error = 1;
    }
    
    if (minor != local_minor){
      if ( !header_printed )
	fprintf(stderr, "\n*** BeeSoft Version Control Report ***\n");
      header_printed=1;
      fprintf(stderr, "WARNING: Minor version number mismatch (%s).\n",
	      reference_text);
      version_warning = 1;
    }
    
  }

  /*
   * print the version numbers, if necessary
   */

  if (version_major_error || version_warning){
    if (!local_printed){
      fprintf(stderr, "\tVersion number of this program: %d.%d.B%d %s\n",
	      local_major, local_minor, local_robot_type, local_date);
      fprintf(stderr, "\tVersion number of tcxlib.a:     %d.%d.B%d %s\n",
	      BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
	      BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE);
      if (external)
	fprintf(stderr, "\tVersion number of tcxServer:    %d.%d.B%d\n",
		beeSoftMaj, beeSoftMin, beeSoftRobotType);
      else
	fprintf(stderr, "\tVersion number of tcxServer:    (could not be verified)\n\n");
      
      local_printed = 1;
    }
    if (!first_check)
      fprintf(stderr, "\tVersion number of %s:     %d.%d.B%d %s\n",
	      reference_text, major, minor, robot_type, date);
  }


  /*
   * if final check: print last report / exit
   */
  
  if (final_check){
    if (!version_major_error && !version_warning)
      fprintf(stderr, "Version number %d.%d/B%d (%s) successfully verified.\n",
	      local_major, local_minor, local_robot_type, local_date);


    fprintf(stderr, "\n");
    if (version_major_error)
      exit (-1);
  }


  first_check = 0;

  return;
}
