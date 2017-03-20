
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/plan/beeSoftVersion.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:47:04 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: beeSoftVersion.h,v $
 * Revision 1.1  2002/09/14 15:47:04  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1997/07/28 15:38:41  tyson
 * fixes to rwi_checkout and updated beeSoftVersion.h files
 *
 * Revision 1.4  1997/07/27 10:00:10  tyson
 * updated rwi_checkout (resolved conflicts with Sebastians update) and other minor tweaks for beta release
 *
 * Revision 1.3  1997/05/06 20:14:22  tyson
 * minor stuff
 *
 * Revision 1.2  1997/04/17 16:10:34  tyson
 * Added support for Ramona plus fixed a couple of minor bugs
 *
 * Revision 1.1.1.1  1997/04/09 16:11:35  thrun
 * Script for checking out a final BeeSoft version.
 *
 * Revision 1.2  1997/02/22 05:16:31  thrun
 * Version numbers are now also checked for libraries.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


/*
 * IMPORTANT NOTICE: Do not edit this file!!!
 *
 *                   This file contains the actual version number
 *                   of the software contained in the current
 *                   directory. 
 */


/* --------------------------------------------------
 *
 * MAJOR VERSION NUMBER
 *
 */

#define	  BEESOFT_VERSION_MAJOR 1
/* --------------------------------------------------
 *
 * MINOR VERSION NUMBER
 *
 */

#define   BEESOFT_VERSION_MINOR 1

/* --------------------------------------------------
 *
 * DATE
 *
 */

#define   BEESOFT_VERSION_DATE "July 28, 1997"

/* --------------------------------------------------
 *
 * ROBOT TYPE
 *
 */

#ifdef B21
#define	  BEESOFT_VERSION_ROBOT_TYPE 21
#else
#define	  BEESOFT_VERSION_ROBOT_TYPE 14
#endif
