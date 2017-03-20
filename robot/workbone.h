
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/robot/workbone.h,v $
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
 * $Log: workbone.h,v $
 * Revision 1.1  2002/09/14 15:55:27  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:15  rhino
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


/*   WorkBone CD Rom Player Software

     Copyright (c) 1994  Thomas McWilliams 
   
     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2, or (at your option)
     any later version.
     
     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.
     
     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

*/

#define CDNULL 0
#define CDPLAY 1
#define CDPAUZ 3
#define CDSTOP 4
#define CDEJECT 5

#ifdef WBLAZY
#define WBS_DELAY 2
#define WBU_DELAY 0
#endif

#ifndef WBS_DELAY
#define WBS_DELAY 1
#endif

#ifndef WBU_DELAY
#define WBU_DELAY 0
#endif

#define UPCUR "\033[1A"
#define REVON "\033[7m"
#define REVOF "\033[27m"

/* kernel 1.0 uses different escape sequence for pc character set */
#define OGON "\033(U"
#define OGOF "\033(B"
#define GON "\033[11m"
#define GOF "\033[10m"

#define MTAB "        "
#define MTAB3 MTAB MTAB MTAB 
#define ROW0 MTAB3 "\263                      \263\n"
#define ROW1 MTAB3 "\263    \376\376    ||    |\020    \263\n"
#define ROW2 MTAB3 "\263    |\021    \036\036    \020|    \263\n"
#define ROW3 MTAB3 "\263    \021\021    ..    \020\020    \263\n"
#define ROW4 MTAB3 "\263   " REVON "   quit   " REVOF "   " REVON " ? " REVOF "   \263\n"
#define HLIN1 "\304\304\304\304\304\304\304\304\304\304\304"
#define HLIN2 "\304\304\304\304"
#define HDR "\264 number pad \303"
#define ROWT MTAB3 "\332" HLIN2 HDR HLIN2 "\277\n" 
#define ROWB MTAB3 "\300" HLIN1 HLIN1 "\331\n" "%s\n" UPCUR "\r"

#define AROW0 MTAB3 "|                      |\n"
#define AROW1 MTAB3 "|    []    ||    =>    |\n"
#define AROW2 MTAB3 "|    <     ^^     >    |\n"
#define AROW3 MTAB3 "|    <<    ..    >>    |\n"
#define AROW4 MTAB3 "|    quit         ?    |\n"
#define AHLIN1 "-----------"
#define AHLIN2 "----"
#define AHDR "| number pad |"
#define AROWT MTAB3 "+" AHLIN2 AHDR AHLIN2 "+\n" 
#define AROWB MTAB3 "+" AHLIN1 AHLIN1 "+\n" "\n" UPCUR "\r"

#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif

void play_cd( int start, int pos, int end) ;
void strmcpy (char **t, const char *s);
int eject_cd( void );
int cd_status( void );
void pause_cd( void );
void stop_cd( void );
void show_terms( const char **p );

