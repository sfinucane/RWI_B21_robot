
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
 ***** Source file:     $Source: /home/rhino/rhino/CVS/bee/src/msp/utils.c,v $
 *****
 ***** Created by:      $Author: rhino $
 *****
 ***** Revision #:      $Revision: 1.1.1.1 $
 *****
 ***** Date of revision $Date: 1996/09/22 16:46:08 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: utils.c,v $
 * Revision 1.1.1.1  1996/09/22 16:46:08  rhino
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
#include <ctype.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <curses.h>

#include "abterm.h"
#include "msputils.h"

void
abtPrintTitles (void)
{
  int col;
  
  /* print the cool looking separators */
  standout();
  move (0, 0);
  for (col = 0; col != COLS; col++) addch (' ');
  move (LINES/2-3, 0);
  for (col = 0; col != COLS; col++) addch (' ');
  move (LINES/2, 0);
  for (col = 0; col != COLS; col++) addch (' ');
  standend();
  
  /* print titles */
  standout();
  move (0, 2);         addstr (TERM_TITLE);
  move (LINES/2-3, 2);   addstr (DBG_TITLE);
  move (LINES/2, 2);   addstr (DEVICE_TITLE);
  standend();
  
  /* display */
  refresh ();
}

void
abtCls (void)
{
  wprintw (termWin, "\n");	/* hack to clear remaining prompt */
  wprintw (devWin, "\n");
  wprintw (dbgWin, "\n");

  werase (stdscr);
  werase (devWin);
  werase (termWin);
  werase (dbgWin);
  clearok (stdscr, 1);
  
  wrefresh (stdscr);
  wrefresh (devWin);
  wrefresh (dbgWin);
  wrefresh (termWin);
  
  /* redraw seperators */
  abtPrintTitles ();
  
  return;
}

void
abtPrintMem (WINDOW *w, unsigned char *dat, int count)
{
  int ii;
  int i2_last = ii + 16;
  int i2;
    
  for (ii=0; ii<count; ii += 16) {
    /* start line with whitespace */
    wprintw (w, "   [");

    i2_last = ii + 16;		/* print in 16 byte chunks */

    /* print hex values */
    for (i2=ii; i2<count && i2<i2_last; i2++) {
      wprintw (w, "%02X", dat[i2]);
    }
    /* print extra space if necessary */
    for (; i2<i2_last; i2++) {
      wprintw (w, "  ");
    }
    
    /* end hex section and start char section*/
    wprintw (w, "]  [");

    /* print char values */
    for (i2=ii; i2<count && i2<i2_last; i2++) {
      wprintw (w, "%c", (isprint(dat[i2])?dat[i2]:'.'));
    }
    /* print extra space if necessary */
    for (; i2<i2_last; i2++) {
      wprintw (w, " ");
    }

    /* end char section and end line with newline */
    wprintw (w, "]\n");
  }

  wrefresh (w);
}

void
abtPrintPacket (WINDOW *w, ABMSG *msg)
{
  /* standard header */
  wprintw (w, "MAJ 0x%02X ", msg->hdr.major);
  wprintw (w, "MIN 0x%02X ", msg->hdr.minor);
  wprintw (w, "LEN 0x%02X ", msg->hdr.msgLen);
  wprintw (w, "DEV 0x%02X ", msg->hdr.devId);
  wprintw (w, "\n");

  /* data */
  abtPrintMem (w, &msg->data[4], msg->hdr.msgLen);
}
