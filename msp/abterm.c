
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
 ***** Source file:     $Source: /home/rhino/rhino/CVS/bee/src/msp/abterm.c,v $
 *****
 ***** Created by:      $Author: tyson $
 *****
 ***** Revision #:      $Revision: 1.2 $
 *****
 ***** Date of revision $Date: 1997/03/11 17:16:38 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: abterm.c,v $
 * Revision 1.2  1997/03/11 17:16:38  tyson
 * added IR simulation and other work
 *
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


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include <sys/types.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <termios.h>
#include <curses.h>

#include "abterm.h"
#include "msputils.h"
#include "version.h"
#include "mspterm.h"

char *usage =
"Yeah!  Right!  ...options coming soon to an mspterm near you.\n";
/* "usage: %s [-s <serial_dev>] [-a <abus_dev>]\n" */

WINDOW *termWin;
WINDOW *devWin;
WINDOW *dbgWin;

int destId;
int fd;

void initWins()
{
  termWin = devWin = dbgWin = NULL;

  stdscr = initscr ();
  termWin = subwin (stdscr, LINES/2-4, COLS, 1, 0);
  dbgWin  = subwin (stdscr, 3, COLS, LINES/2-2, 0);
  devWin  = subwin (stdscr, LINES/2-1, COLS, LINES/2+1, 0);
  if (devWin == NULL) {
    endwin ();
    fprintf (stderr, "unable to initialize devWin\n");
    exit (1);
  } else if (termWin == NULL) {
    endwin ();
    fprintf (stderr, "unable to initialize termWin\n");
    exit (1);
  } else if (dbgWin == NULL) {
    endwin ();
    fprintf (stderr, "unable to initialize dbgWin\n");
    exit (1);
  }

  /* make all windows scrolling */
  scrollok (termWin, TRUE);
  scrollok (devWin, TRUE);
  scrollok (dbgWin, TRUE);

  /* screen setup */
  abtPrintTitles ();
}

void quit (int rc) {

   mspExit(fd);

   if (termWin != NULL) {
      /* little hack to make the prompt go away */
      wprintw (termWin, "\n");

      refresh ();
      move (LINES-1, 0);
      addch ('\n');
      refresh ();
      endwin ();
   }

   exit (rc);
}

int parse_line (char *line) {
  int argc;
  char *argv[100];
  
  if ((line!=(char *)EOF) && (line!=NULL)) {
    for (argc=0; argc<10; argc++) argv[argc]=NULL;
    argc=0;
    while (isspace(*line)) line++;
    if (*line != 0) {
      if (isdigit (*line)) {
	destId = strtoul(line, &line, 16);

	/* XXX check on destId, devId, busId */

      }
      while (isspace(*line)) line++;
      argv[0]=line;
      argc=0;
      if (*line) argc=1;

      do {
	while ((isspace(*line)==0) && *line) line++;
	if (*line==0) break;
	*(line++)=0;
	while (isspace(*line)) line++;
	if (*line) {
	  argv[argc++]=line;
	}
      } while (*line);
      
      if (!(strcmp(argv[0], "quit"))) quit(0);
      
      if (argc) mspU2D(argc, argv, destId);
      wrefresh (termWin);
    }
    return (1);
  }
  quit(0);
  return(0);
}

void
abtSelect (int fd) {
  mspSelect(fd);
  return;
#if 0
  ABMSG msg;
  int count;

  count = read (fd, (char *)&msg, sizeof (msg));
  if (count != sizeof(msg)) {
    wprintw(dbgWin, "select: read() != sizeof(msg)\n\n");
    wrefresh(dbgWin);
    return;
  }

  if (mspM2D(fd, &msg)) defM2D(fd, &msg);
#endif /* 0 */
}

int
main (int argc, char *argv[])
{
  int argn;
  int termFd;
  char cmdline[256];

  /* this is kinda pointless, cause we're probably
     not going to be getting input from anything
     but stdin */
  termFd = 0;
  
  initWins();  /* set up curses interface */
  
  fd = mspInit("/dev/abus");
  if (fd < 0) exit (0);
  
  wprintw (termWin, "mspterm - %s\n", version_date);
  
  /* we also need to cause devId 0x51 to be abmgr, 0x50 to be ab, */
  /* and 0x52 to be serial. Real devs will be numbered after that */
  /* probably this should happen in abmgrInit().                  */
  
  /* handle commandline args */
  
  for (argn=1; argn!=argc; argn++) {
    if (*argv[argn] == '-') {
      if (strcmp (argv[argn], "-h") == 0) {
	fprintf (stderr, usage, argv[0]);
	quit (0);
      }
    }
  }
  
  signal (SIGTERM, quit);
  signal (SIGINT, quit);
  signal (SIGHUP, quit);
  
  
  do {
    fd_set realset, rset;

    wprintw (termWin, PROMPT, destId);
    wrefresh (termWin);
    
    /* we need to reset this each time through */
    FD_ZERO (&realset);
    FD_SET (fd, &realset);
    FD_SET (termFd, &realset);
    
    do {
      /* has to be reset, cause select unsets it */
      rset = realset;
      
      /* block until read ready on one of msps or termWin */
      select (10, &rset, NULL, NULL, NULL);
      
      /* deal with everyone that needs to be read */
      if (FD_ISSET (fd, &rset) && fd != termFd) {
	abtSelect(fd);
	wrefresh(termWin);
      }
      /* feed the user from time to time */
    } while (!FD_ISSET (termFd, &rset));
    wgetstr (termWin, cmdline);
  } while (parse_line (cmdline));
  
  /* exit gracefully */
  quit (0);
  return (0);
}
