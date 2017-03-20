
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/utils/orig/ports/ports.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:51:40 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: ports.c,v $
 * Revision 1.1  2002/09/14 15:51:40  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:21  rhino
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


/*
 * FILE:  ports.c
 * DESC:  Allow a process access to certain IO ports.
 *
 * $Id: ports.c,v 1.1 2002/09/14 15:51:40 rstone Exp $
 *
 * Modification history: $Log: ports.c,v $
 * Modification history: Revision 1.1  2002/09/14 15:51:40  rstone
 * Modification history: *** empty log message ***
 * Modification history:
 * Modification history: Revision 1.1.1.1  1996/09/22 16:46:21  rhino
 * Modification history: General reorganization of the directories/repository, fusion with the
 * Modification history: RWI software.
 * Modification history:
 * Modification history: Revision 1.1.1.1  1996/03/19 16:33:47  tyson
 * Modification history: Inital 1.2 repository
 * Modification history:
 * Revision 1.2  1995/08/06  00:07:05  jal
 * Added #include of <ctype.h> for isxdigit().
 *
 * Install in $(INSTALL_DIR)/bin.
 *
 * Revision 1.1  1995/08/05  23:57:15  jal
 * Allow a process access to the IO ports.
 *
 */
 
#ifndef lint
static char rcsid[] = "$Id: ports.c,v 1.1 2002/09/14 15:51:40 rstone Exp $";
static void *const use_rcsid = (&use_rcsid, &rcsid, 0);
#endif

#include <unistd.h>
#include <stdio.h>
#include <ctype.h>

void
get_port (int port, int nbytes)
{
   int error;

   error = ioperm(port, nbytes, 1);
   if (error)
      {
         fprintf(stderr, "ioperm(port=0x%04X,nbytes=%d): ", port, nbytes);
         perror(NULL);
         exit(1);
      }
}
  
int
main (int argc, char *argv[])
{
   uid_t uid, euid;
   int arg;

   if (argc < 2)
      {
         fprintf (stderr, "usage: ports [port[,nbytes] ...] "
                  "<prog> [arguments ...]\n"
                  "   allows access to the io ports\n");
         exit (1);
      }

   /*
    * Find out who we are
    */
   uid = getuid();
   euid = geteuid();
   
   if (euid != 0)
      /* if we're not setuid root */
      {
         fprintf (stderr, "pll needs to be setuid root!\n");
         exit (1);
      }

   /*
    * Grab our ports
    */
   for (arg = 1; arg < argc; arg++)
      {
         int port = 0, nbytes = 1;
         
         if (!isxdigit(argv[arg][0])) break; /* must be the prog name */
         
         sscanf(argv[arg], "%x,%d", &port, &nbytes);
         
         if (port == 0)
            {
               fprintf(stderr, "bad: %s\n", argv[arg]);
               exit(1);
            }
         
            get_port(port, nbytes);
      }
   
   /*
    * Drop us back down; avoid giving user process
    * root privileges
    */
   setreuid(uid, uid);

   /*
    * A little final sanity checking
    */
   uid = getuid();
   euid = geteuid();

   if (uid == 0 || euid == 0)
      /* if we're still root */
      {
         fprintf (stderr,
                  "pll is not allowed to make a root sub-process!\n");
         exit (1);
      }
   
   /*
    * Run what they want us to run
    */
   execvp (argv[arg], &argv[arg]);

   /* We won't be here, except in an error condition. */
   perror(argv[arg]);
   return (1);
}
