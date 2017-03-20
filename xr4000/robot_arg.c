/* xrdev/examples/robot_arg.c
 *
 * standard command line parsing code for robot example programs
 *
 *                             ** COPYRIGHT NOTICE **
 *
 * Copyright (c) 1999, Nomadic Technologies, Inc.
 *
 * The contents of this file is copyrighted by Nomadic Technologies, Inc, and
 * is protected by United States copyright laws.  Unauthorized copying is
 * prohibited.  All rights reserved.
 *
 */


#include <stdlib.h>
#include <string.h>

#include "robot_arg.h"




/* ------------------------------------------------------------------------
 * Function:     ConsumeArg
 * Purpose:      delete an entry from the argument vector
 * Arguments:    argc : to be decremented by one
 *               argv [] : the argument vector
 *               idx : the entry to delete
 * ------------------------------------------------------------------------ */
static void ConsumeArg(int *argc, char *argv[], int idx)
{
  int i;

  --*argc;
  for (i = idx; i < *argc; ++i)
  {
    argv[i] = argv[i+1];
  }

  return;
}




/* ------------------------------------------------------------------------
 * Function:     ARG_GetStandardArgs
 * Purpose:      parse standard robot command lines arguments from the
 *                 command line, and return a reduced arg vector which
 *                 contains the leftovers to the caller.
 * Arguments:    *argc : the number of arguments in *argv
 *               ***argv : a pointer to the argument vector
 *               sched_hostname_buf : storage for the scheduler hostname
 *               sched_port_buf : storage for the scheduler port
 *               robot_id_buf : storage for the robot id
 * Return:       TRUE if we should display the usage message
 * Note:         initialize the buffers to default values before calling
 *                 this routine, as there is no way of telling if a
 *                 given buffer was filled or not.
 * ------------------------------------------------------------------------ */
BOOL ARG_GetStandardArgs(int *argc, char ***argv, char **sched_hostname_buf,
			 unsigned short *sched_port_buf, long *robot_id_buf)
{
  int i;
  char *cp;
  unsigned short port;

  i = 0;
  while (i < *argc)
  {
    if (strcasecmp((*argv)[i], "-scheduler") == 0 ||
	strcasecmp((*argv)[i], "--scheduler") == 0 ||
	strcasecmp((*argv)[i], "-h") == 0 ||
	strcasecmp((*argv)[i], "-host") == 0 ||
	strcasecmp((*argv)[i], "--host") == 0)
    {
      ConsumeArg(argc, *argv, i);
    
      if (i == *argc)
      {
	return TRUE;
      }

      *sched_hostname_buf = (*argv)[i];
      ConsumeArg(argc, *argv, i);

      /* the scheduler description takes the form hostname[:port]. */
      for (cp = *sched_hostname_buf; *cp != '\0'; ++cp)
      {
	if (*cp == ':')
	{
	  *cp = '\0';
	  ++cp;
	  port = atoi(cp);
	  if (port >= 1024)
	  {
	    *sched_port_buf = port;
	  }
	  break;
	}
      }

      continue;
    }

    if (strcasecmp((*argv)[i], "-robot_id") == 0 ||
	strcasecmp((*argv)[i], "--robot_id") == 0)
    {
      ConsumeArg(argc, *argv, i);
    
      if (i == *argc)
      {
	return TRUE;
      }

      *robot_id_buf = atol((*argv)[i]);
      ConsumeArg(argc, *argv, i);
      continue;
    }

    if (strcasecmp((*argv)[i], "--help") == 0 ||
	strcasecmp((*argv)[i], "-help") == 0 ||
	strcasecmp((*argv)[i], "-?") == 0)
    {
      return TRUE;
    }

    ++i;
  }

  return FALSE;
}




/* ------------------------------------------------------------------------
 * Function:     ARG_GetProgramName
 * Purpose:      remove leading directory prefixes from a program name
 * Arguments:    *argv0 : argv[0] from the command line
 * Return:       the program name
 * ------------------------------------------------------------------------ */
const char *ARG_GetProgramName(const char *argv0)
{
  const char *end;
  return (end = strrchr(argv0, '/')) == NULL ? argv0 : end + 1;
}
