/* xrdev/examples/robot_arg.h
 *
 * public interface to the standard command line parsing code
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

#ifndef XRDEV_EXAMPLES_ROBOT_ARG_H
#define XRDEV_EXAMPLES_ROBOT_ARG_H

#include "Nclient.h"


/* examples/robot_arg.c */
BOOL ARG_GetStandardArgs(int *argc, char ***argv, char **sched_hostname_buf,
			 unsigned short *sched_port_buf, long *robot_id_buf);
const char *ARG_GetProgramName(const char *argv0);


#endif /* XRDEV_EXAMPLES_ROBOT_ARG_H */
