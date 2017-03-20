
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
 ***** Source file:     $Source: /home/rhino/rhino/CVS/bee/src/abus/acb/abus.h,v $
 *****
 ***** Created by:      $Author: rhino $
 *****
 ***** Revision #:      $Revision: 1.1.1.1 $
 *****
 ***** Date of revision $Date: 1996/09/22 16:46:03 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: abus.h,v $
 * Revision 1.1.1.1  1996/09/22 16:46:03  rhino
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
 * libabdriver.h   library for abus device drivers
 *
 * Tyson D. Sawyer
 * tyson@rwii.com
 * Real World Interface, Inc.
 *
 * The creation and continued develoment of this software
 * is sponsored and directed by RWI in the interest of
 * providing the mobile robotics and AI research communities
 * with a well designed and robust Robot Applications
 * Interface (RAI) for the complete line of RWI mobile robots. 
 */

/*
 * Copyright 1995, Real World Interface, Inc.
 *
 * Permission to use and modify this software is herby granted by
 * Real World Interface, Inc. (RWI) for non-commercial uses without
 * fee, provided, however, that the above copyright notice appear in
 * all copies, that both the copyright notice and this permission
 * notice appear in supporting documentation.  This permission may be
 * extended to commercial use of this software with specific written
 * permision from RWI.  RWI makes no representations about the
 * suitability of this software for any purpose.  It is provided
 * "as is" without express or implied warranty.  RWI requests
 * notification of any modifications to this software or its
 * documentation.
 *
 *   ==Contact  support@rwii.com  for further information==
 */

#ifndef _AB_DRIVER_H_
#define _AB_DRIVER_H_

#include <acb/global.h>

#define ABD_MAX_DEVS  (20)

#define ABD_DEV_STATE_NOT 0	/* dev struct not in use */
#define ABD_DEV_STATE_NEW 1	/* dev offered */
#define ABD_DEV_STATE_ACK 2	/* dev accepted, waiting for ack */
#define ABD_DEV_STATE_RDY 3	/* dev ready for use */

typedef struct {
  int state;			/* is this struct in use, etc. */
  int busId;			/* bus address of this dev */
  int fd;			/* fd of this dev */

  DevId     devId;		/* prot, rev, vendor, module, devNum */
  DevProt   devProt;		/* prot, type, model */
  DevStatus devStatus;		/* status */
} abdDevType;

extern abdDevType abdDev[];

/* These send the respective messages with arguments */

int D2M_reset       (void);
int D2M_linkRequest (const char *prot, const char *type, const char *model);
int D2M_linkApprove (int devId, int approve);
int D2M_getLongId   (int devId);
int D2M_getId       (int devId);
int D2M_getType     (int devId);
int D2M_getStatus   (int devId);
int D2M_enable      (int devId, int enable);
int D2M_msgToDevice (ABMSG *msg);
int D2M_disconnect  (int devId);

/* These are normally called by abDriverLibSelect   */
/* They will update the assocated abdDev struct and */
/* call msgToDriverCB().                            */

int M2D_linkReply       (ABMSG *msg);
int M2D_longId          (ABMSG *msg);
int M2D_id              (ABMSG *msg);
int M2D_type            (ABMSG *msg);
int M2D_status          (ABMSG *msg);
int M2D_disconnect      (ABMSG *msg);
int M2D_linkApproveAck  (ABMSG *msg);

/* This is normally called by abDriverLibSelect */
/* It will only call msgToDriverCB()            */

int M2D_msgToDriver     (ABMSG *msg);

/* wrapper for M2D functions */
int abDriverLibSelect(int fd);

int abDriverLibInit(int fd, int (*msgToDriverCB)(ABMSG *msg));

#endif /* _AB_DRIVER_H_ */
