
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/msp/msp.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:44:08 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: msp.h,v $
 * Revision 1.1  2002/09/14 15:44:08  rstone
 * *** empty log message ***
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


#ifndef _LIB_MSP_H_
#define _LIB_MSP_H_

typedef struct {
  int interval;
} mspIrParmsType;

typedef struct {
  int echoCount;		/* number of echos to capture */
  int echoTimeout;		/* time to wait for possible echos 1/150 sec */
  int fireDelay;		/* don't worry about this  */
  int fireInterval;		/* time between sets 1/150 sec */
  int echoBlankTime;		/* between echo blanking */
  int initialBlankTime;		/* ping to look for echo blanking */
  int startDelay;		/* don't worry about this either */
} mspSonParmsType;

typedef struct {
  int *validMspId;		/* list of valid MSP id nums */
				/* 0 terminated. */
  /* These are callbacks */
  int (*mspConnect)   (long mspNum, int connectFlag);
				/* 1 connected, 0 disconnected */

  int (*verRep)       (long mspNum, const char *verString);
  int (*dbgRepStr)    (long mspNum, const char *dbgString);

  int (*dbgRepBin)    (long mspNum, int len, const char *dbgData);
				/* binary data, don't worry about it */

  int (*bmpRep)       (long mspNum, unsigned long bumps);
				/* 1 bit per switch */

  int (*irRep)        (long mspNum, const unsigned long *irs);
				/* 0 terminated.  Currently 8 values */

  int (*irParms)      (long mspNum, const mspIrParmsType *parms);
				/* see mspIrParmsType struct above */

  int (*sonRep)       (long mspNum, const unsigned long *table[]);
				/* table[i][0] is transducer number */
				/* table[i][1] is echo 1 */
				/* table[i][2] is echo 2 */
				/* table[i][...] is echo ... */
				/* table[i][echoCount+1] is 0 */
				/* last table[i] is NULL */

  int (*sonTable)     (long mspNum, const unsigned long *table[]);
				/* table[0] = {xducer, xducer, xducer, 0} */
				/* table[i] = {xducer, xducer, xducer, 0} */
				/* table[last] = NULL */

  int (*sonParms)     (long mspNum, const mspSonParmsType *parms);
} mspOpsType;

/*
 * mspLibInit() returns fd which is used
 * to decide when to call mspSelect().
 */

#define devId2mspNum(a) (abdDev[(a)].devId.devNum)
int mspNum2devId (long mspNum);

int mspLibInit (const char *devFile, const mspOpsType *ops);
int mspLibSelect(void);

int mspReqReset       (long mspNum);
int mspReqVer         (long mspNum);
int mspReqIrParms     (long mspNum);
int mspSetIrParms     (long mspNum, const mspIrParmsType *parms);
int mspReqSonTable    (long mspNum);
int mspSetSonTable    (long mspNum, const unsigned long *table[]);
int mspReqSon         (long mspNum, const unsigned long *list);
				/* This is used to get single  */
				/* readings from a single MSP. */
				/* It is not typically used.   */
				/* See the mspterm.c courses   */
				/* for more info.              */

int mspReqSonStart    (long mspNum);
int mspReqSonStop     (long mspNum);
int mspReqSonParms    (long mspNum);
int mspSetSonParms    (long mspNum, const mspSonParmsType *parms);
				/* set any parms that you don't want */
				/* to change to -1 (0xFFFF) */

#endif /* _LIB_MSP_H_ */
