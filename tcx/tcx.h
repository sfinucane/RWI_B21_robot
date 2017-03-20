
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/tcx/tcx.h,v $
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
 * $Log: tcx.h,v $
 * Revision 1.1  2002/09/14 15:48:16  rstone
 * *** empty log message ***
 *
 * Revision 1.10  1998/10/30 18:49:20  fox
 * Added support for multiple robots.
 *
 * Revision 1.9  1998/06/29 14:19:58  arbuckle
 * Always include <sys/types.h> for definition of size_t.
 *
 * Revision 1.8  1998/04/08 16:36:21  wolfram
 * Changed the concept of multi-robot support. TCX now contains a procedure to
 * generate module names.
 *
 * Revision 1.7  1998/04/07 18:32:11  wolfram
 * Removed Multi calls from tcx. Version runs. Major rework has to be
 * done in order to support multiple robots.
 *
 * Revision 1.6  1998/04/04 03:05:39  wolfram
 * Intermediate version supporting multiple robots
 *
 * Revision 1.5  1998/03/18 08:45:30  arbuckle
 * It's stored as a void *; it's used as a void *; the documentation says
 * it's a void * but ... actually it's an unsigned char *. Just as well
 * nobody ever used it!
 * Changed hndData from unsigned char* to void* in struct TCX_REG_HND_TYPE.
 *
 * Revision 1.4  1998/03/17 14:46:58  arbuckle
 * Changed declaration from tcxRegisterClose which doesn't exist to
 * tcxRegisterCloseHnd which does!
 *
 * Revision 1.3  1997/02/22 05:16:58  thrun
 * Version numbers are now also checked for libraries.
 *
 * Revision 1.2  1997/02/22 00:59:16  thrun
 * Introduced version number support
 *
 * Revision 1.1.1.1  1996/09/22 16:46:02  rhino
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







/******************************************************************************
*
* PROJECT: TCX
* 
* PRELIMINARY RELEASE. COMMENTS AND QUESTIONS CAN BE SENT TO fedor@cs.cmu.edu 
* (c) Copyright 1993 Christopher Fedor. All rights reserved.
*
* MODULE: communications
*
* FILE: tcx.h
*
* ABSTRACT:
* 
* communications
*
* REVISION HISTORY
*
* 23-Nov-91 Christopher Fedor, School of Computer Science, CMU
* created.
*
******************************************************************************/

#ifndef INCtcx
#define INCtcx

#include <sys/types.h>
/* For size_t in particular */

/******** begin changes Sebastian Thrun 93-1-5 ******************/
/******** updated to support VMS - Tyson Sawyer 13 Oct 1994 *****/
#if defined(i386) || defined(VMS)

#ifndef htons
#include <netinet/in.h>
#endif

#define htonChar(c) (c)
#define htonShort(s) (short)htons((u_short)s)
#define htonInt(i) (int)htonl((u_long)i)
#define htonLong(l) (long)htonl((u_long)l)
extern float htonFloat(float f);
extern double htonDouble(double d);

#define ntohChar(c) (c)
#define ntohShort(s) (short)ntohs((u_short)s)
#define ntohInt(i) (int)ntohl((u_long)i)
#define ntohLong(l) (long)ntohl((u_long)l)
extern float ntohFloat(float f);
extern double ntohDouble(double d);

#else

#define htonChar(c) (c)
#define htonShort(s) (s)
#define htonInt(i) (i)
#define htonLong(l) (l)
#define htonFloat(f) (f)
#define htonDouble(d) (d)

#define ntohChar(c) (c)
#define ntohShort(s) (s)
#define ntohInt(i) (i)
#define ntohLong(l) (l)
#define ntohFloat(f) (f)
#define ntohDouble(d) (d)

#endif
/******** end changes Sebastian Thrun 93-1-5 ******************/






#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define TCX_RECV_ALL  1
#define TCX_RECV_NONE 2
#define TCX_RECV_ONCE 3

#define NO_HND       0x0000
#define TCX_HND      0x0001
#define USER_HND     0x0002
#define ALL_HND      (USER_HND | TCX_HND)

typedef struct _module *TCX_MODULE_PTR;

typedef struct _TCX_REF_TYPE *TCX_REF_PTR;

typedef struct _FORMAT_TYPE *TCX_FMT_PTR;

typedef struct {
  char *msgName;
  char *msgFormat;
} TCX_REG_MSG_TYPE;

typedef struct {
  char *name;
  char *format;
  int connections;
} TCX_REG2_MSG_TYPE;
                    
typedef struct {
  char *msgName;
  char *hndName;
  void (*hndProc)();
  int hndControl;
  void *hndData;
} TCX_REG_HND_TYPE;

extern TCX_FMT_PTR tcxCreateFMT;

#ifdef __cplusplus
extern "C" {
#endif
extern void tcxInitialize(char *module, char *server);

extern void tcxInitializeServerRoute(char *module, char *server);

extern void tcxSetModuleName(char *module, char *extension, char *moduleName);
extern void tcxSetModuleNameExtension(char *module);

extern TCX_MODULE_PTR tcxConnectModule(char *module);
extern TCX_MODULE_PTR tcxConnectOptional(char *module);

extern char *tcxModuleName(TCX_MODULE_PTR module);

extern void tcxSendData(TCX_MODULE_PTR module, int id, int ins, void *data, 
			int len, TCX_FMT_PTR fmt);

extern int tcxRecvData(TCX_MODULE_PTR *module, int *id, int *ins, void *data, 
		       TCX_FMT_PTR fmt, int allowHnd, void *timeout);

extern TCX_FMT_PTR tcxParseFormat(char *format);

extern void tcxFree(char *msgName, void *data);
extern void tcxFreeReply(char *msgName, void *data);
extern void tcxFreeByRef(TCX_REF_PTR ref, void *data);
extern void tcxFreeData(TCX_FMT_PTR fmt, void *data);

extern void tcxDecodeData(TCX_FMT_PTR fmt, void *buf, void *data);

extern void tcxRegisterExitHnd(void (*proc)());
  
extern void tcxRegisterConnectHnd(char *name, void (*openHnd)(), 
				  void (*closeHnd)());

extern void tcxRegisterMessages(TCX_REG_MSG_TYPE *msgArray, int size);
extern void tcxRegisterHandlers(TCX_REG_HND_TYPE *hndArray, int size);

extern void tcxRegisterMHnd(char *msgName, void (*proc)(), 
			    int control, int mask, void *data);

extern void tcxRegisterDHnd(int id, TCX_FMT_PTR fmt, 
			    void (*proc)(), int control, int mask, void *data);

extern void tcxRecvFlush(TCX_MODULE_PTR module, int id);

extern int tcxSendMsg(TCX_MODULE_PTR module, char *msgName, void *data);

extern int tcxSendDoubleMsg(TCX_MODULE_PTR module, char *name1, void *data1,
		      char *name2, void *data2);

extern int tcxRecvLoop(void *timeout);

extern void tcxQuery(TCX_MODULE_PTR module, char *msgName, 
		     void *data, char *replyMsgName, void *reply);

extern void tcxReply(TCX_REF_PTR ref, char *replyMsgName, void *reply);

extern void tcxReplyData(TCX_MODULE_PTR module, int id, int ins, void *reply);

extern char *tcxMessageName(int id);
extern int tcxMessageId(char *name);

extern TCX_MODULE_PTR tcxRefModule(TCX_REF_PTR ref);
extern int tcxRefId(TCX_REF_PTR ref);
extern int tcxRefIns(TCX_REF_PTR ref);

extern int tcxRecvMsg(char *msgName, int *ins, void *data, void *timeout);
extern int tcxRecvMsgNoHnd(char *msgName, int *ins, void *data, void *timeout);

extern int tcxRecvMsgE(TCX_MODULE_PTR *module, char *msgName, int *ins, 
		       void *data, int allowHnd, void *timeout);

extern int tcxRecvResp(char *msgName, int *ins, void *data, void *timeout);
extern int tcxRecvRespE(TCX_MODULE_PTR *module, char *msgName, int *ins, 
		       void *data, int allowHnd, void *timeout);

/* 3-nov-93: fedor ***** new stuff for pbk - will need general clean up ****/

extern void tcxRegisterCloseHnd(void (*closeHnd)());

extern int tcxNRecvMsgE(TCX_MODULE_PTR *module, char **msgName, void *data, 
			void *timeout, int *inst, int hndMask);

extern char *tcxCurrentModuleName(void);

/*****************/

extern TCX_REF_PTR tcxRefCopy(TCX_REF_PTR ref);
extern void tcxRefFree(TCX_REF_PTR ref);

extern void tcxSetAutoReconnect();
extern int tcxTestActiveModule(TCX_MODULE_PTR module);

extern void tcxCloseAll();

extern void tcxLock();		/* not implemented */

extern void tcxUnlock();		/* not implemented */

/*****************/

void
check_version_number(int major, int minor, int robot_type, char *date, 
		     char *reference_text, 
		     int final_check);



#ifdef __cplusplus
}
#endif
#endif /* INCtcx */





