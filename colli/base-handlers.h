
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/base-handlers.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:25:05 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: base-handlers.h,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.7  1999/05/19 10:26:00  schulz
 * Added the keyword TRACK_TARGET_WITH_PANTILT.
 * This global flag enables pantilt tracking, if it is contained in
 * colli_modes.ini
 *
 * Revision 1.6  1998/08/18 16:24:21  fox
 * Added support for b18 robot.
 *
 * Revision 1.5  1998/04/12 15:54:31  wolfram
 * Added option -robot for multi robot support
 *
 * Revision 1.4  1997/04/14 08:29:22  fox
 * Fixed a bug in not unibonn version.
 *
 * Revision 1.3  1997/04/09 12:57:47  fox
 * Minor changes.
 *
 * Revision 1.2  1997/02/02 22:32:28  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
 *
 * Revision 1.1.1.1  1996/09/22 16:46:05  rhino
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





#ifndef BASE_HANDLERS_INCLUDED
#define BASE_HANDLERS_INCLUDED


void init_tcx(char *robotName);

void send_automatic_status_update(void);
void send_automatic_action_executed_message(void);
void send_automatic_sonar_update(void);
void send_automatic_laser_update(void);
void send_automatic_ir_update(void);

extern BASE_update_status_reply_type base_tcx_status;
extern SONAR_sonar_reply_type        sonar_tcx_status;
extern LASER_laser_reply_type        laser_tcx_status;
extern IR_ir_reply_type              ir_tcx_status;
extern COLLI_colli_reply_type        colli_tcx_status;

extern BOOLEAN use_simulator;
extern BOOLEAN use_rwi_server;



#define LENGTH_OF_MODULE_NAMES 128
#define NUMBER_OF_TCX_MODULES 8

#define BASE_MODULE           0
#define SERVER_MODULE         1
#define LASER_SERVER_MODULE   2
#define SIMULATOR_MODULE      3
#define BASE_ROUTED_MODULE    4
#define ARM_MODULE            5
#define SUNVIS_MODULE         6
#define SOUND_MODULE          7



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

extern struct bParamList * bParamList;
extern const char *bRobotType;

extern int TRACK_TARGET_WITH_PANTILT;



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/******* auto-reply stuff, data definitions ***************/


#define MAX_N_AUTO_UPDATE_MODULES 100



extern int n_auto_update_modules; /* number of processes to whom
				   * position should be forwarded
				   * automatically upon change */

extern int n_auto_status_update_modules;
extern int n_auto_sonar_update_modules;
extern int n_auto_laser_update_modules;
extern int n_auto_ir_update_modules;
extern int n_auto_colli_update_modules;


typedef struct{
  TCX_MODULE_PTR module;	/* pointer to TCX module */
  int            status;	/* 1=subscribed to regular status updates */
  int            sonar;		/* 1=subscribed to regular sonar updates */
  int            laser;		/* 1=subscribed to regular laser updates */
  int            ir;		/* 1=subscribed to regular ir  updates */
  int            colli;		/* 1=subscribed to regular colli updates */
} auto_update_type;



extern auto_update_type auto_update_modules[MAX_N_AUTO_UPDATE_MODULES];
                                /* collection of TCX-module ptrs */

extern TCX_REF_PTR TCX_sender_of_action;

extern char moduleName[NUMBER_OF_TCX_MODULES][LENGTH_OF_MODULE_NAMES];


void connect_to_SOUND(void);

void SOUND_talk_text(char *text);
void SOUND_play_message( int messageNumber);

#endif
