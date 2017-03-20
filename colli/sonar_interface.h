
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/sonar_interface.h,v $
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
 * $Log: sonar_interface.h,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:06  rhino
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







/*****************************************************************************
 * PROJECT: rhino
 *****************************************************************************/

#ifndef SONAR_INTERFACE_LOADED
#define SONAR_INTERFACE_LOADED

#include "devUtils.h"
#include "handlers.h"


#define SONAR_POLLSECONDS 1
#define CM_PER_SECOND 33000.0
#define SECONDS_PER_CYCLE 3.2552e-6



/************************************************************************
 *  Sonar device type.
 ************************************************************************/

typedef struct {
  DEV_TYPE dev;
  /* put in here whatever is needed for starting up the real device.*/
}  SONAR_TYPE, *SONAR_PTR;


/************************************************************************
 *  Simple sonar commands
 *  NOTE : SONAR_init must be called prior to any other SONAR command.
 ************************************************************************/

int SONAR_init();
void SONAR_Debug(BOOLEAN flag, char *filename);
void SONAR_outputHnd(int fd, long chars_available);
void SONAR_terminate(void);
BOOLEAN start_sonar();
void SONAR_missed(Handler handler, Pointer client_data);
void stop_sonar(void);
void SONAR_LoopStart(unsigned long *mask);
void SONAR_ChangeMask(unsigned long *mask);
void SONAR_LoopEnd(void);
void SONAR_SetLoopIntervall(double i);
void SONAR_Read(int rt_no);
/* Checks wether the sonar returned some information */
void SONAR_look_for_sonar_device(void);



/************************************************************************
 *  vars for sonar loop
 ************************************************************************/

#define MAX_MASKS 10
#define LOOP_RECOVER_TIME 2
#define LOOP_RECOVER_CNT 3


extern float sonar_start_rot;
extern float sonar_start_pos_x;
extern float sonar_start_pos_y;


extern float sonar_readings[];

extern float ir_readings[3][24];

extern unsigned long *sonar_act_mask;
extern unsigned long *sonar_mask_array[];
extern HandlerList sonar_handlers;
/*****************************************************************************
 * EVENTS
 *
 *   To install an event handler, call the function SONAR_InstallHandler
 *
 *   void HandlerEvent1(<type> callback_data, Pointer client_data)
 *   {
 *   ...
 *   }
 *
 *   SONAR_InstallHandler(HandlerEvent1, EVENT1, client_data)
 *
 *****************************************************************************/

void SONAR_InstallHandler(Handler handler, int event, Pointer client_data);

/*
 *      EVENT                       Type of callback data 
 *      *****                       *********************                   
 */

#define SONAR_REPORT       0        /* sonar_readings of one sonar loop complete */
#define SONAR_RT_COMPLETE  1        /* sonar_readings of one RT command complete */
#define SONAR_MISSED       2        /* void *data */

#define SONAR_NUMBER_EVENTS       2

#ifdef DECLARE_SONAR_VARS

SONAR_TYPE   sonar_device =
{
  { FALSE, 
      { "", DEFAULT_PORT},
      { "/dev/ttyS1", 14},
      SONAR_DEV_NAME,
      -1,
      TRUE,
      FALSE,
      (FILE *) NULL,
      (fd_set *) NULL,
      (DEVICE_OUTPUT_HND) SONAR_outputHnd,
      (void (*)(void)) NULL,  
      (DEVICE_SET_TIMEOUT)  setTimeout,
      (DEVICE_CANCEL_TIMEOUT) cancelTimeout,
      (void (*)(void)) NULL,  
      {0, 0},
      {LONG_MAX, 0},
      {LONG_MAX, 0},
      (void (*)(void)) NULL,
      FALSE
      }
};

#else

extern SONAR_TYPE   sonar_device;

#endif

#endif



void SIMULATOR_message_to_sonar_handler(TCX_REF_PTR   ref,
				       char        **message);

