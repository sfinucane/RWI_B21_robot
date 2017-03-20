
#ifndef SIMULATOR_messages_defined
#define SIMULATOR_messages_defined

/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** File name:                   SIMULATOR-messages.h
 *****
 ***** Part of:                     RHINO SOFTWARE
 *****
 ***** Creator:                     The RHINO Team, University of Bonn
 *****
 ***** Date of creation:            Jan 1994
 *****
 *****
 *****
 *****
 ***** $Source: /usr/local/cvs/bee/src/simulator2/SIMULATOR-messages.h,v $
 *****
 ***** $Revision: 1.1 $
 *****
 ***** $Date: 2002/09/14 16:11:07 $
 *****
 ***** $Author: rstone $
 *****
 *****
 *****
 ***** Contact thrun@carbon.cs.bonn.edu or thrun@cs.cmu.edu.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** 
 ***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED BY
 ***** APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING THE
 ***** COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM "AS IS"
 ***** WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED,
 ***** INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 ***** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE ENTIRE
 ***** RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU.
 ***** SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL
 ***** NECESSARY SERVICING, REPAIR OR CORRECTION.
 ***** 
 ***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
 ***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
 ***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
 ***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
 ***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
 ***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO LOSS OF
 ***** DATA OR DATA BEING RENDERED INACCURATE OR LOSSES SUSTAINED BY YOU
 ***** OR THIRD PARTIES OR A FAILURE OF THE PROGRAM TO OPERATE WITH ANY
 ***** OTHER PROGRAMS), EVEN IF SUCH HOLDER OR OTHER PARTY HAS BEEN
 ***** ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: SIMULATOR-messages.h,v $
 * Revision 1.1  2002/09/14 16:11:07  rstone
 * *** empty log message ***
 *
 * Revision 1.14  2000/03/09 09:30:05  schulz
 * Added Saschas multi-robot extension
 *
 * Revision 1.13  1999/08/27 09:13:20  schulz
 * Only changed one comment
 *
 * Revision 1.12  1999/08/27 09:02:54  schulz
 * Fixed TCX-stuff and conflicts with Dieter's changes
 *
 * Revision 1.11  1999/08/26 21:17:29  fox
 * First changes for simulated multi robot localization.
 *
 * Revision 1.10  1999/08/26 16:53:51  schulz
 * -- The robot position can now be specified on the command line.
 * -- SIMULATOR send the robot's global position to modules, which
 *    have registered for it (Not tested yet!).
 *
 * Revision 1.9  1997/03/14 17:21:46  tyson
 * Added tactile support to simulator
 *
 * Revision 1.8  1997/03/11 17:16:42  tyson
 * added IR simulation and other work
 *
 * Revision 1.7  1997/01/28 13:58:27  schulz
 * Added a TCX message port to move the robot from a program.
 * Not yet tested!
 *
 * Revision 1.6  1996/12/04 05:42:29  tyson
 * simulator2 <-> baseServer partially tested.  Cleaning up bumble and sensor tests as demos.  Added frameWork module.
 *
 * Revision 1.5  1996/12/01 05:40:34  tyson
 * More simulator <-> baseServer work.  Servers should now exit with tcxServer
 *
 * Revision 1.4  1996/11/25 21:06:32  tyson
 * Simulator ready for baseServer?  ...time to update baseServer.
 *
 * Revision 1.3  1996/11/18 04:34:58  tyson
 * More baseServer<->simulator2 work. Still not done.
 *
 * Revision 1.2  1996/11/08 09:23:23  tyson
 * Added base support for baseSever<->Simulator2 to Simulator2.  Sonar will be tougher =:-O
 *
 * Revision 1.1.1.1  1996/09/30 16:17:46  schulz
 * reimport on new source tree
 *
 * Revision 1.2  1996/08/26 10:05:30  schulz
 * added laser interface (without visualization, yet) D.S.
 *
 * Revision 1.5  1996/08/16 20:07:43  fox
 * Added simulation of the laser range finders.
 *
 * Revision 1.4  1996/08/15 08:46:09  schulz
 * *** empty log message ***
 *
 * Revision 1.3  1994/07/25 19:07:30  thrun
 * Introduced define to prevent from double reading.
 *
 * Revision 1.2  1994/06/07  15:05:20  wallosse
 * New header files. Don't they look great?
 *
 * Revision 1.1.1.1  1994/06/07  14:47:02  wallosse
 * imulator messages.
 *
 * Revision 1.2  1994/05/31  21:03:08  rhino
 * General reorganization. New header file. New makefiles.
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/




#include "tcx.h"

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* 
 *  Notice: this file is part of BASE-messages.
 */


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** TCX module name - this is a unique identifier for TCX *******/

#define TCX_SIMULATOR_MODULE_NAME "SIMULATOR"



#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR SIMULATOR;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR SIMULATOR;	/* otherwise: reference */

#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** SIMULATOR data types (for the commands listed below)
 **** Make sure the struct defs and the format defs are consistent! ****/




/*
 **** messages from the BASE module to the SIMULATOR. In plain text.
 * 
 * void SIMULATOR_message_from_base_handler(TCX_REF_PTR ref, char **message) 
 *
 */

#define SIMULATOR_message_from_base_format "string"
#define SIMULATOR_message_from_baseServer_format "[char:6]"





/*
 **** messages from the SIMULATOR back to the BASE module. In plain text.
 * 
 * void SIMULATOR_message_to_base_handler( TCX_REF_PTR ref, char **message) 
 *
 */

#define SIMULATOR_message_to_base_format "string"
#define SIMULATOR_message_to_baseServer_format "[char:128]"





/***************************************************************
 ***************************************************************
 ***************************************************************/

#define SIMULATOR_message_from_irServer_format "[char:132]"

typedef struct {
  int count;
  long *values;
} SIMULATOR_message_to_irServer_type;

#define SIMULATOR_message_to_irServer_format "{int, <int:1>}"


/***************************************************************
 ***************************************************************
 ***************************************************************/

#define SIMULATOR_message_from_tactileServer_format "[char:132]"

typedef struct {
  int count;
  long *values;
} SIMULATOR_message_to_tactileServer_type;

#define SIMULATOR_message_to_tactileServer_format "{int, <int:1>}"


/***************************************************************
 ***************************************************************
 ***************************************************************/

#define SIMULATOR_message_from_sonarServer_format "[char:132]"

#if 0 /* this marks a comment */

This struct definition is from sonarClient.h

typedef struct sonarType {
  int value;			/* the distance */
  int mostRecent;		/* was this one in most recent return? */
  struct timeval time;		/* time of capture.  not yet precise */
} sonarType;

#endif /* comment */

#define SIMULATOR_message_to_sonarServer_format "[{int, int, {long, long}}:24]"

/***************************************************************
 ***************************************************************
 ***************************************************************/

/*
 * messages from the SONAR module to the SIMULATOR. In plain text.
 * void SIMULATOR_message_from_sonar_handler(TCX_REF_PTR ref, char **message) 
 */

#define SIMULATOR_message_from_sonar_format "string"

/*
 * messages from the SIMULATOR back to the SONAR module. In plain text.
 * void SIMULATOR_message_to_sonar_handler(TCX_REF_PTR ref, char **message) 
 */

#define SIMULATOR_message_to_sonar_format "string"

/*
 **** messages from the SIMULATOR  to the LASER module in the given format
 * 
 * void SIMULATOR_message_to_sonar_handler(TCX_REF_PTR ref, char **message) 
 *
 */


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

typedef struct {
  int   f_numberOfReadings;
  int*  f_reading;
  float f_startAngle;
  float f_angleResolution;
  int   r_numberOfReadings;
  int*  r_reading;
  float r_startAngle;
  float r_angleResolution;
} SIMULATOR_message_to_laser_type, *SIMULATOR_message_to_laser_ptr;


#define SIMULATOR_message_to_laser_format "{int, <{int} : 1>, float, float, int, <{int} : 5>, float, float}"


typedef struct {
    float x;
    float y;
    float rot;
} SIMULATOR_set_robot_position_type, *SIMULATOR_set_robot_position_ptr;

#define SIMULATOR_set_robot_position_format "{float, float, float}"


typedef struct {
  float posX;
  float posY;
  float posRot;
} SIMULATOR_status_update_type, *SIMULATOR_status_update_ptr;

#define SIMULATOR_status_update_format "{float,float,float}"

typedef struct {
  int update;		   /* >1=subscribe, 
			      0=unsubscribe but keep me on the list!*/
} SIMULATOR_register_auto_update_type, *SIMULATOR_register_auto_update_ptr;

#define SIMULATOR_register_auto_update_format "int"
#define SIMULATOR_cancel_auto_update_format NULL

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** SIMULATOR commands - 
 **** these are the commands/queries understood by SIMULATOR ****/


#define SIMULATOR_messages \
  {"SIMULATOR_message_from_base",  SIMULATOR_message_from_base_format},\
  {"SIMULATOR_message_to_base",    SIMULATOR_message_to_base_format},\
  {"SIMULATOR_message_from_sonar", SIMULATOR_message_from_sonar_format},\
  {"SIMULATOR_message_to_sonar",   SIMULATOR_message_to_sonar_format},\
  {"SIMULATOR_message_to_laser",   SIMULATOR_message_to_laser_format},\
  {"SIMULATOR_message_from_baseServer", \
     SIMULATOR_message_from_baseServer_format},\
  {"SIMULATOR_message_to_baseServer", \
     SIMULATOR_message_to_baseServer_format},\
  {"SIMULATOR_message_from_sonarServer", \
     SIMULATOR_message_from_sonarServer_format},\
  {"SIMULATOR_message_to_sonarServer", \
     SIMULATOR_message_to_sonarServer_format},\
  {"SIMULATOR_message_from_irServer", \
     SIMULATOR_message_from_irServer_format},\
  {"SIMULATOR_message_to_irServer", \
     SIMULATOR_message_to_irServer_format},\
  {"SIMULATOR_message_from_tactileServer", \
     SIMULATOR_message_from_tactileServer_format},\
  {"SIMULATOR_message_to_tactileServer", \
     SIMULATOR_message_to_tactileServer_format},\
  {"SIMULATOR_set_robot_position", \
     SIMULATOR_set_robot_position_format},\
  {"SIMULATOR_register_auto_update", \
     SIMULATOR_register_auto_update_format},\
  {"SIMULATOR_cancel_auto_update", \
     SIMULATOR_cancel_auto_update_format}, \
  {"SIMULATOR_status_update", \
      SIMULATOR_status_update_format}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/




#ifdef DEFINE_REPLY_HANDLERS


/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with SIMULATOR ******/




/******* (a) Procedure headers ******/



void SIMULATOR_message_to_base_handler(TCX_REF_PTR   ref,
				       char **message);

void SIMULATOR_message_to_baseServer_handler(TCX_REF_PTR   ref,
					     char *message);



void SIMULATOR_message_to_sonar_handler(TCX_REF_PTR   ref,
					char **message);

void SIMULATOR_message_to_laser_handler(TCX_REF_PTR   ref,
					SIMULATOR_message_to_laser_ptr	data);

void SIMULATOR_status_update_handler( TCX_REF_PTR ref,
				      SIMULATOR_status_update_ptr status);

/******* (b) Handler array ******/



TCX_REG_HND_TYPE SIMULATOR_reply_handler_array[] = {
  {"SIMULATOR_message_to_base", "SIMULATOR_message_to_base_handler",
   (void (*)()) SIMULATOR_message_to_base_handler, TCX_RECV_ALL, NULL},
  {"SIMULATOR_message_to_sonar", "SIMULATOR_message_to_sonar_handler",
   (void (*)()) SIMULATOR_message_to_sonar_handler, TCX_RECV_ALL, NULL},
  {"SIMULATOR_message_to_laser", "SIMULATOR_message_to_laser_handler",
     (void (*)()) SIMULATOR_message_to_laser_handler, TCX_RECV_ALL, NULL},
  {"SIMULATOR_status_update", "SIMULATOR_status_update_reply_handler",
   (void (*)()) SIMULATOR_status_update_handler, TCX_RECV_ALL, NULL}
};

#endif



/* P.W. */


extern 	TCX_MODULE_PTR MODULE_BASE; 		/* will be != NULL if BASE
					    * has already sent a message.
					    * Only then we will send any
					    * status-messages */
extern 	TCX_MODULE_PTR MODULE_SONAR;     	/* will be != NULL if SONAR
					    * has already sent a message.
					    * Only then we will send any
					    * status-messages */
extern TCX_MODULE_PTR MODULE_IR;
extern TCX_MODULE_PTR MODULE_TACTILE;

#endif
