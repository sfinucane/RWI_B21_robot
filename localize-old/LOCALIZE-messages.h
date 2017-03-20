
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/LOCALIZE-messages.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 17:06:29 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: LOCALIZE-messages.h,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.18  1999/09/06 16:36:02  fox
 * Many changes.
 *
 * Revision 1.17  1999/09/03 22:22:38  fox
 * Changed hadnling of real time script. This version contains both.
 *
 * Revision 1.16  1999/09/03 14:10:37  fox
 * Added timestamp to sample set.
 *
 * Revision 1.15  1999/08/27 22:22:31  fox
 * Better communication for multi robot localization.
 *
 * Revision 1.14  1999/04/21 22:58:00  fox
 * First attempt to get samples from multi back.
 *
 * Revision 1.13  1999/04/21 14:05:59  fox
 * Just an intermediate version.
 *
 * Revision 1.12  1999/04/19 22:40:39  fox
 * Minor changes.
 *
 * Revision 1.11  1999/03/16 16:11:14  fox
 * Added more information to sample set message and created a new subscription
 * message for status, maps, robot position, and samples (maps and robot
 * positions not implemented yet).
 *
 * Revision 1.10  1999/03/12 23:11:37  fox
 * Changed message of sample set.
 *
 * Revision 1.9  1999/03/12 00:41:47  fox
 * Minor changes.
 *
 * Revision 1.8  1999/03/10 15:30:32  schulz
 * Added message for querying samples
 *
 * Revision 1.7  1998/12/10 17:56:05  fox
 * Fixed a bug in displayPositions.
 *
 * Revision 1.6  1998/11/17 23:26:14  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.5  1997/04/30 12:25:37  fox
 * Some minor changes.
 *
 * Revision 1.1  1997/01/29 12:25:44  fox
 * Control LOCALIZE from outside.
 *
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/



#ifndef LOCALIZE_messages_defined
#define LOCALIZE_messages_defined



#include "tcx.h"
#include <sys/time.h>

/* Define this structure for external modules. */
#ifndef LOCALIZE_MAKE
typedef struct {
    float x;
    float y;
    float rot;
} realPosition;

typedef struct {
  realPosition pos;
  double weight;
} sampleType;


#else
#include "general.h"
#endif




/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* 
   Notice: when including this file, you need to have the flag

   TCX_define_variables

   be defined exactly once. This will allocate memory for the
   module pointer and the message arrays.
*/

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** TCX module name - this is a unique identifier for TCX *******/

#define TCX_LOCALIZE_MODULE_NAME "LOCALIZE"



#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR LOCALIZE;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR LOCALIZE;	/* otherwise: reference */

#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define LOCALIZE_quit_format NULL
#define LOCALIZE_start_active_localization_format NULL
#define LOCALIZE_stop_active_localization_format NULL

typedef struct {
  float x;
  float y;
  float rot;
} LOCALIZE_set_robot_position_type, *LOCALIZE_set_robot_position_ptr;

#define LOCALIZE_set_robot_position_format "{float, float, float}"

/**********************************************************************
 *
 * This message allows to subscribe for regular updates 
 *
 */

typedef struct {
  int subscribe;		     /* n>0=subscribe, 0=unsubscribe */
} LOCALIZE_register_auto_update_type, *LOCALIZE_register_auto_update_ptr;

#define LOCALIZE_register_auto_update_format "{int}"

typedef struct {
  int subscribeStatus;          /* Correction params, robot position, local maxima. */
  int subscribeRobotPosition;   /* Frequent updates of robot position */
  int subscribeMap;             /* n>0 subscribe, 0 = unsubscribe */
  int subscribeSamples;         /* n>0 subscribe, 0 = unsubscribe */
  int numberOfSamples;          /* n>0: no samples to be sent, 0 all */
} LOCALIZE_register_update_type, *LOCALIZE_register_update_ptr;

#define LOCALIZE_register_update_format "{int,int,int,int,int}"

/**********************************************************************
 *
 * This message allows other programs to ask for the map used by LOCALIZE.
 * Notice: If you subscribe to auto-updates, LOCALIZE will send you regular 
 * updates on the most recent changes using exactly this reply format.
 */

typedef struct{			
  int    resizedObstacles;
} LOCALIZE_map_query_type, *LOCALIZE_map_query_ptr;

#define LOCALIZE_map_query_format "{int}"


#ifdef own
typedef struct{			
  int    sizeX;
  int    sizeY;
  int resolution;
  unsigned char *char_values;	/* New format for likelihoods/localizes:
				 *  0 = passive,
				 *  1..254 = active, and
				 *  will be translated to floats in [0,1] 
				 *  255 = active, an will never change 
				 *  (used, for example, for points under the
				 *  robot)
				 */
} LOCALIZE_map_reply_type, *LOCALIZE_map_reply_ptr;

#define LOCALIZE_map_reply_format "{int, int, int, <char : 1, 2>}"
#endif

/* This struct is used only for compatibility with map.
   All values apart from size_x, size_y and char_values are
   not used. */
typedef struct{			
  int    first_x;
  int    first_y;
  int    size_x;		/* Size of the map, global map coord. */
  int    size_y;		/* Size of the map, global map coord. */
  float  resolution;
  int    delete_previous_map;	
  int    number_of_map;	  
  unsigned char *char_values;	/* New format for likelihoods/maps:
				 *  0 = passive,
				 *  1..254 = active, and
				 *  will be translated to floats in [0,1] 
				 *  255 = active, an will never change 
				 *  (used, for example, for points under the
				 *  robot)
				 */

} LOCALIZE_map_reply_type, *LOCALIZE_map_reply_ptr;

#define LOCALIZE_map_reply_format "{int, int, int, int, float, int, int, <char : 3, 4>}"



typedef struct{			
  int    numberOfSamples;
  struct timeval timeStamp;
} LOCALIZE_samples_query_type, *LOCALIZE_samples_query_ptr;


#define LOCALIZE_samples_query_format "{int, {long, long}}"

typedef struct{
  int numberOfSamples;
  sampleType *replySamples;
  realPosition boundingBoxMin;
  realPosition boundingBoxMax;
  int numberOfSet;
  struct timeval timeStamp;
} LOCALIZE_samples_reply_type, *LOCALIZE_samples_reply_ptr,
  LOCALIZE_updated_samples_type, *LOCALIZE_updated_samples_ptr;

#define LOCALIZE_samples_reply_format "{int, <{float, float, float, double}:1>, \
{float, float, float},{float, float, float}, int, {long, long}}"

#define LOCALIZE_updated_samples_format "{int, <{float, float, float, double}:1>, \
{float, float, float},{float, float, float}, int, {long, long}}"

typedef struct {
  int numberOfLocalMaxima;
  float probOfGlobalMaximum;
  float corrX;
  float corrY;
  float corrRot;
  int corrType;
  float robotX;
  float robotY;
  float robotRot;
} LOCALIZE_update_status_reply_type, *LOCALIZE_update_status_reply_ptr;

#define LOCALIZE_update_status_reply_format "{int, float, float, float, float, int, float, float, float}"



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** LOCALIZE commands - these are the commands/queries understood by LOCALIZE ****/


#ifdef TCX_define_variables		/* do this exactly once! */


#define LOCALIZE_messages \
  {"LOCALIZE_register_auto_update",        LOCALIZE_register_auto_update_format},\
  {"LOCALIZE_register_update",             LOCALIZE_register_update_format},\
  {"LOCALIZE_update_status_reply",         LOCALIZE_update_status_reply_format},\
  {"LOCALIZE_map_query",                   LOCALIZE_map_query_format},\
  {"LOCALIZE_map_reply",                   LOCALIZE_map_reply_format},\
  {"LOCALIZE_samples_query",               LOCALIZE_samples_query_format},\
  {"LOCALIZE_samples_reply",               LOCALIZE_samples_reply_format},\
  {"LOCALIZE_updated_samples",             LOCALIZE_updated_samples_format},\
  {"LOCALIZE_quit",                        LOCALIZE_quit_format},\
  {"LOCALIZE_set_robot_position",          LOCALIZE_set_robot_position_format},\
  {"LOCALIZE_start_active_localization",   LOCALIZE_start_active_localization_format},\
  {"LOCALIZE_stop_active_localization",    LOCALIZE_stop_active_localization_format}
#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#ifdef DEFINE_REPLY_HANDLERS


/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with LOCALIZE ******/



/******* (a) Procedure headers ******/


void
LOCALIZE_map_reply_handler( TCX_REF_PTR                 ref,
			    LOCALIZE_map_reply_ptr map);

void
LOCALIZE_samples_reply_handler( TCX_REF_PTR                 ref,
			    LOCALIZE_samples_reply_ptr samples);

void
LOCALIZE_update_status_reply_handler( TCX_REF_PTR                 ref,
				      LOCALIZE_update_status_reply_ptr status);

/******* (b) Handler array ******/



TCX_REG_HND_TYPE LOCALIZE_reply_handler_array[] = {
  {"LOCALIZE_map_reply", "LOCALIZE_map_reply_handler",
   (void (*)()) LOCALIZE_map_reply_handler, TCX_RECV_ALL, NULL},
  {"LOCALIZE_samples_reply", "LOCALIZE_samples_reply_handler",
   (void (*)()) LOCALIZE_samples_reply_handler, TCX_RECV_ALL, NULL},
  {"LOCALIZE_update_status_reply", "LOCALIZE_update_status_reply_handler",
   (void (*)()) LOCALIZE_update_status_reply_handler, TCX_RECV_ALL, NULL}
};


#endif

#endif
