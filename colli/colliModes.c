
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/colliModes.c,v $
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
 * $Log: colliModes.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.12  1999/11/15 13:28:06  fox
 * *** empty log message ***
 *
 * Revision 1.11  1999/07/23 19:46:36  fox
 * Added ability to read grid maps for the obstacle server.
 *
 * Revision 1.9  1998/01/22 13:04:22  fox
 * Minor changes.
 *
 * Revision 1.8  1997/11/12 17:07:29  fox
 * Removed some old arm stuff.
 *
 * Revision 1.7  1997/05/14 08:10:57  fox
 * Fixed a bug when setting random mode.
 *
 * Revision 1.6  1997/03/26 18:42:02  fox
 * Incorporated external obstacle information into the collision avoidance.
 *
 * Revision 1.5  1997/02/11 16:39:11  fox
 * Changed COLLI_setmode
 *
 * Revision 1.4  1997/02/04 18:00:33  fox
 * Added paramters to BASE_setMode.
 *
 * Revision 1.3  1997/01/07 13:47:14  fox
 * Improved rotate_away.
 *
 * Revision 1.2  1997/01/02 12:13:50  fox
 * Fixed a bug in load_parameters.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:04  rhino
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




#include "collisionIntern.h"


/* The strcuctures determine the behaviour in the
 * different modes.
 */
mode_structure **mode_structure_array;
mode_structure *ACTUAL_MODE = NULL;
int mode_number;
velocities actual_velocities;

/* If this flag is set the collision avoidance moves backward. */
BOOLEAN colli_go_backward = FALSE;



/**********************************************************************
 **********************************************************************
 *               Functions for the different modes.                   *
 **********************************************************************
 **********************************************************************/


/**********************************************************************
 * Set the direction to move to the next target point.
 **********************************************************************/
void
COLLI_GoForward()
{
#ifdef UNIBONN
  if (use_vision)
    tcxSendMsg(SUNVIS, "SUNVIS_look_forward", NULL);
#endif
    colli_go_backward = FALSE;
}

void
COLLI_GoBackward()
{
#ifdef UNIBONN   
  if (use_vision)
    tcxSendMsg(SUNVIS, "SUNVIS_look_backward", NULL);
#endif
  colli_go_backward = TRUE;
}



void COLLI_SetMode( int mode)
{
  int intMode = (int) mode;
  
  if (intMode >= NUMBER_OF_MODES || intMode < 0) {
    fprintf(stderr, "Wrong mode number (%d). Use default mode.\n", intMode);
    ACTUAL_MODE = mode_structure_array[DEFAULT_MODE];
    mode_number = DEFAULT_MODE;
  }
  else {
    ACTUAL_MODE = mode_structure_array[intMode];
    mode_number = intMode;
    if ( mode_number == ARM_OUT_MODE) {
      armState = OUTSIDE;
/*       COLLI_GoBackward(); */
    }
  }

  fprintf(stderr, "\n\nmode: %d\n", intMode);
 
  fprintf (stderr, "tv      : %f  ", (ACTUAL_MODE->target_max_trans_speed));
  fprintf (stderr, "ta      : %f\n", (ACTUAL_MODE->target_trans_acceleration));
  fprintf (stderr, "rv      : %f  ", (RAD_TO_DEG(ACTUAL_MODE->target_max_rot_speed)));
  fprintf (stderr, "ra      : %f\n", (RAD_TO_DEG(ACTUAL_MODE->target_rot_acceleration)));
  fprintf (stderr, "velocity: %f \n", (ACTUAL_MODE->velocity_factor));
  fprintf (stderr, "angle   : %f\n", (ACTUAL_MODE->angle_factor));
  fprintf (stderr, "distance: %f\n", (ACTUAL_MODE->distance_factor));

  /* Set the corresponding values. */
  BASE_TranslateVelocity(             ACTUAL_MODE->target_max_trans_speed);
  BASE_RotateVelocity( RAD_TO_DEG(    ACTUAL_MODE->target_max_rot_speed));
  BASE_TranslateAcceleration(         ACTUAL_MODE->target_trans_acceleration);
  BASE_RotateAcceleration(RAD_TO_DEG( ACTUAL_MODE->target_rot_acceleration));
}


/************************************************************************
 * Sets all modes to the default value.
************************************************************************/
void
setAllModesToDefault()
{
  int i;
  
  for (i = 0; i < NUMBER_OF_MODES; i++) {
    mode_structure_array[i]         = default_mode_structure();
    if ( i == RANDOM_MODE)
      mode_structure_array[i]->angle_factor = 0.0;
  }
}


    

/************************************************************************
 * Copies the value of the default mode into all other modes. To do this
 * the DEFAULT_MODE must be allocated. Memory for the other modes will
 * be allocated in this function.
 ************************************************************************/
void
copyDefaultMode()
{
    int i;
    mode_structure* mode = mode_structure_array[DEFAULT_MODE];
    
    /* First set the default values. */
    VELOCITY_FACTOR               = mode->velocity_factor;
    ANGLE_FACTOR                  = mode->angle_factor;
    DISTANCE_FACTOR               = mode->distance_factor;

    NUMBER_OF_RVELS               = mode->number_of_rvels;
    NUMBER_OF_TVELS               = mode->number_of_tvels;

    TARGET_TRANS_ACCELERATION     = mode->target_trans_acceleration;
    TARGET_ROT_ACCELERATION       = mode->target_rot_acceleration;
    TARGET_MAX_ROT_SPEED          = mode->target_max_rot_speed;
    TARGET_MAX_TRANS_SPEED        = mode->target_max_trans_speed;

    EXCEPTION_TRANS_ACCELERATION  = mode->exception_trans_acceleration;
    EXCEPTION_TRANS_VELOCITY      = mode->exception_trans_velocity;
    EXCEPTION_ROT_ACCELERATION    = mode->exception_rot_acceleration;
    EXCEPTION_ROT_VELOCITY        = mode->exception_rot_velocity;

    MIN_DIST                      = mode->min_dist;
    SMOOTH_WIDTH                  = mode->smooth_width;
    SECURITY_DIST                 = mode->security_dist;;
    MIN_DIST_FOR_TARGET_WAY_FREE  = mode->min_dist_for_target_way_free;
    MAX_COLLISION_LINE_LENGTH     = mode->max_collision_line_length;
    MAX_RANGE                     = mode->max_range;

    EDGE_PORTION                  = mode->edge_portion;
    MAX_SECURITY_SPEED            = mode->max_security_speed;
    MIN_SECURITY_SPEED            = mode->min_security_speed;
    MAX_SECURITY_DIST             = mode->max_security_dist;

    SECURITY_ANGLE                = mode->security_angle;
    ROTATE_AWAY_PERSISTENCE       = mode->rotate_away_persistence;
    
    /* Now we can allocate memory for the other modes and initialize them
     * with the default values.
     */
    for (i = 0; i < NUMBER_OF_MODES; i++)
	if ( i != DEFAULT_MODE)
	    *(mode_structure_array[i])         = *mode;
}




/************************************************************************
 * Returns a structure for a mode initialized with the default values. *
 ************************************************************************/
mode_structure*
default_mode_structure(void)
{
  mode_structure *mode;

  mode = (mode_structure *) malloc (sizeof(mode_structure));

  mode->velocity_factor = VELOCITY_FACTOR;
  mode->angle_factor = ANGLE_FACTOR;
  mode->distance_factor = DISTANCE_FACTOR;

  mode->number_of_rvels = NUMBER_OF_RVELS;
  mode->number_of_tvels = NUMBER_OF_TVELS;

  mode->target_trans_acceleration = TARGET_TRANS_ACCELERATION;
  mode->target_rot_acceleration = TARGET_ROT_ACCELERATION;
  mode->target_max_rot_speed = TARGET_MAX_ROT_SPEED;
  mode->target_max_trans_speed = TARGET_MAX_TRANS_SPEED;

  mode->exception_trans_acceleration = EXCEPTION_TRANS_ACCELERATION;
  mode->exception_trans_velocity = EXCEPTION_TRANS_VELOCITY;
  mode->exception_rot_acceleration = EXCEPTION_ROT_ACCELERATION;
  mode->exception_rot_velocity = EXCEPTION_ROT_VELOCITY;

  mode->min_dist = MIN_DIST;
  mode->smooth_width = SMOOTH_WIDTH;
  mode->security_dist = SECURITY_DIST;;
  mode->min_dist_for_target_way_free = MIN_DIST_FOR_TARGET_WAY_FREE;
  mode->max_collision_line_length = MAX_COLLISION_LINE_LENGTH;
  mode->max_range = MAX_RANGE;

  mode->edge_portion = EDGE_PORTION;
  mode->max_security_speed = MAX_SECURITY_SPEED;
  mode->min_security_speed = MIN_SECURITY_SPEED;
  mode->max_security_dist = MAX_SECURITY_DIST;

  /* ARM PRAKTIKUM */
  mode->security_angle = SECURITY_ANGLE;
  mode->rotate_away_persistence = ROTATE_AWAY_PERSISTENCE;
  
  return mode;
}






/**********************************************************************/
void COLLI_get_parameters(COLLI_parameter_ptr parameters)
{
  
  if (dumpInfo)
    fprintf( dumpFile, "Received new parameters: VELOCITY_FACTOR : %f   ANGLE_FACTOR: %f\n",
	    parameters->velocity_factor, parameters->angle_factor);
  fprintf(stderr, "Received new parameters: VELOCITY_FACTOR : %f   ANGLE_FACTOR: %f\n",
	  parameters->velocity_factor, parameters->angle_factor);
  
  if (parameters->velocity_factor >= 0.0) 
    ACTUAL_MODE->velocity_factor = parameters->velocity_factor;
  if (parameters->angle_factor >= 0.0) 
    ACTUAL_MODE->angle_factor = parameters->angle_factor;
  if (parameters->distance_factor >= 0.0) 
    ACTUAL_MODE->distance_factor = parameters->angle_factor;
  if (parameters->target_max_trans_speed >= 0.0) {
    ACTUAL_MODE->target_max_trans_speed = parameters->target_max_trans_speed;
    BASE_TranslateVelocity(ACTUAL_MODE->target_max_trans_speed);
  }
  if (parameters->target_max_rot_speed >= 0.0)  {
    ACTUAL_MODE->target_max_rot_speed =  DEG_TO_RAD(parameters->target_max_rot_speed);
    BASE_RotateVelocity(RAD_TO_DEG(ACTUAL_MODE->target_max_rot_speed));
  }
  if (parameters->target_trans_acceleration >= 0.0)  {
    ACTUAL_MODE->target_trans_acceleration = parameters->target_trans_acceleration;
    BASE_TranslateCollisionAcceleration(ACTUAL_MODE->target_trans_acceleration);
  }
  if (parameters->target_rot_acceleration >= 0.0) {
    ACTUAL_MODE->target_rot_acceleration = DEG_TO_RAD(parameters->target_rot_acceleration);
    BASE_RotateCollisionAcceleration(RAD_TO_DEG(ACTUAL_MODE->target_rot_acceleration));
  } 
}





















