
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/colliFiles.c,v $
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
 * $Log: colliFiles.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.15  1999/11/15 13:28:06  fox
 * *** empty log message ***
 *
 * Revision 1.14  1999/10/14 19:56:39  fox
 * IMPORTANT change: if the orientation of the robot is off more than 60 deg.
 * it will prefer rotation a lot.
 *
 * Revision 1.13  1999/09/27 23:17:39  fox
 * Minor changes.
 *
 * Revision 1.12  1999/05/19 10:26:00  schulz
 * Added the keyword TRACK_TARGET_WITH_PANTILT.
 * This global flag enables pantilt tracking, if it is contained in
 * colli_modes.ini
 *
 * Revision 1.11  1998/08/29 21:50:02  fox
 * Adapted exception handling in order to deal with people around the robot.
 *
 * Revision 1.10  1998/08/26 23:23:40  fox
 * Final version before I changed exception handling.
 *
 * Revision 1.9  1997/11/12 17:07:28  fox
 * Removed some old arm stuff.
 *
 * Revision 1.8  1997/04/26 13:56:17  fox
 * Added targetDefined, targetX, and targetY to status report.
 *
 * Revision 1.7  1997/04/09 12:57:49  fox
 * Minor changes.
 *
 * Revision 1.6  1997/03/28 03:48:29  tyson
 * finding .ini files and minor stuff
 *
 * Revision 1.5  1997/01/03 18:08:58  fox
 * Made some minor changes.
 *
 * Revision 1.4  1997/01/02 12:13:49  fox
 * Fixed a bug in load_parameters.
 *
 * Revision 1.3  1996/10/09 13:59:54  fox
 * Changed the directory where to find the parameter file colli_modes.ini.
 *
 * Revision 1.2  1996/09/24 07:04:43  thrun
 * Committed Stefan Waldherr's tactile support.
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
#include <bUtils.h>


BOOLEAN dumpInfo;
FILE* dumpFile = NULL;

extern float DEFAULT_TRANSLATION_SPEED;
extern float DEFAULT_ROTATION_SPEED;


/**********************************************************************
 * If this function is called relevant values will be written to
 * stderr. This is just for debugging.
 **********************************************************************/
void
COLLI_DumpInfo()
{
  dumpFile = fopen("dumpFile", "w");
  dumpInfo = TRUE;
}


/**********************************************************************
 * Sets a mark in the dumpfile.
 **********************************************************************/
void
COLLI_MarkInfo()
{
  fprintf( dumpFile, "*******************************************\n");
  fprintf( dumpFile, "************SET A MARK*********************\n");
  fprintf( dumpFile, "*******************************************\n");
}


/* Writes the status of some variables in the dumpfile. */
void
outputCollisionStatus()
{
  static int current_mode = -1;

  if (dumpInfo) {
    fprintf( dumpFile, "\nNext call to collision avoidance.\n");
    fprintf( dumpFile, "posx:          %f\n", rwi_base.pos_x);
    fprintf( dumpFile, "posy:          %f\n", rwi_base.pos_y);
    fprintf( dumpFile, "rot:           %f\n", rwi_base.rot_position);
    fprintf( dumpFile, "exc-rot-vel:   %f\n", ACTUAL_MODE->exception_rot_velocity);
    fprintf( dumpFile, "exc-rot-acc:   %f\n", ACTUAL_MODE->exception_rot_acceleration);
    fprintf( dumpFile, "rot-vel:       %f\n", rwi_base.rot_set_speed);
    fprintf( dumpFile, "rot-acc:       %f\n", rwi_base.rot_acceleration);
    fprintf( dumpFile, "des-rot-vel:   %f\n", desired_rot_velocity);
    fprintf( dumpFile, "curr-rot-vel:  %f\n", rwi_base.rot_current_speed);
    fprintf( dumpFile, "exc-trans-vel: %f\n", ACTUAL_MODE->exception_trans_velocity);
    fprintf( dumpFile, "exc-trans-acc: %f\n", ACTUAL_MODE->exception_trans_acceleration);
    fprintf( dumpFile, "trans-vel:     %f\n", rwi_base.trans_set_speed);
    fprintf( dumpFile, "trans-acc:     %f\n", rwi_base.trans_acceleration);
    fprintf( dumpFile, "des-trans-vel: %f\n", desired_trans_velocity);
    fprintf( dumpFile, "curr-tr-vel:   %f\n", rwi_base.trans_current_speed);
    fprintf( dumpFile, "target_flag:   %d\n", target_flag);

    fprintf( dumpFile, "halting:       %d\n", haltingFlag);
    fprintf( dumpFile, "achieve_dist:  %d\n", achieveDistanceFlag);
    fprintf( dumpFile, "rotate_away:   %d\n", rotatingAwayFlag);
    fprintf( dumpFile, "rot_wanted:    %d\n", rot_wanted);

    
    fprintf( stderr, "\nNext call to collision avoidance.\n");
    fprintf( stderr, "posx:          %f\n", rwi_base.pos_x);
    fprintf( stderr, "posy:          %f\n", rwi_base.pos_y);
    fprintf( stderr, "rot:           %f\n", rwi_base.rot_position);
    fprintf( stderr, "exc-rot-vel:   %f\n", ACTUAL_MODE->exception_rot_velocity);
    fprintf( stderr, "exc-rot-acc:   %f\n", ACTUAL_MODE->exception_rot_acceleration);
    fprintf( stderr, "rot-vel:       %f\n", rwi_base.rot_set_speed);
    fprintf( stderr, "rot-acc:       %f\n", rwi_base.rot_acceleration);
    fprintf( stderr, "des-rot-vel:   %f\n", desired_rot_velocity);
    fprintf( stderr, "curr-rot-vel:  %f\n", rwi_base.rot_current_speed);
    fprintf( stderr, "exc-trans-vel: %f\n", ACTUAL_MODE->exception_trans_velocity);
    fprintf( stderr, "exc-trans-acc: %f\n", ACTUAL_MODE->exception_trans_acceleration);
    fprintf( stderr, "trans-vel:     %f\n", rwi_base.trans_set_speed);
    fprintf( stderr, "trans-acc:     %f\n", rwi_base.trans_acceleration);
    fprintf( stderr, "des-trans-vel: %f\n", desired_trans_velocity);
    fprintf( stderr, "curr-tr-vel:   %f\n", rwi_base.trans_current_speed);
    fprintf( stderr, "target_flag:   %d\n", target_flag);

    fprintf( stderr, "halting:       %d\n", haltingFlag);
    fprintf( stderr, "achieve_dist:  %d\n", achieveDistanceFlag);
    fprintf( stderr, "rotate_away:   %d\n", rotatingAwayFlag);
    fprintf( stderr, "rot_wanted:    %d\n", rot_wanted);

  }
  
  if ( mode_number != current_mode) {
    current_mode = mode_number;
    switch (mode_number) {
    case APPROACH_OBJECT_MODE: 
      if (dumpInfo)
	fprintf( dumpFile, "APPROACH\n");
      fprintf(stderr, "USING APPROACH or ESCAPE MODE\n");
      break;
    case APPROACH_TRASH_BIN_MODE: 
      if (dumpInfo)
	fprintf( dumpFile, "APPROACH TRASH BIN\n");
      fprintf(stderr, "USING APPROACH TRASH BIN MODE\n");
      break;
    case ARM_OUT_MODE: 
      if (dumpInfo)
	fprintf( dumpFile, "ARM OUT\n");
      fprintf(stderr, "USING ARM OUT MODE\n");
      break;
    case RANDOM_MODE: 
      if (dumpInfo)
	fprintf( dumpFile, "RANDOM\n");
      fprintf(stderr, "USING RANDOM MOTION MODE\n");
      break;
    case ARM_OUT_RANDOM_MODE: 
      if (dumpInfo)
	fprintf( dumpFile, "ARM OUT RANDOM\n");
      fprintf(stderr, "USING ARM OUT RANDOM MODE\n");
      break;
    case FIND_DOOR_MODE: 
      if (dumpInfo)
	fprintf( dumpFile, "DOOR\n");
      fprintf(stderr, "DOOR MODE\n");
      break;
    case FAST_TRAVEL_MODE: 
      if (dumpInfo)
	fprintf( dumpFile, "FAST\n");
      fprintf(stderr, "USING FAST MODE\n");
      break;
    case SERVO_MODE: 
      if (dumpInfo)
	fprintf( dumpFile, "SERVO\n");
      fprintf(stderr, "USING SERVO MODE\n");
      break;
    case ESCAPE_MODE: 
      if (dumpInfo)
	fprintf( dumpFile, "ESCAPE\n");
      fprintf(stderr, "USING ESCAPE MODE\n");
      break;
    case START_MODE: 
      if (dumpInfo)
	fprintf( dumpFile, "START\n");
      fprintf(stderr, "USING START MODE\n");
      break;
    case DEFAULT_MODE: 
      if (dumpInfo)
	fprintf( dumpFile, "DEF\n");
      fprintf(stderr, "USING DEFAULT MODE\n");
      break;
    default:
      if (dumpInfo)
	fprintf( dumpFile, "UNKNOWN\n");
      fprintf(stderr, "UNKNOWN\n");
    }
  }
}



/**********************************************************************
 * Writes time spent sind begint in the dumpfile.
 **********************************************************************/
void
outputTimeInfo( struct timeval* begint)
{
  if (dumpInfo) {
    struct timeval endt;
    
    gettimeofday( &endt, 0);
    
    endt.tv_usec -= begint->tv_usec;
    endt.tv_sec -= begint->tv_sec;
    if (endt.tv_usec < 0) {
      endt.tv_usec += 1000000;
      endt.tv_sec -= 1;
    }
    
    if ( dumpFile != NULL)
      fprintf( dumpFile, "tv %.2f (%.4f)\n", actual_velocities.current_tvel,
	      (float)endt.tv_sec + (float)endt.tv_usec / 1000000.0);
  }
}


    
/* Checks wether the default mode is set already. If this is the first
 * time after having set the default mode we have to copy the values
 * into the other mode structures.
 */
static void
checkForDefaultMode( BOOLEAN* settingDefaultMode,
		     BOOLEAN* defaultModeAlreadySet)
{ 
    /* We have just read the default mode and can now copy the values to
     * the other modes. */
    if ( *settingDefaultMode) {
      copyDefaultMode();
      *settingDefaultMode = FALSE;
      *defaultModeAlreadySet = TRUE;
      return;
    }
    /* If the default mode is not set in the ini file we use the values
     * from the source code. */
    else if ( ! defaultModeAlreadySet) {
      putc( 7, stderr);
      fprintf( stderr, "Warning: the first mode in the parameter file should ");
      fprintf( stderr, "be the default mode.\n");
      return;
    }
    /* Everything is fine. */
}

/************************************************************************
 *
 *   NAME:         load_parameters
 *                 
 *   FUNCTION:     loads parameters for the different modes from a file.
 *                 The default values are replaced by these values.
 *                 
 ************************************************************************/

BOOLEAN
load_parameters(char *filename)
{
  FILE *iop;
  char line[256];
  char filename2[256];
  char *filename3;
  mode_structure *mode;
  BOOLEAN first_line = TRUE;
  BOOLEAN defaultModeAlreadySet = FALSE;
  BOOLEAN settingDefaultMode = FALSE;

  if ( filename == NULL)
    filename = DEFAULT_INITIALIZATION_FILE;
  
  /* Allocate memory for all modes and set the values to default. */
  setAllModesToDefault();
  
  mode = mode_structure_array[DEFAULT_MODE];

  sprintf(filename2, "etc/%s", filename);

  filename3 = bFindFileM(filename2);
  
  if ((iop = fopen(filename3, "r")) == 0){
    fprintf(stderr, "Could not open input file %s. File not loaded.\n",
            filename3);
    fprintf(stderr, "WARNING: Failed to read file %s.\n", filename);
    fprintf(stderr, "All modes will have the same default values!\n");
    free(filename3);
    return 0;
  }

  while (!feof (iop)) {
    if ( fscanf (iop, "%s", line) == EOF)
       break;

    /****************************************************************************
     * Determine the mode.
     ****************************************************************************/
    if (strcmp(line, "DEFAULT_MODE") == 0) {
      /* The following values will be stored in the structure for exploration.*/
      settingDefaultMode = TRUE;
      first_line = FALSE;
      mode = mode_structure_array[DEFAULT_MODE];
    }
    else if (strcmp(line, "FIND_DOOR_MODE") == 0) {
      /* The following values will be stored in the structure for finding doors.*/
      checkForDefaultMode( &settingDefaultMode, &defaultModeAlreadySet);
      first_line = FALSE;
      mode = mode_structure_array[FIND_DOOR_MODE];
    }
    else if (strcmp(line, "FAST_TRAVEL_MODE") == 0) {
      /* The following values will be stored in the structure for fast travel.*/
      checkForDefaultMode( &settingDefaultMode, &defaultModeAlreadySet);
      mode = mode_structure_array[FAST_TRAVEL_MODE];
      first_line = FALSE;
    }
    else if ( (strcmp(line, "APPROACH_OBJECT_MODE") == 0) || (strcmp(line, "ESCAPE_MODE") == 0))  {
      /* The following values will be stored in the structure for approaching objects.*/
      checkForDefaultMode( &settingDefaultMode, &defaultModeAlreadySet);
      mode = mode_structure_array[APPROACH_OBJECT_MODE];
      first_line = FALSE;
    }
    else if ( strcmp(line, "APPROACH_TRASH_BIN_MODE") == 0)  {
      /* The following values will be stored in the structure for approaching trash bins.*/
	checkForDefaultMode( &settingDefaultMode, &defaultModeAlreadySet);
	mode = mode_structure_array[APPROACH_TRASH_BIN_MODE];
	first_line = FALSE;
    }
    else if (strcmp(line, "ARM_OUT_MODE") == 0) {
      /* ... will be stored in the structure for picking up objects.*/
	checkForDefaultMode( &settingDefaultMode, &defaultModeAlreadySet);
	mode = mode_structure_array[ARM_OUT_MODE];
	first_line = FALSE;
    }
    else if ( strcmp(line, "RANDOM_MODE") == 0) {
      /* ... will be stored in the structure for moving randomly.*/
      checkForDefaultMode( &settingDefaultMode, &defaultModeAlreadySet);
      mode = mode_structure_array[RANDOM_MODE];
      first_line = FALSE;
    }
    else if (strcmp(line, "ARM_OUT_RANDOM_MODE") == 0) {
      /* ... will be stored in the structure for moving randomly.*/
      checkForDefaultMode( &settingDefaultMode, &defaultModeAlreadySet);
      mode = mode_structure_array[ARM_OUT_RANDOM_MODE];
      first_line = FALSE;
    }
    else if (strcmp(line, "SERVO_MODE") == 0) {
      /* ...  will be stored in the structure for visual servoing.*/
	checkForDefaultMode( &settingDefaultMode, &defaultModeAlreadySet);
	mode = mode_structure_array[SERVO_MODE];
	first_line = FALSE;
    }
    else if (strcmp(line, "ESCAPE_MODE") == 0) {
      /* ...  will be stored in the structure for visual escapeing.*/
	checkForDefaultMode( &settingDefaultMode, &defaultModeAlreadySet);
	mode = mode_structure_array[ESCAPE_MODE];
	first_line = FALSE;
    }
    else if (strcmp(line, "START_MODE") == 0) {
      /* ...  will be stored in the structure for visual starting.*/
	checkForDefaultMode( &settingDefaultMode, &defaultModeAlreadySet);
	mode = mode_structure_array[START_MODE];
	first_line = FALSE;
    }
    else if (line[0] ==  '#') {
	fgets( line, sizeof(line),iop);
    }
    else if ( first_line) {
      fprintf(stderr,
	      "ERROR in mode ini file: No mode specified. First line must specify a mode!\n");
    }
    
    /****************************************************************************
     * Read the values for the mode.
     ****************************************************************************/

    /* These two values may only be set once are the same for all modes. */
    else if (strcmp(line, "REMEMBER_INTERVAL") == 0)
      fscanf (iop, "%i", &REMEMBER_INTERVAL);

    /* The interval between two updates of the collision avoidance. */
    else if (strcmp(line, "COLLISION_UPDATE_INTERVAL") == 0)  
	fscanf (iop, "%f", &COLLISION_UPDATE_INTERVAL);

    /* Toggles target tracking with the pantilt*/
    else if (strcmp(line, "TRACK_TARGET_WITH_PANTILT") == 0)  
      TRACK_TARGET_WITH_PANTILT = 1;

    /* This velocity is used whenever the robot has to rotate. */
    else if (strcmp(line, "DEFAULT_ROT_SPEED") == 0) {
      fscanf (iop, "%f", &DEFAULT_ROTATION_SPEED);
      fprintf(stderr, "Set default rotation speed to %f deg/sec.\n",
	      DEFAULT_ROTATION_SPEED);
    }
    /* This velocity is used whenever the robot has to rotate. */
    else if (strcmp(line, "DEFAULT_TRANS_SPEED") == 0) {
      fscanf (iop, "%f", &DEFAULT_TRANSLATION_SPEED);
      fprintf(stderr, "Set default translation speed to %f cm/sec.\n",
	      DEFAULT_TRANSLATION_SPEED);
    }


    /* These Values are set local for each Mode */
    else if (strcmp(line, "SECURITY_ANGLE") == 0)
      fscanf (iop, "%f", &(mode->security_angle));
    else if (strcmp(line, "ROTATE_AWAY_PERSISTENCE") == 0)
       fscanf (iop, "%i", &(mode->rotate_away_persistence));
    

    
    /* The parameters of the evaluation function. */
    else if (strcmp(line, "VELOCITY_FACTOR") == 0) 
    fscanf (iop, "%f", &(mode->velocity_factor)); 
    else if (strcmp(line, "ANGLE_FACTOR") == 0)             
      fscanf (iop, "%f", &(mode->angle_factor));
    else if (strcmp(line, "DISTANCE_FACTOR") == 0)             
      fscanf (iop, "%f", &(mode->distance_factor));

    
    /* Number of velocities for the dynamic window. */
    else if (strcmp(line, "NUMBER_OF_RVELS") == 0)  
      fscanf (iop, "%i", &(mode->number_of_rvels));
    else if (strcmp(line, "NUMBER_OF_TVELS") == 0)  
      fscanf (iop, "%i", &(mode->number_of_tvels));

    /* Velocities and accelerations. */
    else if (strcmp(line, "TARGET_MAX_TRANS_SPEED") == 0)    
      fscanf (iop, "%f", &(mode->target_max_trans_speed));
    else if (strcmp(line, "TARGET_TRANS_ACCELERATION") == 0) 
      fscanf (iop, "%f", &(mode->target_trans_acceleration));
    else if (strcmp(line, "TARGET_MAX_ROT_SPEED") == 0)  {
      fscanf (iop, "%f", &(mode->target_max_rot_speed));
      mode->target_max_rot_speed = DEG_TO_RAD(mode->target_max_rot_speed);
    }
    else if (strcmp(line, "TARGET_ROT_ACCELERATION") == 0)   {
      fscanf (iop, "%f", &(mode->target_rot_acceleration));
      mode->target_rot_acceleration = DEG_TO_RAD(mode->target_rot_acceleration);
    }

    /* Velocities and accelerations if the robot is in an exception. */
    else if (strcmp(line, "EXCEPTION_TRANS_VELOCITY") == 0) 
      fscanf (iop, "%f", &(mode->exception_trans_velocity));
    else if (strcmp(line, "EXCEPTION_TRANS_ACCELERATION") == 0) 
      fscanf (iop, "%f", &(mode->exception_trans_acceleration));
    else if (strcmp(line, "EXCEPTION_ROT_VELOCITY") == 0)  {
      fscanf (iop, "%f", &(mode->exception_rot_velocity));
      mode->exception_rot_velocity = DEG_TO_RAD(mode->exception_rot_velocity);
    }
    else if (strcmp(line, "EXCEPTION_ROT_ACCELERATION") == 0)  {
      fscanf (iop, "%f", &(mode->exception_rot_acceleration));
      mode->exception_rot_acceleration = DEG_TO_RAD(mode->exception_rot_acceleration);
    }

    /* Values concerning trajectories. */
    else if (strcmp(line, "MIN_DIST") == 0)     
      fscanf (iop, "%f", &(mode->min_dist));
    else if (strcmp(line, "SMOOTH_WIDTH") == 0)     
      fscanf (iop, "%d", &(mode->smooth_width));
    else if (strcmp(line, "SECURITY_DIST") == 0)
      fscanf (iop, "%f", &(mode->security_dist));
    else if (strcmp(line, "MIN_DIST_FOR_TARGET_WAY_FREE") == 0)
      fscanf (iop, "%f", &(mode->min_dist_for_target_way_free));
    else if (strcmp(line, "MAX_COLLISION_LINE_LENGTH") == 0)  
      fscanf (iop, "%f", &(mode->max_collision_line_length));
    else if (strcmp(line, "MAX_RANGE") == 0)  
      fscanf (iop, "%f", &(mode->max_range));

    else if (strcmp(line, "EDGE_PORTION") == 0)
      fscanf (iop, "%f", &(mode->edge_portion));
    else if (strcmp(line, "MIN_SECURITY_SPEED") == 0)  
      fscanf (iop, "%f", &(mode->min_security_speed));
    else if (strcmp(line, "MAX_SECURITY_SPEED") == 0)  
      fscanf (iop, "%f", &(mode->max_security_speed));
    else if (strcmp(line, "MAX_SECURITY_DIST") == 0)  
      fscanf (iop, "%f", &(mode->max_security_dist));

    else {
      fprintf( stderr, "Wrong format in initialization file for the different modes:");
      fprintf( stderr, "don't know how to process %s\n", line);
      exit;
    }
  }

  fclose (iop);

  /* If only the default is set we have to copy the values. */
  if ( settingDefaultMode)
    copyDefaultMode();
    
  fprintf(stderr, "Successfully read mode structures from %s.\n",
	  filename3);

  return TRUE;
}


