
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/plan/PLAN.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:47:04 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: PLAN.h,v $
 * Revision 1.1  2002/09/14 15:47:04  rstone
 * *** empty log message ***
 *
 * Revision 1.17  1999/10/23 02:48:54  thrun
 * Various improvements at DARPA demo.
 *
 * Revision 1.16  1999/10/14 17:17:29  thrun
 * tried(!) to fix this no-success problem.
 *
 * Revision 1.15  1999/10/07 23:01:33  thrun
 * central map feature.
 *
 * Revision 1.14  1999/03/13 04:22:50  thrun
 * new parameter "robot_specifications->initial_map_file_name",
 * makes it possible to read maps from file.
 *
 * Revision 1.13  1999/02/07 20:52:11  thrun
 * inroduced subgoals, modified PLAN_action_reply to return entire plan.
 *
 * Revision 1.12  1998/09/01 13:51:22  thrun
 * .
 *
 * Revision 1.11  1998/08/22 01:55:49  thrun
 * Increased efficiency, planner can now wait for initial
 * maps and adpot their size/resolution.
 *
 * Revision 1.10  1998/08/09 04:58:47  thrun
 * Fixed autoshift problem (I hope) + initial entropy planning
 * design.
 *
 * Revision 1.9  1998/04/04 03:09:11  wolfram
 * Added support for multiple robots. Intermediate version, do not use!
 *
 * Revision 1.8  1997/06/27 00:47:18  thrun
 * Excluded speech from the official BeeSoft release - this made it
 * necessary to change some of the files (those that included
 * speech stuff)
 *
 * Revision 1.7  1997/05/28 15:17:07  thrun
 * variable size window
 *
 * Revision 1.6  1997/05/03 14:53:37  thrun
 * Fixed the bug that goal points may be removed accidentally, plus
 * cleaned up the output.
 *
 * Revision 1.5  1997/04/28 17:56:56  thrun
 * Fixed a bug that sometimes made the adjusted action come very
 * close to walls. Also introduced a new parameter that makes plan
 * generate new plans everytime a status report is received from
 * colliServer (generate_actions_continuously).
 *
 * Revision 1.4  1997/04/17 14:54:49  fox
 * Added command line parameter -nomap for usage of LOCALIZE and map.
 *
 * Revision 1.3  1997/02/02 22:32:41  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
 *
 * Revision 1.2  1997/01/30 18:05:12  tyson
 * minor cleanups of abus.  PLAN now gets TCXHOST from beeSoft.ini if not set in envronment
 *
 * Revision 1.1.1.1  1996/09/22 16:46:30  rhino
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







#define max_nNUMBER 1



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/
 
/*---- 'PLAN_debug' prints out messages upon receipt of a TCX message ----*/
#define PLAN_debug 0


#define PLAN_verbose 0

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



#define PLAN_INIT_NAME "plan.ini"

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/




/* GENERAL STATE VARIABLE TYPES */


typedef struct{
  int use_tcx;
  int use_map;
  int tcx_initialized;
  int base_connected;
  int map_connected;
  int speech_connected;
  int use_graphics;
  int graphics_initialized;
  int program_initialized;
  int quit;
  int warmer_colder_game;
  int goal_modus;		/* 0 for exploration, 1 for hunting goals */
  int autonomous;
  int target_not_reached;
  int exploration;
  int actual_map_number;
  int reset_descending_utilities_flag;
  int send_automatic_update;
  int busy;
  int interior_mode;

  char *robot_name;
  
  int   bounding_box_defined;
  float bounding_box_min_x;
  float bounding_box_max_x;
  float bounding_box_min_y;
  float bounding_box_max_y;

} PROGRAM_STATE, *PROGRAM_STATE_PTR;


typedef struct{
  float x;
  float y;
  float orientation;
  int stuck;

  float correction_parameter_x;
  float correction_parameter_y;
  float correction_parameter_angle;
  int correction_type;

  int   map_orientation_defined;
  float map_orientation;

} ROBOT_STATE, *ROBOT_STATE_PTR;




typedef struct{
  float global_mapsize_x;	/* size of the world in inches */
  float global_mapsize_y;	/* size of the world in inches */
  int   global_map_dim_x;	/* dimension of the global map in #fields */
  int   global_map_dim_y;	/* dimension of the global map in #fields */
  float resolution;		/* number of inches per displayed pixel */
  
  float robot_size;		/* size of the robot in inches */
  float max_travel_distance[7]; /* vector of max distances given a speed */
  float max_security_dist;

  int   wait_for_initial_map;	/* if 1, planner will wait for initial map */
  int   initial_map_received;	/* set to 1 if initial map received */
  char *initial_map_file_name;	/* file name for initial map */
  int   wait_for_initial_corr;	/* if 1, planner will wait for initial map */
  int   initial_corr_received;	/* set to 1 if initial map received */

  int   autoshift;		/* 1, if autoshift of the map defined, 0 else*/
  float autoshift_distance;	/* amount we shift in autoshift */
  float autoshift_safety_margin;/* minimum distance to walls */
  float autoshifted_x;		/* inches the display has been shifted */
  float autoshifted_y;		/* inches the display has been shifted */
  int   autoshifted_int_x;	/* #field-units the display has been shifted */
  int   autoshifted_int_y;	/* #field-units the display has been shifted */
  int   autoshift_for_maps;	/* if 1, will autoshift when map comes */
  int   autoshift_for_rob_pos;	/* if 1, will autoshift when rob pos. changes*/

  int   min_display_index_x;
  int   max_display_index_x;
  int   min_display_index_y;
  int   max_display_index_y;

  float average_value;
  float average_costs;
  float collision_threshold;
  float average_costs2;
  float costs_exponent;
  float min_base;
  float max_base;
  float max_adjust_angle;
  float max_goal_distance;
  float max_approach_distance;
  float max_final_approach_distance;

  float intermediate_point_dist; /* distance between intermediate points
				  * of a plan*/

  float min_costs;
  float max_costs;

  int   min_plan_index_x;
  int   max_plan_index_x;
  int   min_plan_index_y;
  int   max_plan_index_y;
  int   dp_direction;
  int   number_active_goals[max_nNUMBER];

  int   map_update_frequency;

  float exploration_circle_size; /* size which will be marked as explored, 
				    when the robot moves */

  float min_mapvalue_along_path; /* if the planner generates a path, it
				  * will clip the map values along that
				  * path */


  float border_to_interior;	/* in cm: border to the interior of
				 * a map */
  float exterior_factor;	/* amplifier for the unexplored inside
				 * the map */
  float max_bounding_box_size;	/* from that size on we start exploration
				 * of the interior */

  int fast_exploration;		/* if 1, the robot will atempt to explore
				 * without extensive planning */

  int generate_actions_continuously; /* if 1, new Colli-subgoals will be 
				  * generate continuously */
  float X_window_size;		/* scale factor for the X-window */

  int use_central_map;
  float map_growing_radius;

} ROBOT_SPECIFICATIONS, *ROBOT_SPECIFICATIONS_PTR;



typedef struct{
  float base;
  float turn;
  float absolute_x;
  float absolute_y;
  float potential_next_base;
  float potential_next_turn;
  float potential_next_absolute_x;
  float potential_next_absolute_y;
  int success;
  int no_plan;
  int active_goal_name;
  float goal_x, goal_y;
  int final_action;
  float goal_dist;
} ACTION_TYPE, *ACTION_PTR;



typedef struct{
  ROBOT_STATE_PTR          robot_state;
  PROGRAM_STATE_PTR        program_state;
  ROBOT_SPECIFICATIONS_PTR robot_specifications;
  ACTION_PTR               action;
} ALL_TYPE, *ALL_PTR;


extern ALL_PTR all;			/* this is now global */


/*********************************************************************/

extern int     nNUMBER;
extern int     NUMBER;
extern int BEST_NUMBER;

/*********************************************************************/

typedef float probability;
typedef float mapProbability;
typedef struct{
  float x;
  float y;
} point,position;

typedef struct {
    float x;
    float y;
    float rot;
} realPosition;
typedef unsigned char pixel;

typedef struct {
  char initialized;
  int sizeX;
  int sizeY;
  int origsizeX;
  int origsizeY;
  int shiftedX;
  int shiftedY;
  int resolution;
  float offsetX;
  float offsetY;
  float maxRealX;
  float maxRealY;
  mapProbability average;
  mapProbability unknown;
  mapProbability** prob;
  position* freeSpace;
  int numberOfFreeCells;
} probabilityGrid;

char
readGridMap(char *mapName, probabilityGrid *m);

char
sendMapReplyType( probabilityGrid* map,
		  char inverted,
		  char* name, int type);

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* CONSTRAINTS */

#define max_n_constraints 200

extern PLAN_constraints_message_type constraints[max_n_constraints];

extern int n_constraints;


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

extern struct timeval TCX_waiting_time;

extern struct timeval block_waiting_time;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* BUTTONS and other graphic objects */


extern int GLOBAL_BACKGROUND;
extern int MAPVALUES;
extern int COSTS;
extern int ENTROPY;
extern int UTILITY;
extern int DISPLAY_MAPVALUES_BUTTON;
extern int DISPLAY_COSTS_BUTTON;
extern int DISPLAY_ENTROPY_BUTTON;
extern int DISPLAY_UTILITY_BUTTON;
extern int ROBOT;
extern int ADJUSTED_ACTION;
extern int GOALS[max_nNUMBER];
extern int PLAN_DISPLAY;
extern int PLAN_MARKERS;
extern int PLAN_BOX;
extern int PLAN_BUTTON;
extern int CONSTRAINTS;
extern int QUIT_BUTTON;
extern int BASE_CONNECTED_BUTTON;
extern int MAP_CONNECTED_BUTTON;
extern int AUTONOMOUS_BUTTON;
extern int EXPLORATION_TYPE_BUTTON;
extern int BOUNDING_BOX;
extern int EXPLORATION_INTERIOR_MODE_BUTTON;


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



/* MAPS */


extern int maps_allocated;

extern float *global_values;
extern int   *global_active;
extern int   *global_explored;
extern int   *global_visited;
extern int   *global_succ;
extern float *global_costs;
extern float *global_entropy;
extern float *global_costs2;
extern int   *global_goal;
extern float *global_utility;
/*extern int   *global_interior;*/

extern int   *global_goal_table[max_nNUMBER];
extern float *global_utility_table[max_nNUMBER];
extern int   *global_succ_table[max_nNUMBER];



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

extern struct bParamList * bParamList;
extern const char *bRobotType;



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void count_auto_update_modules();

static int add_auto_update_module(TCX_MODULE_PTR module, int status);

static int remove_auto_update_module(TCX_MODULE_PTR module);

void send_automatic_status_update(TCX_MODULE_PTR recipient);

void PLAN_quit_message_handler(TCX_REF_PTR   ref,
			       void         *msg);

void PLAN_start_autonomous_message_handler(TCX_REF_PTR   ref,
					   int          *expl);

void PLAN_stop_autonomous_message_handler(TCX_REF_PTR   ref,
					  int          *stop_base);

void PLAN_goal_message_handler(TCX_REF_PTR   ref,
		       PLAN_goal_message_ptr goal_msg);

void PLAN_new_robot_pos_message_handler(TCX_REF_PTR                 ref,
					PLAN_new_robot_pos_message_ptr
					robot_msg);

void PLAN_action_query_handler(TCX_REF_PTR   ref,
			       PLAN_action_query_ptr question_msg);

void MAP_partial_map_reply_handler(TCX_REF_PTR                 ref,
				   MAP_partial_map_reply_ptr map_msg);

void MAP_correction_parameters_reply_handler(TCX_REF_PTR                 ref,
				     MAP_correction_parameters_reply_ptr corr);

void BASE_update_status_reply_handler(TCX_REF_PTR                   ref,
				      BASE_update_status_reply_ptr status);

void BASE_action_executed_reply_handler(TCX_REF_PTR                    ref,
					BASE_action_executed_reply_ptr pos);

void BASE_robot_position_reply_handler(TCX_REF_PTR                   ref,
				       BASE_robot_position_reply_ptr pos);

void SONAR_sonar_reply_handler(TCX_REF_PTR                ref,
			       SONAR_sonar_reply_ptr      data);

void PLAN_constraints_message_handler(TCX_REF_PTR   ref,
				      PLAN_constraints_message_ptr 
				      new_constraint);

void init_tcx(PROGRAM_STATE_PTR  program_state);

void connect_to_MAP(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		    PROGRAM_STATE_PTR program_state);


void connect_to_BASE(PROGRAM_STATE_PTR program_state);

void tcx_base_goto_relative(PROGRAM_STATE_PTR program_state,
			    ROBOT_SPECIFICATIONS_PTR robot_specifications,
			    ROBOT_STATE_PTR robot_state,
			    ACTION_PTR action);

void tcx_base_stop(PROGRAM_STATE_PTR program_state);

void tcx_base_goto_absolute(PROGRAM_STATE_PTR program_state,
			    ROBOT_SPECIFICATIONS_PTR robot_specifications,
			    ROBOT_STATE_PTR robot_state,
			    ACTION_PTR action,
			    int new_target);

void PLAN_close_handler(char *name, TCX_MODULE_PTR module);

void init_program(PROGRAM_STATE_PTR        program_state,
		  ROBOT_SPECIFICATIONS_PTR robot_specifications,
		  ROBOT_STATE_PTR          robot_state,
		  ACTION_PTR               action,
		  ALL_PTR                  all);

void allocate_maps(ROBOT_STATE_PTR          robot_state,
		   PROGRAM_STATE_PTR program_state, 
		   ROBOT_SPECIFICATIONS_PTR robot_specifications);

void check_commandline_parameters(int argc, char **argv, 
				  PROGRAM_STATE_PTR program_state, 
				  ROBOT_SPECIFICATIONS_PTR 
				  robot_specifications);

void init_graphics(ROBOT_STATE_PTR    robot_state,
		   PROGRAM_STATE_PTR  program_state,
		   ROBOT_SPECIFICATIONS_PTR robot_specifications);

void display_all(int name,
		 ROBOT_STATE_PTR robot_state,
		 ROBOT_SPECIFICATIONS_PTR robot_specifications, 
		 PROGRAM_STATE_PTR program_state);

void display_robot(ROBOT_STATE_PTR robot_state, 
		   PROGRAM_STATE_PTR program_state);

void get_global_range(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		 float *global_array,
		 int   *global_active,
		 float *min_value, 
		 float *max_value);

void check_index(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	    int copy_flag);

int search_for_inconsistencies_in_succ(ROBOT_SPECIFICATIONS_PTR 
				       robot_specifications,
				       char *text);

void reset_descending_utilities(ROBOT_SPECIFICATIONS_PTR robot_specifications,
				PROGRAM_STATE_PTR program_state,
				ROBOT_STATE_PTR robot_state);


void autoshift_display_by(ROBOT_STATE_PTR robot_state,
			  PROGRAM_STATE_PTR  program_state,
			  ROBOT_SPECIFICATIONS_PTR robot_specifications,
			  int shift_x, 
			  int shift_y);

void autoshift_display(float point_x, 
		       float point_y,
		       ROBOT_STATE_PTR robot_state,
		       PROGRAM_STATE_PTR  program_state,
		       ROBOT_SPECIFICATIONS_PTR robot_specifications);

void fake_map_update(ROBOT_STATE_PTR robot_state,
		     PROGRAM_STATE_PTR  program_state,
		     ROBOT_SPECIFICATIONS_PTR robot_specifications,
		     float robot_x, float robot_y);

void count_goals_and_reset_utilities(PROGRAM_STATE_PTR  program_state,
				     ROBOT_SPECIFICATIONS_PTR 
				     robot_specifications,
				     ROBOT_STATE_PTR robot_state);

 
int find_nearest_goal(float robot_x, float robot_y, float *dist, int *name,
		      float *goal_x, float *goal_y);

int check_if_goal_reached(PROGRAM_STATE_PTR program_state,
			  ROBOT_STATE_PTR robot_state,
			  ROBOT_SPECIFICATIONS_PTR robot_specifications,
			  float robot_x, float robot_y, 
			  float min_dist);

  
int modify_goal_set(PROGRAM_STATE_PTR program_state,
		    ROBOT_STATE_PTR robot_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications,
		    float x,
		    float y,
		    int add,	/* 1=set, 0=remove */
		    int name);

float costs_function(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		     float value, float entropy);

float utility_function(float costs, int goal);

void dynamic_programming(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			 PROGRAM_STATE_PTR program_state,
			 ROBOT_STATE_PTR robot_state);

void display_plan_box(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		      PROGRAM_STATE_PTR program_state);

void compute_plan(ROBOT_STATE_PTR robot_state,
		  ROBOT_SPECIFICATIONS_PTR robot_specifications,
		  ACTION_PTR  action,
		  PROGRAM_STATE_PTR program_state,
		  int display_plan, int iteration);

void adjust_action(ROBOT_STATE_PTR robot_state,
		   ROBOT_SPECIFICATIONS_PTR robot_specifications,
		   ACTION_PTR  action,
		   PROGRAM_STATE_PTR program_state,
		   int display_action);

void
compute_plan_intermediate_points(ROBOT_STATE_PTR robot_state,
				 ROBOT_SPECIFICATIONS_PTR robot_specifications);


int check_if_point_reachable_by_straight_line(float target_x, float target_y,
					      ROBOT_STATE_PTR robot_state,
					      ROBOT_SPECIFICATIONS_PTR robot_specifications);

void generate_action(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		     PROGRAM_STATE_PTR program_state,
		     ROBOT_STATE_PTR robot_state,
		     ACTION_PTR action,
		     int plan_till_success,
		     int display_plan);

int add_exploration_goal(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			  PROGRAM_STATE_PTR program_state,
			  ROBOT_STATE_PTR robot_state);

int check_if_point_free(float point_x, float point_y,
			ROBOT_SPECIFICATIONS_PTR robot_specifications);

void compute_forward_correction(float robot_x, 
				float robot_y, 
				float robot_orientation,
				float corr_x, 
				float corr_y, 
				float corr_angle, /* in deg. */
				int   corr_type,
				float *corr_robot_x,
				float *corr_robot_y, 
				float *corr_orientation);

void compute_backward_correction(float corr_robot_x, 
				 float corr_robot_y, 
				 float corr_robot_orientation,
				 float corr_x, 
				 float corr_y, 
				 float corr_angle, /* in deg. */
				 int   corr_type,
				 float *robot_x,
				 float *robot_y, 
				 float *orientation);

void clear_maps(ROBOT_SPECIFICATIONS_PTR robot_specifications);

void change_table(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		  int NEW_NUMBER,
		  PROGRAM_STATE_PTR program_state);

float evaluate_constraints(ROBOT_SPECIFICATIONS_PTR robot_specifications);

void create_new_goal_point(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		      float *x, 
		      float *y);

int search_for_goal_points(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			    PROGRAM_STATE_PTR program_state,
			    ROBOT_STATE_PTR robot_state,
			    ACTION_PTR action);

int mouse_test_loop(ROBOT_STATE_PTR          robot_state,
		    PROGRAM_STATE_PTR        program_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications,
		    ACTION_PTR               action);

void PLAN_register_auto_update_handler(TCX_REF_PTR  ref,
				      PLAN_register_auto_update_ptr data);

void remove_all_goals(PROGRAM_STATE_PTR program_state,
		      ROBOT_STATE_PTR robot_state,
		      ROBOT_SPECIFICATIONS_PTR robot_specifications);

void reset_internal_exploration_table(PROGRAM_STATE_PTR program_state, 
				      ROBOT_SPECIFICATIONS_PTR 
				      robot_specifications);

void compute_interior(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		      PROGRAM_STATE_PTR program_state);

int check_if_inside_interior(int x, int y, 
			     ROBOT_SPECIFICATIONS_PTR robot_specifications,
			     ROBOT_STATE_PTR robot_state,
			     PROGRAM_STATE_PTR program_state);

float exploration_utility_factor(int x, int y, 
				 ROBOT_SPECIFICATIONS_PTR robot_specifications,
				 ROBOT_STATE_PTR robot_state,
				 PROGRAM_STATE_PTR program_state);

#ifdef UNIBONN
void tcx_speech_talk_text(PROGRAM_STATE_PTR program_state, char *text);
#endif

void connect_to_SPEECH(PROGRAM_STATE_PTR program_state);

void PLAN_reset_exploration_table_handler(TCX_REF_PTR   ref,
					  void         *msg);

int refresh_goal_points(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			    PROGRAM_STATE_PTR program_state,
			    ROBOT_STATE_PTR robot_state);


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/
