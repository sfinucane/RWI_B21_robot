
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** File:                        multirobot.cc
 *****
 ***** Part of:                     RHINO SOFTWARE
 *****
 ***** Creator:                     Sascha Ferrein, University of Aachen
 *****
 ***** Date of creation:            Feb 2000
 *****
 *****
 *****
 *****
 ***** $Source: /usr/local/cvs/bee/src/simulator2/multirobot.c,v $
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
 * $Log: multirobot.c,v $
 * Revision 1.1  2002/09/14 16:11:07  rstone
 * *** empty log message ***
 *
 * Revision 1.2  2000/03/18 15:47:41  schulz
 * Changed define for robot radius
 *
 * Revision 1.1  2000/03/09 09:30:08  schulz
 * Added Saschas multi-robot extension
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "tcx.h"
#include "tcxP.h"
#include "simxw.h"
#include "bUtils.h"

#define MAX_ROBOTS 20
#define OTHER_ROBOT_RADIUS bRobot.base_radius
#include "SIMULATOR-messages.h"

/* some globals */
TCX_MODULE_PTR other_simulator_modules[MAX_ROBOTS];
extern char m_robotName[MAX_ROBOTS][80];
extern int NumberOfRobots;
char other_simulator_module_names[MAX_ROBOTS][80];
char *robotColors[MAX_ROBOTS]={"red", "green", "navy blue", "snow", 
  "dark green", "dark olive green", "dark sea green", 
  "SeaGreen", "medium sea green" "light sea green"};

typedef struct  {
  Figure Robot_Figure;
  Figure Robot_Orientation;
  float x;
  float y;
  float rot;
} otherRobot_Figure;
 
otherRobot_Figure OtherRobots[MAX_ROBOTS];

 
/* some extern variables */
extern Widget simCanvas;
extern char robotNameString[80];
extern Boolean displaynames;


/* some extern functions */
extern void new_circle(float , float , float );
extern void RemoveObstacle(float , float );
extern Figure expose_circle(float,float,float, char*);


/* get the ref-number of the other simulator-modules */
int moduleNumber(TCX_REF_PTR ref, TCX_MODULE_PTR *module){
  /* determine the module */
  unsigned int i = 1;
  char found = FALSE;
  while (i <= NumberOfRobots && !found)
    if (ref->module != module[i])
      i++;
    else
      found = TRUE;
  
  if (!found){
    fprintf(stderr, "Error: module not found\n");
    exit(0);
  }
  return i;
}


/* handler-routine for SIMULATOR-status-updates */
void RobotPos_handler(TCX_REF_PTR ref, SIMULATOR_status_update_ptr status)
{
    int mod, i;
    int CurrentModule_Updating=0;
    Figure Robot_Orientation;
    float x2, y2;
    char *currentRobotName;

    /* determine, which simulator updates its coordinates */
    CurrentModule_Updating = moduleNumber(ref, other_simulator_modules);
    
    /* if one robot has new coordinates, remove its old figure */
    if (OtherRobots[CurrentModule_Updating].Robot_Figure != NULL &&
    	OtherRobots[CurrentModule_Updating].x != status->posX &&
    	OtherRobots[CurrentModule_Updating].y != status->posY)      {
	RemoveFigure(OtherRobots[CurrentModule_Updating].Robot_Figure);
	RemoveFigure (OtherRobots[CurrentModule_Updating].Robot_Orientation);
	RemoveObstacle(OtherRobots[CurrentModule_Updating].x, 
		       OtherRobots[CurrentModule_Updating].y);      
      }
    
    /* instert new obstacle (circle) as figure of other robot , 
       only if new position is received */ 
    if (OtherRobots[CurrentModule_Updating].x != status->posX &&
    	OtherRobots[CurrentModule_Updating].y != status->posY) {
      new_circle(status->posX, status->posY, OTHER_ROBOT_RADIUS);
      
      /* Draw circle of robotColors over the grey obstacle */
      OtherRobots[CurrentModule_Updating].Robot_Figure=
	expose_circle(status->posX, status->posY, 
		      OTHER_ROBOT_RADIUS,	      
		      robotColors[CurrentModule_Updating]);

      /* Draw robot's orientation line */
      x2 = status->posX+ OTHER_ROBOT_RADIUS * (float) cos (status->posRot);
      y2 = status->posY+ OTHER_ROBOT_RADIUS * (float) sin (status->posRot);    
      OtherRobots[CurrentModule_Updating].Robot_Orientation = 
	DW_DrawLine(simCanvas, status->posX, status->posY, x2, y2);
      
      /* Update coordinates of current robot */
      OtherRobots[CurrentModule_Updating].x = status->posX;
      OtherRobots[CurrentModule_Updating].y = status->posY;
    }
    
    /* free update-message */
    tcxFree("SIMULATOR_status_update", status);
      
}


/* register RobotPos_handler as handler for coordinate-updates from other 
   simulators */
TCX_REG_HND_TYPE multirobot_handler_array[]={
  {"SIMULATOR_status_update", "RobotPos", RobotPos_handler, TCX_RECV_ALL, NULL}
}; 


/* register SIMULATOR-messages from other simulators */

TCX_REG_MSG_TYPE multirobot_message_array[]={
  SIMULATOR_messages
};


/* show robotFigure infront of robot's name information */
void displayRobotFigure(float x, float y, float rotation,  char *color)
{
  float x2, y2;
  Figure current;
  
  current=expose_circle(x, y, OTHER_ROBOT_RADIUS, color);
  x2 = x+ OTHER_ROBOT_RADIUS * (float) cos (rotation);
  y2 = y+ OTHER_ROBOT_RADIUS * (float) sin (rotation);    
  DW_DrawLine(simCanvas, x, y, x2, y2);
  change_color(current, color);
}


/* scheduler calls this routine to handle other robots */
void multirobotReport()
{
  int i;
  SIMULATOR_register_auto_update_ptr *update_interval;
  char displayText[80];
  Figure displayRobots[MAX_ROBOTS];

  /* update interval for coordinate-updates */
  (int*)update_interval=5;
  
  /* show robot's color and name in simulator */
  if (!displaynames) { 
    fprintf(stderr, "Other Robots are\n");
    fprintf(stderr, "Number\tName\tColor, :\n");

    strcpy(m_robotName[0], robotNameString);

    for (i=0; i <= NumberOfRobots; i++) {
      fprintf(stderr, "%i.\t%s\t(%s)\n", i,m_robotName[i], robotColors[i]);
      sprintf(displayText, "Robot: %s ", m_robotName[i], robotColors[i]);
      displayRobots[i]=DW_DrawText(simCanvas, FALSE, 400, 
				   1800-(100*i), displayText);
      displayRobotFigure(300, 1825-(100*i), 0, robotColors[i]); 
    }
  }

  /* connect other simulators */
  for (i=1; i <= NumberOfRobots; i++) {
    tcxSetModuleName(TCX_SIMULATOR_MODULE_NAME, m_robotName[i], 
		     other_simulator_module_names[i]);
    other_simulator_modules[i]=tcxConnectModule(other_simulator_module_names[i]);
    printf("Connecting to %s\n", tcxModuleName(other_simulator_modules[i])); 
  } 
    
  
  tcxRegisterHandlers(multirobot_handler_array, 
		      sizeof(multirobot_handler_array) / 
		      sizeof(TCX_REG_HND_TYPE));

  /* get coordinate-updates from other simulators */
  for (i=1; i <= NumberOfRobots; i++) {
    tcxSendMsg(other_simulator_modules[i],
	       "SIMULATOR_register_auto_update", &update_interval);
  }
}


















