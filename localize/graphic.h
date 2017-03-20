
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/graphic.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:39 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: graphic.h,v $
 * Revision 1.1  2002/09/14 20:45:39  rstone
 * *** empty log message ***
 *
 * Revision 1.39  2000/01/02 15:33:15  fox
 * Should work.
 *
 * Revision 1.38  1998/11/24 23:05:26  fox
 * Implemented furhter routines for condensation and vision.
 *
 * Revision 1.37  1998/11/19 03:14:26  fox
 * Newest version with integrated mapping and localization. If MAP is connected,
 * then the correction parameters are NOT sent to PLAN.
 * Furthermore, the size of the onlinemap adapts to the partial maps sent by MAP.
 *
 * Revision 1.36  1998/11/17 23:26:20  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.35  1998/09/25 04:02:55  fox
 * First version of CONDENSATION!!!!
 *
 * Revision 1.34  1998/08/20 00:22:58  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.33  1998/01/22 13:06:15  fox
 * First version after selection-submission.
 *
 * Revision 1.32  1997/12/02 15:20:38  fox
 * Nothing remarkable.
 *
 * Revision 1.31  1997/11/21 15:36:04  fox
 * Modifications in graphic
 *
 * Revision 1.30  1997/11/07 12:39:39  fox
 * Added some graphic features.
 *
 * Revision 1.29  1997/10/28 12:34:19  wolfram
 * Removed unused variables and parameters
 *
 * Revision 1.28  1997/09/26 17:02:09  wolfram
 * Intermediate version, do not use
 *
 * Revision 1.27  1997/09/25 16:37:17  wolfram
 * Fixed a bug in setMaxFactor
 *
 * Revision 1.26  1997/08/08 19:52:13  wolfram
 * Robot window now displays the simulator map as seen from the lasers.
 * Simulator map can be displayed at a certain ZRange.
 *
 * Revision 1.25  1997/08/02 16:51:03  wolfram
 * 1. Changed the order of indexes of the grid (and only of the grid):
 * The order now is grid->prob[rot][x][y].  This results in a significant
 * speed-up for different operations such as normalization and
 * integration of new sensory data.  Reimplemented the ConvolveThirdDim
 * procedure for convolving over rot.
 *
 * 2. Changed the algorithm to detect linear alignments of readings.  Now
 * we use the approach of Lu.
 *
 * 3. Linear alignments of readings is also checked for laser readings.
 *
 * 4. Expected distances are now computed given the simulator map if
 * available.  For that purpose the library libGetDistance is included.
 *
 * 5. Graphic output now displays the simulator map (if available). This
 * concernes the map overlay as well as the robot window.
 *
 * 6. Fixed some minor bugs.
 *
 * 7. Added different parameters to the ini-file (see example.ini).
 *
 * Revision 1.24  1997/06/25 14:16:39  fox
 * Changed laser incorporation.
 *
 * Revision 1.23  1997/04/27 15:48:20  wolfram
 * Changes in script.c
 *
 * Revision 1.22  1997/04/08 14:56:23  fox
 * Changed the reset function for new planes and the prob function.
 *
 * Revision 1.21  1997/03/24 06:55:28  wolfram
 * Cleaned up graphic.c and added a graphic window as a global variable
 * in graphic.c
 *
 * Revision 1.20  1997/03/14 17:58:19  fox
 * This version should run quite stable now.
 *
 * Revision 1.19  1997/03/14 11:20:54  wolfram
 * Added windows for different maps
 *
 * Revision 1.18  1997/03/13 17:36:36  fox
 * Temporary version. Don't use!
 *
 * Revision 1.17  1997/01/29 12:23:08  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.16  1997/01/19 19:31:16  fox
 * yeah
 *
 * Revision 1.15  1997/01/17 13:21:06  fox
 * Added timer for non relevant functions.
 *
 * Revision 1.14  1997/01/16 12:42:49  fox
 * Fixed a bug in scan masks.
 *
 * Revision 1.13  1997/01/14 16:53:23  fox
 * Removed a very very nasty bug for sensor selection.
 *
 * Revision 1.12  1997/01/07 13:14:28  wolfram
 * Movement decisions are made on the basis of a list of real positions
 *
 * Revision 1.11  1996/12/02 10:32:05  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.10  1996/11/29 15:33:38  fox
 * ok
 *
 * Revision 1.9  1996/11/27 15:51:17  fox
 * For Woflram.
 *
 * Revision 1.8  1996/11/27 12:18:18  fox
 * Nothing special.
 *
 * Revision 1.7  1996/11/26 14:00:46  wolfram
 * position of grid window is now computet correctly given a real position
 *
 * Revision 1.6  1996/11/26 11:08:12  fox
 * Improved version.
 *
 * Revision 1.5  1996/11/25 09:47:24  wolfram
 * Added graphic support for movment errors
 * Improved procedure for computing the entropy of a grid
 *
 * Revision 1.4  1996/11/22 10:30:24  wolfram
 * Added graphic for expected errors of movements
 *
 * Revision 1.3  1996/11/15 17:44:06  ws
 * *** empty log message ***
 *
 * Revision 1.2  1996/10/25 13:05:22  ws
 * Added MIN_WINDOW_SCALE keyword for map and grid windows
 *
 * Revision 1.1.1.1  1996/09/22 16:46:32  rhino
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


#ifndef GRAPHIC_INCLUDE
#define GRAPHIC_INCLUDE

#include "EZX11.h"

#include "sensings.h"
#include "general.h"
#include "sonar.h"
#include "movement.h"
#include "angle.h"
#include "probGridTools.h"

#define OVERLAY_VALUE      1
#define NON_OVERLAY_VALUE  0

/**************************************************************************
 **************************************************************************
 * Structures.
 **************************************************************************
 **************************************************************************/

typedef struct {
  EZXW_p window;
  int startX;
  int startY;
  int sizeX;
  int sizeY;
  int gridSizeX;
  int gridSizeY;
  int gridOffsetX;
  int gridOffsetY;
  int gridResolution;
  int scale;
  int planeType;
  bool firstTimePosition;
  realPosition pos;
}  gridWindow;

typedef struct {
  EZXW_p window;

  /* Size in pixels. */
  int sizeX;
  int sizeY;

  /* Global x and y coordinate of the lower left corner. */
  float minX; 
  float minY;

  /* Scale between cm and pixels. */
  float cmPerPixel;
}  positionWindow;


typedef struct {
  bool useGraphic;
  gridWindow* mapWindow;
  gridWindow* sonarMapWindow;
  gridWindow* laserMapWindow;
  gridWindow* initialPositionProbsWindow;
  gridWindow* robotWindow;
  gridWindow* xyMaxWindow;
  gridWindow* angleWindow;
  gridWindow* planeWindow;
  gridWindow* visionMapWindow;
  positionWindow* sampleWindow;
  angleProbTable* angleProbs;
  bool showRobotInMap;
  robot robotInMap;
  int showMap;
  int showSonarMap;
  int showLaserMap;
  int showInitialPositionProbs;
  int showAngles;
  int showRobotZoom;
  bool showSelectedSensings;
  int createMapOverlay;
  int minWindowScale;  
  int robotZoomScale;
  int displaySkip;
  int dumpRobWin;
  int dumpXYMax;
  int showExpectedDistances;
} informationsFor_GRAPHIC;


/**************************************************************************
 **************************************************************************
 * Other functions.
 **************************************************************************
 **************************************************************************/


void
updateGlobalRobotWindow( actionInformation* actionInfo,
			 sensingActionMask* mask);


void
initialize_GRAPHICS( char* fileName,
		     actionInformation* actionInfo,
		     sensingActionMask* actionMask,
		     informationsFor_GRAPHIC* graphicInfo);

void
accumulateMovements_GRAPHICS( movement delta,
			      informationsFor_GRAPHIC* graphicInfo);

void
update_SENSING_GRAPHICS(  actionInformation* actionInfo,
			  sensingActionMask* mask,
			  informationsFor_GRAPHIC* graphicInfo);

void
update_GRAPHICS( actionInformation* actionInfo,
		 sensingActionMask* mask,
		 informationsFor_GRAPHIC* graphicInfo);

void
checkPosition( actionInformation* info, sensingActionMask* mask);


gridWindow *
createVisionMapWindow(visionMap *m, char* text, int x, int y, int minScale);

void
displayVisionMapWindow(visionMap *m, gridWindow *mapwin);

gridWindow *
createMapWindow(probabilityGrid *m, char* text, int x, int y, int scale);

void
displayMapWindow(probabilityGrid *m, gridWindow *mapwin);

void
displayMapWindowMax(probabilityGrid *m, gridWindow *mapwin);

void
destroyMapWindow(gridWindow *mapwin);

realPosition
positionRobotinWindow(int mode, gridWindow mapwindow,
		      probabilityGrid *map,
		      simulatorMap *simMap,
		      robot rob, int x, int y,
		      int destroyPositionTool);

gridWindow *
createGridWindow(positionProbabilityGrid *m, int planeType, 
		 char *title, int x, int y, int minScale);
    

positionWindow *
createPositionWindow( sampleSet* samples, char *title,
		      int x, int y, float maxCmPerPixel);

void
displayGridCellList( gridWindow *win, gridCellList *maxima);

void
displayRealCellList( gridWindow *win, realCellList *maxima);

gridPosition
displayXYMax( positionProbabilityGrid *m,
	      probabilityGrid *map,
	      simulatorMap *simMap,
	      gridWindow *mapwin,
	      bool createMapOverlay,
	      float distanceTraveled);

void
clearMapWindow(gridWindow *win);

void
displaySamples();

void
displayPositions( sampleSet* samples,
		  probabilityGrid *map,
		  visionMap* visMap,
		  positionWindow *mapwin,
		  bool createMapOverlay,
		  float distanceTraveled);

void
dumpXYMaxGraphic( positionProbabilityGrid *m,
		  probabilityGrid *map,
		  realPosition scriptPos,
		  realCellList* localMaxima);

void
displayGridPosition( gridWindow* win,
		     positionProbabilityGrid* grid,
		     float radius,
		     gridPosition pos);

void
displayRobot(gridWindow* win, robot rob, int col, bool fillRobot);

gridWindow *
createRobotWindow( probabilityGrid *map, char* text,
		   int size, int x, int y, int scale);

void
displayRobotWindow( gridWindow* win,
		    robot robotToDisplay,
		    actionInformation* actionInfo,
		    sensingActionMask* actionMask,
		    informationsFor_GRAPHIC* graphicInfo);

gridWindow *
createAngleWindow(angleProbTable *grid, char* text, int x, int y);

void
displayAngleWindow(angleProbTable *grid, gridWindow *anglewin,
		   double referenceValue);


void
showAllColors();

void
displayMovementField( gridWindow *mapwin,
		      probabilityGrid *m,
		      realCellList* cellList);


realPosition
measuredRobotPositionInMap();

#define SONAR_TAG 0
#define LASER_TAG 1

#endif



