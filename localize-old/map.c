
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/map.c,v $
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
 * $Log: map.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.87  2000/03/06 20:00:45  wolfram
 * Changed the code of the vision routines.
 *
 * Revision 1.86  2000/01/26 22:56:43  fox
 * Might work. If anything weird happens, contact me.
 *
 * Revision 1.85  2000/01/10 19:04:21  fox
 * DON'T USE!
 *
 * Revision 1.84  2000/01/02 15:33:15  fox
 * Should work.
 *
 * Revision 1.83  1999/12/28 17:19:24  fox
 * Changed computation of free space for samples. Define OBSTACLE_DISTANCE
 * sets the minimum distance to the next obstacle.
 *
 * Revision 1.82  1999/12/16 16:13:59  fox
 * Several preparation changes for angles.
 *
 * Revision 1.81  1999/12/15 16:16:40  fox
 * First attempt to extract p(l | o).
 *
 * Revision 1.80  1999/12/10 15:20:16  wolfram
 * Improved version for on-line computation of expected distances
 *
 * Revision 1.79  1999/11/02 18:12:35  fox
 * Version of multi localize journal paper.
 *
 * Revision 1.78  1999/09/30 11:50:39  wolfram
 * Once again removed the libGetDistance (and I won't include it again)
 *
 * Revision 1.77  1999/09/29 16:06:11  fox
 * Should work.
 *
 * Revision 1.76  1999/09/26 21:20:58  fox
 * Nothing special.
 *
 * Revision 1.75  1999/06/23 16:22:02  fox
 * Added robot type urban.
 *
 * Revision 1.74  1999/04/29 17:24:38  fox
 * Removed the EDIT_MAP stuff.
 *
 * Revision 1.73  1999/04/29 17:21:37  fox
 * Added feature to edit a map when setting SET_ROBOT_POSITION. At each
 * mouse press the corresponding pixel will be set to free space. When done,
 * the map is stored in testmap. Define EDIT_MAP.
 *
 * Revision 1.72  1999/03/19 16:04:40  wolfram
 * Added REAL_TIME simulation in script.c
 *
 * Revision 1.71  1999/03/12 00:41:49  fox
 * Minor changes.
 *
 * Revision 1.70  1999/01/22 17:48:06  fox
 * Final version of sampling paper and denver demo.
 *
 * Revision 1.69  1999/01/14 23:39:31  wolfram
 * Added sensorMaxRange and standardDeviaton to the sensor model for lasers
 *
 * Revision 1.68  1999/01/14 00:33:01  wolfram
 * Changes for vision
 *
 * Revision 1.67  1999/01/11 19:47:52  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.65  1999/01/07 01:07:09  wolfram
 * Changes to integrate scan matching
 *
 * Revision 1.64  1998/12/16 14:59:03  wolfram
 * First version without libGetDistance. Use with caution.
 *
 * Revision 1.63  1998/12/16 08:51:49  wolfram
 * This version does not need the libGetDistance to compute the expected Distances
 *
 * Revision 1.62  1998/12/10 17:56:07  fox
 * Fixed a bug in displayPositions.
 *
 * Revision 1.61  1998/11/27 09:12:03  wolfram
 * Some changes for Frank
 *
 * Revision 1.60  1998/11/25 16:29:40  wolfram
 * Added higher resolution for vision maps. resolution is now read from file.
 * Couldn't integrate it consistently into graphic.c.
 *
 * Revision 1.59  1998/11/24 23:05:27  fox
 * Implemented furhter routines for condensation and vision.
 *
 * Revision 1.58  1998/11/23 21:19:26  fox
 * Fixed some minor bugs.
 *
 * Revision 1.57  1998/11/23 19:45:09  fox
 * Latest version.
 *
 * Revision 1.56  1998/11/19 03:14:27  fox
 * Newest version with integrated mapping and localization. If MAP is connected,
 * then the correction parameters are NOT sent to PLAN.
 * Furthermore, the size of the onlinemap adapts to the partial maps sent by MAP.
 *
 * Revision 1.55  1998/11/17 23:26:22  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.54  1998/09/18 15:44:27  fox
 * First attempt to get a running system after museum.
 *
 * Revision 1.53  1998/09/05 22:06:56  wolfram
 * Changes regarding vision!
 *
 * Revision 1.52  1998/08/31 22:29:21  wolfram
 * Several changes
 *
 * Revision 1.51  1998/08/24 07:39:50  wolfram
 * final version for Washington
 *
 * Revision 1.50  1998/08/23 00:01:01  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.49  1998/08/20 00:23:00  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.48  1998/06/12 10:16:33  fox
 * Implemented virutal sensor.
 *
 * Revision 1.47  1998/04/19 10:40:36  wolfram
 * Added graphical display to multi localization
 *
 * Revision 1.46  1998/04/06 19:44:13  wolfram
 * Added -robot option for multi-robot support
 *
 * Revision 1.45  1998/03/09 10:07:45  wolfram
 * slight changes
 *
 * Revision 1.44  1998/03/09 09:36:27  wolfram
 * LOCALIZE now checks the consistency of the various maps.
 *
 * Revision 1.43  1998/02/13 14:12:22  fox
 * Minor changes.
 *
 * Revision 1.42  1997/11/23 15:50:18  wolfram
 * Changes because of robotDump
 *
 * Revision 1.41  1997/10/31 15:27:20  wolfram
 * Offset of grid maps is set to zero
 *
 * Revision 1.40  1997/10/07 16:39:51  wolfram
 * Fixed a bug in computeGridMap
 *
 * Revision 1.39  1997/10/05 15:37:01  wolfram
 * Fixed a bug in computing grid maps of simulator maps.
 *
 * Revision 1.38  1997/09/29 10:45:24  wolfram
 * Added default values for all values in .ini file
 *
 * Revision 1.37  1997/09/26 17:02:10  wolfram
 * Intermediate version, do not use
 *
 * Revision 1.36  1997/08/22 02:14:55  wolfram
 * If a beam reaches an unknown field we assume that maxRange is measured
 *
 * Revision 1.35  1997/08/16 22:59:50  fox
 * Last version before I change selsection.
 *
 * Revision 1.34  1997/08/16 21:52:02  wolfram
 * Grid maps can now be cad maps.  Obstacles are enlarged accordingly
 *
 * Revision 1.33  1997/08/16 18:16:53  wolfram
 * Planmap and initprobs are now saved in every case
 *
 * Revision 1.32  1997/08/14 23:29:42  wolfram
 * Fixed a bug in writeMap
 *
 * Revision 1.31  1997/08/08 19:52:13  wolfram
 * Robot window now displays the simulator map as seen from the lasers.
 * Simulator map can be displayed at a certain ZRange.
 *
 * Revision 1.30  1997/08/02 16:51:04  wolfram
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
 * Revision 1.29  1997/04/24 21:25:43  wolfram
 * Ooops, fourth parameter of CUBE's is radius and not diameter!
 *
 * Revision 1.28  1997/04/10 13:01:03  fox
 * Fixed a bug.
 *
 * Revision 1.27  1997/03/31 21:18:14  wolfram
 * Localize now reads cylinders from the simulator map, small bug fix in graphic.c
 *
 * Revision 1.26  1997/03/17 18:41:14  fox
 * First version with sensor selection in cluttered environments.
 *
 * Revision 1.25  1997/03/14 17:58:20  fox
 * This version should run quite stable now.
 *
 * Revision 1.24  1997/03/13 17:36:21  fox
 * Temporary version. Don't use!
 *
 * Revision 1.23  1997/03/03 12:59:22  wolfram
 * Initial position probabilities are computed out of the simulator map if
 * the simulator map is available
 *
 * Revision 1.22  1997/02/11 11:04:10  wolfram
 * Faster Computation of expected distances, height of sensors is considered
 *
 * Revision 1.21  1997/01/30 17:17:25  fox
 * New version with integrated laser.
 *
 * Revision 1.20  1997/01/29 12:23:09  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.19  1997/01/24 17:27:40  wolfram
 * Added additional object size to readSimulatorMap
 *
 * Revision 1.18  1997/01/14 16:53:23  fox
 * Removed a very very nasty bug for sensor selection.
 *
 * Revision 1.17  1997/01/08 15:52:56  fox
 * Rotations are not any longer treated like movements.
 * Send parameters to PLAN.
 *
 * Revision 1.16  1996/12/31 09:19:24  fox
 * Last version before I switched exploration mode in PLAN to random
 * mode in COLLI.
 *
 * Revision 1.15  1996/12/20 15:29:39  fox
 * Added four parameters.
 *
 * Revision 1.14  1996/12/18 09:11:06  wolfram
 * Added clipping of simulator maps
 *
 * Revision 1.13  1996/12/13 13:55:37  fox
 * Implemented connection to PLAN.
 *
 * Revision 1.12  1996/12/13 10:29:53  wolfram
 * computed grid maps are now automatically dumped
 *
 * Revision 1.11  1996/12/13 09:44:52  wolfram
 * Fixed a bug in readSimulatorMap
 *
 * Revision 1.10  1996/12/12 17:28:09  wolfram
 * Localize can read simulator maps now
 *
 * Revision 1.9  1996/12/02 10:32:08  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.8  1996/11/29 15:33:38  fox
 * ok
 *
 * Revision 1.7  1996/11/28 17:56:22  fox
 * *** empty log message ***
 *
 * Revision 1.6  1996/11/28 12:40:29  wolfram
 * Wolframs version.
 *
 * Revision 1.5  1996/11/27 16:36:13  wolfram
 * added procedure for filling the unknown fields of a grid
 *
 * Revision 1.4  1996/11/27 12:18:18  fox
 * Nothing special.
 *
 * Revision 1.3  1996/10/24 12:07:11  fox
 * Fixed a bug.
 *
 * Revision 1.2  1996/10/24 09:56:54  fox
 * LOCALIZE also works in a write protected directory.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:31  rhino
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

#include "general.h"
#include "map.h"
#include "file.h"
#include "allocate.h"
#include "probFunction.h"
#include "function.h"
#include "allocate.h"
#include "probGrid.h"

#define MAP_FILE_TOKEN                 0
#define MAP_UNKNOWN_TOKEN              1
#define CAD_MAP_TOKEN                  2
#define DESIRED_RESOLUTION_TOKEN       3
#define SECURITY_DIST_TOKEN            4
#define ONLINE_LOCALIZATION_TOKEN      5


#define SIMULATOR_MAP            0
#define GRID_MAP                 1
#define GRID_MAP_MARK "global_map[0]"
#define SIMULATOR_MAP_MARK "MAP"
#define SIMULATOR_ROBOT_MARK "ROBOT"
#define RECTANGLE_MARK "RECTANGLE"
#define CUBE_MARK "CUBE"
#define CYLINDER_MARK "CYLINDER"
#define NUMBER_OF_CORNERS 4
#define ZOOM 3
#define CLIP_THRESHOLD 0.4
#define PLAN_MAP_EXTENSION ".planmap"
#define ENTROPY_MAP_EXTENSION ".entropymap"
#define POTENTIAL_MAP_EXTENSION ".potentialmap"


mapParameters globalMapParameters;

typedef struct{
  float centerX;
  float centerY;
  float width;
  float depth;
  float rot;
} rectangle;

    

/********************************************************************
 ********************************************************************
 * Forward declarations of internal functions.
 ********************************************************************
 ********************************************************************/

int
readMap(char* mapname, probabilityGrid *m);

int
shrinkMap(probabilityGrid *m);

void
setStatistics( probabilityGrid *m);

int
initializeProbabilityGrid(probabilityGrid *m);

void
computeGridMap( simulatorMap *simMap,
		int mapResolution,
		float additionalObjectSize,
		float fromZ,
		float toZ,
		probabilityGrid *m);

bool
readSimulatorMap( char *mapName, simulatorMap *simMap);


bool
readGridMap(char *mapname, char *extension, probabilityGrid *m);

probabilityGrid
planMap( probabilityGrid* map, float additionalSize);

static void
initializeOnlineMap( probabilityGrid* map, char* mapFileName);


/**************************************************************************
 **************************************************************************
 * Mandatory functions.
 **************************************************************************
 **************************************************************************/


void
initialize_MAP( char* fileName,
		actionInformation* actionInfo)
{

  bool simulatorMapRead = FALSE, gridMapRead = FALSE;
  float securityDistance = 0.0;
  token tok[NUMBER_OF_MAP_PARAMETERS];
    
    /**********************************************************
     * Get the parameters from the file. 
     **********************************************************/

  getDirectory(globalMapParameters.mapFileName);
  globalMapParameters.unknown = -1;
  globalMapParameters.cadMap = 1;
  globalMapParameters.desiredResolution = 15;
  securityDistance = 15;

  setTokensInitialized(tok, NUMBER_OF_MAP_PARAMETERS);

    
    /**********************************************************
     * Get the parameters from the file. 
     **********************************************************/


  tok[MAP_FILE_TOKEN].format   = STRING_FORMAT;
  tok[MAP_FILE_TOKEN].variable = &(globalMapParameters.mapFileName);
  tok[MAP_FILE_TOKEN].keyWord  = MAP_KEYWORD;

  tok[MAP_UNKNOWN_TOKEN].format   = FLOAT_FORMAT;
  tok[MAP_UNKNOWN_TOKEN].variable = &(globalMapParameters.unknown);
  tok[MAP_UNKNOWN_TOKEN].keyWord  = MAP_UNKNOWN_KEYWORD;

  tok[CAD_MAP_TOKEN].format   = INT_FORMAT;
  tok[CAD_MAP_TOKEN].variable = &(globalMapParameters.cadMap);
  tok[CAD_MAP_TOKEN].keyWord  = CAD_MAP_KEYWORD;

  tok[DESIRED_RESOLUTION_TOKEN].format   = FLOAT_FORMAT;
  tok[DESIRED_RESOLUTION_TOKEN].variable = &(globalMapParameters.desiredResolution);
  tok[DESIRED_RESOLUTION_TOKEN].keyWord  = DESIRED_RESOLUTION_KEYWORD;

  tok[SECURITY_DIST_TOKEN].format   = FLOAT_FORMAT;
  tok[SECURITY_DIST_TOKEN].variable = &securityDistance;
  tok[SECURITY_DIST_TOKEN].keyWord  = SECURITY_DIST_KEYWORD;

  tok[ONLINE_LOCALIZATION_TOKEN].format   = INT_FORMAT;
  tok[ONLINE_LOCALIZATION_TOKEN].variable = &(actionInfo->onlineMapping);
  tok[ONLINE_LOCALIZATION_TOKEN].keyWord  = ONLINE_LOCALIZATION_KEYWORD;
  
  readTokens( fileName, tok, NUMBER_OF_MAP_PARAMETERS, FALSE); 

    /**********************************************************
     * done.
     **********************************************************/

  simulatorMapRead = readSimulatorMap( globalMapParameters.mapFileName,
				       &actionInfo->simMap);
  
  if ( ! actionInfo->onlineMapping) {
    
    /* Read the map. */
    if (0) simulatorMapRead = readSimulatorMap( globalMapParameters.mapFileName,
						&actionInfo->simMap);
    
    gridMapRead = readGridMap( globalMapParameters.mapFileName,
			       GRID_MAP_EXTENSION,
			       &actionInfo->map);
    if (0) writeGridMap(globalMapParameters.mapFileName,
			".shrinkedmap",
			&actionInfo->map);
    if ( ! gridMapRead){
      if ( !simulatorMapRead){
	fprintf(stderr, "Error: could not load simulator map or grid map\n");
	closeLogAndExit(1);
      }
      else {
	actionInfo->simMap.initialized = TRUE;
	computeGridMap( &actionInfo->simMap,
			globalMapParameters.desiredResolution,
			0,
			0,
			ROB_HEIGHT,
			&actionInfo->map);
	(void) writeGridMap( globalMapParameters.mapFileName,
			     GRID_MAP_EXTENSION,
			     &actionInfo->map);
      }
    }
    actionInfo->map.initialized = TRUE;    
      
    if (!readGridMap( globalMapParameters.mapFileName,
		      PLAN_MAP_EXTENSION,
		      &actionInfo->planMap)){
      if (actionInfo->simMap.initialized){
	computeGridMap( &actionInfo->simMap,
			globalMapParameters.desiredResolution,
			ROB_RADIUS + securityDistance,
			0.0, 120.0,
			&(actionInfo->planMap));
	(void) writeGridMap( globalMapParameters.mapFileName,
			     PLAN_MAP_EXTENSION,
			     &(actionInfo->planMap));
      }
      else {
	actionInfo->planMap = planMap( &(actionInfo->map),
				       securityDistance);
	(void) writeGridMap( globalMapParameters.mapFileName,
			     PLAN_MAP_EXTENSION,
			     &(actionInfo->planMap)); 
	/*
	  positionProbabilityPlanMaps( &(actionInfo->map),
	  securityDistance);
	  invertMap( &(actionInfo->planMap)); */
      } 
    }
    else
  
    if (readGridMap( globalMapParameters.mapFileName,
		     ENTROPY_MAP_EXTENSION,
		     &actionInfo->entropyMap))
      actionInfo->entropyMap.initialized = TRUE;
    if (readGridMap( globalMapParameters.mapFileName,
		     POTENTIAL_MAP_EXTENSION,
		     &actionInfo->potentialMap))
      actionInfo->potentialMap.initialized = TRUE;
    
    actionInfo->planMap.initialized = TRUE;
    actionInfo->planMap.offsetX = actionInfo->map.offsetX;
    actionInfo->planMap.offsetY = actionInfo->map.offsetY;
    
    /* Set default values for the online map. */
    actionInfo->onlineMap = actionInfo->map;
      
    checkMapConsistency(actionInfo->map.sizeX,
			actionInfo->map.sizeY,
			actionInfo->map.resolution,
			actionInfo->planMap.sizeX,
			actionInfo->planMap.sizeY,
			actionInfo->planMap.resolution,
			globalMapParameters.mapFileName, GRID_MAP_EXTENSION,
			PLAN_MAP_EXTENSION);
  }
  else {

    initializeOnlineMap( & actionInfo->onlineMap, globalMapParameters.mapFileName);
    
    /* Set all maps to the online map. */
    actionInfo->map      = actionInfo->onlineMap;
    actionInfo->planMap = actionInfo->onlineMap;

    if(0) actionInfo->simMap.initialized = FALSE;
  }
}



/**************************************************************************
 **************************************************************************
 * Local variables and defines.
 **************************************************************************
 **************************************************************************/

/********************************************************************
 ********************************************************************
 * Global functions.
 ********************************************************************
 ********************************************************************/

/********************************************************************
 ********************************************************************
 * The local functions.
 ********************************************************************
 ********************************************************************/


bool
readGridMap(char *mapName, char *extension, probabilityGrid *m){
   int x,y;
   float temp;
   char line[MAX_STRING_LENGTH], fileName[MAX_STRING_LENGTH];
   FILE *fp;

   sprintf(fileName, "%s%s", mapName, extension);

   if ((fp = fopen(fileName, "rt")) == NULL) {
     fprintf(stderr,"# Could not open file %s\n", fileName);
     return FALSE;
   }

   fprintf(stderr, "# Reading grid map from %s\n", fileName);
   
   /* seek header for map data in rhino-Type file,
      analyze robot specifications */
   while ((fgets(line,MAX_STRING_LENGTH,fp) != NULL)
	  && (strncmp("global_map[0]", line , 13) != 0)){
     if (strncmp(line,"robot_specifications->resolution",32) == 0) {
	 if (sscanf(&line[32],"%d",&(m->resolution)) != 0 ) {
	   writeLog("# Map resolution: %d cm\n",m->resolution);
	 }
     }
     if (strncmp(line,"robot_specifications->autoshifted_x",35) == 0) {
       if (sscanf(&line[35],"%g",&(m->offsetX)) != 0 ) {
	 m->offsetX = m->offsetX;
	 writeLog("# Map offsetX: %g cm\n",m->offsetX);
       }
     }
     if (strncmp(line,"robot_specifications->autoshifted_y",35) == 0) {
       if (sscanf(&line[35],"%g",&(m->offsetY)) != 0 ) {
	 m->offsetY = m->offsetY;
	 writeLog("# Map offsetY: %g cm\n",m->offsetY);
       }
     }
   }
   if (sscanf (line,"global_map[0]: %d %d",&m->sizeY, &m->sizeX)
       != 2 ) {
     fprintf(stderr,"ERROR: corrupted file %s\n",fileName);
     fclose(fp);
     return FALSE;
   }
   
   writeLog("# Map size: %d %d\n",m->sizeX,m->sizeY);
   
   m->unknown = globalMapParameters.unknown;
   m->offsetX = m->offsetY = 0.0;
   m->origsizeX = m->sizeX;
   m->origsizeY = m->sizeY;
   m->shiftedX = 0;
   m->shiftedY = 0;
   m->maxRealX = m->offsetX + m->sizeX * m->resolution;
   m->maxRealY = m->offsetY + m->sizeY * m->resolution;
   m->numberOfFreeCells = 0;
   m->freeSpace = NULL;
   
   m->prob = (mapProbability**) allocate2D(m->sizeX,
					   m->sizeY,
					   MAP_PROBABILITY);

   if (m->prob == (mapProbability**) NULL){
      fprintf(stderr,
              "ERROR: Not enough memory for loading map %s\n", fileName);
      fclose(fp);
      return FALSE;
   }
   
   for (x=0;x<m->sizeX; x++)
     for (y=0;y<m->sizeY; y++) {
	 fscanf(fp,"%e",&temp);
	 if (temp < 0.0 || temp > 1.0)
	   m->prob[x][y] = globalMapParameters.unknown;
	 else {
	   m->prob[x][y] = 1-temp;	   
	   if (m->prob[x][y] < MINIMUM_MAPPROBABILITY)
	     m->prob[x][y] = MINIMUM_MAPPROBABILITY;
	   else if (m->prob[x][y] > MAXIMUM_MAPPROBABILITY)
	     m->prob[x][y] = MAXIMUM_MAPPROBABILITY;
	 }
      }

   /* #define THIN_MAP */
#ifdef THIN_MAP
   {
     mapProbability** tmp = (mapProbability**) allocate2D(m->sizeX,
							  m->sizeY,
							  MAP_PROBABILITY);
     probabilityGrid thinMap;
     
     for (x=0;x<m->sizeX; x++)
       for (y=0;y<m->sizeY; y++) {
	 tmp[x][y] = m->prob[x][y];
	 if ( m->prob[x][y] != m->unknown)
	   m->prob[x][y] = 1.0 - m->prob[x][y];
       }

     thinMap = planMap( m, -16.7);
     
     for (x=0;x<m->sizeX; x++)
       for (y=0;y<m->sizeY; y++) {
	 if ( m->prob[x][y] != m->unknown) {
	   m->prob[x][y] = 1.0 - thinMap.prob[x][y];
	   if ( m->prob[x][y] < 0.5)
	     m->prob[x][y] = 0.0;
	 }
       }
   }
   writeGridMap( "floor-thin", "map", m);
#endif
   
   shrinkMap(m);
   setStatistics(m);
   fprintf(stderr, "# done\n");
   return TRUE;
}


void
checkMapConsistency( int map1SizeX, int map1SizeY, int map1Res,
		     int map2SizeX, int map2SizeY, int map2Res,
		     char* fileName, char* mapExtension, char* extension)
{
  if (map1Res != map2Res ||
      map1SizeX != map2SizeX ||
      map1SizeY != map2SizeY){
    fprintf(stderr, "# Error: %s%s and %s%s are not consistent!\n",
	    fileName, mapExtension, fileName, extension);
      fprintf(stderr, "# Remove %s%s and restart!\n",
	      fileName, extension);
      exit(0);
  }
}


void
invertMap( probabilityGrid *map)
{
  int x,y;

  for (x = 0; x < map->sizeX; x++)
    for (y = 0; y < map->sizeY; y++)
      if (map->prob[x][y] != map->unknown)
	map->prob[x][y] = 1.0 - map->prob[x][y];
  setStatistics(map);
}

probabilityGrid
invertedMap( probabilityGrid *map)
{
  int x,y;
  probabilityGrid inverted = *map;

  inverted.prob = (mapProbability**) allocate2D( map->sizeX, map->sizeY, MAP_PROBABILITY);

  for (x = 0; x < map->sizeX; x++)
    for (y = 0; y < map->sizeY; y++)
      if (map->prob[x][y] != map->unknown)
	inverted.prob[x][y] = 1.0 - map->prob[x][y];
      else
	inverted.prob[x][y] = IMPOSSIBLE;
  setStatistics(&inverted);
  return inverted;
}

void
rotpnt(float angle, float *x, float *y) 
{ 
    float t = *x; 
    *x = *x * cos(angle) - *y * sin(angle); 
    *y = t  * sin(angle) + *y * cos(angle); 
}


void
absolute_points(rectangle *rect, float* inside_x, float* inside_y)
{
  int i;
  inside_x[0] = rect->width*0.5;
  inside_y[0] = -rect->depth*0.5;
  rotpnt(rect->rot,&inside_x[0],&inside_y[0]);
  inside_x[1] = rect->width*0.5;
  inside_y[1] = rect->depth*0.5;
  rotpnt(rect->rot,&inside_x[1],&inside_y[1]);
  inside_x[2] = -rect->width*0.5;
  inside_y[2] = rect->depth*0.5;
  rotpnt(rect->rot,&inside_x[2],&inside_y[2]);
  inside_x[3] = -rect->width*0.5;
  inside_y[3] = -rect->depth*0.5;
  rotpnt(rect->rot,&inside_x[3],&inside_y[3]);
  for(i = 0; i < NUMBER_OF_CORNERS; i++) {
    inside_x[i] += rect->centerX;
    inside_y[i] += rect->centerY;
  }
}

int
inHalfPlane(float xp, float yp, float x1, float y1, float x2, float y2)
{
    float m,c;
  if(x1 == x2) {
    if(y1 <= y2)
      return (xp <= x1);
    else
      return (xp >= x1);
  }
  else {
    if( x1 < x2 ) {
      m  = (y2 - y1) / (x2 - x1);
      c = y1 - m*x1;
      return (yp >= m*xp+c);
    }
    else {
      m  = (y1 - y2) / (x1 - x2);
      c = y1 - m*x1;
      return (yp <= m*xp+c);
    }
  }
}

int
inside(float xi, float yi, float* inside_x, float* inside_y)
{
    int i;
    for(i = 0; i < NUMBER_OF_CORNERS; i++) {
	if(!inHalfPlane(xi,yi,
			inside_x[i],inside_y[i],
			inside_x[(i+1)%NUMBER_OF_CORNERS],
			inside_y[(i+1)%NUMBER_OF_CORNERS]))
	    return FALSE;
    }
    return TRUE;
}

void
insertRectangle(probabilityGrid *map, rectangle rect)
{
  int x, y, minX, maxX, minY, maxY;

  float inside_x[NUMBER_OF_CORNERS];
  float inside_y[NUMBER_OF_CORNERS];
  
  absolute_points( &rect, inside_x, inside_y);

  minX = maxX = inside_x[0];
  minY = maxY = inside_y[0];
  
  for (x = 1; x < NUMBER_OF_CORNERS; x++){
    if (inside_x[x] < minX)
      minX = inside_x[x];
    if (inside_x[x] > maxX)
      maxX = inside_x[x];
    if (inside_y[x] < minY)
      minY = inside_y[x];
    if (inside_y[x] > maxY)
      maxY = inside_y[x];
  }
  maxX++;
  maxY++;
  
  minX = iMax(0, minX);
  minX = iMin(minX, map->sizeX - 1);
  maxX = iMax(0, maxX);
  maxX = iMin(maxX, map->sizeX - 1);
  minY = iMax(0, minY);
  minY = iMin(minY, map->sizeY - 1);
  maxY = iMax(0, maxY);
  maxY = iMin(maxY, map->sizeY - 1);

  for (x = minX; x < maxX; x++)
    for (y = minY; y < maxY; y++)
      if (inside( (float) x, (float) y, inside_x, inside_y))
      {
	 map->prob[x][y] = MAXIMUM_PROBABILITY;
      }
}






void
insertCylinder(probabilityGrid *map, rectangle rect)
{
  int x, y, minX, maxX, minY, maxY;
  float radius = rect.width;
  
  minX = rect.centerX - rect.width;
  minY = rect.centerY - rect.width;
  maxX = rect.centerX + rect.width + 1;
  maxY = rect.centerY + rect.width + 1;
  
  minX = iMax(0, minX);
  minX = iMin(minX, map->sizeX - 1);
  maxX = iMax(0, maxX);
  maxX = iMin(maxX, map->sizeX - 1);
  minY = iMax(0, minY);
  minY = iMin(minY, map->sizeY - 1);
  maxY = iMax(0, maxY);
  maxY = iMin(maxY, map->sizeY - 1);

  for (x = minX; x <= maxX; x++)
    for (y = minY; y <= maxY; y++)
      if (sqrt(fSqr(rect.centerX - x) + fSqr(rect.centerY - y)) < radius)
      {
	 map->prob[x][y] = MAXIMUM_PROBABILITY;
      }
}




int 
checkIntersection(float p_x1, float p_y1, float p_x2, float p_y2,
		   float q_x1, float q_y1, float q_x2, float q_y2,
		   float *x, float *y)
{
  float mu, lambda;

  float p_dx;
  float p_dy;
  float q_dx;
  float q_dy;
  float n;
  float d;

  p_dx = p_x2 - p_x1;
  p_dy = p_y2 - p_y1;
  q_dx = q_x2 - q_x1;
  q_dy = q_y2 - q_y1;

  n = (q_x1 * q_dy) - (q_y1 * q_dx) - (p_x1 * q_dy) + (p_y1 * q_dx);
  d = (p_dx * q_dy) - (p_dy * q_dx);
  
  if (d == 0.0)			/* collinear */
    return FALSE;
  
  lambda = n/d;
  
  n = (p_x1 * p_dy) - (p_y1 * p_dx) - (q_x1 * p_dy) + (q_y1 * p_dx);
  d = (q_dx * p_dy) - (q_dy * p_dx);
  
  if (d == 0.0)			/* ...shouldn't happen here! */
    return FALSE;
  
  mu = n/d;
  
  
  /*  fprintf(stderr, "### P=(%6.4f %6.4f ->%6.4f %6.4f) Q=(%6.4f %6.4f ->%6.4f %6.4f)   %6.4f %6.4f, lambda=%6.4f mu=%6.4f -> %d\n",
      p_x1,  p_y1,  p_x2,  p_y2, q_x1,  q_y1,  q_x2,  q_y2,
      p_x1 + (lambda * p_dx) - q_x1 - (mu * q_dx),
      p_y1 + (lambda * p_dy) - q_y1 - (mu * q_dy), 
      lambda, mu, 
      (lambda >= 0.0 && lambda <= 1.0 && mu >= 0.0 && mu <= 1.0));
  */
  
  if (lambda >= 0.0 && lambda <= 1.0 && mu >= 0.0 && mu <= 1.0){
    *x = p_x1 + (lambda * p_dx);
    *y = p_y1 + (lambda * p_dy);
    return TRUE;
  }
  else
    return FALSE;
  
}



static void          
fswap(float *x, float *y){
        float   swap;

        swap = *x;
        *x = *y;
        *y = swap;

} /* fswap() */


bool
cutCircleAndLine(float cx, float cy, float r, float lx1, float ly1,
		    float lx2, float ly2, 
		    float *dist)
{
  float 	dx, dy, mue, lambda, tmp, norm;
  float         S0x, S0y, S1x, S1y;
  float		swap;


  dx = lx2 - lx1;
  dy = ly2 - ly1;

  norm = (float) (sqrt((double) (fSqr(dx) + fSqr(dy))));

  if (norm < 1e-6)		/*** the two points of the line are too close to each other ! ***/
  	return FALSE;

  dx /= norm;
  dy /= norm;

  /*** First compute the distance between midpoint of the circle
   *** and the line.
   *** The following formulas are the solution to:
   *** line.pt1.x + lambda*dx = M.x + mue*dy   and
   *** line.pt1.y + lambda*dy = M.y - mue*dx.
   *** In this solution the minimal distance between M and line is mue.
   ***/

  mue = (dy * (lx1 - cx) + dx * (cy - ly1)) / (fSqr(dy) + fSqr(dx));

  if (fSqr(r)-fSqr(mue) < 0.0)
    return 0;

  /*** Now compute the two points of intersection ***/  
  
  tmp = (float) (sqrt((double) (fSqr(r) - fSqr(mue))));

  if (dy != 0.0) {
    lambda = (cy - mue*dx - ly1) / dy;

    S0x = lx1 + (lambda - tmp) * dx;
    S0y = ly1 + (lambda - tmp) * dy;
    S1x = lx1 + (lambda + tmp) * dx;
    S1y = ly1 + (lambda + tmp) * dy;
  }
  else { 
    S0x = cx - tmp;
    S0y = ly1;
    S1x = cx + tmp;
    S1y = ly1;
  }

  /*** only used for sonar :  dist, angle ***/

  *dist = pointDistance(lx1, ly1, S0x, S0y);

  swap =  pointDistance(lx1, ly1, S1x, S1y);
  if (swap < *dist) {
  	*dist = swap;
  }

  if (lx1 > lx2) 
  	fswap(&lx1, &lx2);

  if (ly1 > ly2) 
  	fswap(&ly1, &ly2);

  if (((S0x >= lx1) && (S0x <= lx2) && (S0y >= ly1) && (S0y <= ly2)) ||	    /*** infinit length of line ***/
      ((S1x >= lx1) && (S1x <= lx2) && (S1y >= ly1) && (S1y <= ly2))) 
      	return TRUE;

  return FALSE;

} /* cut_circle_and_line() */




bool
cutLineAndLine(float line1x1, float line1y1, 
	       float line1x2, float line1y2, 
	       float line2x1, float line2y1, 
	       float line2x2, float line2y2, 
	       float *dist)
{
  
  point cutPoint;
  bool cut;

  cut = checkIntersection(line1x1, line1y1, line1x2, line1y2, 
			  line2x1, line2y1, line2x2, line2y2,
			  &cutPoint.x, &cutPoint.y);
  if (cut){
    *dist = pointDistance(line1x1, line1y1, cutPoint.x, cutPoint.y);
  }
  
  return cut;

} 


float
rectangleDistance(rectangle *rect,
		  lineSegment beam,
		  float maxRange){
  float dist = maxRange;
  int i,j;
  
  
  float inside_x[NUMBER_OF_CORNERS];
  float inside_y[NUMBER_OF_CORNERS];
  
  absolute_points( rect, inside_x, inside_y);

  
  for (i = 0; i < NUMBER_OF_CORNERS; i++){
    float dist1;
    
    j = i+1;
    if (j == NUMBER_OF_CORNERS)
      j = 0;

    if (cutLineAndLine(beam.from.x, beam.from.y, beam.to.x, beam.to.y, 
		       inside_x[i], inside_y[i], inside_x[j],  inside_y[j], 
		       &dist1)){
      /* fprintf(stderr, "Line: %f %f %f %f\n",
	      inside_x[i], inside_y[i],
	      inside_x[j],  inside_y[j]); */
      
      
      if (dist1 < dist)
	dist = dist1;
    }
    
  }
  
  return dist;
}



float
simulatorObjectDistance(simulatorMap *simMap,
			float posX, float posY,
			float sensorHeight,
			float cosRot, float sinRot, float maxRange){
  
  int i;
  simulatorObject object;
  float dist = maxRange;
  lineSegment beam;
  beam.from.x = posX;
  beam.from.y = posY;
  beam.to.x = posX + maxRange * cosRot;
  beam.to.y = posY + maxRange * sinRot;

  /* fprintf(stderr, "Beam: %f %f %f %f\n", beam.from.x, beam.from.y,
     beam.to.x, beam.to.y); */
  for (i = 0; i < simMap->numberOfObjects; i++){
    object = simMap->object[i];
    if (
	sensorHeight >= object.posZ - 0.5*object.height &&
	sensorHeight <= object.posZ + 0.5*object.height){
      if (object.type == CYLINDER){
	
      }
      
      else if (object.type == CUBE){
	float xBox, yBox, xFrom, xTo, yFrom, yTo;
	if (object.rot == 0.0){
	  xBox = object.width * 0.5;
	  yBox = object.depth * 0.5;
	}
	else {
	  if (object.width > object.depth)
	    xBox = yBox = object.width * 0.5;
	  else
	    xBox = yBox = object.depth * 0.5;
	}
	
	if (beam.from.x < beam.to.x){
	  xFrom = beam.from.x;
	  xTo = beam.to.x;
	}
	else{
	  xFrom = beam.to.x;
	  xTo = beam.from.x;
	}
	
	if (beam.from.y < beam.to.y){
	  yFrom = beam.from.y;
	  yTo = beam.to.y;
	}
	else{
	  yFrom = beam.to.y;
	  yTo = beam.from.y;
	}
	
	if (intersection(xFrom, xTo, object.posX - xBox,
			 object.posX + xBox)
	    &&
	    intersection(yFrom, yTo,
			 object.posY - yBox,
			 object.posY + yBox)){
	  rectangle rect;
	  float dist1;
	  rect.centerX = object.posX;
	  rect.centerY = object.posY;
	  rect.width = object.width;
	  rect.depth = object.depth;
	  rect.rot = object.rot;
	  
	  dist1 = rectangleDistance(&rect, beam, maxRange);
	  
	  if (dist1 < dist)
	    dist = dist1;
	}
      }
    }
    
  }
  
  return dist;
}


float
integratedOccupancyProbability(int x, int y, int delta, probabilityGrid *map)
{
  int i, j;
  int startI, startJ, endI, endJ;

  float prob = 0.0, dist, weightSum = 0.0, weight, maxDist;
  
  startI = iMax(0, x - delta);
  endI = iMin(map->sizeX, x + delta + 1);

  startJ = iMax(0, y - delta);
  endJ = iMin(map->sizeY, y + delta + 1);

  maxDist = pointDistance(delta, delta, 0, 0);

  for (i = startI; i < endI; i++)
    for (j = startJ; j < endJ; j++){
      if (map->prob[i][j] >= 0){
	dist = pointDistance(i,j,x,y);
	weight = fabs(maxDist - dist) / maxDist;
	weightSum += weight;
	prob += weight * map->prob[i][j];
      }
    }
  if (weightSum == 0.0){
     prob = map->prob[x][y];
     fprintf(stderr, "Null: %d %d\n", x, y);
  }
  else
     prob /= weightSum;

  if (prob != globalMapParameters.unknown) {
      if (prob < MINIMUM_PROBABILITY)
	  prob = MINIMUM_PROBABILITY;
      else if (prob > MAXIMUM_PROBABILITY)
	  prob = MAXIMUM_PROBABILITY;
  }
  
  return prob;
}



bool
readSimulatorMap( char *mapName, simulatorMap *simMap)
{
  char fileName[MAX_STRING_LENGTH];
  sprintf(fileName, "%s.sim", mapName);
  return readSimulatorMapFile( fileName, simMap);
}

  
bool
readSimulatorMapFile( char *fileName, simulatorMap *simMap)
{

  FILE *fp;
  char line[MAX_STRING_LENGTH];
  int markLength;
  int found;
  realPosition pos;
  simulatorObject simObject;
  bool stop;

  if ((fp = fopen(fileName,"r")) == NULL) {
    fprintf(stderr,"# Could not open file %s\n", fileName);
    return FALSE;
  }

  strcpy(simMap->fileName, fileName);

  fprintf(stderr, "# Reading Simulator map from %s\n", simMap->fileName);
  stop = FALSE;
  while ((fgets(line,MAX_STRING_LENGTH,fp) != NULL) && !stop){
    if(strncmp(SIMULATOR_MAP_MARK, line, strlen(SIMULATOR_MAP_MARK)) == 0){
      stop = TRUE;
    }
  }
  
  if (!stop){
    fprintf(stderr,
	    "# File %s has not simulator map format\n", simMap->fileName);
    fclose(fp);
    return FALSE;
  }
  
  rewind(fp);
  
  found = FALSE;
  while ((fgets(line,MAX_STRING_LENGTH,fp) != NULL) && !feof(fp)){
    if (strncmp(line,SIMULATOR_MAP_MARK,strlen(SIMULATOR_MAP_MARK)) == 0){
      markLength = strlen(SIMULATOR_MAP_MARK);
      if (sscanf(&line[markLength],"%f %f %f %f",
		 &(simMap->fromX), &(simMap->fromY),
		 &(simMap->toX), &(simMap->toY)) == 4){
	fprintf(stderr, "# Map size: (%.2fx%.2f) -> (%.2fx%.2f)\n",
		simMap->fromX, simMap->fromY,
		simMap->toX,  simMap->toY);
	found = TRUE;
      }
    }
    if (strncmp(line,SIMULATOR_ROBOT_MARK,
		strlen(SIMULATOR_ROBOT_MARK)) == 0){
      markLength = strlen(SIMULATOR_ROBOT_MARK);
      if (sscanf(&line[markLength],"%f %f %f", &pos.x, &pos.y,
		 &pos.rot) == 3){
	fprintf(stderr, "# Simulator robot position: X=%.2f Y=%.2f O=%.2f\n", 
		pos.x, pos.y, pos.y);
      }
      
    }
  }
  
  if (!found){
    fprintf(stderr, "Error: Map size not found!\n");
    fclose(fp);
    return(FALSE);
  }

  rewind(fp);
  

  simMap->sizeX = simMap->toX - simMap->fromX;
  simMap->sizeY = simMap->toY - simMap->fromY;
  simMap->sizeZ = 0.0;
  simMap->numberOfObjects = 0;
  
  while ((fgets(line,MAX_STRING_LENGTH,fp) != NULL)
	 && simMap->numberOfObjects < NUMBER_OF_SIMULATOR_OBJECTS ){
    found = FALSE;
    if (strncmp(line,RECTANGLE_MARK,
		markLength = strlen(RECTANGLE_MARK)) == 0){
      simObject.type = CUBE;
      if (sscanf(&line[markLength],"%f %f %f %f %f", &simObject.posX,
		 &simObject.posY, &simObject.width, &simObject.depth,
		 &simObject.rot) == 5){
	simObject.height = MAXFLOAT;
	simObject.posZ = 0.0;
	found = TRUE;
      }
      else if (sscanf(&line[markLength],"%f %f %f %f", &simObject.posX,
		      &simObject.posY,
		      &simObject.width,
		      &simObject.depth) == 4){
	simObject.height = MAXFLOAT;
	simObject.posZ = 0.0;
	simObject.rot = 0.0;
	found = TRUE;
      }
    }
    
    else if (strncmp(line,CUBE_MARK,
		     markLength = strlen(CUBE_MARK)) == 0){
      simObject.type = CUBE;
      if (sscanf(&line[markLength],"%f %f %f %f %f %f %f",
		 &simObject.posX,
		 &simObject.posY,
		 &simObject.posZ,
		 &simObject.width,
		 &simObject.depth,
		 &simObject.height,
		 &simObject.rot) == 7){
	found = TRUE;
      }
      else if (sscanf(&line[markLength],"%f %f %f %f %f %f",
		      &simObject.posX,
		      &simObject.posY,
		      &simObject.posZ,
		      &simObject.width,
		      &simObject.depth,
		      &simObject.height) == 6){
	simObject.rot = 0.0;
	found = TRUE;
      }
    }
    else if (strncmp(line,CYLINDER_MARK,
		     markLength = strlen(CYLINDER_MARK)) == 0){
      simObject.type = CYLINDER;
      if (sscanf(&line[markLength],"%f %f %f %f %f",
		 &simObject.posX,
		 &simObject.posY,
		 &simObject.posZ,
		 &simObject.width,
		 &simObject.height) == 5){
	simObject.depth = simObject.width;
	simObject.rot = 0.0;
	found = TRUE;
	
      }
    }
    
    if (found && (simObject.width > 10 || simObject.depth > 10)){
      simObject.posX -= simMap->fromX;
      simObject.posY -= simMap->fromY;
      simMap->object[simMap->numberOfObjects] = simObject;
      simMap->numberOfObjects++;
    }
  }
  
  fprintf(stderr, "# Found %d objects\n", simMap->numberOfObjects);
  fclose(fp);
  simMap->initialized = TRUE;
  return TRUE;
}




void
computeGridMap( simulatorMap *simMap,
		int mapResolution,
		float additionalObjectSize,
		float fromZ,
		float toZ,
		probabilityGrid *m)
{
  probabilityGrid tmpMap;
  rectangle rect;
  simulatorObject object;
  int i, j;
  int delta;

  tmpMap.sizeX = (int) (simMap->sizeX + 1);
  tmpMap.sizeY = (int) (simMap->sizeY + 1);
  
  tmpMap.prob = (mapProbability**)
    allocate2D(tmpMap.sizeX, tmpMap.sizeY, MAP_PROBABILITY);
  
  /* filling everything white */
  
  for (i = 0; i < tmpMap.sizeX; i++)
      for (j = 0; j < tmpMap.sizeY; j++)
	tmpMap.prob[i][j] = MINIMUM_MAPPROBABILITY;
  
  for ( i=0; i < simMap->numberOfObjects; i++){
     object = simMap->object[i];
     if (intersection(fromZ, toZ, object.posZ - object.height/2,
		      object.posZ + object.height/2)){
       rect.centerX = object.posX;
       rect.centerY = object.posY;
       rect.rot = object.rot;
       switch (object.type){
       case CUBE:
	 rect.width = object.width + 2.0 * additionalObjectSize;
	 rect.depth = object.depth + 2.0 * additionalObjectSize;
	 insertRectangle(&tmpMap, rect);
	 break;
       case CYLINDER:
	 rect.width = object.width + additionalObjectSize;
	 rect.depth = object.depth + additionalObjectSize;
	 insertCylinder(&tmpMap, rect);
	 break;
       default:
	 break;
       }
     }
  }
   
   m->sizeX = tmpMap.sizeX / (float) mapResolution + 1;
   m->sizeY = tmpMap.sizeY / (float) mapResolution + 1;
   m->resolution = mapResolution;
   setStatistics( &tmpMap);
   /* writeGridMap( "tmp", "map", &tmpMap); */
   m->unknown = globalMapParameters.unknown;
   m->origsizeX = m->sizeX;
   m->origsizeY = m->sizeY;
   m->offsetX = m->offsetY = 0.0;
   m->maxRealX = m->offsetX + m->sizeX * m->resolution;
   m->maxRealY = m->offsetY + m->sizeY * m->resolution;
   m->numberOfFreeCells = 0;
   m->freeSpace = NULL;

   m->prob = (mapProbability**)
      allocate2D(m->sizeX, m->sizeY, MAP_PROBABILITY);
	   

  for (i = 0; i < m->sizeX; i++)
      for (j = 0; j < m->sizeY; j++)
	m->prob[i][j] = MINIMUM_MAPPROBABILITY;

   if ((mapResolution % 2) == 0)
     delta = mapResolution+1;
   else
     delta = mapResolution;
   delta = (delta - 1) / 2;
   
   for (i = 0; i < m->sizeX; i++){
     int posX;
     posX = i*mapResolution + delta;
     for (j = 0; j < m->sizeY; j++){
       int posY;
       posY = j*mapResolution + delta;
       if ((posX < tmpMap.sizeX) && (posY < tmpMap.sizeY)){
	 m->prob[i][j] =
	   integratedOccupancyProbability(posX, posY, delta, &tmpMap);
	 if (m->prob[i][j] < MINIMUM_MAPPROBABILITY)
	   m->prob[i][j] = MINIMUM_MAPPROBABILITY;
	 else if (m->prob[i][j] > MAXIMUM_MAPPROBABILITY)
	   m->prob[i][j] = MAXIMUM_MAPPROBABILITY;
       }
     }
   }
   free2D((void **) tmpMap.prob, tmpMap.sizeX, MAP_PROBABILITY);
   setStatistics( m);
}



int
shrinkMap(probabilityGrid *m)
{
  register int x,y;
  register int empty = TRUE;

  int min_x,max_x,min_y,max_y;

  int oldsizeX;
  int oldsizeY;

  for (min_x = 0; min_x < m->sizeX && empty; min_x++)
    for (y=0; y < m->sizeY; y++)
      empty = empty && (m->prob[min_x][y] == m->unknown);
  min_x--;

  empty = TRUE;
  for (max_x = m->sizeX-1; max_x >=0 && empty; max_x--)
    for (y=0; y < m->sizeY; y++)
      empty = empty && (m->prob[max_x][y] == m->unknown);
  max_x++;
  
  empty = TRUE;
  for (min_y = 0; min_y < m->sizeY && empty; min_y++)
    for (x=0; x < m->sizeX; x++)
      empty = empty && (m->prob[x][min_y] == m->unknown);
  min_y--;
    
  empty = TRUE;
  for (max_y = m->sizeY-1; max_y >=0 && empty; max_y--)
    for (x=0; x < m->sizeX; x++)
      empty = empty && (m->prob[x][max_y] == m->unknown);
  max_y++;
  
  oldsizeX = m->sizeX;
  oldsizeY = m->sizeY;
  
  m->sizeX = max_x - min_x + 1;
  m->sizeY = max_y - min_y + 1;

  m->shiftedX = min_x;
  m->shiftedY = min_y;
  
  writeLog("# Map shrinked: [%d %d] --> [%d %d]  offset: (%.2f, %.2f)\n",
	   oldsizeX, oldsizeY, m->sizeX,m->sizeY,
	   m->offsetX, m->offsetY);
  writeLog(
	  "# Map shrinked: origin %d %d, size %d %d, resolution %d\n",
	  min_x, min_y,
	  m->sizeX, m->sizeY,
	  m->resolution);
  fprintf( stderr, "# Map shrinked: [%d %d] --> [%d %d]  offset: (%.2f, %.2f)\n",
	   oldsizeX, oldsizeY, m->sizeX,m->sizeY,
	   m->offsetX, m->offsetY);
  fprintf(stderr,
	  "# Map shrinked: origin %d %d, size %d %d, resolution %d\n",
	  min_x, min_y,
	  m->sizeX, m->sizeY,
	  m->resolution);

  for(x=0; x<m->sizeX; x++)
    for(y=0; y<m->sizeY; y++)
      m->prob[x][y] = m->prob[x+min_x][y+min_y];

  
  /*  m->offsetX += m->resolution * min_x;
  m->offsetY += m->resolution * min_y; */
  m->offsetX = m->offsetY = 0;
  m->maxRealX = m->offsetX + m->sizeX * m->resolution;
  m->maxRealY = m->offsetY + m->sizeY * m->resolution;
  
  return(1);
}


void
setStatistics( probabilityGrid *m)
{
  int y, x;
  
  int knownFields = 0;
  probability sum = 0.0;

  for (x=0; x<m->sizeX; x++)
    for (y=0; y<m->sizeY; y++)
      if ( m->prob[x][y] != m->unknown) {
	knownFields++;
	sum += m->prob[x][y];
      }
  
  if ( knownFields > 0)
    m->average = sum / knownFields;
  else
    m->average = m->unknown;
}


/* Normalizes the map such that each value is between 0 and 1. */
void
normalizeMap( probabilityGrid* map)
{
  int x, y, knownFields = 0;
  probability min = 1e20, max = -1e20;
  probability scaleFactor;
  
  for ( x = 0; x < map->sizeX; x++)
    for ( y = 0; y < map->sizeY; y++) 
      if ( map->prob[x][y] != map->unknown) {
	knownFields++;
	if (map->prob[x][y] < min)
	  min = map->prob[x][y];
	if (map->prob[x][y] > max) {
	  max = map->prob[x][y];
	}
      }
  
  if ( max == min || knownFields == 0)
    return;
  
  scaleFactor = 1.0 / ( max - min);
  
  for ( x = 0; x < map->sizeX; x++)
    for ( y = 0; y < map->sizeY; y++)
      if ( map->prob[x][y] != map->unknown) {
	map->prob[x][y] = ( map->prob[x][y] - min) * scaleFactor;
      }
  
  map->average = ( map->average - min) * scaleFactor;
}


/* Normalizes the map such that the sum of the cells is desiredSum. */
void
normalizeMapToSum( probabilityGrid* map,
		   probability desiredSum)
{
  int x, y;
  probability sum = 0.0;
  probability scaleFactor;

  for ( x = 0; x < map->sizeX; x++)
    for ( y = 0; y < map->sizeY; y++) 
      if ( map->prob[x][y] != map->unknown) 
	sum += map->prob[x][y];
      else
	sum += map->average;
  
  if ( sum == 0.0)
    return;
  
  scaleFactor = desiredSum / sum;
  
  for ( x = 0; x < map->sizeX; x++)
    for ( y = 0; y < map->sizeY; y++)
      if ( map->prob[x][y] != map->unknown)
	map->prob[x][y] *= scaleFactor;
  
  map->average *= scaleFactor;
}

bool
writeGridMap(char* mapFileName, char *extension, probabilityGrid *map)
{
  int x,y;
  char fileName[80];
  FILE *fp;

  
  sprintf(fileName, "%s%s", mapFileName, extension);
  
  if ((fp = fopen(fileName,"wt")) == NULL) {
      fprintf(stderr,"ERROR: Could not open file %s\n",fileName);
      return FALSE;
    }

  fprintf(fp, "robot_specifications->resolution %d\n", map->resolution);
  fprintf(fp, "robot_specifications->autoshifted_x %f\n",
	  (float) map->offsetX);
  fprintf(fp, "robot_specifications->autoshifted_y %f\n",
	  (float) map->offsetY);
  fprintf(fp, "global_map[0]: %d %d\n", map->sizeY, map->sizeX);

  for (x = 0; x < map->sizeX; x++){
    for (y = 0; y < map->sizeY; y++)
      if (map->prob[x][y] == globalMapParameters.unknown)
	fprintf(fp, "%.3f ", globalMapParameters.unknown);
      else
	fprintf(fp, "%.3f ", 1-map->prob[x][y]);
    fprintf(fp, "\n");
  }

  fclose(fp);
  
  return(TRUE);
}

int
initializeProbabilityGrid(probabilityGrid *m)
{  
  m->prob = NULL;
  return(1);
}


mapProbability
mapAverage(probabilityGrid m){
  register int y, n = 0, x;
  mapProbability sum = (mapProbability) 0;
  for (x=0;x<m.sizeX;x++)
    for (y=0;y<m.sizeY;y++)
      if (m.prob[x][y] >= 0){
        n++;
        sum += m.prob[x][y];
      }
  if (n > 0)
    return((mapProbability) sum/n);
  else
    return((mapProbability) -1);
}

bool
writeVisionMap(char *mapname, char *extension, visionMap *m)
{ 
  FILE *fp; 
  unsigned int x,y;
  char fileName[MAX_STRING_LENGTH];
  
  sprintf(fileName, "%s%s", mapname, extension);

  if ((fp = fopen(fileName, "w")) == NULL)
    {
      printf("# Error: could not open %s\n",fileName);
      return FALSE;
    }
  
    /*--- the header ---------------*/
  
  fprintf(fp,"P5\n");
  fprintf(fp, "# resolution: %d\n", m->resolution);
  
  fprintf(fp,"%03d %03d\n",m->sizeX, m->sizeY);
  fprintf(fp,"255\n");
  
  for (y = 0; y < m->sizeY; y++){
      for (x = 0; x < m->sizeX; x++){
	fputc(m->pix[x][y],fp);
      }
  }
  return TRUE;

}

  
bool
readVisionMap(char *mapname, char *extension, visionMap *m,
	      probabilityGrid *map)
{
  FILE *fp; 
  char buf[MAX_STRING_LENGTH];
  char fileName[MAX_STRING_LENGTH];
  bool error = FALSE, stop = FALSE;
  unsigned int x, y;
  unsigned fullSizeX, fullSizeY;
  pixel **pix = NULL;
  m->offsetX = m->offsetY = 0;
  
  sprintf(fileName, "%s%s", mapname, extension);

  if ((fp = fopen(fileName, "r")) == NULL)
    {
      printf("# Error: could not open %s\n",fileName);
      return FALSE;
    }
  
  /*--- the header ---------------*/
  

  if (!error)
    error = (fgets(buf, MAX_STRING_LENGTH, fp) == NULL);

  if (!error)
    error = (fgets(buf, MAX_STRING_LENGTH, fp) == NULL);
  
  if (!error)
    if (sscanf(buf, "# resolution: %d", &(m->resolution)) != 1){
      error = TRUE;
      m->resolution = map->resolution;
    }

  fprintf(stderr, "# Vision map resolution: %d\n",m->resolution);
  
  rewind(fp);
  error = (fgets(buf, MAX_STRING_LENGTH, fp) == NULL);

  while (!error && !stop){
    error = fgets(buf, MAX_STRING_LENGTH, fp) == NULL;
    if (!error)
      stop = (buf[0] != '#');
  }
  
  /*------ the dimension --------*/
  if (!error){
    error = (sscanf(buf,"%d %d",&fullSizeX,&fullSizeY) != 2);
  }

  fprintf( stderr, "# Size of %s: %d %d\n", fileName, fullSizeX, fullSizeY);

#ifdef WOLFRAM
  m->sizeX = map->sizeX;
  m->sizeY = map->sizeY;
#else
  m->sizeX = fullSizeX;
  m->sizeY = fullSizeY;
#endif
  
  if (!error){
    pix = (pixel **) allocate2D(fullSizeX, fullSizeY, PIXEL);
    m->pix = (pixel **) allocate2D(m->sizeX, m->sizeY, PIXEL);
    error = (fgets(buf, MAX_STRING_LENGTH, fp) == NULL);
    /*--- the image ---------------*/
    for (y = 0; y < fullSizeY && !error; y++){
      for (x = 0; x < fullSizeX && !error; x++){
	pix[x][y] = fgetc(fp);
      }
    }
  }

  fclose(fp);

#define SMALLERMAP 0
  if (SMALLERMAP){
    int offsetX = 35, offsetY = 52;
    for (x = 0; x < m->sizeX; x++)
      for (y = 0; y < m->sizeY; y++)
	m->pix[x][y] = 0;
    for (x = 0; x < fullSizeX-offsetX; x++)
      for (y = 0; y < fullSizeY-offsetY; y++)
	m->pix[x+offsetX][y+offsetY] = pix[x][y];
  }
  else
    for (x = 0; x < m->sizeX; x++)
      for (y = 0; y < m->sizeY; y++)
	m->pix[x][y] = pix[x+map->shiftedX][y+map->shiftedY];
  
  if (pix != NULL)
    free2D((void **) pix, fullSizeX, PIXEL);
  
  return !error;
}

/*****************************************************************************
 Save a pgm image
*****************************************************************************/

static 
void WritePGM(unsigned char *data, int width, int height, const char *filename)
{
  FILE *fp; 
  fp = fopen(filename, "wb");
  if (fp==0)
    {
      printf("WritePGM: could not open %s for writing\n",filename);
      exit(1);
    }

  /*--- the header ---------------*/
	fprintf(fp,"P5\n");
	fprintf(fp,"%03d %03d\n",width, height);
	fprintf(fp,"255\n");

  /*--- write image out to file ---------------*/
  fwrite(data, 1, width*height, fp);

  fclose(fp);
}

/*****************************************************************************/


float
realCoordinateOfMapCoordinate( int x, float offset, int resolution){
  return (x + 0.5) * resolution + offset;
}

float 
floatMapCoordinateOfRealCoordinate( float x, float offset, int resolution) {
  return ( x - offset) / resolution;
}

int
mapCoordinateOfRealCoordinate( float x, float offset, int resolution) {
  return ( x - offset) / resolution;
}

int
coordinateInMap(int x, int y, probabilityGrid m) {
    return (x >= 0 && x < m.sizeX && y >= 0 && y < m.sizeY); 
}

probability
probOfRealCoordinate( float x, float y, probabilityGrid* map)
{
  int gridX = mapCoordinateOfRealCoordinate( x,
					     map->offsetX,
					     map->resolution);
  int gridY = mapCoordinateOfRealCoordinate( y,
					     map->offsetY,
					     map->resolution);

  return map->prob[gridX][gridY];
}  


float
positionProbabilityMap( int x, int y, probabilityGrid m, float additionalSize)
{
  /* If it is not a cad map the walls are thickened by the robot size. */
  if ( ! globalMapParameters.cadMap) {

    if ( coordinateInMap(x, y, m) && (m.prob[x][y] != m.unknown))
      return 1.0 - m.prob[x][y];
    else
      return UNEXPLORED;
  }
  /* In this case we take the maximum probability of the neighbouring cells. */
  else {
    /* Is the point in the map? */
    if ( coordinateInMap(x, y, m) && (m.prob[x][y] != m.unknown)) {
      int robotRadius = round( (ROB_RADIUS + additionalSize)/ m.resolution);
      int xCnt, yCnt;
      probability minProb = 1.0, tmpProb;

      /* Now look at the neighbouring cells. */
      for ( xCnt = - robotRadius; xCnt <= robotRadius; xCnt++)
	for ( yCnt = - robotRadius; yCnt <= robotRadius; yCnt++)
	  if ( coordinateInMap( x + xCnt, y + yCnt, m)) {
	    float dist = sqrt( fSqr( m.resolution * xCnt) +
			       fSqr( m.resolution * yCnt));
	    if ( dist < ROB_RADIUS + additionalSize) {
	      tmpProb = 1.0 - m.prob[x+xCnt][y+yCnt];
	      if ( tmpProb < minProb) 
		minProb = tmpProb;
	    }
	  }
      return minProb;
    }
    else
      return UNEXPLORED;
  }
}


mapProbability
occupancyProbabilityMapUnknown(int x, int y, probabilityGrid m) {
  if (coordinateInMap(x, y, m))
    return m.prob[x][y];
  else
    return m.unknown;
}



mapProbability
occupancyProbabilityMap(int x, int y, probabilityGrid m) {
  mapProbability prob = 0;
  if (m.initialized){
    prob  = occupancyProbabilityMapUnknown(x, y, m);
    if (prob == m.unknown)
      prob = m.average;
  }
  return prob;
}


/***********************************************************************
 * Computes the weighted occupancy probability of a position.
 ***********************************************************************/
probability
weightedOccupancyProbability( realCellList* cellList,
			      probabilityGrid* initialOccupancyProbs)
{
  probability weightedProb = 0.0;
  probability prob;
  int i;
  
  for ( i = 0; i < cellList->numberOfCells; i++) {

    if ( cellList->inMap[i]) 
      prob = probOfRealCoordinate( cellList->cell[i].pos.x,
				   cellList->cell[i].pos.y,
				   initialOccupancyProbs);
    else
      prob = 1.0 - UNEXPLORED;
    
    weightedProb += cellList->cell[i].prob * prob;
  }

  return weightedProb;
}

/************************************************************
 * replaces the unknown fields of a Map
 ***********************************************************/


static mapProbability
neighborAverage(probabilityGrid *m, int x, int y){
  register int i, j;
  int minX, maxX, minY, maxY;
  
  
  mapProbability average, sum = 0.0;
  int count = 0;

  minX = iMax(0, x-1);
  minY = iMax(0, y-1);
  maxX = iMin(m->sizeX-1, x+1);
  maxY = iMin(m->sizeY-1, y+1);
  
  for (i = minX; i < maxX; i ++)
    for (j = minY; j < maxY; j++)
      if (m->prob[i][j] != m->unknown){
	sum += m->prob[i][j];
	count++;
      }
  if (count > 0)
    average = sum / count;
  else
    average = m->unknown;

  return average;
}
      

void
fillProbabilityGrid(probabilityGrid *m) {
  register int x,y;
  mapProbability **prob;

  prob = (mapProbability **) allocate2D(m->sizeX, m->sizeY, MAP_PROBABILITY);

  for (x = 0; x < m->sizeX; x++)
    for (y = 0; y < m->sizeY; y++)
      if (m->prob[x][y] == m->unknown) { fprintf(stderr, "filled %d %d\n", x, y);
      prob[x][y] = neighborAverage(m, x, y); }
      else
	prob[x][y] = m->prob[x][y];

  for (x = 0; x < m->sizeX; x++)
    for (y = 0; y < m->sizeY; y++)
      m->prob[x][y] = prob[x][y];

  free(prob);
}



probability
positionProbabilityPlanMap( int x, int y,
			    probabilityGrid m,
			    float additionalSize)
{
  mapProbability cellProb = 0.0;
  bool closeToObstacle = FALSE;
  int xCnt, yCnt;
  mapProbability threshold = 0.8;
  mapProbability planProb = 0.9;
  
  /* Is the point in the map? */
  if ( coordinateInMap(x, y, m)){
    if (m.prob[x][y] > threshold)
      cellProb = m.prob[x][y];
    else {
      int radius = round( (ROB_RADIUS + additionalSize)/ m.resolution);
      
      /* Now look at the neighbouring cells. */
      for ( xCnt = - radius; xCnt <= radius && !closeToObstacle; xCnt++)
	for ( yCnt = - radius; yCnt <= radius && !closeToObstacle; yCnt++)
	  if ( coordinateInMap( x + xCnt, y + yCnt, m)) {
	    if ((m.prob[x+xCnt][y+yCnt]) > threshold)
	      closeToObstacle = TRUE;
	  }
      if (closeToObstacle){
	if (m.prob[x][y] > planProb)
	  cellProb = m.prob[x][y];
	else
	  cellProb = planProb;
      }
      else
	cellProb = m.prob[x][y];
    }
  }
  else
    cellProb = 0.9999;
  
  return cellProb;
}




probabilityGrid
planMap( probabilityGrid* map, float additionalSize)
{
  int x,y;
  probabilityGrid planMap = *map;
  planMap.prob = (mapProbability**)
    allocate2D( map->sizeX, map->sizeY, MAP_PROBABILITY);
  
  for (x = 0; x < planMap.sizeX; x++)
    for (y = 0; y < planMap.sizeY; y++)
      planMap.prob[x][y] =
	positionProbabilityPlanMap( x, y,
				    *map, additionalSize);
  
  return planMap;

}


/* We assume that the robot is two grid cells away from the closest obstacle. */
#define OBSTACLE_DISTANCE 0

void
setFreeSpace( probabilityGrid* positionProbs)
{
  int x, y, dx, dy;
  int numberOfFreeCells = 0;
  probability unknown = positionProbs->unknown;

  /* Allocate memory. */
  if ( positionProbs->freeSpace != NULL)
    free1D( positionProbs->freeSpace, POSITION);

  positionProbs->freeSpace = (position*)
    allocate1D( positionProbs->sizeX * positionProbs->sizeY, POSITION);

  for ( x = OBSTACLE_DISTANCE; x < positionProbs->sizeX - OBSTACLE_DISTANCE; x++)
    for ( y = OBSTACLE_DISTANCE; y < positionProbs->sizeY - OBSTACLE_DISTANCE; y++) {
      probability prob = positionProbs->prob[x][y];
      if ( prob != unknown && prob > 0.9) {
	int foundObstacle = FALSE;
	for ( dx = x - OBSTACLE_DISTANCE; dx <= x + OBSTACLE_DISTANCE && ! foundObstacle; dx++) 
	  for ( dy = y - OBSTACLE_DISTANCE; dy <= y + OBSTACLE_DISTANCE && ! foundObstacle; dy++) {
	    probability tmpProb = positionProbs->prob[dx][dy];
	    if ( tmpProb == unknown || tmpProb < 0.9) 
	      foundObstacle = TRUE;
	  }
	      
	if ( ! foundObstacle) {
	  positionProbs->freeSpace[numberOfFreeCells].x = x;
	  positionProbs->freeSpace[numberOfFreeCells].y = y;
	  numberOfFreeCells++;
	}
      }
    }
  
  positionProbs->numberOfFreeCells = numberOfFreeCells;

  fprintf(stderr, "# Found %d (%f %%) free cells.\n", numberOfFreeCells,
	  (float) (numberOfFreeCells * 100) / (positionProbs->sizeX * positionProbs->sizeY));
}


static void
initializeOnlineMap( probabilityGrid* onlineMap, char* mapFileName)
{
  /* If keyword ONLINE_MAP is set, then don't read a grid map. */
  if ( strncmp( mapFileName, ONLINE_MAP, strlen(ONLINE_MAP)) == 0) {
    
#define DEFAULT_ONLINE_MAP_SIZE 700
#define DEFAULT_ONLINE_MAP_CENTER 350

    onlineMap->sizeX = DEFAULT_ONLINE_MAP_SIZE;
    onlineMap->sizeY = DEFAULT_ONLINE_MAP_SIZE;
    
    onlineMap->unknown = globalMapParameters.unknown;
    
    onlineMap->resolution = globalMapParameters.desiredResolution;
    
    onlineMap->offsetX = 0.0;
    onlineMap->offsetY = 0.0;
    
    onlineMap->maxRealX = onlineMap->sizeX * onlineMap->resolution;
    onlineMap->maxRealY = onlineMap->sizeY * onlineMap->resolution;
    
    onlineMap->initialized = FALSE;
  }
  else {
    if ( ! readGridMap( mapFileName, GRID_MAP_EXTENSION, onlineMap))
      exit(0);
    else 
      /* We read the map and preprocess the inverted map. */
      onlineMap->initialized = TRUE;
  }
}



void
incorporatePartialMap( unsigned char* partialProbs,
		       int partialSizeX, int partialSizeY,
		       int partialFirstX, int partialFirstY,
		       float resolution,
		       probabilityGrid* map)
{
  /* These are the boarders of the map. */
  static int minIndexX = 0, endIndexX = 0;
  static int minIndexY = 0, endIndexY = 0;

  int x, y;
  int shiftX, shiftY;
  
  /* The boarders of the map have changed. */
  if ( ! map->initialized) {

    fprintf(stderr, "Map not initialized.\n");
    minIndexX  = partialFirstX;
    minIndexY  = partialFirstY;
    endIndexX  = partialFirstX + partialSizeX;
    endIndexY  = partialFirstY + partialSizeY;
    map->sizeX = partialSizeX;
    map->sizeY = partialSizeY;
    
    map->prob = (mapProbability**) allocate2D( partialSizeX, partialSizeY, MAP_PROBABILITY);

    map->offsetX   = minIndexX * resolution;
    map->offsetY   = minIndexY * resolution;
    map->maxRealX  = endIndexX * resolution;
    map->maxRealY  = endIndexY * resolution;

    map->initialized = TRUE;
  }  
  else if ( partialFirstX < minIndexX || partialFirstY < minIndexY
	    || (partialFirstX + partialSizeX) > endIndexX
	    || (partialFirstY + partialSizeY) > endIndexY) {
    
    /* Find the new boarders. */
    int newMinIndexX = iMin( partialFirstX, minIndexX);
    int newMinIndexY = iMin( partialFirstY, minIndexY);
    int newEndIndexX = iMax( partialFirstX + partialSizeX, endIndexX);
    int newEndIndexY = iMax( partialFirstY + partialSizeY, endIndexY);
    int newSizeX     = newEndIndexX - newMinIndexX;
    int newSizeY     = newEndIndexY - newMinIndexY;
    
    mapProbability** newProbs = (mapProbability**)
      allocate2D( newSizeX, newSizeY, MAP_PROBABILITY);

    writeLog( "# Resize: (%d -> %d) (%d -> %d)  (%d -> %d)  (%d -> %d)  (%d -> %d)  (%d -> %d)\n",
	      minIndexX, newMinIndexX, minIndexY, newMinIndexY, endIndexX, newEndIndexX,
	      endIndexY, newEndIndexY, map->sizeX, newSizeX, map->sizeY, newSizeY);
    
    /* Shift between old and new map. */
    shiftX = newMinIndexX - minIndexX;
    shiftY = newMinIndexY - minIndexY;
    
    writeLog( "# Maps are shifted by %d %d cells.\n", shiftX, shiftY);
    
    /* Copy the old stuff into the new map. */
    for ( x = 0; x < newSizeX; x++) {
      
      int xInOldMap = x + shiftX;

      /* Is the x index in the old map? */
      if ( xInOldMap < 0 || xInOldMap >= map->sizeX) {
      	for ( y = 0; y < newSizeY; y++) 
	  newProbs[x][y] = map->unknown;
      }
      else {
      	for ( y = 0; y < newSizeY; y++) {
	  int yInOldMap = y + shiftY;

	  /* Is the y index in the old map? */
	  if ( yInOldMap < 0 || yInOldMap >= map->sizeY) 
	    newProbs[x][y] = map->unknown;
	  else
	    newProbs[x][y] = map->prob[xInOldMap][yInOldMap];
	}
      }
    }
    
    /* Free memory of the old map. */
    free2D( (void**) map->prob, map->sizeX, MAP_PROBABILITY);

    /* Readjust all relevant values in the map structure. */
    map->offsetX   = newMinIndexX * resolution;
    map->offsetY   = newMinIndexY * resolution;
    map->maxRealX  = newEndIndexX * resolution;
    map->maxRealY  = newEndIndexY * resolution;
    map->sizeX     = newSizeX;
    map->sizeY     = newSizeY;

    /* Store values for next function call. */
    minIndexX = newMinIndexX;
    minIndexY = newMinIndexY;
    endIndexX = newEndIndexX;
    endIndexY = newEndIndexY;
    
    map->prob      = newProbs;
  }

  writeLog( "# New map boarders: [%f %f] -- [%f %f]  size %d %d\n",
	   map->offsetX, map->maxRealX, map->offsetY, map->maxRealY, map->sizeX,
	   map->sizeY);

  /* Shift between partial and global map in grid cells. */
  shiftX = partialFirstX - minIndexX;
  shiftY = partialFirstY - minIndexY;

  /* Copy the new partial map into the global map. */
  for ( x = 0; x < partialSizeX; x++) {

    int xDest = x + shiftX;

    for ( y = 0; y < partialSizeY; y++) {
      int yDest = y + shiftY;
      int mapIndex = x * partialSizeY + y;
      
      if ( partialProbs[mapIndex] == 0)
	map->prob[xDest][yDest] = map->unknown;
      else {
	map->prob[xDest][yDest] = fNorm( (float) partialProbs[mapIndex],
					 255, 1, 0, 1);
      }
    }
  }  
}



