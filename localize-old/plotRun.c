
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/plotRun.c,v $
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
 * $Log: plotRun.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.13  1998/10/02 15:16:40  fox
 * Several improvements of condensation. Updated the function calls to the
 * new version of the distancServer.
 *
 * Revision 1.12  1998/09/22 15:40:48  wolfram
 * Copied function from function.c so that it can be linked again.
 *
 * Revision 1.11  1998/09/18 15:44:28  fox
 * First attempt to get a running system after museum.
 *
 * Revision 1.10  1997/10/06 22:19:45  wolfram
 * Adopted to latest version of Localize
 *
 * Revision 1.9  1997/08/16 22:59:51  fox
 * Last version before I change selsection.
 *
 * Revision 1.8  1997/08/14 04:56:00  wolfram
 * Added alignmaps.c
 *
 * Revision 1.7  1997/06/20 07:36:11  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.6  1997/04/02 08:57:34  fox
 * Display the robot right after sensor selection.
 *
 * Revision 1.5  1997/01/27 14:23:18  fox
 * Xfig format can be dumped.
 *
 * Revision 1.4  1997/01/19 19:31:17  fox
 * yeah
 *
 * Revision 1.3  1996/12/02 10:32:10  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.2  1996/10/25 07:28:46  fox
 * Improved dump of graphic output.
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


#include "graphic.h"
#include "general.h"
#include "probGrid.h"
#include "map.h"
#include "script.h"
#include "proximityTools.h"
#include "file.h"
#include "allocate.h"
#include "fileNames.h"

/**************************************************************************
**************************************************************************
* Local variables and defines.
 **************************************************************************
 **************************************************************************/

#define MYMAXCOLORS     155
#define COL_BACK        0                  /* background of window       */
#define COL_UNKNOWN     C_YELLOW           /* color of unknown area      */
#define COL_RED         C_RED
#define COL_BLACK       54
#define COL_WHITE       154

#define COL_ROBODIR    C_BLACK
#define COL_BACKGROUND C_BLUE

#define ESTIMATED_ROB_COLOR C_YELLOW
#define SCRIPT_ROB_COLOR    C_RED

typedef struct{
  int x;
  int y;
} windowPosition;

/********************************************************************
 ********************************************************************
 * Forward declarations of internal functions.
 ********************************************************************
 ********************************************************************/

static int
gridWindowScale(int sizeX, int sizeY);

static windowPosition
mapWindowPosition(gridWindow* win, realPosition pos);


/********************************************************************
 ********************************************************************
 * Global functions.
 ********************************************************************
 ********************************************************************/

distance
realPositionDistance(realPosition pos1, realPosition pos2)
{
  return (distance) sqrt(fSqr(pos1.x - pos2.x) + fSqr(pos1.y - pos2.y));
}

long
round(double x) {
  return (long) floor(x + 0.5);
}

/*****************************************************************************/
/*   float-Normierung: b = norm (a,...)                                      */
/*****************************************************************************/
float 
fNorm( float a, float amin, float amax, float bmin, float bmax)
{
  if (amin==amax) {
    fprintf( stderr, "Warning: cannot norm from empty interval.\n");
    return( a < amin ? bmin : bmax);
  }
  else
    if (bmin==bmax)
      return(bmax);
    else
      return( (a-amin) / (amax-amin) * (bmax-bmin) + bmin);
}



gridWindow *
createWindow(probabilityGrid *m, char* text) {
/* creates a window under EZX depending on map_size_x, map_size_y and
   scale (size of a pixel) */

    gridWindow *mapwin;
    
    mapwin = (gridWindow *) malloc(sizeof(gridWindow));
    mapwin->scale = gridWindowScale(m->sizeX, m->sizeY);

    mapwin->gridSizeX = m->sizeX;
    mapwin->gridSizeY = m->sizeY;
    mapwin->gridResolution = m->resolution;
    
    mapwin->sizeX = m->sizeX * mapwin->scale;
    mapwin->sizeY = m->sizeY * mapwin->scale;

    mapwin->firstTimePosition = TRUE;

    EZX_NoMotionEvents();
    mapwin->window = EZX_MakeWindow(text,mapwin->sizeX,mapwin->sizeY,"0,0");
    EZX_SetWindowBackground (mapwin->window,C_RED);
    EZX_Flush();

    mapwin->startX = 0;
    mapwin->startY = 0;
    return(mapwin);
}

void
displayMapWindow( probabilityGrid *m,
		  gridWindow *mapwin)
{
  
/* plots map in window */

  probability maxProb = 0.0;
  int x,y;
  
  int winX, winY;
  int endX = mapwin->startX + m->sizeX * mapwin->scale;
  int endY = mapwin->startY + m->sizeY * mapwin->scale;

  for ( x = 0; x < m->sizeX; x++)
    for ( y = 0; y < m->sizeY; y++)
      if ( m->prob[x][y] > maxProb) 
	maxProb = m->prob[x][y];

  y=m->sizeY;
  for (winY=mapwin->startY; winY <= endY; winY += mapwin->scale){
    y--;
    x=0;
    for (winX=mapwin->startX; winX < endX; winX += mapwin->scale){
      if ( m->prob[x][y] <= 0.0 || maxProb == 0.0)
	EZX_SetColor( COL_WHITE);
      else {
	EZX_SetColor( COL_WHITE -
		      (int) round((double) (m->prob[x][y]) / maxProb
				  * (COL_WHITE-COL_BLACK)));
      }
      EZX_FillRectangle(mapwin->window,winX,winY,
			mapwin->scale,mapwin->scale);
      x++;
    }
  }
  
  EZX_Flush();
  
}

void
displayOverlayWindow( probabilityGrid *m, gridWindow *mapwin) {
/* plots map in window */

    int x,y;
    
    int winX, winY;
    int endX = mapwin->startX + m->sizeX * mapwin->scale;
    int endY = mapwin->startY + m->sizeY * mapwin->scale;

    /*     EZX_SetColor(COL_UNKNOWN); */
    /*     EZX_FillRectangle(mapwin->window,mapwin->startX,mapwin->startY,endX,endY); */
    
    y=m->sizeY;
    for (winY=mapwin->startY; winY <= endY; winY += mapwin->scale){
	y--;
	x=0;
	for (winX=mapwin->startX; winX < endX; winX += mapwin->scale){
	  if ( m->prob[x][y] > 0.99) {  
	    EZX_SetColor( COL_BACKGROUND);
	    EZX_FillRectangle(mapwin->window,winX,winY,
			      mapwin->scale,mapwin->scale);
	  }
	  x++;
	}
      }
    
    EZX_Flush();
}
   

void
displayRob(gridWindow *win, robot rob, int color, int fill) {

    windowPosition pos1 = mapWindowPosition( win, rob.pos);

    windowPosition pos2;

    realPosition pos;
    
    pos.x = rob.pos.x + (rob.radius-3) * cos(rob.pos.rot);
    pos.y = rob.pos.y + (rob.radius-3) * sin(rob.pos.rot);
    pos.rot = rob.pos.rot;
    
    pos2 = mapWindowPosition( win, pos);
    EZX_SetColor(color);
    EZX_SetLineWidth(3);
    if ( fill)
      EZX_FillCircle(win->window, pos1.x, pos1.y,
		     (float) rob.radius/ (float) win->gridResolution* (float) win->scale);
    else
      EZX_DrawCircle(win->window, pos1.x, pos1.y,
		     (float) rob.radius/ (float) win->gridResolution* (float) win->scale);
      
    EZX_SetColor(color);
    EZX_DrawLine(win->window, pos1.x, pos1.y, pos2.x, pos2.y);

    EZX_Flush();
}

void
displayRobPath(gridWindow *win, robot* rob,
	       int start, int end,
	       int color, int fill)
{
  int cnt;
  int firstPoint = TRUE;
  windowPosition oldPos;

#ifdef PLOT_ALL_ROBOTS
  /* Plots a robot at each position. */
  for ( cnt = start; cnt <= end; cnt++) {

    windowPosition pos1 = mapWindowPosition( win, rob[cnt].pos);
    windowPosition pos2;

    realPosition pos;
    
    pos.x = rob[cnt].pos.x + (rob[cnt].radius-3) * cos(rob[cnt].pos.rot);
    pos.y = rob[cnt].pos.y + (rob[cnt].radius-3) * sin(rob[cnt].pos.rot);
    pos.rot = rob[cnt].pos.rot;

    pos2 = mapWindowPosition( win, pos);
    EZX_SetColor(color);
    if ( fill)
      EZX_FillCircle( win->window, pos1.x, pos1.y,
		      (float) rob[cnt].radius / (float) win->gridResolution * (float) win->scale);
    else
      EZX_DrawCircle( win->window, pos1.x, pos1.y,
		      (float) rob[cnt].radius / (float) win->gridResolution * (float) win->scale);
    EZX_SetColor(C_BLACK);
    EZX_DrawLine(win->window, pos1.x, pos1.y, pos2.x, pos2.y);
  }
#endif
fprintf(stderr, "%d %d\n", start, end);
  /* This version connects the position with lines. */
  for ( cnt = start; cnt <= end; cnt++) {
    
    windowPosition actualPos = mapWindowPosition( win, rob[cnt].pos);
    
    EZX_SetColor(color);

    if ( ! firstPoint &&
	 ( actualPos.x - oldPos.x < 1000) && ( actualPos.y - oldPos.y < 1000) ) {
      EZX_DrawLine(win->window, actualPos.x, actualPos.y, oldPos.x, oldPos.y);
    }
    firstPoint = FALSE;

    oldPos = actualPos;
  }
  
  EZX_Flush();
}
gridWindow *
createRobotWindow( probabilityGrid *map, char* text,
		   int size, int x, int y, int scale) {
  /* creates a window under EZX for the robot and sensor information */

  char corner[80];
  gridWindow *robwin;
  
  robwin = (gridWindow *) malloc(sizeof(gridWindow));

  robwin->gridSizeX = size / map->resolution;
  if (robwin->gridSizeX % 2 == 0)
    robwin->gridSizeX ++;

  robwin->gridSizeY = robwin->gridSizeX;
  
  robwin->scale = scale;
  
  robwin->sizeX = robwin->gridSizeX * robwin->scale;
  robwin->sizeY = robwin->gridSizeY * robwin->scale;
  robwin->gridResolution = map->resolution;

  
  robwin->startX = 0;
  robwin->startY = 0;
  robwin->planeType = XYPLANE;

  sprintf(corner,"+%d+%d",x,y);
    
  EZX_NoMotionEvents();
  robwin->window = EZX_MakeWindow(text,robwin->sizeX,robwin->sizeY,corner);
  EZX_SetWindowBackground (robwin->window,C_BLACK);
  EZX_Flush();

  return(robwin);
}


windowPosition
robotWindowPosition(gridWindow* robwin, windowPosition center,
		    realPosition pos) {
    windowPosition winPos;
    winPos.x = 
      robwin->startX
	+ (int) round(((pos.x -
			((int) (center.x - robwin->gridSizeX/2) )
			* robwin->gridResolution)/ robwin->gridResolution + 0.5)
		      * robwin->scale);
    winPos.y =
      robwin->startY
	+ (int) round(((robwin->gridSizeY -
		       (pos.y - ((int) (center.y - robwin->gridSizeY/2) -2 )
			* robwin->gridResolution)/ robwin->gridResolution + 0.5))
		      * robwin->scale);
    return winPos;
}


/********************************************************************
 ********************************************************************
 * Local functions.
 ********************************************************************
 ********************************************************************/


static int
gridWindowScale(int sizeX, int sizeY) {
/*     int scaleX = (int) (1280 / sizeX); */
/*     int scaleY = (int) (960 / sizeY); */
    int scaleX = (int) (800 / sizeX);
    int scaleY = (int) (600 / sizeY);

    if (scaleX < 1)
	scaleX = 1;
    if (scaleY < 1)
        scaleY = 1;

    if (scaleY < scaleX)
	return(scaleY);
    else
	return(scaleX);
}

static windowPosition
mapWindowPosition(gridWindow* win, realPosition pos)
{
  windowPosition winPos;

  winPos.x = win->startX + (int) round((pos.x / win->gridResolution + 0.5) * win->scale);
  winPos.y = win->startY + (int) round((win->gridSizeY - (pos.y / win->gridResolution + 0.5)) * win->scale);
  return winPos;
}


void
readPath( char* fileNameStart, int start, int end,
	  robot* scriptRob, robot* estimatedRob)
{
  int cnt;

  for ( cnt = start; cnt < end; cnt++) {
    int sizeX, sizeY, resolution,numberOfMax;
    probability unknown;
    char fileName[MAX_FILE_NAME];
    FILE* fp;
    
    /* File including extender. */
    addExtenderNumber( fileNameStart, fileName, cnt);
    fprintf( stderr, "Get position from %s.\n", fileName);
    if ( (fp = fopen( fileName, "r")) == NULL)
      exit(-1);
    
    /* Fixed header. */
    fscanf (fp, "%d %d %d %g", &(sizeX), &(sizeY),
	    &(resolution), &(unknown));
    fscanf (fp, "%f %f %f",
	    &(scriptRob->pos.x), &(scriptRob->pos.y), &(scriptRob->pos.rot));
    
    fscanf (fp, "%d ", &numberOfMax);

    /* Now get the interesting thing. */
    if ( numberOfMax > 0)
      fscanf (fp, "%f %f %f",
	      &(estimatedRob[cnt].pos.x),
	      &(estimatedRob[cnt].pos.y),
	      &(estimatedRob[cnt].pos.rot));

    fclose( fp);
  }
}


void
readProbabilities( char* fileNameStart,
		   probabilityGrid* map,
		   probabilityGrid* overlay,
		   robot* scriptRob,
		   robot* estimatedRob,
		   int plotCnt,
		   int* numberOfMax,
		   float* maxX,
		   float* maxY,
		   float* maxRot,
		   float* maxProb)
{
  int x, y;
  FILE* fp;
  char fileName[MAX_FILE_NAME];
  int prob;
  int cnt;

  addExtenderNumber( fileNameStart, fileName, plotCnt);
  if ( (fp = fopen( fileName, "r")) == NULL) {
    fprintf( stderr, "Cannot open %s.\n", fileName);
    exit(-1);
  }

  /* Fixed header. */
  fscanf (fp, "%d %d %d %g", &(map->sizeX), &(map->sizeY),
	  &(map->resolution), &(map->unknown));
  fscanf (fp, "%f %f %f",
	  &(scriptRob->pos.x), &(scriptRob->pos.y), &(scriptRob->pos.rot));

  fscanf (fp, "%d ", numberOfMax);
  for ( cnt = 0; cnt < *numberOfMax; cnt++) {
    fscanf (fp, "%f %f %f %f", &(maxX[cnt]), &(maxY[cnt]), &(maxRot[cnt]), &(maxProb[cnt]));
  }
  if ( *numberOfMax > 0) {
    estimatedRob->pos.x = maxX[0];
    estimatedRob->pos.y = maxY[0];
    estimatedRob->pos.rot = maxRot[0];
  }
  
  /* Set the sizes and allocate memory if not done yet. */
  if ( map->prob == NULL)
    map->prob = (mapProbability**) allocate2D( map->sizeX, map->sizeY, MAP_PROBABILITY);
  
  /* Set the sizes and allocate memory if not done yet. */
  overlay->sizeX = map->sizeX;
  overlay->sizeY = map->sizeY;
  if ( overlay->prob == NULL)
    overlay->prob =
      (mapProbability**) allocate2D( overlay->sizeX, overlay->sizeY, MAP_PROBABILITY);
  
  /* Get the probabilities. */
  for ( y = map->sizeY-1; y >= 0; y--)    
    for ( x = 0; x < map->sizeX; x++) 
      fscanf( fp, "%f ", &(map->prob[x][y]));

  /* In the first plot file the overlay is also given. */
  if ( plotCnt == 0) {
    for ( y = overlay->sizeY-1; y >= 0; y--)    
      for ( x = 0; x < overlay->sizeX; x++) {
	fscanf( fp, "%d ", &prob);
	overlay->prob[x][y] = prob; 
      }
  }

  fclose( fp);
}


#define XFIG_SCALE          4.5
#define XFIG_HEADER         "#FIG 3.1\nPortrait\nCenter\nMetric\n1200 2\n"

static probability
plotXfigHeader( FILE* fp, probabilityGrid* map)
{
  int x, y;
  int minSet = FALSE;
  int minX=0, maxX=map->sizeX, minY=0, maxY=map->sizeY;
  probability max = 0.0;
  
  int lastY = XFIG_SCALE * map->sizeY * map->resolution;

  for ( x = 0; x < map->sizeX; x++)
    for ( y = 0; y < map->sizeY; y++) 
      if ( map->prob[x][y] > map->unknown) {
	if ( map->prob[x][y] > max)
	  max = map->prob[x][y];
	if ( ! minSet) {
	  minX = XFIG_SCALE * x * map->resolution;
	  minY = XFIG_SCALE * y * map->resolution;
	  minSet = TRUE;
	}
	maxX = XFIG_SCALE * x * map->resolution;
	maxY = XFIG_SCALE * y * map->resolution;
      }

  fprintf( fp, XFIG_HEADER);

  /* Start of the group. */
  fprintf( fp, "6 %d %d %d %d\n", minX, lastY - maxY, maxX, lastY - minY);

  return max;
}


static void
plotXfigRectangle( FILE* fp,
		   int x, int y,
		   probabilityGrid *map,
		   probability max)
{
#define XFIG_FORMAT_STRING  "2 2 0 0 7 0 5 0 %d 0.000 0 0 -1 0 0 5\n       "
#define MAX_XFIG_INTENSITY  20.0

  /* Position of lower left corner. */
  int realX = XFIG_SCALE * x * map->resolution;
  int realY = XFIG_SCALE * y * map->resolution;
  int size =  XFIG_SCALE * map->resolution;
  int maxY  = XFIG_SCALE * map->resolution * map->sizeY;

  int intensity = (int) fNorm( map->prob[x][y], 0.0, max, 0.0, MAX_XFIG_INTENSITY);

  if ( intensity > 0) {
    intensity = iMin( MAX_XFIG_INTENSITY, intensity * 3);
    
    fprintf( fp, XFIG_FORMAT_STRING, intensity);
    
    /* Lower left */
    fprintf( fp, "%d %d ", realX, maxY - realY);
    
    /* Lower right */
    fprintf( fp, "%d %d ", realX + size, maxY - realY);
    
    /* Upper right */
    fprintf( fp, "%d %d ", realX + size, maxY - realY - size);
    
    /* Upper left */
    fprintf( fp, "%d %d ", realX, maxY - realY - size);
    
    /* Lower left */
    fprintf( fp, "%d %d\n", realX, maxY - realY);
  }
}


#define XFIG_EXTENDER "fig"
static void
dumpXfig( char* fileNameStart,
	  probabilityGrid* map,
	  int plotCnt,
	  int* numberOfMax,
	  float* maxX,
	  float* maxY,
	  float* maxRot,
	  float* maxProb)
{
  int x, y;
  FILE* fp;
  char tmpName[MAX_FILE_NAME], fileName[MAX_FILE_NAME];
  probability max = 0.0;

  addExtenderNumber( fileNameStart, tmpName, plotCnt);
  addExtender( tmpName, fileName, XFIG_EXTENDER);
  if ( (fp = fopen( fileName, "w")) == NULL) {
    fprintf( stderr, "Cannot open %s.\n", fileName);
    exit(-1);
  }

  max = plotXfigHeader( fp, map);

  for ( x = 0; x < map->sizeX; x++)
    for ( y = 0; y < map->sizeY; y++) 
      if ( map->prob[x][y] > map->unknown) 
	plotXfigRectangle( fp, x, y, map, max);

  /* Mark the end of the group. */
  fprintf( fp, "-6\n");

  fclose(fp);
}



#define MAX_NUMBER_OF_PLOTS 2000

int
main( int argc, char** argv)

{
  int i;
  char* fileNameStart = "plot";
  gridWindow* win = NULL;
  probabilityGrid map;
  probabilityGrid overlay;
  robot scriptRob[MAX_NUMBER_OF_PLOTS], estimatedRob[MAX_NUMBER_OF_PLOTS];

  int currentPlot = 0;
  int startNumber = 0;
  int showPath = FALSE;
  int showScriptRob = FALSE;
  int showOverlay = TRUE;
  int showMaxima = TRUE;
  
#define MAX_NUMBER_OF_MAX 50
#define MIN_PROB_OF_MAX 0.05
  int numberOfMax, max;
  float maxX[MAX_NUMBER_OF_MAX], maxY[MAX_NUMBER_OF_MAX];
  float maxRot[MAX_NUMBER_OF_MAX], maxProb[MAX_NUMBER_OF_MAX]; 
  
  map.prob = overlay.prob = NULL;
  map.resolution = overlay.resolution = 15.0;

  for ( i = 0; i < MAX_NUMBER_OF_PLOTS; i++)
    scriptRob[i].radius = estimatedRob[i].radius = 50.0;

  for (i=1; i<argc; i++) {
    if ((strcmp(argv[i],"start")==0))
      if ( argc < i + 2)
	fprintf ( stderr, "Need a number following keyword \"start\"\n");
      else {
	startNumber = atoi( argv[++i]);
	currentPlot = startNumber;
      }
    else if ((strcmp(argv[i],"+path")==0)) 
      showPath = TRUE;
    else if ((strcmp(argv[i],"-nomax")==0))
      showMaxima = FALSE;
    else if ((strcmp(argv[i],"-overlay")==0))
      showOverlay = FALSE;
    else {
      fprintf(stderr,
	      "\nusage: plotRun [start <n> +path -overlay -nomax\n");
      exit(-1);
    }
  }      

  /* This is done to get the overlay from the 0 plot. */
  if ( currentPlot != 0) {
    readProbabilities( fileNameStart,
		       &map, &overlay,
		       scriptRob, estimatedRob,
		       0,
		       &numberOfMax,
		       maxX, maxY, maxRot, maxProb);
    if ( 0 && showPath) 
      readPath( fileNameStart, 0, currentPlot, scriptRob, estimatedRob);
  }
  
  
  while (1) {

    /* Read the file. */
    readProbabilities( fileNameStart,
		       &map, &overlay,
		       &scriptRob[currentPlot], &estimatedRob[currentPlot],
		       currentPlot,
		       &numberOfMax,
		       maxX, maxY, maxRot, maxProb);
    
    /* Dump the file in xfig format. */
    if (1) dumpXfig( fileNameStart,
		     &map, 
		     currentPlot,
		     &numberOfMax,
		     maxX, maxY, maxRot, maxProb);
    
    /* Create the window. */
    if ( win == NULL)
      win = createWindow( &map, "plots");

    /* Display the probabilities. */
    displayMapWindow( &map, win);
    getchar();

    /* Create an overlay of the map. */
    if ( showOverlay)
      displayOverlayWindow( &overlay, win);

    if ( showMaxima) {
      robot maxRob;
      maxRob.radius = 50.0;
      for ( max = 0; max < numberOfMax; max++) {
	if ( maxProb[max] > MIN_PROB_OF_MAX) {
	  maxRob.pos.x = maxX[max];
	  maxRob.pos.y = maxY[max];
	  maxRob.pos.rot = maxRot[max];
	  displayRob( win, maxRob, SCRIPT_ROB_COLOR, FALSE);
	}
      }
    }
    
    /* Show the robot path. */
    if (showPath)
/*       displayRobPath( win, scriptRob, currentPlot, SCRIPT_ROB_COLOR, FALSE); */
      displayRobPath( win, estimatedRob, startNumber, currentPlot, SCRIPT_ROB_COLOR, FALSE);
    else if ( showScriptRob)
      displayRob( win, scriptRob[currentPlot], SCRIPT_ROB_COLOR, FALSE);
    
    fprintf( stderr, "done no %d.\n", currentPlot);
    getchar();

    currentPlot++;
  }    
}

