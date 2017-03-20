
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/shrinkMap.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:40 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: shrinkMap.c,v $
 * Revision 1.1  2002/09/14 20:45:40  rstone
 * *** empty log message ***
 *
 * Revision 1.4  1996/12/02 10:32:14  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.3  1996/10/24 12:07:14  fox
 * Fixed a bug.
 *
 * Revision 1.2  1996/10/24 09:56:56  fox
 * LOCALIZE also works in a write protected directory.
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


#include "general.h"
#include "map.h"


#include <stdlib.h>

#define BUFFLEN 512
#define UNKNOWN -1.0
#define TRUE 1


void**
allocate2D( int dim1, int dim2)
{
    int x;
    mapProbability** pt;
    if ( (pt = (mapProbability**)
          malloc( dim1 * sizeof( mapProbability*))) == NULL) {
        return (void**) NULL;
    }
    for ( x = 0; x < dim1; x++)
        if ( (pt[x] = (mapProbability*)
              malloc( dim2 * sizeof(mapProbability))) == NULL) {
            return (void**) NULL;
        }
    return (void**) pt;
}
#


int
shrinkMap(probabilityGrid *m)
{
  register int x,y;
  register int empty = TRUE;

  int min_x,max_x,min_y,max_y;

  int oldsizeX;
  
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
  
  m->sizeX = max_x - min_x + 1;
  m->sizeY = max_y - min_y + 1;

  if ( logfile != NULL) 
    fprintf(logFile,"# Map shrinked: %d %d\n",m->sizeX,m->sizeY);
  fprintf(stderr,"# Map shrinked: %d %d\n",m->sizeX,m->sizeY);

  m->offsetX = min_x;
  m->offsetY = min_y;

  for(x=0; x<m->sizeX; x++)
    for(y=0; y<m->sizeY; y++)
      m->prob[x][y] = m->prob[x+m->offsetX][y+m->offsetY];

  return(1);
  
}


int
readMap(char* mapname, probabilityGrid *m)
{
  register int x,y;
  float temp;
  
  char line[255];
  FILE *fp;


  if ((fp = fopen(mapname,"rt")) == NULL) {
      fprintf(stderr,"ERROR: Could not open file %s\n",mapname);
      return(0);
    }

  m->resolution = 15;
  /* seek header for map data in rhino-Type file,
     analyze robot specifications */
  while ((fgets(line,255,fp) != NULL)
         && (strncmp("global_map[0]", line , 13) != 0)){
    if (strncmp(line,"robot_specifications->resolution",32) == 0) {
        if (sscanf(&line[32],"%d",&(m->resolution)) != 0 ) {
	  if ( logfile != NULL) 
            fprintf(logFile,"# Map resolution: %d cm\n",m->resolution);
        }
    }
/*     if (strncmp(line,"robot_specifications->robot_size",32) == 0) { */
/*      if (sscanf(&line[32],"%d",&(r->radius)) != 0 ) { */
/*          fprintf(stderr,"Robot radius: %d cm\n",r->radius); */
/*      } */
/*     } */
  }
  if (sscanf (line,"global_map[0]: %d %d",&m->sizeX,&m->sizeY)
      == EOF) {
      fprintf(stderr,"ERROR: corrupted file %s\n",mapname);
      fclose(fp);
      return(0);
  }
    
  if ( logfile != NULL) 
    fprintf(logFile,"# Map size: %d %d\n",m->sizeX,m->sizeY);
  
  m->unknown = UNKNOWN;
  m->origsizeX = m->sizeX;
  m->origsizeY = m->sizeY;
  m->offsetX = m->offsetY = 0;
    
  m->prob = (mapProbability**) allocate2D( m->sizeX, m->sizeY);
  
  if (m->prob == (mapProbability**) NULL){
    fprintf(stderr,
              "ERROR: Not enough memory for loading map %s\n",mapname);
    fclose(fp);
    return(0);
  }

  for (y=m->sizeY-1;y>=0; y--)
    for (x=0; x<m->sizeX; x++){
        fscanf(fp,"%e",&temp);
        m->prob[x][y] = (mapProbability) (temp);
    }
  
  fclose(fp);

  shrinkMap(m);
  
  return(1);
}

int
main( int argc, char** argv)
{
 
   int fromX, fromY, toX, toY;
   int i,j;
   
   FILE* fp;
   
   probabilityGrid map;

   for (i=0; i<argc; i++)
       fprintf(stderr,"%s\n",argv[i]);
   
   (void) readMap(argv[1], &map);

   fromX = atoi(argv[2]);
   toX = atoi(argv[3]);
   fromY = atoi(argv[4]);
   toY = atoi(argv[5]);

   fprintf(stdout, "Shrinking map (X: %d->%d), (Y: %d->%d)\n",
           fromX, toX, fromY, toY);

   if ((fp = fopen("shrinkedMap","wt")) != NULL){
     
     fprintf(fp,"robot_specifications->resolution %d\n", map.resolution);
     fprintf(fp, "global_map[0]: %d %d\n", toX - fromX + 1, toY - fromY + 1);

     for (j=toY;j>=fromY; j--) {
         for (i=fromX; i<=toX; i++)
             fprintf(fp, "%f ", map.prob[i][j]);
         fprintf(fp, "\n");
     }
     
     fclose(fp);
     
   }
   
   closeLogAndExit(0);
}

