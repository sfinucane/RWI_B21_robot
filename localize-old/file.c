
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/file.c,v $
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
 * $Log: file.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.23  2000/01/02 15:33:14  fox
 * Should work.
 *
 * Revision 1.22  1999/01/11 19:47:48  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.20  1999/01/08 22:28:45  wolfram
 * Better integration of scanAlignment
 *
 * Revision 1.19  1998/10/02 15:16:37  fox
 * Several improvements of condensation. Updated the function calls to the
 * new version of the distancServer.
 *
 * Revision 1.18  1998/09/25 17:53:29  fox
 * Improved version of condensation.
 *
 * Revision 1.17  1998/08/31 22:29:19  wolfram
 * Several changes
 *
 * Revision 1.16  1998/08/20 00:22:57  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.15  1998/01/05 10:37:10  fox
 * Several changes especially in selection parts.
 *
 * Revision 1.14  1997/12/02 15:20:36  fox
 * Nothing remarkable.
 *
 * Revision 1.13  1997/09/30 10:40:42  wolfram
 * Fixed a bug in initialization of graphic.c
 *
 * Revision 1.12  1997/09/30 07:59:38  wolfram
 * slight change
 *
 * Revision 1.11  1997/09/29 10:45:22  wolfram
 * Added default values for all values in .ini file
 *
 * Revision 1.10  1997/09/26 17:02:08  wolfram
 * Intermediate version, do not use
 *
 * Revision 1.9  1997/08/02 16:51:01  wolfram
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
 * Revision 1.8  1997/03/14 17:58:17  fox
 * This version should run quite stable now.
 *
 * Revision 1.7  1997/01/29 12:23:05  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.6  1997/01/16 19:43:22  fox
 * And another bug ...
 *
 * Revision 1.5  1996/12/20 15:29:37  fox
 * Added four parameters.
 *
 * Revision 1.4  1996/12/02 10:32:04  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.3  1996/11/26 16:08:25  fox
 * Nothing special.
 *
 * Revision 1.2  1996/10/24 12:07:09  fox
 * Fixed a bug.
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


#include <sys/param.h>
#include <unistd.h>

char *getwd(char *pathname);

#include "general.h"
#include "sensings.h"
#include "file.h"
#include "inline.h"

FILE* logFile;

/*-------------------------------------------------------------------
 *-------------------------------------------------------------------
 * Handling of log files.
 *-------------------------------------------------------------------
 *-------------------------------------------------------------------*/

void
closeLogAndExit( int sig)
{
  if ( logFile != NULL)
    fclose( logFile);
  exit( sig);
}


void
openLogFile( char* logFileName)
{
  /* try to open file */
  if ( strncmp( logFileName, STDOUT_NAME, 6) != 0) {
    if ( (logFile = fopen( logFileName, "w")) == NULL) {
      putc( 7, stderr);
      fprintf(stderr, "WARNING: Cannot open log file %s.\n", logFileName);
      return;
    }
    
  }
  else
    logFile = stdout;
}


int
writeLog( char* fmt, ...)
{
 va_list                args;
 int			bytes;

 if ( logFile != NULL) {
   
   va_start(args, fmt);
   bytes = vfprintf(logFile, fmt, args);
   va_end(args);
   
#define IMMEDIATE_WRITE  
#ifdef IMMEDIATE_WRITE
   fflush(logFile);
#endif
   
   return bytes;
 }
 else
   return 0;
}


/********************************************************************
 ********************************************************************
 * Parses parameter files.
 ********************************************************************
 ********************************************************************/

void
readTokens( char* file,
	    token* tokens, int numberOfTokens,
	    bool allTokenHaveToBeGiven)
{
  FILE* fp;
  char line[MAX_STRING_LENGTH];
  int tokenCnt;

  if ((fp = fopen( file, "r")) == NULL)
    if (allTokenHaveToBeGiven){
      fprintf(stderr, "Could not open ini file %s.\n", file);
      closeLogAndExit(0);
    }
  else
    return;

  if (allTokenHaveToBeGiven)
    for ( tokenCnt = 0; tokenCnt < numberOfTokens; tokenCnt++)
      tokens[tokenCnt].initialized = FALSE;
  
  while (!feof (fp) &&  (fgets(line, MAX_STRING_LENGTH, fp) != NULL)){

    /*---------------------------------------------------------------------------
     * Determine the keyword.
     *--------------------------------------------------------------------------*/
    for ( tokenCnt = 0; tokenCnt < numberOfTokens; tokenCnt++){
      if ( strncmp(line, tokens[tokenCnt].keyWord,
		   strlen(tokens[tokenCnt].keyWord)) == 0) {
	if ( strcmp(tokens[tokenCnt].format, MULTI_VALUE_FORMAT) == 0){
	  strcpy(tokens[tokenCnt].variable,
		 &line[strlen(tokens[tokenCnt].keyWord)]);
	  tokens[tokenCnt].initialized = TRUE;
	}
	else if (sscanf( &line[strlen(tokens[tokenCnt].keyWord)],
			 tokens[tokenCnt].format,
			 tokens[tokenCnt].variable) == 1){
	  tokens[tokenCnt].initialized = TRUE;
	}

	if (tokens[tokenCnt].initialized == FALSE){
	  fprintf( stderr, "Error when parsing ini file %s (", file);

	  fprintf( stderr, tokens[tokenCnt].format, tokens[tokenCnt].variable);
	  fprintf( stderr, ").\n");
	  closeLogAndExit(0);
	}
      }
    }
  }

  fclose( fp);
    
  /* If not all tokens are given we closeLogAndExit. */
  if ( allTokenHaveToBeGiven) {
    for ( tokenCnt = 0; tokenCnt < numberOfTokens; tokenCnt++) 
      if ( ! tokens[tokenCnt].initialized) {
	fprintf(stderr, "Error: keyword %s not given.\n",
		tokens[tokenCnt].keyWord);
	closeLogAndExit(0);
      }
  }
}


void
getInitValuesFloat( char *string, float *value, int size){
  int i, n, len;

  
  len = strlen(string);
  i = n = 0;

  while (i < len  && n < size){
    while(i < len && isspace(string[i]))
      i++;
    
    if (sscanf(&string[i], "%f", &value[n]) == 1)
      n++;

    while (i < len && !isspace(string[i]))
      i++;
  }
  if (n != size){
    fprintf(stderr, "Error: Could not find %d values in %s.", size, string);
    closeLogAndExit(0);
  }
}	


void
setTokensInitialized(token *tok, int numberOfValues)
{
  while( --numberOfValues >= 0)
    tok[numberOfValues].initialized = TRUE;
}


/* take the directory name as fileName */
void
getDirectory(char *name){
  char path[MAX_STRING_LENGTH];
  int i;
  
  if (getcwd(path, (size_t) MAX_STRING_LENGTH) != NULL){
    i = strlen(path);
    
    while (i >= 0 && path[i] != '/') i--;
    i++;
    strncpy(name, &path[i], iMin(strlen(name), MAX_STRING_LENGTH));
  }
  else
    strcpy(name, "");
}

realPosition
markerPosition( int markerNumber){
  FILE *fp;
  realPosition markerPos;
  markerPos.x = markerPos.y = markerPos.rot = 0;
  if ((fp = fopen("marker.dat", "r")) != NULL){
    int marker;
    char line[80];
    bool found = FALSE;
    while (!feof(fp) && fgets(line, 80, fp) && !found){
      if (sscanf(line, "%d %f %f %f", &marker,
		 &markerPos.x, &markerPos.y, &markerPos.rot) == 4){
	found = (marker == markerNumber);
      }
    }
    fclose(fp);
  }
  return markerPos;
}


void
dumpDistanceToMarker( realPosition estimatedPosition, int markerNumber)
{
  
  FILE *fp;
  if ((fp = fopen("marker.dat", "r")) != NULL){
    int marker;
    realPosition markerPos;
    char line[80];
    bool found = FALSE;
    while (!feof(fp) && fgets(line, 80, fp) && !found){
      if (sscanf(line, "%d %f %f %f", &marker,
		 &markerPos.x, &markerPos.y, &markerPos.rot) == 4){
	found = (marker == markerNumber);
      }
    }
    fclose(fp);
    if (found) {
      fprintf(stderr, "# Distance to marker: %d %f\n",
	      marker,
	      realPositionDistance(markerPos, estimatedPosition));
      writeLog("%d %.2f %.2f %.2f %.2f %.2f %.2f %.2f # marker\n",
	       marker,
	       estimatedPosition.x,
	       estimatedPosition.y,
	       estimatedPosition.rot,
	       markerPos.x - estimatedPosition.x,
	       markerPos.y - estimatedPosition.y,
	       markerPos.rot - estimatedPosition.rot,
	       realPositionDistance(markerPos, estimatedPosition));
    }
  }
}
