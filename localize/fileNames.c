
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/fileNames.c,v $
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
 * $Log: fileNames.c,v $
 * Revision 1.1  2002/09/14 20:45:39  rstone
 * *** empty log message ***
 *
 * Revision 1.2  2000/03/06 20:00:43  wolfram
 * Changed the code of the vision routines.
 *
 * Revision 1.1  1997/01/29 12:23:06  fox
 * First version of restructured LOCALIZE.
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "fileNames.h"

/*******************************************************************
*  Copies the characters beginning at the last / in the string
*  into fileName.
*  fileName has to be long enough!!!!
********************************************************************/
void extractFileName(const char* string, char* fileName)
{
  char* c;

  /*  we look for the last "/" in the string  */
  if ( (c = strrchr(string, '/')) != NULL)
    /* next character is beginning of the filename  */
    fileName = strcpy(fileName, c+1);
  else   /* there is no path and we have to copy the whole string  */
    fileName = strcpy(fileName, string);    
}


/*******************************************************************
*  Copies the characters until the last / in the string
*  into fileName.
*  fileName has to be long enough!!!!
********************************************************************/
void extractPathName(const char* string, char* pathName)
{
  char* c;

  /* we look for the last "/" in the string  */
  if ( (c = strrchr(string, '/')) != NULL) {
    /* we copy string till "/"  */
    pathName = strncpy(pathName, string, c-string+1);
    pathName[c-string+1] = '\0';
  }
  else        /* there is no path, so we return an empty string */
    pathName[0] = '\0';
}


/********************************************************************
*  Replaces the last extender (behind a dot) with ext and copies
*  the resulting string into fileName.
*  fileName has to be long enough!!!!
********************************************************************/
void replaceExtender( const char* orgFileName,
		      char* newFileName,
		      const char* ext)
{
  char* c;

  /* we look for the last "." in the string  */
  if ( (c = strrchr(orgFileName, '.')) != NULL) {
    /* we copy string till "."  */
    newFileName = strncpy(newFileName, orgFileName, c-orgFileName+1);
    newFileName[c-orgFileName+1] = '\0';
  }
  else {       /* there is no extender, so we need not replace anything. */
      newFileName = strcpy(newFileName, orgFileName);
      newFileName = strcat(newFileName, ".");
  }
  
  newFileName = strcat(newFileName, ext);

}

/*******************************************************************
*  Replaces the last extender (behind a dot) with ext and copies
*  the resulting string into fileName.
*  fileName has to be long enough!!!!
********************************************************************/
void addExtender( const char* orgFileName,
		  char* newFileName,
		  const char* ext)
{
  newFileName = strcpy(newFileName, orgFileName);
  newFileName = strcat(newFileName, ".");
  newFileName = strcat(newFileName, ext);
}

void addExtenderNumber( const char* orgFileName,
			char* newFileName,
			const int extNumber)
{
  char ext[MAX_FILE_NAME];
  sprintf( ext, "%d", extNumber);
  newFileName = strcpy(newFileName, orgFileName);
  newFileName = strcat(newFileName, ".");
  newFileName = strcat(newFileName, ext);
}


/*******************************************************************
* Checks wether the file has the extender <ext>.
*  Returns 1 if the extender is the same.
*  Returns 0 otherwise.
********************************************************************/
int checkForExtender( const char* fileName,
		      const char* ext)
{
    char* c;
    
    /* we look for the last "." in the string */
    if ((c = strrchr(fileName, '.')) != NULL) {
      if ( strcmp( c+1, ext) != 0)
	return 0;
      else
	return 1;
    }
    else /* there is no extender */
      return 0;
}


/*******************************************************************
*  Adds the path at the head of orgFileName.
*  newFileName has to be long enough!!!!
********************************************************************/
void addPath( const char* orgFileName,
	      char* newFileName,
	      const char* path)
{
  newFileName = strcpy(newFileName, path);
  newFileName = strcat(newFileName, orgFileName);
}


/*******************************************************************
*  Replaces the path of the orgFileName with path and copies
*  the resulting string into newFileName. If orgFileName contains
*  no path the result is the same as addPath().
*  newFileName has to be long enough!!!!
********************************************************************/
void replacePath( const char* orgFileName,
		  char* newFileName,
		  const char* path)
{
  char tmp[MAX_FILE_NAME];

  /* First we extract the name of the file. */
  extractFileName( orgFileName, tmp);

  /* Now we only have to add the path. */
  addPath( tmp, newFileName, path);
}

