#ifndef FILE_NAMES_LOADED
#define FILE_NAMES_LOADED

#include <stdio.h>
#include "general.h"

#define MAX_FILE_NAME 255

/***********************************************************
* Some functions to extract or replace parts of a file name.
* The string for the rsult of the functions has to be long enough
* to keep the result!!!
***********************************************************/
   


/***********************************************************
* Copies the characters beginning at the last / in the string
* into fileName.
* fileName has to be long enough!!!!
***********************************************************/
void extractFileName(const char* string, char* fileName);


/***********************************************************
* Copies the characters until the last / in the string
* into fileName.
* fileName has to be long enough!!!!
***********************************************************/
void extractPathName(const char* string, char* pathName);


/***********************************************************
* Replaces the last extender (behind a dot) with ext and copies
* the resulting string into fileName.
* fileName has to be long enough!!!!
***********************************************************/
void replaceExtender( const char* orgFileName,
		      char* newFileName,
		      const char* ext);

/***********************************************************
* Adds an extender.
* fileName has to be long enough!!!!
***********************************************************/
void addExtender( const char* orgFileName,
		  char* newFileName,
		  const char* ext);

void addExtenderNumber( const char* orgFileName,
			char* newFileName,
			const int extNumber);

/***********************************************************
* Checks wether the file has the extender <ext>.
* Returns 1 if the extender is the same.
* Returns 0 otherwise.
***********************************************************/
int checkForExtender( const char* fileName,
		      const char* ext);

/***********************************************************
* Adds the path at the head of orgFileName.
* newFileName has to be long enough!!!!
***********************************************************/
void addPath( const char* orgFileName,
	      char* newFileName,
	      const char* path);

/***********************************************************
* Replaces the path of the orgFileName with path and copies
* the resulting string into newFileName. If orgFileName contains
* no path the result is the same as addPath().
* newFileName has to be long enough!!!!
***********************************************************/
void replacePath( const char* orgFileName,
		  char* newFileName,
		  const char* path);



#endif







