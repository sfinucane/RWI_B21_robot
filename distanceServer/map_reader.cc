// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/map_reader.cc,v $
//
//  Created by:      $Author: rstone $
//
//  Revision:        $Revision: 1.1 $
//
//  Date of revision $Date: 2002/09/14 16:52:31 $
//
//  This file is part of the Robotic Telelabor Project.
//
// ============================================================================
//
//                 ------------- HISTORY -----------------
//
//  $Log: map_reader.cc,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.7  2000/02/07 13:09:49  schulz
//  Some fixes regarding network updates
//
//  Revision 1.6  1999/11/18 12:49:01  schulz
//  Fixed framebuffer, added stuff for humans
//
//  Revision 1.5  1998/10/12 13:31:54  schulz
//  This is a complete new version
//
//  Revision 1.1  1998/06/19 13:57:49  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.9  1997/09/12 12:07:35  schulz
//  fixed several bugs
//
//  Revision 1.8  1997/09/08 14:00:22  schulz
//  intermediate update, wont run
//
//  Revision 1.7  1997/06/09 13:30:22  schulz
//  database modifications
//
//  Revision 1.6  1997/06/07 14:19:13  schulz
//  new obstacle database
//
//  Revision 1.5  1997/06/03 12:15:35  schulz
//  to many changes to mention
//
//  Revision 1.4  1997/05/16 14:57:56  schulz
//  source restructioring in progress
//  things won't compile for now
//
//  Revision 1.3  1997/03/13 11:12:03  schulz
//  compiled a generic GetDistance library
//
//  Revision 1.2  1997/03/04 17:15:11  schulz
//  Added file header and revision history
//
//
// ----------------------------------------------------------------------------

#include <string.h>

#include "mapreader.hh"

static const char spacechars[] = " \t";

struct obstacleNames t_mapReader::obstacle_type[NO_OBSTACLE_TYPES] = {
  {"CUBE", CUBE},			
  {"CYLINDER", CYLINDER},		    
  {"DOOR", DOOR},
  {"HUMAN", HUMAN},
  {"CAMERA", CAMERA},
  {"begin", COMPOUND},
  {"end", COMPOUND_END}};

t_mapReader::t_mapReader(FILE* mF)
{
  mapFile = mF;
  unread = 0;
}

t_ObstacleRecord*
t_mapReader::nextObstacle()
{
  bool found = FALSE, endOfFile = FALSE;
  do
    if(!(endOfFile = (read_line() == 0)))
      if(get_type()) 
	if(newobst.type == COMPOUND_END)
	  found = TRUE;
	else {
	  get_label();
	  if(get_position())
	    if(get_dimensions())
	      if(get_orientation()) {
		get_yrotation();
		if(get_color())
		  if(get_texture())
		    found = TRUE;
	      }
	}
  while (!found && !endOfFile);
  
  if(found)
    return &newobst;
  else {
    if(newobst.label) delete [] newobst.label;
    if(newobst.texture) delete [] newobst.texture;    
    return NULL;
  }
}

bool
t_mapReader::getMapDimensions(float &x1, float &y1, float &x2, float &y2)
{
  char map_string[BUFFLEN];
  if(!read_line()) {
    return FALSE;
  }	
  if(!get_string(map_string," ")) {
    unread_line();
    return FALSE;
  }
  if(strcasecmp(map_string, "MAP") != 0) {
    unread_line();
    return FALSE;
  }
  if(get_float(x1," ") &&
     get_float(y1," ") &&
     get_float(x2," ") &&
     get_float(y2) )
    return TRUE;
  unread_line();
  return FALSE;
}

bool
t_mapReader::getRobotPosition(float &x, float &y, float &rot)
{
  char robot_string[BUFFLEN];
  if(!read_line()) 
    return FALSE;
  if(!get_string(robot_string," ")) {
    unread_line();
    return FALSE;
  }
  if(strcasecmp(robot_string, "ROBOT") != 0) {
    unread_line();
    return FALSE;
  }
  if(get_float(x," ") &&
     get_float(y," ") &&
     get_float(rot) )
    return TRUE;
  unread_line();
  return FALSE;
}

bool
t_mapReader::get_type()
{
  bool found = FALSE;
  char type_string[BUFFLEN];
  newobst.type = UNKNOWN;
  if(!get_string(type_string," \t\n"))
    return FALSE;
  for(int i = 0; i < NO_OBSTACLE_TYPES && !found; i++) {
    if( strcasecmp(obstacle_type[i].token, type_string) == 0) {
      newobst.type = obstacle_type[i].id;
      found = TRUE;
    }
  }
  return found;
}

bool
t_mapReader::get_label()
{
  float f;
  char label[256];
  if(get_float(f)) {
    if(newobst.label) delete [] newobst.label;    
    newobst.label = NULL;
    return TRUE;
  }
  if(!get_string(label," ")) return FALSE;
  if(newobst.label) delete [] newobst.label;
  newobst.label = new char[strlen(label)+1];
  strcpy(newobst.label, label);
  return TRUE;
}

bool
t_mapReader::get_position()
{
  if(! (get_float(newobst.position.cx," ") &&
	get_float(newobst.position.cy," ") &&
	get_float(newobst.position.cz," ")))
    return FALSE;
  return TRUE;
}

bool
t_mapReader::get_dimensions()
{
  bool retVal = FALSE;
  switch(newobst.type) {
  case COMPOUND:
    retVal = TRUE;
    break;
  case CYLINDER:
  case HUMAN:
    if(get_float(newobst.dimensions.r," ") &&
       get_float(newobst.dimensions.h," ")) {
      retVal = TRUE;
    }
    break;
  case CUBE:
  case DOOR:
  case CAMERA:
    if(get_float(newobst.dimensions.w," ") &&
       get_float(newobst.dimensions.d," ") &&
       get_float(newobst.dimensions.h," ")) {
      retVal = TRUE;
    }
    break;
  default:
    retVal = FALSE;
  }
  return retVal;
}

bool
t_mapReader::get_orientation()
{
  bool retVal = FALSE;
  if(get_float(newobst.orientation)) {
    retVal = TRUE;
  }
  else {
    newobst.orientation = 0;
    retVal = TRUE;
  }
  return retVal;
}

bool
t_mapReader::get_yrotation()
{
  bool retVal = FALSE;
  if(skip_to(' ') && get_float(newobst.yrotation)) {
    retVal = TRUE;
  }
  else {
    newobst.yrotation = 0;
    retVal = TRUE;
  }
  return retVal;
}

bool
t_mapReader::get_color()
{
  bool retVal = FALSE;
  newobst.color_defined = FALSE;
  if(skip_to('(') &&
     get_float(newobst.color.r," ") &&
     get_float(newobst.color.g," ") &&
     get_float(newobst.color.b,")")) {
    newobst.color_defined = TRUE;
    retVal = TRUE;
  }
  else {
    newobst.color.r = 1.0;
    newobst.color.g = 1.0;
    newobst.color.b = 1.0;
    newobst.color_defined = FALSE;	
    retVal = TRUE;
  }
  return retVal;
}

bool
t_mapReader::get_texture()
{
  char texture[80];
  if(sscanf(cpos, "%s", texture) > 0) {
    if(newobst.texture) delete [] newobst.texture;
    newobst.texture = new char[strlen(texture)+1];
    strcpy(newobst.texture, texture);
  }
  else {
    if(newobst.texture) delete [] newobst.texture;
    newobst.texture = NULL;
  }
  return TRUE;
}

bool
t_mapReader::match_char(char c)
{
  skip_whitespace();
  if (*cpos == '\0')
    return FALSE;
  if(*cpos == c) {
    cpos++;
    return TRUE;
  }
  return FALSE; 
}

bool
t_mapReader::skip_to(char c)
{
  while(*cpos != c && *cpos != '\0') {
    cpos++;
  }
  if(*cpos == c) {
    cpos++;
    return TRUE;
  }
  return FALSE;  
}



bool
t_mapReader::get_float(float &f, char* delim)
{
  char *float_string;
  skip_whitespace();
  if (*cpos == '\0')
    return FALSE;
  if(!(float_string = strtok(cpos, delim)) )
    return FALSE;
  if(sscanf(float_string, "%f", &f) != 1)
    return FALSE;
  cpos += strlen(float_string) + strlen(delim);
  return TRUE;
}

bool
t_mapReader::get_float(float &f)
{
  char *float_string;
  skip_whitespace();
  if (*cpos == '\0')
    return FALSE;
  if(sscanf(cpos, "%f", &f) != 1)
    return FALSE;
  return TRUE;
}

bool
t_mapReader::get_string(char *s, char* delim)
{
  skip_whitespace();
  char *s2;
  if(! (s2 = strtok(cpos, delim)))
    return FALSE;
  strcpy(s,s2);
  cpos += strlen(s) + 1;
  return TRUE;
}

void
t_mapReader::skip_whitespace()
{
  while(memchr((const void*) spacechars, *cpos, 2) && *cpos != '\0') cpos++;
}

void
t_mapReader::unread_line()
{
  unread = 1;
}

bool
t_mapReader::read_line()
{
  bool empty = TRUE;

  if(unread) {
    unread = 0;
    cpos = scanLine;
    skip_whitespace();
    return TRUE;
  }
  while (!feof(mapFile) && empty){
    if (fgets(scanLine, BUFFLEN, mapFile) != NULL) {
      cpos = scanLine;
      skip_whitespace();
      empty = (*cpos == '\n' || *cpos == '\0' || *cpos == '#');
    }
  }

  return !empty;
}
