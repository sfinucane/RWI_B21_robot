// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/mapreader.hh,v $
//
//  Created by:      $Author: rstone $
//
//  Revision:        $Revision: 1.1 $
//
//  Date of revision $Date: 2002/09/14 16:52:31 $
//
// ============================================================================
//
//                 ------------- HISTORY -----------------
//
//  $Log: mapreader.hh,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.2  1998/10/12 13:31:55  schulz
//  This is a complete new version
//
//  Revision 1.1  1998/06/19 13:57:49  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.9  1997/09/29 16:02:55  schulz
//  Made source files SUN C++ compatible
//
//  Revision 1.8  1997/09/08 14:00:24  schulz
//  intermediate update, wont run
//
//  Revision 1.7  1997/06/07 14:19:13  schulz
//  new obstacle database
//
//  Revision 1.6  1997/06/03 12:15:36  schulz
//  to many changes to mention
//
//  Revision 1.5  1997/05/16 14:57:56  schulz
//  source restructioring in progress
//  things won't compile for now
//
//  Revision 1.4  1997/03/23 10:28:38  schulz
//  changes for 970318 demo
//
//  Revision 1.3  1997/03/13 11:12:04  schulz
//  compiled a generic GetDistance library
//
//  Revision 1.2  1997/03/04 17:28:28  schulz
//  Added file header to *.hh files
//  Removed some old "Zombie" files
//
//
//
// ----------------------------------------------------------------------------

#ifndef MAPREADER_HH_
#define MAPREADER_HH_

#include <stdio.h>
#include "obstacles.hh"



#ifndef FALSE
#define FALSE  0
#endif
#ifndef TRUE
#define TRUE  1
#endif


#define NO_OBSTACLE_TYPES 7
#define BUFFLEN 128


class t_mapReader {
public:

    t_mapReader(FILE *mapFile);
    // mapFile has to be an opened simulator mapfile
    
    t_ObstacleRecord* nextObstacle();
    // reads the next obstacle from file into an internal obstacle
    // record and returns a pointer to the record on success
    // returns NULL on error or end of file

    bool getMapDimensions(float &xll, float &yll, float &xur, float &yur);
    // map dimensions are return by lower-left and upper-right corner
    
    bool getRobotPosition(float &x, float &y, float &rot);
    
private:

    FILE *mapFile;

    char scanLine[BUFFLEN];
    char *cpos;
    // points into scanLine to the current scan position

    static struct obstacleNames obstacle_type[];
    
    bool readLine();
    // read net line from file into buffer

    t_ObstacleRecord newobst;
    bool get_type();
    bool get_position();
    bool get_dimensions();
    bool get_orientation();
    bool get_yrotation();
    bool get_color();
    bool get_texture();
    // read corresponding entries from scanLine into newobst!
    
    bool get_string(char *s, char* delim);
    // read next string token from internal buffer;
    // delim is the expected delimiter

    bool get_label();
  // succeeds if the obstacle on the current line has got a label
  
    bool get_float(float &f, char* delim);
    // fill f with float value read from scanLine
    // delim is the expected delimiter

    bool get_float(float &f);
    // same as last one but no delimiter (e.g. value at line end)

    bool match_char(char c);
    // check if next char form scanLine is c
    // consume this char on success
    // return FALSE otherwise

    bool skip_to(char c);
    // skip to the char next to the next c char   
  
    bool read_line();
    // read next line from mapFile into scanLine;
    // returns FLASE on eof

    void unread_line();
    // marks current line buffer unread;
  
    void skip_whitespace();
    // only skips " \t" !!!

  int unread;
};

#endif
