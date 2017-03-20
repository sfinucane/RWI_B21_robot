// Time-stamp: <98/02/12 16:05:41 derr>
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
/////
///// File name:                   scriptData.hh
/////
///// Part of:                     RHINO SOFTWARE
/////
///// Creator:                     Andreas Derr, University of Bonn
/////
///// Date of creation:            Oct 1997
/////
/////
/////
/////
///// $Source: /usr/local/cvs/bee/src/localize/rawData/scriptData.hh,v $
/////
///// $Revision: 1.1 $
/////
///// $Date: 2002/09/14 17:06:31 $
/////
///// $Author: rstone $
/////
/////
/////
///// Contact derr@informatik.uni-bonn.de
/////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
/////
///// THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED BY
///// APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING THE
///// COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM "AS IS"
///// WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED,
///// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
///// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE ENTIRE
///// RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU.
///// SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL
///// NECESSARY SERVICING, REPAIR OR CORRECTION.
/////
///// IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
///// WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
///// MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
///// LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
///// INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
///// INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO LOSS OF
///// DATA OR DATA BEING RENDERED INACCURATE OR LOSSES SUSTAINED BY YOU
///// OR THIRD PARTIES OR A FAILURE OF THE PROGRAM TO OPERATE WITH ANY
///// OTHER PROGRAMS), EVEN IF SUCH HOLDER OR OTHER PARTY HAS BEEN
///// ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
/////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
//
//
//                  ----- REVISION HISTORY -----
//
// $Log: scriptData.hh,v $
// Revision 1.1  2002/09/14 17:06:31  rstone
// *** empty log message ***
//
// Revision 1.1  1998/02/12 18:14:02  derr
// Added library for reading laserint- sonarint- and 'new'-scripts.
//
// Revision 1.1  1998/02/12 15:59:51  derr
// New library librawData for reading laserint-, sonarint and 'new'-script.
// Will replace bee/src/localize/script.c.
//
//
//
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
#ifndef CLASS_SCRIPTDATA_INCLUDE
#define CLASS_SCRIPTDATA_INCLUDE
// load some general definitions
#include <fstream.h>
//#include <stdio.h>
//#include <sys/time.h>
//#include <unistd.h>

#include "rawData.hh"

class t_ScriptData : public t_RawData
{
public:
  // use script file <scriptFileName> with error action CONTINUE_ON_ERROR
  t_ScriptData( const char *scriptFileName,
                const float timeFactor);

  // use script file <scriptFileName> and <timeFactor>
  t_ScriptData( const char *scriptFileName,
                const float timeFactor,
                const t_ErrorAction e);

  // use script file <scriptFileName> with error action CONTINUE_ON_ERROR,
  // logging output to stdout, stderr _and_ <logFile>
  t_ScriptData( const char *scriptFileName,
                const float timeFactor,
                FILE *logFile);

  // use script file <scriptFileName>,
  // logging output to stdout, stderr _and_ <logFile>
  t_ScriptData( const char *scriptFileName,
                const float timeFactor,
                const t_ErrorAction e,
                FILE *logFile);

  // get version
  t_ScriptData( const unsigned int length, char *versionStr);

  // Destructor
  ~t_ScriptData();

  bool getBasePosition( t_RealPosition &basePos);
  bool getBump( t_RealPosition &bumpDelta);
  void getCounts( unsigned long &sonarCnt,
                  unsigned long &laserCnt,
                  unsigned long &posCnt,
                  unsigned long &markerCnt,
                  unsigned long &noMoveCnt);
  void getDistanceTraveled(float &dist);
  bool getFrontLaser( t_Reading &reading);
  bool getMarker( long &number, t_RealPosition &markerPos);
  bool getMovement( t_RealPosition &delta);
  bool getRearLaser( t_Reading &reading);
  bool getSonar( t_Reading &reading);
  void getStartPosition(t_RealPosition &pos);

  /////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////
  //
  //              redefinition of virtual functions
  //
  /////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////

  // new data arrived?
  t_Result checkNext( const float nonRelevantTime=0.0);

  // get new data
  // TRUE:  parameter contains new data
  // FALSE: parameter unchanged
  unsigned long getActualLine();
  bool getBump( t_RealPosition *bumpDelta);
  bool getBasePosition( t_RealPosition *basePos);
  void getCounts( unsigned long *sonarCnt,
                  unsigned long *laserCnt,
                  unsigned long *posCnt,
                  unsigned long *markerCnt,
                  unsigned long *noMoveCnt);
  float getElapsedTime();
  bool getFrontLaser( t_Reading *reading);
  bool getMarker( long &number, t_RealPosition *&markerPos);
  bool getMovement( t_RealPosition *&delta);
  bool getRearLaser( t_Reading *reading);
  bool getSonar( t_Reading *reading);
  void getStartPosition( t_RealPosition *pos);

  // TRUE: everything is allright
  bool getStatus();
  void resetTimer();

  /////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////
  //
  //                       STOP READING HERE
  //
  /////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////


protected:
private:
  typedef enum {
    UNKNOWN_PATTERN,
    BUTTON_PATTERN,
    INFRARED_PATTERN,
    LASER_PATTERN,
    MARKER_PATTERN,
    SONAR_PATTERN,
    TACTILE_PATTERN
  } t_secondScriptPatterns;

  typedef struct {
    unsigned int hour;
    unsigned int minute;
    float second;
  } t_ScriptTime;

  typedef struct {
    char *name;
    char *description;
    char *type;
  } t_ScriptInfoStruct;

  typedef struct _t_ScriptStruct {
    t_ScriptInfoStruct info;
    float          distanceOffset;
    t_RawDataType  type;
    t_ScriptTime   startTimeOfScriptOne;  // for orig_script
    t_ScriptTime   currentTimeOfScriptOne;
    struct timeval startTimeOfScriptTwo;   // for second script type
    struct timeval currentTimeOfScriptTwo;
    struct timeval lastAccessTime;
    unsigned long  actualLine;
    unsigned long  sonarCount;
    unsigned long  laserCount;
    unsigned long  positionCount;
    unsigned long  markerCount;
    unsigned long  noMoveCount;
  } t_ScriptStruct;


  t_Status       dStatus;
  char          *ScriptName;
  istream       *ScriptStream;
  ifstream       ScriptStreamNotStdin;
  bool           bReadScriptFromStdin;

  t_ScriptStruct ScriptSettings;

  void     add2LengthOfTrajectorie(realPosition newPos);
  void     convertNeededLine2Str( const char *line, char *&str);
  bool     copyString( char *&target, const char *source,
                       const char *errorText);
  void     destroy();     // called by destructor
  t_secondScriptPatterns getPatternType(const char *line);
  bool     getReadings( const char *line,
                        const unsigned int numberOfReadings,
                        float *sensor);
  bool      getReadingsInverse( const char *line,
                                const unsigned int numberOfReadings,
                                float *sensor);
  void     init();        // called by constructor
  // orig script

  bool     bumperReadingLine(const char* line);
  bool     descriptionReadingLine( const char* line);
  bool     laserReadingLine( const char* line);
  bool     markerReadingLine( const char* line);
  bool     nameReadingLine( const char* line);
  float    normedAngleRAD(const float angle);
  bool     positionReadingLine( const char *line);
  void     processMarkerReading( const long markerNumber);
  void     processPositionReading( t_RealPosition &newPos);

  // second_script
  bool     patternEndReadingLine( const char* line);
  bool     patternNameReadingLine( const char* line);
  bool     patternSetButtonReadingLine( const char* line, t_Button *b);
  bool     patternSetEndReadingLine( const char* line);
  bool     patternSetInfraredReadingLine( const char* line, t_Infrared *i);
  bool     patternSetLaserReadingLine( const char* line, float *sensor,
                                       int &frontSize, int &rearSize);
  bool     patternSetMarkerReadingLine( const char* line, unsigned long &m);
  bool     patternSetNameReadingLine( const char* line);
  bool     patternSetPositionReadingLine( const char* line,
                                          t_RealPosition &newPos);
  bool     patternSetSonarReadingLine( const char* line, float *sensor,
                                       int &sonarReading);
  bool     patternSetStartReadingLine( const char* line);
  bool     patternSetTactileReadingLine( const char* line, t_Tactile *t);
  bool     patternSetTimeReadingLine( const char* line, struct timeval &t);
  bool     patternStartReadingLine( const char* line);
  bool     patternTypeReadingLine( const char* line);

  bool     openScript();  // called by constructor
  bool     readLine( char *line, const unsigned int maxLen);
  t_Result readOrigScript( const float nonRelevantTime);
  t_Result readSecondScript( const float nonRelevantTime);
  void     scriptPosition2RobotPosition( t_RealPosition &scriptPos);
  float    scriptTimeDiff( const t_ScriptTime *newTime,
                           const t_ScriptTime *oldTime);
  // orig_script
  bool     sonarReadingLine(const char* line);
  bool     timeReadingLine( const char *line);
  void     updateLaser( const float *sensor, const unsigned int n);
  void     updateFloat( const float *sensor,
                        t_Reading *r1, const unsigned int n1,
                        t_Reading *r2, const unsigned int n2);
  void     updateFloat( const float *sensor,
                        t_Reading *r1, const unsigned int n1);
  void     updateMovement( t_RealPosition &newPos);
  void     updateSonar(const float *sensor, const unsigned int n);

  char    *version();
}; // end of class t_ScriptData
#endif

// End of CLASS_SCRIPTDATA_HEADER

