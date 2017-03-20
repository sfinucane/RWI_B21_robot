// time-stamp value _has to_ reside in the first 8 lines
static char *t_ScriptDataTimeStamp=" CLASS   scriptData  \
Time-stamp: <98/02/12 16:25:19 derr>";
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
/////
///// File name:                   t_ScriptData.cc
/////
///// Part of:                     RHINO SOFTWARE
/////
///// Creator:                     AnD, University of Bonn
/////
///// Date of creation:            Oct 1997
/////
/////
/////
/////
///// $Source: /usr/local/cvs/bee/src/localize/rawData/scriptData.cc,v $
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
// $Log: scriptData.cc,v $
// Revision 1.1  2002/09/14 17:06:31  rstone
// *** empty log message ***
//
// Revision 1.1  1998/02/12 18:14:02  derr
// Added library for reading laserint- sonarint- and 'new'-scripts.
//
// Revision 1.1  1998/02/12 15:59:50  derr
// New library librawData for reading laserint-, sonarint and 'new'-script.
// Will replace bee/src/localize/script.c.
//
//
//
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

#include <iostream.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <sys/types.h>

#include "rawData.hh"
#include "scriptData.hh"

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::t_ScriptData
// Purpose   : (Constructor) use script file <scriptFileName> with error
//             action CONTINUE_ON_ERROR
//
// Parameters: const char *scriptFileName :
//             const float timeFactor   :
//
// Return    : ---                      :
//
// Remarks   :
//
t_ScriptData::t_ScriptData( const char *scriptFileName,
                            const float timeFactor) :
  t_RawData(timeFactor)
{
  dStatus=BAD;
  if(BAD == getProtectedStatus())
    return;

  init();

  if(BAD==dStatus)
    return;

  if(!copyString(ScriptName,scriptFileName,"copying script name")) {
    dStatus=BAD;
    return;
  }

  if(!openScript()) {
    dStatus=BAD;
    return;
  }
}

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::t_ScriptData
// Purpose   : (Constructor) use script file <scriptFileName>
//
// Parameters: const char *scriptFileName :
//             const float timeFactor   :
//             const t_ErrorAction e    :
//
// Return    : ---                      :
//
// Remarks   :
//
t_ScriptData::t_ScriptData( const char *scriptFileName,
                            const float timeFactor,
                            const t_ErrorAction e) :
  t_RawData(timeFactor,e)
{
  dStatus=BAD;
  if(BAD == getProtectedStatus())
    return;

  init();

  if(BAD==dStatus)
    return;

  if(!copyString(ScriptName,scriptFileName,"copying script name")) {
    dStatus=BAD;
    return;
  }

  if(!openScript()) {
    dStatus=BAD;
    return;
  }
}

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::t_ScriptData
// Purpose   : (Constructor) use script file <scriptFileName> with error
//             action CONTINUE_ON_ERROR, log output to stdout,
//             stderr _and_ <logFile>
//
// Parameters: const char *scriptFileName :
//             const float timeFactor   :
//             FILE *logFile            :
//
// Return    : ---                      :
//
// Remarks   :
//
t_ScriptData::t_ScriptData( const char *scriptFileName,
                            const float timeFactor,
                            FILE *logFile) :
  t_RawData(timeFactor,logFile)
{
  dStatus=BAD;
  if(BAD == getProtectedStatus())
    return;

  init();

  if(BAD==dStatus)
    return;

  if(!copyString(ScriptName,scriptFileName,"copying script name")) {
    dStatus=BAD;
    return;
  }

  if(!openScript()) {
    dStatus=BAD;
    return;
  }
}

  // use script file <scriptFileName>,
  // logging output to stdout, stderr _and_ <logFile>
//-----------------------------------------------------------------------------
// Function  : t_ScriptData::t_ScriptData
// Purpose   : (Constructor) use script file <scriptFileName> with error
//             action CONTINUE_ON_ERROR, log output to stdout,
//             stderr _and_ <logFile>
//
// Parameters: const char *scriptFileName :
//             const float timeFactor   :
//             const t_ErrorAction e    :
//             FILE *logFile            :
//
// Return    : ---                      :
//
// Remarks   :
//
t_ScriptData::t_ScriptData( const char *scriptFileName,
                            const float timeFactor,
                            const t_ErrorAction e,
                            FILE *logFile) :
  t_RawData(timeFactor,e,logFile)
{
  dStatus=BAD;
  if(BAD == getProtectedStatus())
    return;

  init();

  if(BAD==dStatus)
    return;

  if(!copyString(ScriptName,scriptFileName,"copying script name")) {
    dStatus=BAD;
    return;
  }

  if(!openScript()) {
    dStatus=BAD;
    return;
  }
}

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::t_ScriptData
// Purpose   : return version of scriptData.
//             scriptData will not be initialized
//
// Parameters: const unsigned int length :
//             char *&versionStr       :
//
// Return    : ---                      :
//
// Remarks   :
//
t_ScriptData::t_ScriptData( const unsigned int length, char *versionStr) :
  t_RawData(length,versionStr)
{
  dStatus=VERSION_ONLY;

  if(!versionStr)
    return;

  if(strlen(version())+strlen(versionStr)+2 > length) {
    if(strlen(version()) > length) {
      strncpy(versionStr,version(),length);
      versionStr[length-1]=0;
    } else {
      strcpy(versionStr,version());
    }
  } else {
    sprintf(&versionStr[strlen(versionStr)],"\n%s",version());
  }
}

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::~t_ScriptData
// Purpose   : destructor
//
// Parameters: ---
//
// Return    : ---                      :
//
// Remarks   :
//
t_ScriptData::~t_ScriptData()
{
  if(VERSION_ONLY != dStatus)
    destroy();
}

void t_ScriptData::add2LengthOfTrajectorie(realPosition newPos)
{
  static realPosition oldPos;
  static bool firstTime=TRUE;

  if(firstTime) {
    oldPos=newPos;
    firstTime=FALSE;
    fDistanceTraveled=0.0;
    return;
  }

  fDistanceTraveled += sqrt ( (oldPos.x - newPos.x) * (oldPos.x - newPos.x)+
                              (oldPos.y - newPos.y) * (oldPos.y - newPos.y)
                              );
  oldPos=newPos;
} // add2LengthOfTrajectorie


bool
t_ScriptData::bumperReadingLine(const char* line)
{
  return (strncmp(line,SCRIPTBUMPERMARK,
		  strlen(SCRIPTBUMPERMARK)) == 0);
} // bumperReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::checkNext
// Purpose   :
//
// Parameters:  const float nonRelevantTime :
//
// Return    : t_RawData::t_Result      :
//
// Remarks   :
//
t_Result
t_ScriptData::checkNext( const float nonRelevantTime)
{
  DeltaIsNew      = FALSE;
  SonarIsNew      = FALSE;
  FrontLaserIsNew = FALSE;
  RearLaserIsNew  = FALSE;
  MarkerIsNew     = FALSE;

  if(INITIALIZED != dStatus) {
    return ERROR;
  }

  // reset timer if it is the first time
  static bool firstTime = TRUE;

  if (firstTime) {
    firstTime = FALSE;
    gettimeofday( &ScriptSettings.lastAccessTime, 0);
  }

  t_Result result = ERROR;
  switch (ScriptSettings.type) {
    case ORIG_SCRIPT_TYPE:
      result=readOrigScript(nonRelevantTime);
      break;
    case SECOND_SCRIPT_TYPE:
      result=readSecondScript(nonRelevantTime);
      break;
    default:
      logMessage("t_ScriptData::checkNext ERROR unsupported type\n");
      chooseErrorAction();
  } // switch

  //cout << "# ScriptLine: " << ScriptSettings.actualLine
  //     << ", length of trajectorie: " << fDistanceTraveled / 100.0
  //     << endl;

  return result;
} // checkNext

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::convertNeededLine2Str
// Purpose   :
//
// Parameters: const char *line        :
//             char *&str               :
//
// Return    : void                     :
//
// Remarks   :
//
void
t_ScriptData::convertNeededLine2Str( const char *line, char *&str)
{
  if(NULL != str) {
    delete str;
    str=NULL;
  }

  if(NULL == (str=new char[strlen(line)+1])) {
    logMessage("convertNeededLines2Str: out of memory!\n");
    chooseErrorAction();
    exit(1);
  }
  strcpy(str,line);
} // convertNeededLine2Str

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::copyString
// Purpose   :
//
// Parameters: char *&target            :
//             const char *source       :
//             const char *errorText    :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::copyString(char *&target, const char *source,
                         const char *errorText)
{
  if(!strlen(source)) {
    logMessage(errorText,"\n");
    chooseErrorAction();
    return FALSE;
  }
  if(target) {
    delete target;
    target=NULL;
  }
  if(!(target=new char[strlen(source)+1])) {
    logMessage("ERROR: copyString out of memory for %s\n", errorText);
    chooseErrorAction();
    return FALSE;
  }

  sprintf(target,"%s",source);
  return TRUE;
} // copyString

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::descriptionReadingLine
// Purpose   :
//
// Parameters: const char* line         :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::descriptionReadingLine( const char* line)
{
  return (strncmp(line,ORIG_SCRIPTDESCRIPTIONMARK,
		  strlen(ORIG_SCRIPTDESCRIPTIONMARK)) == 0);
} // descriptionReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::destroy
// Purpose   : free memory
//
// Parameters: ---
//
// Return    : void                     :
//
// Remarks   :
//
void
t_ScriptData::destroy()
{
  if(NULL != ScriptSettings.info.name) {
    delete ScriptSettings.info.type;
    ScriptSettings.info.type=NULL;
  }

  if(NULL != ScriptSettings.info.description) {
    delete ScriptSettings.info.description;
    ScriptSettings.info.description=NULL;
  }

  if(NULL != ScriptSettings.info.name) {
    delete ScriptSettings.info.name;
    ScriptSettings.info.name=NULL;
  }

  if(bReadScriptFromStdin)
    if(ScriptStreamNotStdin.is_open())
      ScriptStreamNotStdin.close();

  if(ScriptName)
    delete ScriptName;
  ScriptName=NULL;

  ScriptStream=NULL;

  dStatus=BAD;
} // destroy

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::getActualLine
// Purpose   :
//
// Parameters: ---
//
// Return    : unsigned long            :
//
// Remarks   :
//
unsigned long
t_ScriptData::getActualLine()
{
  return ScriptSettings.actualLine;
} // getActualLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::getBasePosition
// Purpose   : BasePosition is always valid even if robot has not moved
//             between last checkNext() and actual checkNext()
//
// Parameters:  t_RealPosition *basePos :
//
// Return    : bool                     : TRUE: robot has moved between
//                                              last checkNext()-calls
//
// Remarks   :
//
bool
t_ScriptData::getBasePosition( t_RealPosition *basePos)
{
  basePos->x   = BasePosition.x;
  basePos->y   = BasePosition.y;
  basePos->rot = BasePosition.rot;

  return DeltaIsNew;
}
bool
t_ScriptData::getBasePosition( t_RealPosition &basePos)
{
  t_RealPosition *pB=&basePos;
  return getBasePosition(pB);
} // getBasePosition

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::getBump
// Purpose   : Get bump generated with addNoise (D.Fox)
//
// Parameters:  t_RealPosition *bumpDelta :
//
// Return    : bool                     : TRUE: bump occured
//
// Remarks   :
//
bool
t_ScriptData::getBump( t_RealPosition *bumpDelta)
{
  bumpDelta->x   = fBumpForward;
  bumpDelta->y   = fBumpSideward;
  bumpDelta->rot = fBumpRot;

  return BumpIsNew;
}
bool
t_ScriptData::getBump( t_RealPosition &bumpDelta)
{
  t_RealPosition *pB=&bumpDelta;
  return getBump(pB);
} // getBasePosition

void t_ScriptData::getCounts( unsigned long &sonarCnt,
                              unsigned long &laserCnt,
                              unsigned long &posCnt,
                              unsigned long &markerCnt,
                              unsigned long &noMoveCnt)
{
   unsigned long *sC=&sonarCnt;
   unsigned long *lC=&laserCnt;
   unsigned long *pC=&posCnt;
   unsigned long *mC=&markerCnt;
   unsigned long *nmC=&noMoveCnt;
   getCounts(sC,lC,pC,mC,nmC);
} // getCounts

void t_ScriptData::getCounts( unsigned long *sonarCnt,
                              unsigned long *laserCnt,
                              unsigned long *posCnt,
                              unsigned long *markerCnt,
                              unsigned long *noMoveCnt)
{
   *sonarCnt=ScriptSettings.sonarCount;
   *laserCnt=ScriptSettings.laserCount;
   *posCnt=ScriptSettings.positionCount;
   *markerCnt=ScriptSettings.markerCount;
   *noMoveCnt=ScriptSettings.noMoveCount;
} // getCounts

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::getDistanceTraveled
// Purpose   : BasePosition is always valid even if robot has not moved
//             between last checkNext() and actual checkNext()
//
// Parameters:  t_RealPosition *&basePos :
//
// Return    : ---
//
// Remarks   :
//
void
t_ScriptData::getDistanceTraveled( float &dist)
{
  dist = fDistanceTraveled;
} // getDistanceTraveled

float t_ScriptData::getElapsedTime()
{
  switch (ScriptSettings.type) {
    case ORIG_SCRIPT_TYPE:
      return scriptTimeDiff(&ScriptSettings.currentTimeOfScriptOne,
                            &ScriptSettings.startTimeOfScriptOne);
      break;
    case SECOND_SCRIPT_TYPE:
      return timeDiff(&ScriptSettings.currentTimeOfScriptTwo,
                      &ScriptSettings.startTimeOfScriptTwo);
      break;
    default:
      return 0.0;
  } // switch
}
//-----------------------------------------------------------------------------
// Function  : t_ScriptData::getFrontLaser
// Purpose   :
//
// Parameters:  t_Reading *reading      :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::getFrontLaser( t_Reading *reading)
{
  if(FrontLaserIsNew) {
    if(reading->maxNumberOfReadings < FrontLaser->actualNumberOfReadings) {
      reading->actualNumberOfReadings=reading->maxNumberOfReadings;
      logMessage("# getFrontLaser WARNING: max number of readings %u "
                 "below actual number of readings %u.\n",
                 reading->maxNumberOfReadings,
                 FrontLaser->actualNumberOfReadings);
    } else
      reading->actualNumberOfReadings=FrontLaser->actualNumberOfReadings;
    for (unsigned int i = 0; i < reading->actualNumberOfReadings ; i++)
      reading->reading[i].dist=FrontLaser->reading[i].dist;
  }
  return FrontLaserIsNew;
}
bool
t_ScriptData::getFrontLaser( t_Reading &reading)
{
  t_Reading *pR=&reading;
  return (getFrontLaser(pR));
} // getFrontLaser

//--------------------------------------------------------------------y---------
// Function  : t_ScriptData::getMarker
// Purpose   :
//
// Parameters:  long &number            :
//             t_RealPosition *&markerPos :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::getMarker( long &number, t_RealPosition *&markerPos)
{
  if(MarkerIsNew) {
    number=MarkerNumber;
    markerPos->x   = MarkerPos.x;
    markerPos->y   = MarkerPos.y;
    markerPos->rot = MarkerPos.rot;
  }
  return MarkerIsNew;
}
bool
t_ScriptData::getMarker( long &number, t_RealPosition &markerPos)
{
  t_RealPosition *pM=&markerPos;
  return getMarker(number,pM);
} // getMarker

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::getMovement
// Purpose   :
//
// Parameters:  t_RealPosition *&delta  :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::getMovement( t_RealPosition *&delta)
{
  if(DeltaIsNew) {
    delta->x   = Delta.x;
    delta->y   = Delta.y;
    delta->rot = Delta.rot;
  }
  return DeltaIsNew;
}
bool
t_ScriptData::getMovement( t_RealPosition &delta)
{
  t_RealPosition *pD=&delta;
  return getMovement(pD);
} // getMovement

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::getPatternType
// Purpose   :
//
// Parameters: const char *line         :
//
// Return    : t_secondScriptPatterns   :
//
// Remarks   :
//
t_ScriptData::t_secondScriptPatterns
t_ScriptData::getPatternType(const char *line)
{
  if(!patternSetNameReadingLine(line))
    return UNKNOWN_PATTERN;

  else if(0==strncmp(&line[strlen(SECOND_SCRIPT_PAT_NAME)],
                     SECOND_SCRIPT_PAT_TYPE_BUTTON,
                     strlen(SECOND_SCRIPT_PAT_TYPE_BUTTON)))
    return BUTTON_PATTERN;

  else if(0==strncmp(&line[strlen(SECOND_SCRIPT_PAT_NAME)],
                     SECOND_SCRIPT_PAT_TYPE_INFRARED,
                     strlen(SECOND_SCRIPT_PAT_TYPE_INFRARED)))
    return INFRARED_PATTERN;

  else if(0==strncmp(&line[strlen(SECOND_SCRIPT_PAT_NAME)],
                     SECOND_SCRIPT_PAT_TYPE_LASER,
                     strlen(SECOND_SCRIPT_PAT_TYPE_LASER)))
    return LASER_PATTERN;

  else if(0==strncmp(&line[strlen(SECOND_SCRIPT_PAT_NAME)],
                     SECOND_SCRIPT_PAT_TYPE_MARKER,
                     strlen(SECOND_SCRIPT_PAT_TYPE_MARKER)))
    return MARKER_PATTERN;

  else if(0==strncmp(&line[strlen(SECOND_SCRIPT_PAT_NAME)],
                     SECOND_SCRIPT_PAT_TYPE_SONAR,
                     strlen(SECOND_SCRIPT_PAT_TYPE_SONAR)))
    return SONAR_PATTERN;

  else if(0==strncmp(&line[strlen(SECOND_SCRIPT_PAT_NAME)],
                     SECOND_SCRIPT_PAT_TYPE_TACTILE,
                     strlen(SECOND_SCRIPT_PAT_TYPE_TACTILE)))
    return TACTILE_PATTERN;

  else return UNKNOWN_PATTERN;
} // getPatternType

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::getReadings
// Purpose   :
//
// Parameters: const char *line         :
//             const unsigned int numberOfReadings :
//             float *sensor            :
//
// Return    : bool                     :
//
// Remarks   :
//

bool
t_ScriptData::getReadings( const char *line,
                           const unsigned int numberOfReadings,
                           float *sensor)
{
  bool bError = FALSE;
  unsigned int i = 0;
  const char *p = line;

  while (!bError && (i < numberOfReadings)){
    bError = (sscanf(p, "%e", &sensor[i++]) != 1);
    if (!bError) {
      do {
        p++;
      } while ((*p != ' ') && (*p != '\0') && (*p != '\n'));
    }
    if (*p != ' ') {
      if(i != numberOfReadings) {
        logMessage("# t_ScriptData::getReadings ERROR end of reading line "
                   "reached...\n");
        // no chooseErrorAction() cause calling function will log further
        // information
        bError = TRUE;
      }
    }
  }

  return i == numberOfReadings;
} // getReadings

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::getReadingsInverse
// Purpose   :
//
// Parameters: const char *line         :
//             const unsigned int numberOfReadings :
//             float *sensor            :
//
// Return    : bool                     :
//
// Remarks   : get <numberOfReadings> readings in descending order
//

bool
t_ScriptData::getReadingsInverse( const char *line,
                           const unsigned int numberOfReadings,
                           float *sensor)
{
  bool bError = FALSE;
  if(0==numberOfReadings) {
    logMessage("# t_ScriptData::getReadingsInverse warning: 0 readings to get. "
               "\n");
    return FALSE;
  }
  int i = numberOfReadings-1;
  const char *p = line;

  while (!bError && (i >= 0)) {
    bError = (sscanf(p, "%e", &sensor[i--]) != 1);
    if (!bError) {
      do {
        p++;
      } while ((*p != ' ') && (*p != '\0') && (*p != '\n'));
    }
    if (*p != ' ') {
      if(i != -1) {
        logMessage("# t_ScriptData::getReadingsInverse ERROR end of reading line "
                   "reached...\n");
        // no chooseErrorAction() cause calling function will log further
        // information
        bError = TRUE;
      }
    }
  }

  return i == -1;
} // getReadingsInverse

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::getRearLaser
// Purpose   :
//
// Parameters:  t_Reading *reading      :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::getRearLaser( t_Reading *reading)
{
  if(RearLaserIsNew) {
    if(reading->maxNumberOfReadings < RearLaser->actualNumberOfReadings) {
      reading->actualNumberOfReadings=reading->maxNumberOfReadings;
      logMessage("# getRearLaser WARNING: max number of readings %u "
                 "below actual number of readings %u.\n",
                 reading->maxNumberOfReadings,
                 RearLaser->actualNumberOfReadings);

    } else
      reading->actualNumberOfReadings=RearLaser->actualNumberOfReadings;
    for (unsigned int i = 0; i < reading->actualNumberOfReadings ; i++)
      reading->reading[i].dist=RearLaser->reading[i].dist;
  }
  return RearLaserIsNew;
}
bool
t_ScriptData::getRearLaser( t_Reading &reading)
{
  t_Reading *pR=&reading;
  return (getRearLaser(pR));
} // getRearLaser

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::getSonar
// Purpose   :
//
// Parameters:  t_Reading *reading      :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::getSonar( t_Reading *reading)
{
  if(SonarIsNew) {
    if(reading->maxNumberOfReadings < Sonar->actualNumberOfReadings) {
      reading->actualNumberOfReadings=reading->maxNumberOfReadings;
      logMessage("# getSonar WARNING: max number of readings %u "
                 "below actual number of readings %u.\n",
                 reading->maxNumberOfReadings,
                 Sonar->actualNumberOfReadings);

    } else
      reading->actualNumberOfReadings=Sonar->actualNumberOfReadings;
    for (unsigned int i = 0; i < reading->actualNumberOfReadings ; i++)
      reading->reading[i].dist=Sonar->reading[i].dist;
  }
  return SonarIsNew;
}
bool
t_ScriptData::getSonar( t_Reading &reading)
{
  t_Reading *pS=&reading;
  return (getSonar(pS));
} // getSonar

void t_ScriptData::getStartPosition(t_RealPosition &pos)
{
   t_RealPosition *pP=&pos;
   getStartPosition(pP);
} // getStartPosition

void t_ScriptData::getStartPosition(t_RealPosition *pos)
{
   *pos=StartPosition;
} // getStartPosition

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::getStatus
// Purpose   :
//
// Parameters: ---
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::getStatus()
{
  if(INITIALIZED != getProtectedStatus())
    return FALSE;
  if(INITIALIZED != dStatus)
    return FALSE;

  return TRUE;
} // getStatus

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::init
// Purpose   :
//
// Parameters: ---
//
// Return    : void                     :
//
// Remarks   :
//
void
t_ScriptData::init()
{
  dStatus=BAD;
  ScriptName=NULL;
  bReadScriptFromStdin=FALSE;

  ScriptSettings.distanceOffset               = 0.0;
  ScriptSettings.type                         = UNKNOWN_TYPE;
  ScriptSettings.startTimeOfScriptOne.hour       = 0;
  ScriptSettings.startTimeOfScriptOne.minute     = 0;
  ScriptSettings.startTimeOfScriptOne.second     = 0.0;
  ScriptSettings.startTimeOfScriptTwo.tv_sec     = 0;
  ScriptSettings.startTimeOfScriptTwo.tv_usec    = 0;
  ScriptSettings.currentTimeOfScriptOne.hour     = 0;
  ScriptSettings.currentTimeOfScriptOne.minute   = 0;
  ScriptSettings.currentTimeOfScriptOne.second   = 0.0;
  ScriptSettings.currentTimeOfScriptTwo.tv_sec   = 0;
  ScriptSettings.currentTimeOfScriptTwo.tv_usec  = 0;
  ScriptSettings.lastAccessTime.tv_sec        = 0;
  ScriptSettings.lastAccessTime.tv_usec       = 0;
  ScriptSettings.actualLine                   = 0;
  ScriptSettings.sonarCount                   = 0;
  ScriptSettings.laserCount                   = 0;
  ScriptSettings.positionCount                = 0;
  ScriptSettings.markerCount                  = 0;
  ScriptSettings.noMoveCount                  = 0;
  ScriptSettings.info.name                    = NULL;
  ScriptSettings.info.description             = NULL;
  ScriptSettings.info.type                    = NULL;

  BumpIsNew = FALSE;

  dStatus=INITIALIZED;
} // init

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::laserReadingLine
// Purpose   :
//
// Parameters: const char* line         :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::laserReadingLine( const char* line)
{
  return (strncmp(line,ORIG_SCRIPTLASERMARK,
		  strlen(ORIG_SCRIPTLASERMARK)) == 0);
} // laserReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::nameReadingLine
// Purpose   :
//
// Parameters: const char* line         :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::nameReadingLine( const char* line)
{
  return (strncmp(line,ORIG_SCRIPTNAMEMARK,
		  strlen(ORIG_SCRIPTNAMEMARK)) == 0);
} // nameReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::normedAngleRAD
// Purpose   :
//
// Parameters: float angle              :
//
// Return    : float                    :
//
// Remarks   :
//
float
t_ScriptData::normedAngleRAD(const float angle)
{
  float newAngle=angle;
  while (0 > newAngle)
    newAngle = newAngle + 6.283185307; // 2pi

  while (6.283185307 <= newAngle)
    newAngle = newAngle - 6.283185307;

  return(newAngle);
} // normedAngleRAD


//-----------------------------------------------------------------------------
// Function  : t_ScriptData::markerReadingLine
// Purpose   :
//
// Parameters: const char* line         :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::markerReadingLine( const char* line)
{
  return (strncmp(line,ORIG_MARKERSCRIPTMARK,
		  strlen(ORIG_MARKERSCRIPTMARK)) == 0);
} // markerReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::openScript
// Purpose   :
//
// Parameters: ---
//
// Return    : bool                     :
//
// Remarks   :
//

bool
t_ScriptData::openScript()
{
  bool eof;
  bool timeRead;
  bool positionRead;

  bool patternSetRead;

  char line[DATA_BUFFLEN];

  float startX;
  float startY;
  float startRot;

  // try to open file

  if((0==strcmp(ScriptName,STDIN_SCRIPT_NAME)) ||
     (0==strcmp(ScriptName,"-")) ) {
    bReadScriptFromStdin=TRUE;
    logMessage("# Reading script from stdin!\n");
    ScriptStream=&cin;
    if(ScriptStream->bad()) {
      logMessage("# t_ScriptData::openScript ERROR: Could not open stdin!\n");
      chooseErrorAction();
      return FALSE;
    }
  } else {
    bReadScriptFromStdin=FALSE;
    ScriptStreamNotStdin.open(ScriptName,ios::in);
    if(ScriptStreamNotStdin.bad()) {
      logMessage("# t_ScriptData::openScript ERROR: Could not open "
                 "file '%s'!\n",ScriptName);
      chooseErrorAction();
      return FALSE;
    }
    ScriptStream=&ScriptStreamNotStdin;
  }

  // if ((ScriptFile = fopen(ScriptName,"rt")) == NULL) {
  //     logMessage("ERROR: Could not open file '", ScriptName, "'!\n");
  //     return FALSE;
  //   }

  eof = FALSE;
  timeRead = FALSE;
  positionRead = FALSE;
  patternSetRead = FALSE;

  struct timeval startTime;

  // check file, load start data

  while (!eof && !((timeRead && positionRead) || (patternSetRead))){
    eof = !readLine(line,DATA_BUFFLEN);
    if (!eof){
      if (positionReadingLine(line)){
	if (sscanf(&line[strlen(ORIG_SCRIPTPOSMARK)],"%f %f %f",
	       &startX, &startY, &startRot) == 3)
	  positionRead = TRUE;
      }
      if (timeReadingLine(line))  {
	if (sscanf(&line[strlen(ORIG_TIMEMARK)+9], "%d:%d:%f",
		   &(ScriptSettings.currentTimeOfScriptOne.hour),
		   &(ScriptSettings.currentTimeOfScriptOne.minute),
		   &(ScriptSettings.currentTimeOfScriptOne.second)) == 3)
	  timeRead = TRUE;
      }
      if (nameReadingLine(line)) {
        convertNeededLine2Str(&line[strlen(ORIG_SCRIPTNAMEMARK)],
                              ScriptSettings.info.name);
      }
      if (descriptionReadingLine(line)) {
        convertNeededLine2Str(&line[strlen(ORIG_SCRIPTDESCRIPTIONMARK)],
                              ScriptSettings.info.description);
      }
      if (patternSetStartReadingLine(line))  {
	  patternSetRead = TRUE;
      }
    }
  }

  if (timeRead && positionRead){
    ScriptSettings.type = ORIG_SCRIPT_TYPE;

    StartPosition.x  =startX;
    StartPosition.y  =startY;
    StartPosition.rot=startRot;

    scriptPosition2RobotPosition(StartPosition);

    BasePosition=StartPosition;

    ScriptSettings.startTimeOfScriptOne = ScriptSettings.currentTimeOfScriptOne;
    if( NULL != ScriptSettings.info.name ){
      logMessage("# OrigScript descriptive name: %s.\n",
                 ScriptSettings.info.name);
    } else {
      logMessage("# OrigScript descriptive name: unknown.\n");
    }

    if(0) // this output is senseless for ORIG_SCRIPT_TYPE
      if( NULL != ScriptSettings.info.type ){
        logMessage("# OrigScript type: %s.\n", ScriptSettings.info.type);
      } else {
        logMessage("# OrigScript type: unknown.\n");
      }

    if( NULL != ScriptSettings.info.description ){
      logMessage("# OrigScript description: %s.\n",
                 ScriptSettings.info.description);
    } else {
      logMessage("# OrigScript description: none.\n");
    }

    logMessage("# Script starts with X: %.3f  Y: %.3f  Rot: %.3f\n",
               StartPosition.x,
               StartPosition.y,
               StartPosition.rot);
    gettimeofday( &ScriptSettings.lastAccessTime, 0);

    return TRUE;
  } else if( patternSetRead ) {
    bool patternRead = FALSE;

    ScriptSettings.type = SECOND_SCRIPT_TYPE;
    if(doOdometryCorrection()) {
      logMessage("# Disable odometry correction with second-script.\n");
      setOdometryCorrection(1.0);
    }
    while (!eof && !(patternRead)) {
      eof = !readLine(line,DATA_BUFFLEN);
      if (!eof){
        if (patternNameReadingLine(line)) {
          convertNeededLine2Str(&line[strlen(SECOND_SCRIPT_NAME_STR)],
                                ScriptSettings.info.name);
        } else if (patternTypeReadingLine(line)) {
          convertNeededLine2Str(&line[strlen(SECOND_SCRIPT_TYPE_STR)],
                                ScriptSettings.info.type);
        } else if (patternStartReadingLine(line)){
          // found first pattern.
          // get time and pos of pattern
          bool foundTime = FALSE;
          bool foundPos = FALSE;
          bool patternEnd = FALSE;

          while (!eof && !(patternEnd)) { // read till patternEnd
            eof = !readLine(line,DATA_BUFFLEN);
            if( patternSetTimeReadingLine(line,startTime)) {
              foundTime = TRUE;
              ScriptSettings.currentTimeOfScriptTwo = startTime;
              ScriptSettings.startTimeOfScriptTwo = ScriptSettings.currentTimeOfScriptTwo;
            } else if( patternSetPositionReadingLine(line,StartPosition)) {
              foundPos = TRUE;
              scriptPosition2RobotPosition(StartPosition);
              BasePosition=StartPosition;
            } else if( patternSetEndReadingLine(line)) {
              patternEnd = TRUE;
            }
          } // while

          if(foundTime && foundPos) {
            patternRead = TRUE;
          }
        }
      }
    }
    if( NULL != ScriptSettings.info.name ){
      logMessage("# SecondScript descriptive name: %s.\n",
                 ScriptSettings.info.name);
    } else {
      logMessage("# SecondScript descriptive name: unknown.\n");
    }

    if( NULL != ScriptSettings.info.type ){
      logMessage("# SecondScript type: %s.\n", ScriptSettings.info.type);
    } else {
      logMessage("# SecondScript type: unknown.\n");
    }

    if( NULL != ScriptSettings.info.description ){
      logMessage("# SecondScript description: %s.\n",
                 ScriptSettings.info.description);
    } else {
      logMessage("# SecondScript description: none.\n");
    }

    logMessage("# Script starts with X: %.3f  Y: %.3f  Rot: %.3f\n",
               StartPosition.x,
               StartPosition.y,
               StartPosition.rot);
    gettimeofday( &ScriptSettings.lastAccessTime, 0);

    return TRUE;
  } else {
    if(bReadScriptFromStdin) {
      logMessage("t_ScriptData::openScript ERROR: stdin is no script file.\n",
                 ScriptName);
    } else {
      logMessage("t_ScriptData::openScript ERROR: %s is no script file.\n",
                 ScriptName);
      ScriptStreamNotStdin.close();
    }
    ScriptSettings.type = UNKNOWN_TYPE;
    chooseErrorAction();
    return FALSE;
  }
} // openScript

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::patternEndReadingLine
// Purpose   :
//
// Parameters:  const char* line        :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::patternEndReadingLine( const char* line)
{
  return (strncmp(line,SECOND_SCRIPT_END,
		  strlen(SECOND_SCRIPT_END)) == 0);
} // patternEndReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScritpData::patternNameReadingLine
// Purpose   :
//
// Parameters:  const char* line        :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::patternNameReadingLine( const char* line)
{
  return (strncmp(line,SECOND_SCRIPT_NAME_STR,
		  strlen(SECOND_SCRIPT_NAME_STR)) == 0);
} // patternNameReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::patternSetButtonReadingLine
// Purpose   :
//
// Parameters: const char* line         :
//             t_Button *b              :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::patternSetButtonReadingLine( const char* line, t_Button *b)
{
  if(strncmp(line,SECOND_SCRIPT_PAT_BUTTON,
             strlen(SECOND_SCRIPT_PAT_BUTTON)) == 0) {
    //t_Button newB;

    static bool firstTime=TRUE;
    if(firstTime) {
      firstTime=FALSE;
      cerr << "# WARNING: buttons not implemented\n";
    }
    b=b;
    //exit(1);
    //if(2 == sscanf(&line[strlen(SECOND_SCRIPT_PAT_BUTTON)], "%ld %ld",
    //              &newT.tv_sec, &newT.tv_usec)) {
    //b=newB;
    //  return TRUE;
    //}
  }
  return FALSE;

} // patternSetButtonReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::patternSetEndReadingLine
// Purpose   :
//
// Parameters:  const char* line        :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::patternSetEndReadingLine( const char* line)
{
  return (strncmp(line,SECOND_SCRIPT_PAT_END,
		  strlen(SECOND_SCRIPT_PAT_END)) == 0);
} // patternSetEndReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::patternSetInfraredReadingLine
// Purpose   :
//
// Parameters: const char* line         :
//             t_Infrared *i            :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::patternSetInfraredReadingLine( const char* line, t_Infrared *i)
{
  if(strncmp(line,SECOND_SCRIPT_PAT_INFRARED,
             strlen(SECOND_SCRIPT_PAT_INFRARED)) == 0) {
    //t_Infrared newI;

    static bool firstTime=TRUE;
    if(firstTime) {
      firstTime=FALSE;
      cerr << "# WARNING: infrared not implemented\n";
    }

    i=i;
    //exit(1);
    //if(2 == sscanf(&line[strlen(SECOND_SCRIPT_PAT_INFRARED)], "%ld %ld",
    //              &newT.tv_sec, &newT.tv_usec)) {
    //i=newI;
    //  return TRUE;
    //}
  }
  return FALSE;

} // patternSetInfraredReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::patternSetLaserReadingLine
// Purpose   :
//
// Parameters: const char* line         :
//             float *sensor            :
//             int &frontSize           :
//             int &rearSize            :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::patternSetLaserReadingLine( const char* line, float *sensor,
                                          int &frontSize, int &rearSize)

{
  if(strncmp(line,SECOND_SCRIPT_PAT_LASER,
             strlen(SECOND_SCRIPT_PAT_LASER)) == 0) {

    int markLength = strlen(SECOND_SCRIPT_PAT_LASER);
    frontSize=0;
    rearSize=0;
    int numberOfInts = sscanf( &line[markLength], "%d %d :",
                               &frontSize, &rearSize);

    if ( numberOfInts == 1) {
      frontSize = frontSize / 2;
      rearSize = frontSize;
    }

    // Set the mark behind the second ":".
    if(NULL == strstr(&line[markLength],":")) {
      logMessage("# ERROR 11 while scanning script %s\n" , ScriptName);
      logMessage("# Cannot scan line %lu:\n%s\n", ScriptSettings.actualLine,
                 line);
      chooseErrorAction();
    } else {
      // there may be a ":" in "SECOND_SCRIPT_PAT_LASER", too.
      markLength += strlen(&line[markLength]) -
                    strlen(strstr(&line[markLength],":")) +
                    1;
    }
    if((unsigned int)markLength >= strlen(line)) {
      logMessage("# ERROR 12 while scanning script %s\n" , ScriptName);
      logMessage("# Cannot scan line %lu:\n%s\n", ScriptSettings.actualLine,
                 line);
      chooseErrorAction();
      return FALSE;
    }

    if( getReadings( &line[markLength], frontSize + rearSize, sensor)) {
      bool checkFront=TRUE;
      bool checkRear=TRUE;
      for ( int cnt = 0; cnt < frontSize && checkFront; cnt++)
        if(-1 == sensor[cnt])
          checkFront= FALSE;
      for ( int cnt = frontSize; cnt < frontSize+rearSize && checkRear; cnt++)
        if(-1 == sensor[cnt])
          checkRear= FALSE;
      return checkFront | checkRear;
    }

  }
  return FALSE;

} // patternSetLaserReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::patternSetMarkerReadingLine
// Purpose   :
//
// Parameters: const char* line         :
//             unsigned long &m         :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::patternSetMarkerReadingLine( const char* line, unsigned long &m)
{
  if(strncmp(line,SECOND_SCRIPT_PAT_MARKER,
             strlen(SECOND_SCRIPT_PAT_MARKER)) == 0) {
    unsigned long newM;

    if(1 == sscanf(&line[strlen(SECOND_SCRIPT_PAT_MARKER)], "%ld",
                  &newM)) {
      m=newM;
      return TRUE;
    }
  }
  return FALSE;

} // patternSetMarkerReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScritpData::patternSetNameReadingLine
// Purpose   :
//
// Parameters:  const char* line        :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::patternSetNameReadingLine( const char* line)
{
  return (strncmp(line,SECOND_SCRIPT_PAT_NAME,
		  strlen(SECOND_SCRIPT_PAT_NAME)) == 0);
} // patternSetNameReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::patternSetPositionReadingLine
// Purpose   :
//
// Parameters:  const char* line        :
//             t_RealPosition &newPos   :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::patternSetPositionReadingLine( const char* line,
                                          t_RealPosition &newPos)
{
  bool retVal=(strncmp(line,SECOND_SCRIPT_PAT_POSITION,
                       strlen(SECOND_SCRIPT_PAT_POSITION)) == 0);
  if(retVal) {
    t_RealPosition pos;
    if (retVal = (sscanf(&line[strlen(SECOND_SCRIPT_PAT_POSITION)],"%f %f %f",
                         &pos.x, &pos.y, &pos.rot) == 3)) {
      newPos.x   = pos.x;
      newPos.y   = pos.y;
      newPos.rot = pos.rot;
    }
  }
  return retVal;
} // patternSetPositionReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::patternSetSonarReadingLine
// Purpose   :
//
// Parameters: const char* line         :
//             float *sensor            :
//             int &sonarReading        :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::patternSetSonarReadingLine( const char* line, float *sensor,
                                          int &sonarReading)
{
  if(strncmp(line,SECOND_SCRIPT_PAT_SONAR,
             strlen(SECOND_SCRIPT_PAT_SONAR)) == 0) {

    int markLength = strlen(SECOND_SCRIPT_PAT_SONAR);

    int sonarSize=0;
    int numberOfInts = sscanf( &line[markLength], "%d :",
                               &sonarSize);

    if ( numberOfInts != 1) {
      logMessage("# ERROR 21 while scanning script %s\n" , ScriptName);
      logMessage("# Cannot scan line %lu:\n%s\n", ScriptSettings.actualLine,
                 line);
      chooseErrorAction();
      return FALSE;
    }

    if ( sonarSize > (int)NUMBER_OF_SONARS) {
      logMessage("# ERROR 22 while scanning script %s\n" , ScriptName);
      logMessage("# Maximum allowed sonars: %d.\n",NUMBER_OF_SONARS);
      logMessage("# Num sonars of this pattern in line %lu: %d\n",
                 ScriptSettings.actualLine,sonarSize);
      chooseErrorAction();
      return FALSE;
    }

    // Set the mark behind the second ":".
    if(NULL == strstr(&line[markLength],":")) {
      logMessage("# ERROR 23 while scanning script %s\n" , ScriptName);
      logMessage("# Cannot scan line %lu:\n%s\n", ScriptSettings.actualLine,
                 line);
      chooseErrorAction();
      return FALSE;
    } else {
      // there may be a ":" in "SECOND_SCRIPT_PAT_SONAR", too.
      markLength += strlen(&line[markLength]) -
                    strlen(strstr(&line[markLength],":")) +
                    1;
    }
    if((unsigned int)markLength >= strlen(line)) {
      logMessage("# ERROR 24 while scanning script %s\n" , ScriptName);
      logMessage("# Cannot scan line %lu:\n%s\n", ScriptSettings.actualLine,
                 line);
      chooseErrorAction();
      return FALSE;
    }

    if(getReadingsInverse(&line[markLength],
                          sonarSize, sensor)) {

      // Substract the size of the robot.
      for ( int cnt = 0; cnt < sonarSize; cnt++)
        sensor[cnt] = fMax( 0.0, sensor[cnt] - getDistanceOffset());

      sonarReading=sonarSize;
      return TRUE;
    }
  }
  return FALSE;

} // patternSetSonarReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::patternSetStartReadingLine
// Purpose   :
//
// Parameters:  const char* line        :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::patternSetStartReadingLine( const char* line)
{
  return (strncmp(line,SECOND_SCRIPT_START,
		  strlen(SECOND_SCRIPT_START)) == 0);
} // patternSetStartReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::patternSetTactileReadingLine
// Purpose   :
//
// Parameters: const char* line         :
//             t_Tactile *t             :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::patternSetTactileReadingLine( const char* line, t_Tactile *t)
{
  if(strncmp(line,SECOND_SCRIPT_PAT_TACTILE,
             strlen(SECOND_SCRIPT_PAT_TACTILE)) == 0) {
    //t_Reading newT;

    static bool firstTime=TRUE;
    if(firstTime) {
      firstTime=FALSE;
      cerr << "# WARNING: tactile not implemented\n";
    }
    t=t;
    //exit(1);
    //if(2 == sscanf(&line[strlen(SECOND_SCRIPT_PAT_TACTILE)], "%ld %ld",
    //              &newT.tv_sec, &newT.tv_usec)) {
    //i=newI;
    //  return TRUE;
    //}
  }
  return FALSE;

} // patternSetTactileReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::patternSetTimeReadingLine
// Purpose   :
//
// Parameters: const char* line         :
//             struct timeval &t        :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::patternSetTimeReadingLine( const char* line, struct timeval &t)
{
  if(strncmp(line,SECOND_SCRIPT_PAT_TIME,
             strlen(SECOND_SCRIPT_PAT_TIME)) == 0) {
    struct timeval newT;

    // some machines use long int, some use int...
    long sec,usec;
    if(2 == sscanf(&line[strlen(SECOND_SCRIPT_PAT_TIME)],
                   "%ld %ld",
                   &(sec), &(usec))) {
      newT.tv_sec=sec;
      newT.tv_usec=usec;
      t=newT;
      return TRUE;
    }
  }
  return FALSE;

} // patternSetTimeReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::patternStartReadingLine
// Purpose   :
//
// Parameters:  const char* line        :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::patternStartReadingLine( const char* line)
{
  return (strncmp(line,SECOND_SCRIPT_PAT_START,
		  strlen(SECOND_SCRIPT_PAT_START)) == 0);
} // patternStartReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::patternTypeReadingLine
// Purpose   :
//
// Parameters:  const char* line        :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::patternTypeReadingLine( const char* line)
{
  return (strncmp(line,SECOND_SCRIPT_TYPE_STR,
		  strlen(SECOND_SCRIPT_TYPE_STR)) == 0);
} // patternTypeReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::processMarkerReading
// Purpose   :
//
// Parameters: ---
//
// Return    : void                     :
//
// Remarks   :
//
void t_ScriptData::processMarkerReading(const long markerNumber)
{

  MarkerPos.x   = BasePosition.x;
  MarkerPos.x   = BasePosition.y;
  MarkerPos.rot = BasePosition.rot;

  MarkerNumber=markerNumber;

  cerr << "# Found marker: " << markerNumber << endl;

  if( isWanted(MARKER_DATA) )
    MarkerIsNew = TRUE;
} // processMarkerReading

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::processPositionReading
// Purpose   :
//
// Parameters: t_RealPosition &newPos   :
//
// Return    : void                     :
//
// Remarks   :
//
void t_ScriptData::processPositionReading( t_RealPosition &newPos)
{
  // This is a copy of the base status position.
  // Convert log angle to robot angle.

  scriptPosition2RobotPosition(newPos);

  updateMovement(newPos); // to set delta, DeltaIsNew and BasePosition

  BasePosition=newPos;
} // processPositionReading

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::positionReadingLine
// Purpose   :
//
// Parameters: const char *line         :
//
// Return    : static bool              :
//
// Remarks   :
//
bool
t_ScriptData::positionReadingLine(const char *line)
{
  return (strncmp(line,ORIG_SCRIPTPOSMARK,
		  strlen(ORIG_SCRIPTPOSMARK)) == 0);
} // positionReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::readLine
// Purpose   : read maximum <maxLen> bytes in <ScriptStream>. Stop reading
//             if EOF or eol (== <END_OF_LINE> is reached.
//
//
// Parameters: char *line               :
//             const unsigned int maxLen :
//
// Return    : bool                     : TRUE on success
//                                        FALSE: any ERROR or
//                                               trying to read behind EOF
//
// Remarks   :
//
bool
t_ScriptData::readLine( char *line, const unsigned int maxLen)
{
  if(ScriptStream->bad()) {
    logMessage("t_ScriptData:readLine WARNIG script bad.\n");
    chooseErrorAction();
    return FALSE;
  }
  if(ScriptStream->eof())
    return FALSE;

  ScriptSettings.actualLine++;
  ScriptStream->get(line,maxLen,END_OF_LINE);

  // check if current line fits in <line>
  char c;
  if(ScriptStream->get(c) &&  END_OF_LINE != c) {
    logMessage("t_ScriptData:readLine WARNING inputline %lu to long\n",
               ScriptSettings.actualLine);
    ScriptSettings.actualLine--;
    ScriptStream->putback(c);
  }
  return TRUE;
} // readLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::readOrigScript
// Purpose   :
//
// Parameters: const float nonRelevantTime :
//
// Return    : t_Result                 :
//
// Remarks   :
//
t_Result
t_ScriptData::readOrigScript( const float nonRelevantTime)
{
  char line[DATA_BUFFLEN];
  char positionLine[DATA_BUFFLEN];
  char markerLine[DATA_BUFFLEN];
  char sonarLine[DATA_BUFFLEN];
  char laserLine[DATA_BUFFLEN];

  bool newPositionLine = FALSE;
  bool newSonarLine    = FALSE;
  bool newLaserLine    = FALSE;
  bool newMarkerLine   = FALSE;
  bool newTimeLine     = FALSE;

  bool eof             = FALSE;
  bool timeSkipped     = FALSE;


  unsigned long positionLineNumber = ScriptSettings.actualLine;
  unsigned long sonarLineNumber    = ScriptSettings.actualLine;
  unsigned long laserLineNumber    = ScriptSettings.actualLine;
  unsigned long markerLineNumber   = ScriptSettings.actualLine;
  unsigned long timeLineNumber     = ScriptSettings.actualLine;

  float sensor[MAX_VALUESPERSCAN];

  t_Result  returnValue = NO_NEW_DATA;
  unsigned int cnt      = 0;

  t_RealPosition newPos;
  struct timeval currentTime;
  gettimeofday(&currentTime, 0);
  t_ScriptTime newTimeOfScript;


  TactileIsNew    = FALSE;
  DeltaIsNew      = FALSE;
  SonarIsNew      = FALSE;
  FrontLaserIsNew = FALSE;
  RearLaserIsNew  = FALSE;
  MarkerIsNew     = FALSE;
  ButtonIsNew     = FALSE;
  InfraredIsNew   = FALSE;
  BumpIsNew       = FALSE;

  float elapsedTime = 0.0;


  // skip to next sensing line

  unsigned int sCnt=0;
  unsigned int mCnt=0;

  long markerNumber=0;

  if (fTimeFactor > 0.0) {
    elapsedTime = timeDiff(&currentTime, &ScriptSettings.lastAccessTime)
                  / fTimeFactor;
    elapsedTime = fMax( elapsedTime - nonRelevantTime, 0.0);
  } else {
    // if fTimeFactor <= 0 we take every reading
    elapsedTime = 0.0;
  }
  // Skip to the next reading line after the elapsed time.
  // During this store the latest readings.

  while (!eof && !timeSkipped) { // && !newPositionLine){
    eof = !readLine(line,DATA_BUFFLEN);
    if (!eof){
      if(markerReadingLine(line)) {
        markerLineNumber = ScriptSettings.actualLine;
        strcpy(markerLine,line);
        newMarkerLine=TRUE;
        cnt++;
        mCnt++;
        if(newTimeLine)
          timeSkipped=TRUE;
      }
      else if (positionReadingLine(line)){
        positionLineNumber = ScriptSettings.actualLine;
	strcpy(positionLine, line);
	newPositionLine = TRUE;
	cnt++;
        mCnt++;
        if (sscanf(&positionLine[strlen(ORIG_SCRIPTPOSMARK)],"%f %f %f",
                   &newPos.x, &newPos.y, &newPos.rot) == 3) {
          if(doOdometryCorrection()) {
            newPos.x *= getOdometryCorrection();
            newPos.y *= getOdometryCorrection();
          }
          add2LengthOfTrajectorie(newPos);
        }
      }
      else if (sonarReadingLine(line)){
        sonarLineNumber = ScriptSettings.actualLine;
	strcpy(sonarLine, line);
	newSonarLine = TRUE;
	cnt++;
        sCnt++;
      }
      else if (laserReadingLine(line)){
        laserLineNumber = ScriptSettings.actualLine;
	strcpy(laserLine, line);
	newLaserLine = TRUE;
	cnt++;
      }
      else if (bumperReadingLine(line)){
	if ( sscanf(&line[strlen(SCRIPTBUMPERMARK)],
		    "%f %f %f", &(fBumpForward), &(fBumpSideward),
                    &(fBumpRot)) < 3)
	  logMessage( "Wrong Bumper line: %s", line);
	else
	  BumpIsNew = TRUE;
      }
      else if (cnt > 0 && timeReadingLine(line)) {
	if (sscanf(&line[strlen(ORIG_TIMEMARK)+9], "%d:%d:%f",
		   &newTimeOfScript.hour,
		   &newTimeOfScript.minute,
		   &newTimeOfScript.second) == 3){
          timeLineNumber = ScriptSettings.actualLine;
	  timeSkipped = ( scriptTimeDiff(&newTimeOfScript,
                                         &ScriptSettings.currentTimeOfScriptOne)
                          >= elapsedTime) || ( -1.0 == fTimeFactor );
          newTimeLine=TRUE; // for marker
          if(newMarkerLine) // marker will always be returned
            timeSkipped=TRUE;
	}
      }
    }
  }

  if (cnt > 1) {
    cnt--;
    if(0)
      if (cnt == 1)
        logMessage("# Skipped %d reading.\n", cnt);
      else
        logMessage("# Skipped %d readings.\n", cnt);
  }

  if (eof) {
    logMessage("# End of file reached.\n");
    return END_REACHED;
  }

  ScriptSettings.currentTimeOfScriptOne.hour   = newTimeOfScript.hour;
  ScriptSettings.currentTimeOfScriptOne.minute = newTimeOfScript.minute;
  ScriptSettings.currentTimeOfScriptOne.second = newTimeOfScript.second;

  gettimeofday(&ScriptSettings.lastAccessTime, 0);

  if (newMarkerLine){
    if (sscanf(&markerLine[strlen(ORIG_MARKERSCRIPTMARK)],"%ld",
	       &markerNumber) == 1) {
      processMarkerReading(markerNumber);
      returnValue=NEW_DATA;
      ScriptSettings.markerCount++;
    }
    else {
      logMessage("# ERROR 1 while scanning script %s\n" , ScriptName);
      logMessage("# Cannot scan line %lu:\n%s\n", markerLineNumber, markerLine);
      chooseErrorAction();
      MarkerIsNew = FALSE;
      returnValue = ERROR;
    }
  }

  if ((END_REACHED != returnValue) &&
      (ERROR != returnValue) && (newPositionLine)){
    if (sscanf(&positionLine[strlen(ORIG_SCRIPTPOSMARK)],"%f %f %f",
	       &newPos.x, &newPos.y, &newPos.rot) == 3) {
      // processPositionReading(newPos, s, actualSensing);
      if(doOdometryCorrection()) {
        newPos.x *= getOdometryCorrection();
        newPos.y *= getOdometryCorrection();
      }
      processPositionReading(newPos);
      ScriptSettings.positionCount++;
      if(DeltaIsNew) {
        returnValue=NEW_DATA;
     }
    }
    else {
      logMessage("# ERROR 2 while scanning script %s\n" , ScriptName);
      logMessage("# Cannot scan line %lu:\n%s\n", positionLineNumber,
                 positionLine);
      chooseErrorAction();
      DeltaIsNew = FALSE;
      returnValue = ERROR;
    }
  }

  if ((END_REACHED != returnValue) && (ERROR != returnValue) && newSonarLine){
    unsigned int cnt2;
    if (sscanf(sonarLine,
               "#SONAR 24: %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e",
               &sensor[23],&sensor[22],&sensor[21],&sensor[20],
               &sensor[19],&sensor[18],&sensor[17],&sensor[16],
               &sensor[15],&sensor[14],&sensor[13],&sensor[12],
               &sensor[11],&sensor[10],&sensor[ 9],&sensor[ 8],
               &sensor[ 7],&sensor[ 6],&sensor[ 5],&sensor[ 4],
               &sensor[ 3],&sensor[ 2],&sensor[ 1],&sensor[ 0])
        == NUMBER_OF_SONARS) {

      // Substract the size of the robot.
      for ( cnt2 = 0; cnt2 < NUMBER_OF_SONARS; cnt2++)
        sensor[cnt2] = fMax( 0.0, sensor[cnt2] - getDistanceOffset());

      // updateSonar(s, &(actualSensing->sonar), sensor, NUMBER_OF_SONARS);
      updateSonar(sensor, NUMBER_OF_SONARS);

      ScriptSettings.sonarCount++;
      if( isWanted(SONAR_DATA) ) {
        SonarIsNew = TRUE;
        returnValue=NEW_DATA;
      }
    }
    else {
      logMessage("# ERROR 3 while scanning script %s\n" , ScriptName);
      logMessage("# Cannot scan line %lu:\n%s\n", sonarLineNumber,
                 sonarLine);
      chooseErrorAction();
      SonarIsNew = FALSE;
      returnValue = ERROR;
    }
  }

  if ((END_REACHED != returnValue) && (ERROR != returnValue) && newLaserLine){
    // dSkippedReadingsLaser++;

    int frontSize, rearSize;
    int markLength = strlen(ORIG_SCRIPTLASERMARK);
    int numberOfInts = sscanf( &laserLine[markLength], "%d %d:",
                               &frontSize, &rearSize);

    if ( numberOfInts == 1) {
      frontSize = frontSize / 2;
      rearSize = frontSize;
    }

    FrontLaser->actualNumberOfReadings = frontSize;
    RearLaser->actualNumberOfReadings = rearSize;

    // Set the mark behind the second ":".
    if(NULL == strstr(&laserLine[markLength],":")) {
      logMessage("# ERROR 4 while scanning script %s\n" , ScriptName);
      logMessage("# Cannot scan line %lu:\n%s\n", laserLineNumber,
                 laserLine);
      chooseErrorAction();
    } else {
      markLength += strlen(&laserLine[markLength]) -
                    strlen(strstr(&laserLine[markLength],":")) +
                    1;
    }
    if((unsigned int)markLength >= strlen(laserLine)) {
      logMessage("# ERROR 5 while scanning script %s\n" , ScriptName);
      logMessage("# Cannot scan line %lu:\n%s\n", laserLineNumber,
                 laserLine);
      chooseErrorAction();
    }

    if ( getReadings( &laserLine[markLength],
                      FrontLaser->actualNumberOfReadings +
                      RearLaser->actualNumberOfReadings,
                      sensor)) {

      updateLaser( sensor,
                   FrontLaser->actualNumberOfReadings +
                   RearLaser->actualNumberOfReadings);

      ScriptSettings.laserCount++;
      if( FrontLaserIsNew  || RearLaserIsNew ) {
        returnValue=NEW_DATA;
      }
    }
    else {
      logMessage("# ERROR 6 while scanning script %s\n" , ScriptName);
      logMessage("# Cannot scan line %lu:\n%s\n", laserLineNumber,
                 laserLine);
      chooseErrorAction();
      FrontLaserIsNew = FALSE;
      RearLaserIsNew = FALSE;
      returnValue = ERROR;
    }
  }


  return returnValue;
} // readOrigScript

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::resetTimer
// Purpose   : reset timer to allow different user interaction before reading
//             script data
//
// Parameters: ---
//
// Return    : void                     :
//
// Remarks   : use only once and only before _firsS_ reading!
//
void
t_ScriptData::resetTimer()
{
  if(!dStatus) {
    return;
  }
  static bool bMessage=TRUE;
  if(bMessage) {
    cerr << "# Script: timer reset.\n";
    bMessage=FALSE;
  }
  gettimeofday( &(ScriptSettings.lastAccessTime), 0);
} // resetTimer

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::readSecondScript
// Purpose   :
//
// Parameters: const float nonRelevantTime :
//
// Return    : t_Result                 :
//
// Remarks   :
//
t_Result
t_ScriptData::readSecondScript( const float nonRelevantTime)
{
  char line[DATA_BUFFLEN];
  struct timeval patTime;
  t_RealPosition patPos;

  bool bPatPos  = FALSE;
  bool bPatTime = FALSE;

  bool bPatButton;
  bool bPatInfrared;
  bool bPatLaser;
  bool bPatMarker;
  bool bPatSonar;
  bool bPatTactile;

  int frontReading;
  int rearReading;
  int sonarReading;

  bool scriptEnd       = FALSE;
  bool patternEnd      = FALSE;
  bool eof             = FALSE;
  bool timeSkipped     = FALSE;

  int cnt=0;

  float sensor[MAX_VALUESPERSCAN];

  unsigned long currentPatternLineNumber   = ScriptSettings.actualLine;

  unsigned long buttonLineNumber   = ScriptSettings.actualLine;
  unsigned long infraredLineNumber = ScriptSettings.actualLine;
  unsigned long laserLineNumber    = ScriptSettings.actualLine;
  unsigned long markerLineNumber   = ScriptSettings.actualLine;
  unsigned long sonarLineNumber    = ScriptSettings.actualLine;
  unsigned long tactileLineNumber  = ScriptSettings.actualLine;

  t_Result  returnValue = NO_NEW_DATA;

  struct timeval currentTime;
  gettimeofday(&currentTime, 0);
  struct timeval newTimeOfScript;


  TactileIsNew    = FALSE;
  DeltaIsNew      = FALSE;
  SonarIsNew      = FALSE;
  FrontLaserIsNew = FALSE;
  RearLaserIsNew  = FALSE;
  MarkerIsNew     = FALSE;
  ButtonIsNew     = FALSE;
  InfraredIsNew   = FALSE;
  BumpIsNew       = FALSE;

  t_RealPosition latestPos;
  struct timeval latestPosTime;
  bool bLatestPos=FALSE;
  latestPosTime.tv_sec=0;
  latestPosTime.tv_usec=0;

  static bool infoButton=TRUE;
  static bool infoInfrared=TRUE;
  static bool infoTactile=TRUE;

  t_secondScriptPatterns patternType = UNKNOWN_PATTERN;

  float elapsedTime = 0.0;

  // skip to next sensing line

  if (fTimeFactor > 0.0) {
    elapsedTime = timeDiff(&currentTime, &ScriptSettings.lastAccessTime)
                  / fTimeFactor;
    elapsedTime = fMax( elapsedTime - nonRelevantTime, 0.0);
  } else {
    // if fTimeFactor <= 0 we take every reading
    elapsedTime = 0.0;
  }

  // Skip to the next reading line after the elapsed time.
  // During this store the latest readings.

  while (!eof && !timeSkipped && !scriptEnd) {// && !newPositionLine){
    patternEnd=FALSE;
    do {
      eof = !readLine(line,DATA_BUFFLEN);
      scriptEnd=patternEndReadingLine(line);
    } while(!eof && !scriptEnd && !patternStartReadingLine(line));


    if(!eof && !scriptEnd) { // found start of pattern

      // search name (== type) of pattern
        do {
          eof = !readLine(line,DATA_BUFFLEN);
          scriptEnd=patternEndReadingLine(line);
          patternEnd=patternSetEndReadingLine(line);
        } while (!eof && !scriptEnd &&
                 !patternEnd && !patternSetNameReadingLine(line));

        if(!eof && !scriptEnd && !patternEnd) { // found name of pattern
          patternType=getPatternType(line);
          currentPatternLineNumber = ScriptSettings.actualLine;
          if(UNKNOWN_PATTERN == patternType) {
            logMessage("WARNING: unknown pattern in scriptline %d: %s\n",
                       ScriptSettings.actualLine,line);
            patternEnd=TRUE;
          } else { // read pattern
            bPatPos     = FALSE;
            bPatTime    = FALSE;

            bPatButton   = FALSE;
            bPatInfrared = FALSE;
            bPatLaser    = FALSE;
            bPatMarker   = FALSE;
            bPatSonar    = FALSE;
            bPatTactile  = FALSE;

            while(!eof && !scriptEnd && !patternEnd) {
              eof = !readLine(line,DATA_BUFFLEN);
              if(!eof) {
                if((patternSetTimeReadingLine(line,patTime))) {
                  bPatTime=TRUE;
                } else if((patternSetPositionReadingLine(line,patPos))) {
                  bPatPos=TRUE;
                } else if (bumperReadingLine(line)){
                  if ( sscanf(&line[strlen(SCRIPTBUMPERMARK)],
                              "%f %f %f", &(fBumpForward), &(fBumpSideward),
                              &(fBumpRot)) < 3)
                    logMessage( "Wrong Bumper line: %s", line);
                  else
                    BumpIsNew = TRUE;
                } else {
                  switch(patternType) {
                    case BUTTON_PATTERN:
                      if((patternSetButtonReadingLine(line,Buttons))) {
                        bPatButton=TRUE;
                      }
                      break;
                    case INFRARED_PATTERN:
                      if((patternSetInfraredReadingLine(line,Infrareds))) {
                        bPatInfrared=TRUE;
                      }
                      break;
                    case LASER_PATTERN:
                      if((patternSetLaserReadingLine(line,sensor, frontReading,
                                                     rearReading))) {
                        bPatLaser=TRUE;
                      }
                      break;
                    case MARKER_PATTERN:
                      if((patternSetMarkerReadingLine(line,MarkerNumber))) {
                        bPatMarker=TRUE;
                      }
                      break;
                    case SONAR_PATTERN:
                      if((patternSetSonarReadingLine(line,sensor,
                                                     sonarReading))) {
                        bPatSonar=TRUE;
                      }
                      break;
                    case TACTILE_PATTERN:
                      if((patternSetTactileReadingLine(line,Tactiles))) {
                        bPatTactile=TRUE;
                      }
                      break;
                    default:
                      logMessage("# WARNING: unsupported pattern in "
                                 "scriptline %ld: %s\n",
                                 ScriptSettings.actualLine,line);
                      patternEnd=TRUE;
                  } // switch
                  if(!(patternEnd=patternSetEndReadingLine(line)))
                    scriptEnd=patternEndReadingLine(line);
                }
              }
            } // while

            // keep position if newer
            if(bPatPos) {
              if((!bLatestPos) || (0 < timeDiff(&patTime,&latestPosTime))) {
                latestPos=patPos;
                bLatestPos=TRUE;
                latestPosTime=patTime;
                add2LengthOfTrajectorie(patPos);
                ScriptSettings.positionCount++;
              }
            }
            switch(patternType) {
              case BUTTON_PATTERN:
                if(bPatPos && bPatTime && bPatButton) {
                  buttonLineNumber = currentPatternLineNumber;
                  ButtonPos        = patPos;
                  ButtonTime       = patTime;
                  if( isWanted(BUTTON_DATA) ) {
                    ButtonIsNew    = TRUE;
                    returnValue    = NEW_DATA;
                  }
                } else {
                  // somthing is missing!
                  if(infoButton) {
                    infoButton=FALSE;
                    logMessage("########## Warning: button not implemented\n");
                    //logMessage("# WARNING: button needs time, buttons and position"
                    //         ". Bad pattern in line %ld\n",
                    //         currentPatternLineNumber);
                  }
                }
                break;
              case INFRARED_PATTERN:
                if(bPatPos && bPatTime && bPatInfrared) {
                  infraredLineNumber = currentPatternLineNumber;
                  InfraredPos        = patPos;
                  InfraredTime       = patTime;
                  if( isWanted(LASER_DATA) ) {
                    InfraredIsNew    = TRUE;
                    returnValue      = NEW_DATA;
                  }
                } else {
                  // somthing is missing!
                  if(infoInfrared) {
                    infoInfrared=FALSE;
                    logMessage("########## Warning: infrared not implemented\n");
                    //logMessage("# WARNING: Infrareds need time, infrareds and "
                    //           "position. Bad pattern in line %ld\n",
                    //           currentPatternLineNumber);
                  }
                }
                break;
              case LASER_PATTERN:
                if(bPatPos && bPatTime && bPatLaser) {
                  laserLineNumber = currentPatternLineNumber;
                  FrontLaserPos   = patPos;
                  RearLaserPos    = patPos;
                  FrontLaserTime  = patTime;
                  RearLaserTime   = patTime;
                  FrontLaser->actualNumberOfReadings = frontReading;
                  RearLaser->actualNumberOfReadings  = rearReading;
                  updateLaser( sensor,
                               FrontLaser->actualNumberOfReadings +
                               RearLaser->actualNumberOfReadings);
                  ScriptSettings.laserCount++;
                  if( isWanted(LASER_DATA) ) {
                    FrontLaserIsNew = TRUE;
                    RearLaserIsNew  = TRUE;
                    returnValue     = NEW_DATA;
                  }
                } else {
                  // somthing is missing!
                  // no message is laser pattern is missing
                  if(!bPatPos  || !bPatTime)
                    logMessage("# WARNING: Laser need time, lasers and "
                               "position. Bad pattern in line %ld\n",
                               currentPatternLineNumber);
                }
                break;
              case MARKER_PATTERN:
                if(bPatPos && bPatTime && bPatMarker) {
                  markerLineNumber = currentPatternLineNumber;
                  MarkerPos        = patPos;
                  MarkerTime       = patTime;
                  ScriptSettings.markerCount++;
                  if( isWanted(MARKER_DATA) ) {
                    MarkerIsNew    = TRUE;
                    returnValue    = NEW_DATA;
                  }
                } else {
                  // somthing is missing!
                  logMessage("# WARNING: Marker needs time, markernum and "
                             "position. Bad pattern in line %ld\n",
                             currentPatternLineNumber);
                }
                break;
              case SONAR_PATTERN:
                if(bPatPos && bPatTime && bPatSonar) {
                  sonarLineNumber = currentPatternLineNumber;
                  SonarPos        = patPos;
                  SonarTime       = patTime;
                  returnValue     = NEW_DATA;
                  Sonar->actualNumberOfReadings = sonarReading;
                  updateSonar(sensor, sonarReading);
                  ScriptSettings.sonarCount++;
                  if( isWanted(SONAR_DATA) ) {
                    SonarIsNew    = TRUE;
                    returnValue   = NEW_DATA;
                  }
                } else {
                  // somthing is missing!
                  logMessage("# WARNING: Sonar need time, sonars and "
                             "position. Bad pattern in line %ld\n",
                             currentPatternLineNumber);
                }
                break;
              case TACTILE_PATTERN:
                if(bPatPos && bPatTime && bPatTactile) {
                  tactileLineNumber = currentPatternLineNumber;
                  TactilePos        = patPos;
                  TactileTime       = patTime;
                  if( isWanted(TACTILE_DATA) ) {
                    TactileIsNew    = TRUE;
                    returnValue     = NEW_DATA;
                  }
                } else {
                  // somthing is missing!
                  if(infoTactile) {
                    infoTactile=FALSE;
                    logMessage("########## Warning: tactile not implemented\n");
                    //logMessage("# WARNING: Tactiles need time, tactiles and "
                    //           "position. Bad pattern in line %ld\n",
                    //           currentPatternLineNumber);
                  }
                }
                break;
              default:
                ; // nothing
            } // switch
          }

        }
        cnt++;

        if((bPatTime) &&
           ( ( TactileIsNew) ||( DeltaIsNew) || ( SonarIsNew) ||
             ( FrontLaserIsNew) || ( RearLaserIsNew) || ( MarkerIsNew) ||
             ( ButtonIsNew) || ( InfraredIsNew) ) ) {
          newTimeOfScript=patTime;
          timeSkipped = ( timeDiff(&newTimeOfScript,
                                   &ScriptSettings.currentTimeOfScriptTwo)
                          >= elapsedTime) || ( -1.0 == fTimeFactor );
        }

    }
  }
  if (cnt > 1) {
    cnt--;
    if (cnt == 1)
      logMessage("# Skipped %d reading.\n", cnt);
    else
      logMessage("# Skipped %d readings.\n", cnt);
  }

  if (eof) {
    logMessage("# End of file reached.\n");
    return END_REACHED;
  }

  ScriptSettings.currentTimeOfScriptTwo = newTimeOfScript;

  gettimeofday(&ScriptSettings.lastAccessTime, 0);
  if(NEW_DATA == returnValue)
    processPositionReading(latestPos);

  return returnValue;
} // readSecondScript

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::scriptPosition2RobotPosition
// Purpose   : Converts a position from the sonarint script (DEG) into the
//             robot's coordinates in RADIAN
//
// Parameters: t_ScriptCoord scriptPos  :
//
// Return    : ---
//
// Remarks   : real position rot is stored in DEG and will be changed to RAD!
//
void
t_ScriptData::scriptPosition2RobotPosition( t_RealPosition &scriptPos)
{
  scriptPos.rot  = normedAngleRAD( deg2rad( 90.0 - scriptPos.rot));
} // scriptPosition2RobotPosition

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::scriptTimeDiff
// Purpose   :
//
// Parameters: const t_ScriptTime *newTime :
//             const t_ScriptTime *oldTime :
//
// Return    : float                    :
//
// Remarks   :
//
float
t_ScriptData::scriptTimeDiff( const t_ScriptTime *newTime,
                              const t_ScriptTime *oldTime)
{

    float newSeconds, oldSeconds;

    newSeconds = newTime->hour * 3600 + newTime->minute * 60 + newTime->second;

    oldSeconds = oldTime->hour * 3600 + oldTime->minute * 60 + oldTime->second;

    if (newSeconds >= oldSeconds)
	return (newSeconds - oldSeconds);
    else
	return (86400 + newSeconds - oldSeconds);

} // scriptTimeDiff

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::sonarReadingLine
// Purpose   :
//
// Parameters: const char* line         :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_ScriptData::sonarReadingLine(const char* line)
{
  return (strncmp(line,ORIG_SCRIPTSONARMARK,
		  strlen(ORIG_SCRIPTSONARMARK)) == 0);
} // sonarReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::timeReadingLine
// Purpose   :
//
// Parameters: const char *line         :
//
// Return    : static bool              :
//
// Remarks   :
//

bool
t_ScriptData::timeReadingLine(const char *line)
{
  return (strncmp(line, ORIG_TIMEMARK, strlen(ORIG_TIMEMARK)) == 0);
} // timeReadingLine

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::updateFloat
// Purpose   :
//
// Parameters: const float *sensor      :
//             t_Reading *r1            :
//             const unsigned int n1    :
//             t_Reading *r2            :
//             const unsigned int n2    :
//
// Return    : void                     :
//
// Remarks   :
//
void
t_ScriptData::updateFloat( const float *sensor,
                           t_Reading *r1, const unsigned int n1,
                           t_Reading *r2, const unsigned int n2)
{
  updateFloat(sensor,r1,n1);
  updateFloat(&sensor[n1],r2,n2);
}

void
t_ScriptData::updateFloat( const float *sensor,
                           t_Reading *r1, const unsigned int n1)
{
  if(n1 > r1->maxNumberOfReadings) {
    logMessage("# t_ScriptData::updateFloat ERROR script readings: %u\n",
               n1);
    logMessage("#                           allocatedReadings: "
               "%u (r1)\n",
               r1->maxNumberOfReadings);
    chooseErrorAction();
    // continue, if error action means continue
  }
  unsigned int i;

  for (i=0; i < r1->actualNumberOfReadings ; i++){
     r1->reading[i].dist = sensor[i];
  }
} // updateFloat
#ifdef 0
//-----------------------------------------------------------------------------
// Function  : t_ScriptData::updateInfrared
// Purpose   :
//
// Parameters: const int *sensor        :
//             t_Infrareds  *i          :
//             const unsigned int n1    :
//             const unsigned int n2    :
//             const unsigned int n3    :
//
// Return    : void                     :
//
// Remarks   :
//
void
t_ScriptData::updateInt( const int *sensor,
                         t_DistanceReading **i, const unsigned int n1,
                         const unsigned int n2,const unsigned int n3)
{
  if(n1+n2+n3 > i->maxNumberOfReadings) {
    logMessage("# t_ScriptData::updateInfrared ERROR script readings: %u\n",
               n);
    logMessage("#                           allocatedReadings: "
               "%u (i)\n",
               i->maxNumberOfReadings);
    chooseErrorAction();
    // continue, if error action means continue
  }
  unsigned int i;

  for (i=0; i < i->actualNumberOfReadings ; i++){
     i->reading[i]->dist = sensor[i];
  }
} // updateInfrared
#endif
//-----------------------------------------------------------------------------
// Function  : t_ScriptData::updateLaser
// Purpose   :
//
// Parameters: const float *sensor      :
//             const unsigned int n     :
//
// Return    : void                     :
//
// Remarks   :
//
void
t_ScriptData::updateLaser( const float *sensor, const unsigned int n)
{
  if(n > FrontLaser->maxNumberOfReadings + RearLaser->maxNumberOfReadings) {
    logMessage("# t_ScriptData::updateLaser ERROR script laser readings: %u\n",
               n);
    logMessage("#                           allocatedReadings: "
               "%u (front laser)\n",
               FrontLaser->maxNumberOfReadings);
    logMessage("#                           allocatedReadings: "
               "%u (rear laser)\n",
               RearLaser->maxNumberOfReadings);
    chooseErrorAction();
    return;
  }
  unsigned int i;

  FrontLaserIsNew = FrontLaser->actualNumberOfReadings > 0;
  RearLaserIsNew = RearLaser->actualNumberOfReadings > 0;

  for (i=0; i < FrontLaser->actualNumberOfReadings &&  FrontLaserIsNew; i++){
     FrontLaser->reading[i].dist = sensor[i];
     FrontLaserIsNew = (FrontLaser->reading[i].dist >= 0);
  }

  for (i=0; i < RearLaser->actualNumberOfReadings &&  RearLaserIsNew; i++) {
    RearLaser->reading[i].dist = sensor[i+FrontLaser->actualNumberOfReadings];
    RearLaserIsNew = (RearLaser->reading[i].dist >= 0);
  }

  if( !isWanted(LASER_DATA) ) {
    FrontLaserIsNew = FALSE;
    RearLaserIsNew = FALSE;
  }
  //if (FrontLaserIsNew)
  //  FrontLaser->elapsedTime =
  //    scriptTimeDiff(&TimeOfScript, &StartTimeOfScript);

  //if (RearLaserIsNew)
  // RearLaser->elapsedTime =
  //  scriptTimeDiff(&TimeOfScript, &StartTimeOfScript);

  //if (FrontLaserIsNew || RearLaserIsNew)
    //dLaserCount++;
} // updateLaser

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::updateMovement
// Purpose   :
//
// Parameters: const t_RealPosition &newPos :
//
// Return    : void                     :
//
// Remarks   :
//
void
t_ScriptData::updateMovement( t_RealPosition &newPos)
{
  // to calculate delti, you have to rotate old and new position back to
  // angle which it was before call of scriptPosition2RobotPosition()
  t_RealPosition tmpNew  = newPos;
  t_RealPosition tmpBase = BasePosition;

  tmpBase.rot = normedAngleRAD(- tmpBase.rot + M_PI/2.0);
  tmpNew.rot  = normedAngleRAD(- tmpNew.rot  + M_PI/2.0);

  // compute forward and sideward movement
  Delta.x =
    + (tmpNew.y - tmpBase.y) * sin(tmpBase.rot)
    + (tmpNew.x - tmpBase.x) * cos(tmpBase.rot);

  Delta.y =
    - (tmpNew.y - tmpBase.y) * cos(tmpBase.rot)
    + (tmpNew.x - tmpBase.x) * sin(tmpBase.rot);

  Delta.rot = tmpNew.rot - tmpBase.rot;

  if( (0 != Delta.x) || (0 != Delta.y ) || (0 != Delta.rot) )  {
    ScriptSettings.noMoveCount=0;
    if( isWanted(MOVEMENT_DATA | BASEPOSITION_DATA) )
      DeltaIsNew = TRUE;
    // fprintf(stdout, "delta %f %f %f -- %f %f %f --> %f %f %f\n",
    //         tmpBase.x, tmpBase.y, Rad2Deg(tmpBase.z),
    //         tmpNew.x, tmpNew.y, Rad2Deg(tmpNew.z),
    //         Delta.forward, Delta.sideward, Rad2Deg(Delta.rotation));
  } else {
    ScriptSettings.noMoveCount++;
    DeltaIsNew = FALSE;
  }
  // dPositionCount++;
} // updateMovement

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::updateSonar
// Purpose   :
//
// Parameters: const float *sensor      :
//             const unsigned int n     :
//
// Return    : void                     :
//
// Remarks   :
//
void t_ScriptData::updateSonar(const float *sensor, const unsigned int n)
{
  if(n > Sonar->maxNumberOfReadings) {
    logMessage("# t_ScriptData::updateSonar ERROR script sonar readings: %u\n"
               "#                           allocatedReadings: %u\n",
               n,Sonar->maxNumberOfReadings);
    chooseErrorAction();
    // continue, if error action means continue
  }

  //dSonarCount++;


  for (unsigned int i=0; (i < n) && (i < Sonar->maxNumberOfReadings) ; i++)
    Sonar->reading[i].dist = sensor[i] + ROB_RADIUS;

  if(n < Sonar->maxNumberOfReadings)
    Sonar->actualNumberOfReadings=n;
  else
    Sonar->actualNumberOfReadings=Sonar->maxNumberOfReadings;

  // Sonar->elapsedTime = scriptTimeDiff(&TimeOfScript, &StartTimeOfScript);
  if( isWanted(SONAR_DATA) )
    SonarIsNew = TRUE;
} // updateSonar

//-----------------------------------------------------------------------------
// Function  : t_ScriptData::version
// Purpose   : Return version string of t_ScriptData
//
// Parameters: ---
//
// Return    : char *                   : version string of t_ScriptData
//
// Remarks   :
//
char *t_ScriptData::version()
{
#ifdef DEBUG_CODE
  return t_ScriptDataTimeStamp;
#else
  return GetRCSVersionAndDateC("$Id: scriptData.cc,v 1.1 2002/09/14 17:06:31 rstone Exp $","Feb 1998","# t_scriptData          ");
  // prevent warning
  t_ScriptDataTimeStamp=t_ScriptDataTimeStamp;
#endif
} // version

// End of t_ScriptData //
