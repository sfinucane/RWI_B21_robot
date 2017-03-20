// time-stamp value _has to_ reside in the first 8 lines
static char *t_RawDataTimeStamp=" CLASS   rawData     \
Time-stamp: <1998-02-12 09:43:01 derr>";
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
/////
///// File name:                   t_RawData.cc
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
///// $Source: /usr/local/cvs/bee/src/localize/rawData/rawData.cc,v $
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
// $Log: rawData.cc,v $
// Revision 1.1  2002/09/14 17:06:31  rstone
// *** empty log message ***
//
// Revision 1.1  1998/02/12 18:14:00  derr
// Added library for reading laserint- sonarint- and 'new'-scripts.
//
// Revision 1.1  1998/02/12 15:59:49  derr
// New library librawData for reading laserint-, sonarint and 'new'-script.
// Will replace bee/src/localize/script.c.
//
//
//
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

#include <iostream.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <sys/fcntl.h>
#include <sys/types.h>
//#include <varargs.h>

#include "rawData.hh"
#include "channelData.h"
#include "constDef.icc"

//-----------------------------------------------------------------------------
// Function  : t_RawData::t_RawData
// Purpose   : (Constructor) with error action CONTINUE_ON_ERROR
//
// Parameters: const float timeFactor   :
//
// Return    : ---                      :
//
// Remarks   :
//
t_RawData::t_RawData( const float timeFactor)
{
  init(CONTINUE_ON_ERROR);

  if(BAD==dStatus)
    return;

  if(!setTimeFactor(timeFactor))
    dStatus=BAD;
}

//-----------------------------------------------------------------------------
// Function  : t_RawData::t_RawData
// Purpose   : (Constructor)
//
// Parameters: const float timeFactor   :
//             const t_ErrorAction e    :
//
// Return    : ---                      :
//
// Remarks   :
//
t_RawData::t_RawData( const float timeFactor,
                      const t_ErrorAction e)
{
  init(e);

  if(BAD==dStatus)
    return;

  if(!setTimeFactor(timeFactor))
    dStatus=BAD;
}

//-----------------------------------------------------------------------------
// Function  : t_RawData::t_RawData
// Purpose   : (Constructor) with error action CONTINUE_ON_ERROR,
//             log output to stdout, stderr _and_ <logFile>
//
// Parameters: const float timeFactor   :
//             FILE *logFile            :
//
// Return    : ---                      :
//
// Remarks   :
//
t_RawData::t_RawData( const float timeFactor,
                      FILE *logFile)
{
  init(CONTINUE_ON_ERROR,logFile);

  if(BAD==dStatus)
    return;

  if(!setTimeFactor(timeFactor))
    dStatus=BAD;
}

//-----------------------------------------------------------------------------
// Function  : t_RawData::t_RawData
// Purpose   : (Constructor) with error action CONTINUE_ON_ERROR,
//             log output to stdout, stderr _and_ <logFile>
//
// Parameters: const float timeFactor   :
//             const t_ErrorAction e    :
//             FILE *logFile            :
//
// Return    : ---                      :
//
// Remarks   :
//
t_RawData::t_RawData( const float timeFactor,
                      const t_ErrorAction e,
                      FILE *logFile)
{
  init(e,logFile);

  if(BAD==dStatus)
    return;

  if(!setTimeFactor(timeFactor))
    dStatus=BAD;
}

//-----------------------------------------------------------------------------
// Function  : t_RawData::t_RawData
// Purpose   : return version of rawData.
//             rawData will not be initialized
//
// Parameters: const unsigned int length :
//             char *&versionStr       :
//
// Return    : ---                      :
//
// Remarks   :
//
t_RawData::t_RawData( const unsigned int length, char *versionStr)
{
  dStatus=VERSION_ONLY;

  if(!versionStr)
    return;

  if(strlen(version()) > length) {
    strncpy(versionStr,version(),length);
    versionStr[length-1]=0;
  }
  else {
    strcpy(versionStr,version());
  }
}

//-----------------------------------------------------------------------------
// Function  : t_RawData::~t_RawData
// Purpose   : destructor
//
// Parameters: ---
//
// Return    : ---                      :
//
// Remarks   :
//
t_RawData::~t_RawData()
{
  if(VERSION_ONLY != dStatus)
    destroy();
}

//-----------------------------------------------------------------------------
// Function  : t_RawData::allocateReading
// Purpose   :
//
// Parameters: t_Reading *&reading      :
//             const unsigned int maxNumberOfReadings :
//
// Return    : bool                     :
//
// Remarks   :
//

bool
t_RawData::allocateReading( t_Reading *&reading,
                            const unsigned int maxNumberOfReadings)
{
  if(NULL == (reading = new t_Reading)) {
    cerr << "# allocateReading ERROR out of memory!\n";
    return FALSE;
  }
  reading->maxNumberOfReadings=0;
  reading->actualNumberOfReadings=0;
  reading->reading=NULL;

  if(0 == maxNumberOfReadings)
    return TRUE; // senseless, but ...

  if(NULL == (reading->reading = new (t_DistanceReading[maxNumberOfReadings]
                                      ))) {
    cerr << "# allocateReading ERROR out of memory!!\n"
         << "#                       impossible to allocate "
         << maxNumberOfReadings << "\n";
    return FALSE;
  }

  reading->maxNumberOfReadings=maxNumberOfReadings;

  return TRUE;
} // allocateReading

//-----------------------------------------------------------------------------
// Function  : t_RawData::chooseErrorAction
// Purpose   : check <ErrorAction> and continue, stop or generate core-dump
//
// Parameters: ---
//
// Return    : void                     :
//
// Remarks   :
//
void
t_RawData::chooseErrorAction()
{
  char *c=NULL; // for "core-dump"
  switch (ErrorAction) {
    case CONTINUE_ON_ERROR:
      // nothing to do
      break;
    case STOP_ON_ERROR:
      logMessage("rawData:chooseErrorAction stop on error...\n");
      exit(1);
      break;
    case CORE_ON_ERROR:
      logMessage("rawData:chooseErrorAction trying to generate core-dump\n"
                 "                          with segment violation...\n");
      c[0]=0;
      c[1]=0;
      logMessage("rawData:chooseErrorAction WARNING: impossible to generate\n"
                 "                                   core...\n");
      exit(1);
      break;
    default:
      logMessage("rawData:chooseErrorAction WARNING: unknown action "
                 "to choose!\n");
  } // switch
} // chooseErrorAction

//-----------------------------------------------------------------------------
// Function  : t_RawData::deg2rad
// Purpose   : calculate radian of x
//
// Parameters: const float x            : angle in degree
//
// Return    : float                    : angle in radian
//
// Remarks   :
//
float
t_RawData::deg2rad( const float x)
{
  return x * 0.017453293;
} // deg2rad

//-----------------------------------------------------------------------------
// Function  : t_RawData::destroy
// Purpose   :
//
// Parameters: ---
//
// Return    : void                     :
//
// Remarks   :
//
void
t_RawData::destroy()
{
  if(INITIALIZED != dStatus)
    return;

  if(NULL == Sonar) {
    delete Sonar;
    Sonar=NULL;
  }
  if(NULL == FrontLaser) {
    delete FrontLaser;
    FrontLaser=NULL;
  }
  if(NULL == RearLaser) {
    delete RearLaser;
    RearLaser=NULL;
  }

  dStatus=BAD;

  // nothing to do
} // destroy

//-----------------------------------------------------------------------------
// Function  : t_RawData::fMax
// Purpose   : return maximum of x and y
//
// Parameters: float x                  :
//             float y                  :
//
// Return    : float                    : maximum of x and y
//
// Remarks   :
//
float
t_RawData::fMax( float x, float y)
{
 return ( (x) > (y) ? (x) : (y));
} // fMax

//-----------------------------------------------------------------------------
// Function  : t_RawData::getProtectedStatus
// Purpose   :
//
// Parameters: ---
//
// Return    : t_Status                 :
//
// Remarks   :
//
t_RawData::t_Status
t_RawData::getProtectedStatus()
{
  return dStatus;
} // getProtectedStatus

//-----------------------------------------------------------------------------
// Function  : t_RawData::init
// Purpose   :
//
// Parameters: const t_ErrorAction e    :
//             FILE *fp                 :
//
// Return    : void                     :
//
// Remarks   :
//
void
t_RawData::init(const t_ErrorAction e, FILE *fp)
{
  LogFile=fp;
  ErrorAction=e;
  dStatus=BAD;
  fTimeFactor=1.0;
  fOdometryCorrection=1.0;
  bOdometryCorrection=FALSE;
  WantedData=ALL_DATA;

  DeltaIsNew      = FALSE;
  SonarIsNew      = FALSE;
  FrontLaserIsNew = FALSE;
  RearLaserIsNew  = FALSE;
  MarkerIsNew     = FALSE;

  fDistanceTraveled = 0.0;

  Sonar         = NULL;
  FrontLaser    = NULL;
  RearLaser     = NULL;

  if(!allocateReading(Sonar,NUMBER_OF_SONARS))
    return;

  if(!allocateReading(FrontLaser,NUMBER_OF_SCANS_PER_LASER))
    return;

  if(!allocateReading(RearLaser,NUMBER_OF_SCANS_PER_LASER))
    return;

  dStatus=INITIALIZED;
} // init

//-----------------------------------------------------------------------------
// Function  : t_RawData::isWanted
// Purpose   : Check if user wants to get data-type specified with <which>
//
// Parameters: const unsigned int which :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_RawData::isWanted(const unsigned int which)
{
  return WantedData & which;
} // isWanted

//-----------------------------------------------------------------------------
// Function  : t_RawData::logMessage
// Purpose   :
//
// Parameters: const char *printfParams :
//             ...                      :
//
// Return    : void                     :
//
// Remarks   :
//
void
t_RawData::logMessage(const char *printfParams ...)
{
  va_list ap;
  va_start (ap,printfParams);   // arg startup

  // Ausgabe
  if(NULL != LogFile)
    vfprintf(LogFile,printfParams,ap);
  vfprintf(stderr,printfParams,ap);

  va_end (ap);                 // arg cleanup

} // logMessage

//-----------------------------------------------------------------------------
// Function  : t_RawData::normedAngleDEG
// Purpose   : keep angle in [0..360[ DEG
//
// Parameters: const float angle        :
//
// Return    : float                    :
//
// Remarks   :
//

float t_RawData::normedAngleDEG( const float angle)
{
  float newAngle=angle;

  while (newAngle < 0.0)
    (newAngle) += DEG_360;

  while (newAngle >= DEG_360)
    (newAngle) -= DEG_360;

  return(newAngle);
} // normedAngleDEG

//-----------------------------------------------------------------------------
// Function  : t_RawData::normedAngleRAD
// Purpose   : keep angle in [0..360[ RAD
//
// Parameters: const float angle        :
//
// Return    : float                    :
//
// Remarks   :
//

float t_RawData::normedAngleRAD( const float angle)
{
  float newAngle=angle;
  while (0 > newAngle)
    newAngle = newAngle + 6.283185307; // 2pi

  while (6.283185307 <= newAngle)
    newAngle = newAngle - 6.283185307;

  return(newAngle);
} // normedAngleRAD

//-----------------------------------------------------------------------------
// Function  : t_RawData::setDistanceOffset
// Purpose   :
//
// Parameters: const float newOffset    :
//
// Return    : void                     :
//
// Remarks   :
//
void
t_RawData::setDistanceOffset(const float newOffset)
{
  DistanceOffset=newOffset;

  logMessage("# new distance offset: %.2f",newOffset);
} // setDistanceOffset

void t_RawData::setOdometryCorrection(float odoCorrection)
{
  if(1.0 != odoCorrection) {
    logMessage("# Odometry correction on.\n");
    fOdometryCorrection=odoCorrection;
    bOdometryCorrection=TRUE;
  } else {
    logMessage("# Odometry correction disabled.\n");
    fOdometryCorrection=odoCorrection;
    bOdometryCorrection=FALSE;
  }
} // setOdometryCorrection

//-----------------------------------------------------------------------------
// Function  : t_RawData::setTimeFactor
// Purpose   :
//
// Parameters: const float timeFactor   :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
t_RawData::setTimeFactor(const float timeFactor)
{
  if(BAD == dStatus) {
    return FALSE;
  }
  if( (0 == timeFactor) ||
      ((0 > timeFactor) && (-1 != timeFactor)) ) {
    logMessage("# setTimeFactor: WARNING new timefactor %f bad!\n"
               "#                        timefactor has to be -1 or > 0\n",
               timeFactor);
    return FALSE;
  }
  logMessage("# New time factor: %f\n", timeFactor);
  fTimeFactor=timeFactor;
  return TRUE;
} // setTimeFactor

//-----------------------------------------------------------------------------
// Function  : t_RawData::setWantedData
// Purpose   :
//
// Parameters: unsigned int which       :
//
// Return    : void                     :
//
// Remarks   :
//

void
t_RawData::setWantedData(unsigned int which)
{
  if(0 == which) {
    logMessage("rawData.setWantedData  WARNING: ignoring 0!\n");
    return;
  }
  WantedData=which;
  logMessage("%s%s%s%s%s%s\n",
             "# used data: ",
             (which&SONAR_DATA?"SONAR ":""),
             (which&LASER_DATA?"LASER ":""),
             (which&MOVEMENT_DATA?"MOVEMENT ":""),
             (which&BASEPOSITION_DATA?"BASEPOSITION ":""),
             (which&MARKER_DATA?"MARKER ":""),
             (which&TACTILE_DATA?"TACTILE ":""),
             (which&INFRARED_DATA?"INFRARED ":""),
             (which&BUTTON_DATA?"BUTTON ":"")
             );
} // setWantedData

//-----------------------------------------------------------------------------
// Function  : t_RawData::timeDiff
// Purpose   :
//
// Parameters: ---
//
// Return    : float                    :
//
// Remarks   :
//
float
t_RawData::timeDiff(struct timeval *newTime,struct timeval *oldTime)
     //struct timeval* t1, struct timeval* t2)
{
  float diff;

  diff =  (float) (newTime->tv_usec - oldTime->tv_usec) / 1000000.0;
  diff += (float) (newTime->tv_sec - oldTime->tv_sec);

  return diff;
} // timeDiff

//-----------------------------------------------------------------------------
// Function  : t_RawData::version
// Purpose   : Return version string of t_RawData
//
// Parameters: ---
//
// Return    : char *                   : version string of t_RawData
//
// Remarks   :
//
char *t_RawData::version()
{
#ifdef DEBUG_CODE
  return t_RawDataTimeStamp;
#else
  return GetRCSVersionAndDateC("$Id: rawData.cc,v 1.1 2002/09/14 17:06:31 rstone Exp $","Feb 1998","# t_rawData             ");
  // prevent warning
  t_RawDataTimeStamp=t_RawDataTimeStamp;
#endif
} // version

// End of t_RawData //
