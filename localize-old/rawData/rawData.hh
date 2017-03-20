// Time-stamp: <98/02/12 16:04:39 derr>
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
/////
///// File name:                   rawData.hh
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
///// $Source: /usr/local/cvs/bee/src/localize/rawData/rawData.hh,v $
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
// $Log: rawData.hh,v $
// Revision 1.1  2002/09/14 17:06:31  rstone
// *** empty log message ***
//
// Revision 1.1  1998/02/12 18:14:01  derr
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
#ifndef CLASS_RAWDATA_INCLUDE
#define CLASS_RAWDATA_INCLUDE
// load some general definitions
#include <iostream.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#include "channelData.h"
#include "typedefRawData.h"
#include "constDef.h"

class t_RawData
{
public:
  // with error action CONTINUE_ON_ERROR
  t_RawData( const float timeFactor);

  // <timeFactor> and error action <e>
  t_RawData( const float timeFactor,
             const t_ErrorAction e);

  // use error action CONTINUE_ON_ERROR,
  // logging output to stdout, stderr _and_ <logFile>
  t_RawData( const float timeFactor,
             FILE *logFile);

  // logging output to stdout, stderr _and_ <logFile>
  t_RawData( const float timeFactor,
             const t_ErrorAction e,
             FILE *logFile);

  // get version
  t_RawData( const unsigned int length,  char *versionStr);

  // Destructor
  virtual ~t_RawData();

  void chooseErrorAction();
  bool doOdometryCorrection(){return bOdometryCorrection;};
  float getOdometryCorrection(){return fOdometryCorrection;};
  bool isWanted(const unsigned int which);
  void logMessage(const char *printfparams ...);

  // timeFactor -1 (get each reading) or > 0
  bool setTimeFactor(const float timeFactor);

  void setDistanceOffset(const float newOffset);
  void setOdometryCorrection(float odoCorrection);
  void setWantedData(const unsigned int which);

  /////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////
  //
  //                       virtual functions
  //
  /////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////

  // new data arrived?
  virtual t_Result checkNext( const float nonRelevantTime=0.0) = 0;
;//  {logMessage("t_rawData::checkNext VIRTUAL\n"); return ERROR;}

  virtual unsigned long getActualLine() = 0;
  // get new data
  // TRUE:  parameter contains new data
  // FALSE: parameter unchanged
  virtual bool getBasePosition( t_RealPosition *basePos) = 0;
    ;//{logMessage("t_rawData::getBasePosition VIRTUAL\n"); return FALSE;}

  // get bump generated with d. fox 'addNoise'
  virtual bool getBump( t_RealPosition *bumpDelta) = 0;

  virtual void getCounts( unsigned long *sonarCnt,
                          unsigned long *laserCnt,
                          unsigned long *posCnt,
                          unsigned long *markerCnt,
                          unsigned long *noMoveCnt) = 0;

  // get distance traveled in cm
  virtual void getDistanceTraveled(float &dist) =0;

  // get elapsed time (stored in script)
  virtual float getElapsedTime() = 0;

  virtual bool getFrontLaser( t_Reading *reading ) = 0;
  ;//{logMessage("t_rawData::getFrontLaser VIRTUAL\n"); return FALSE;}

  virtual bool getMarker( long &number, t_RealPosition *&markerPos) = 0;
;//  {logMessage("t_rawData::getMarker VIRTUAL\n"); return FALSE;}

  virtual bool getMovement( t_RealPosition *&delta) = 0;
;//  {logMessage("t_rawData::getMovement VIRTUAL\n"); return FALSE;}

  virtual bool getRearLaser( t_Reading *reading) = 0;
;//  {logMessage("t_rawData::getRearLaser VIRTUAL\n"); return FALSE;}

  virtual bool getSonar( t_Reading *reading) = 0;
;//  {logMessage("t_rawData::getSonar VIRTUAL\n"); return FALSE;}

  virtual void getStartPosition( t_RealPosition *pos) = 0;

  virtual bool getStatus() = 0;
;//  {logMessage("t_rawData::getStatus VIRTUAL\n"); return FALSE;}

  virtual void resetTimer() = 0;
  /////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////
  //
  //                       STOP READING HERE
  //
  /////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////


protected:
  enum {DATA_BUFFLEN = 4096};

  typedef enum {
    VERSION_ONLY  = 0,
    BAD,
    INITIALIZED
  } t_Status;

  t_RealPosition  Delta;
  t_RealPosition  BasePosition;
  t_RealPosition  StartPosition;
  struct timeval  DeltaTime;
  bool            DeltaIsNew;

  float           fDistanceTraveled;

  t_Reading      *Sonar;
  t_RealPosition  SonarPos;
  struct timeval  SonarTime;
  bool            SonarIsNew;

  t_Reading      *FrontLaser;
  t_RealPosition  FrontLaserPos;
  struct timeval  FrontLaserTime;
  bool            FrontLaserIsNew;

  t_Reading      *RearLaser;
  t_RealPosition  RearLaserPos;
  struct timeval  RearLaserTime;
  bool            RearLaserIsNew;

  unsigned long   MarkerNumber;
  t_RealPosition  MarkerPos;
  struct timeval  MarkerTime;
  bool            MarkerIsNew;

  t_Tactile      *Tactiles;
  t_RealPosition  TactilePos;
  struct timeval  TactileTime;
  bool            TactileIsNew;

  t_Button       *Buttons;
  t_RealPosition  ButtonPos;
  struct timeval  ButtonTime;
  bool            ButtonIsNew;

  t_Infrared     *Infrareds;
  t_RealPosition  InfraredPos;
  struct timeval  InfraredTime;
  bool            InfraredIsNew;

  bool            BumpIsNew;
  float           fBumpForward;
  float           fBumpSideward;
  float           fBumpRot;

  float           fTimeFactor; // -1 (get each reading) or > 0
  float           fOdometryCorrection;
  bool            bOdometryCorrection;

  bool     allocateReading( t_Reading *&reading,
                            const unsigned int maxNumberOfReadings);
  float    deg2rad( const float x);
  float    fMax( float x, float y);
  t_Status getProtectedStatus();
  float    normedAngleDEG( const float angle);
  float    normedAngleRAD( const float angle);
  float    timeDiff( struct timeval *newTime,struct timeval *oldTime);


  inline float    getDistanceOffset() {return DistanceOffset;}

private:

  t_Status       dStatus;
  t_ErrorAction  ErrorAction;
  FILE          *LogFile;
  float          DistanceOffset;
  unsigned int   WantedData;

  void     destroy();                                  // called by destructor
  void     init(const t_ErrorAction e, FILE *fp=NULL); // called by constructor

  char    *version();
}; // end of class t_RawData
#endif

// End of CLASS_RAWDATA_HEADER

