// time-stamp value _has to_ reside in the first 8 lines
static char *t_ChannelDataTimeStamp=" WRAPPER channelData \
Time-stamp: <98/02/12 16:11:41 derr>";
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
/////
///// File name:                   channelData.cc
/////
///// Part of:                     RHINO SOFTWARE
/////
///// Creator:                     AnD, University of Bonn
/////
///// Date of creation:            Oct 1997
/////
///// Purpose:                     WRAPPER for rawData
/////
/////
///// $Source: /usr/local/cvs/bee/src/localize/rawData/channelData.cc,v $
/////
///// $Revision: 1.1 $
/////
///// $Date: 2002/09/14 17:06:31 $
/////
///// $Author: rstone $
/////
/////
///// Remarks: This wrapper uses a pointer to some struct (in this
/////          case <t_Channel> to point to class <t_RawData>. This is
/////          some kind of hack and may result in future errors.
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
// $Log: channelData.cc,v $
// Revision 1.1  2002/09/14 17:06:31  rstone
// *** empty log message ***
//
// Revision 1.1  1998/02/12 18:13:58  derr
// Added library for reading laserint- sonarint- and 'new'-scripts.
//
// Revision 1.1  1998/02/12 15:59:46  derr
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
#include <sys/types.h>

#include "channelData.h"
#include "rawData.hh"
#include "scriptData.hh"


/******************* scpecial wrapper feature for localize only */
static bool localizeWrapperIsInitialized=FALSE;
t_Channel *localizeWrapperChannel;
float elapsedScriptTime;
float nonRelevantTime;
realPosition measuredPosition = {0.0, 0.0, 0.0};
extern FILE *logFile;

/**************************************************************************
 * Computes the end point given a start point and a sensing_MOVEMENT.
 * (copied (movement.c))
 **************************************************************************/
realPosition calculateEndPoint( realPosition start, movement move)
{
  realPosition end;
  float cosRot;
  float sinRot;
  float angle;

  cosRot =  cos(start.rot);
  sinRot =  sin(start.rot);
  if ( move.forward == 0.0 && move.sideward == 0.0 && move.rotation == 0.0)
    return start;

  /* This is the correct formula. */
  /*   end.x = start.x + */
  /*     cos( start.rot) * move.forward + cos( start.rot - DEG_90) * move.sideward; */

  /*   end.y = start.y + */
  /*     sin( start.rot) * move.forward + sin( start.rot - DEG_90) * move.sideward; */


  /* Replaced cos( r - 90) by sin( r) and sin( r - 90) by -cos( r) */
  end.x = start.x + cosRot * move.forward + sinRot * move.sideward;
  end.y = start.y + sinRot * move.forward - cosRot * move.sideward;

  angle=start.rot + move.rotation;
  while (angle < 0.0)
    (angle) += DEG_360;

  while (angle >= DEG_360)
    (angle) -= DEG_360;

  end.rot = angle;

  return end;
} // calculateEndPoint

void channelChooseErrorAction( t_Channel *pChannel)
{
  if(!pChannel)
    return;

  ((t_RawData *)pChannel)->chooseErrorAction();

} // channelChooseErrorAction

//-----------------------------------------------------------------------------
// Function  : channelStatus
// Purpose   :
//
// Parameters:  t_Channel channel       :
//
// Return    : bool                     :
//
// Remarks   :
//
bool
channelStatus( t_Channel *pChannel)
{
  if(!pChannel)
    return FALSE;

  return ((t_RawData *)pChannel)->getStatus();
} // channelStatus

//-----------------------------------------------------------------------------
// Function  : channelVersion
// Purpose   :
//
// Parameters: unsigned int length      :
//             char *versionStr         :
//             t_ChannelType type       :
//
// Return    : void                     :
//
// Remarks   :
//
void
channelVersion(unsigned int length, char *versionStr,t_ChannelType type)
{
  char *str[]={"not yet implemented","unknown channel type"};
  char *tmpstr=NULL;

  switch(type) {
    case SCRIPT_CHANNEL:
      {
        t_ScriptData script(length,versionStr);
      }
      break;
    case TCX_CHANNEL:
      if(strlen(str[0]) > length) {
        strncpy(versionStr,str[0],length);
        versionStr[length-1]=0;
      } else {
        strcpy(versionStr,str[0]);
      }
      break;
    default:
      if(strlen(str[1]) > length) {
        strncpy(versionStr,str[1],length);
        versionStr[length-1]=0;
      } else {
        strcpy(versionStr,str[1]);
      }
  } // switch

  char *idStr=GetRCSVersionAndDateC("$Id: channelData.cc,v 1.1 2002/09/14 17:06:31 rstone Exp $","Feb 1998","# channelData (WRAPPER) ");
  t_ChannelDataTimeStamp=t_ChannelDataTimeStamp;
  if(strlen(idStr)+1+strlen(versionStr) < length) {
    if(NULL != (tmpstr=new char[length])) {
      sprintf(tmpstr,"%s\n%s",idStr,versionStr);
      strcpy(versionStr,tmpstr);
      delete tmpstr;
      tmpstr=NULL;
    }
  }
} // channelVersion

//-----------------------------------------------------------------------------
// Function  : channelCheckNext
// Purpose   :
//
// Parameters:  t_Channel *pChannel      :
//             float nonRelevantTime    :
//
// Return    : t_Result                 :
//
// Remarks   :
//
t_Result
channelCheckNext( t_Channel *pChannel, float nonRelevantTime)
{
  if(!pChannel)
    return ERROR;

  return ((t_RawData *)pChannel)->checkNext(nonRelevantTime);

} // channelCheckNext

/*-----------------------------------------------------------------------------
** Function  : *GetRCSVersionAndDateC
** Purpose   :
**
** Parameters: const char *str          : contains $Id:...
**             const char *since        : somthing like "Feb 96"
**             const char *prefix       : something like progamname
**
** Return    : char *                   :
**
** Remarks   : GetRCSVersionAndDateC is valid until next call, 'cause result
**             becomes stored in static char
**             GetRCSVersionAndDateC works fine with RCS Version 5.7 :-)
**
**             out of AnDbasic.c
*/
char *
GetRCSVersionAndDateC(const char *str,const char *since,const char *prefix)
{
  char *unknown="version unknown";
  int month;
  char year[5];
  static char result[200];
  const char *search;

  strcpy(result,prefix);

  /*
  ** String to small or no RCS-Id string?
  */
  if(!str) return unknown;
  if(strlen(str)<=strlen("$Id: ")) return unknown;
  if(strncmp(str,"$Id: ",5)) return unknown;

  search=&str[strlen("$Id: ")];

  /*
  ** _Id: xxxxxx.xx,v yy.zz jjjj/mm/dd hh:mm:ss user Exp user _
  ** searching for " " before jjjj
  */
  search=strstr(search," ");
  if(NULL==search) return unknown;
  search=&search[1];
  search=strstr(search," ");
  if(NULL==search) return unknown;
  search=&search[1];
  if('\0'==search[0]) return unknown;
  if(5>strlen(search)) return unknown;

  strncpy(year,search,4);
  year[4]=0;                                                /* year */

  /*
  ** _Id: xxxxxx.xx,v yy.zz jjjj/mm/dd hh:mm:ss user Exp user _
  ** searching for "/" before mm
  */
  search=strstr(search,"/");
  if(NULL==search) return unknown;

  search=&search[1];
  if('\0'==search[0]) return unknown;
  if(3>strlen(search)) return unknown;
  {
    char temp[3];
    char *errorstr=NULL;
    strncpy(temp,search,2);
    temp[2]=0;
    month=strtol(temp,&errorstr,10);
    if(strlen(errorstr)) return unknown;
    if((1>month)||(12<month)) return unknown;
  }                                                        /* month */
  /*
  ** _Id: xxxxxx.xx,v yy.zz jjjj/mm/dd hh:mm:ss user Exp user _
  ** searching for " " behind ",v" before yy.zz
  */
  search=&str[strlen("$Id: ")];
  search=strstr(search," ");
  search=&search[1];
  {
    const char *temp;
    int   i;
    temp=strstr(search," "); /* Space behind version */
    strncat(result,search,i=strlen(search)-strlen(temp)); /* yy.zz */
    i+=strlen(prefix);
    result[i]=0;
  }

  /*
  *+ put em together
  */
  {
    char *months[]={"Jan","Feb","Mar","Apr","May","Jun",
                    "Jul","Aug","Sep","Oct","Nov","Dec"};
    strcat(result," ");
    if(since)
    {
      strcat(result,since);
      strcat(result," / ");
    }
    strcat(result,months[month -1]);
    strcat(result," ");
    strcat(result,year);
  }
  return result;
} /* GetRCSVersionAndDateC */

int readScript(script* s, rawSensings* actualSensings)
{
  if(localizeWrapperIsInitialized) {
    t_Result res;
    if((END_REACHED == (res = channelCheckNext(localizeWrapperChannel,
                                    s->nonRelevantTime))) ||
       (ERROR == res)) {
      elapsedScriptTime=((t_RawData *)localizeWrapperChannel)->getElapsedTime();
      return 0;
    } else {
      // fill in script reading
      updateScript(s);
      updateRawSensings(actualSensings);
      elapsedScriptTime=((t_RawData *)localizeWrapperChannel)->getElapsedTime();
      return 1;
    }
  } else {
    cerr << "readScript: -- ERROR -- no open script." << endl;
    return 0;
  }
} // readScript


//-----------------------------------------------------------------------------
// Function  : closeChannel
// Purpose   : free memory
//
// Parameters: t_Channel *pChannel       :
//
// Return    : void                     :
//
// Remarks   :
//
void
closeChannel(t_Channel *pChannel)
{
  if(!pChannel)
    return;
  delete ((t_RawData *)pChannel);

  pChannel=NULL;
} // closeChannel

int closeScript(script *s)
{
  if(localizeWrapperIsInitialized) {
    updateScript(s);
    closeChannel(localizeWrapperChannel);
    return 1;
  }
  return 0;
} // closeScript


//-----------------------------------------------------------------------------
// Function  : getChannelBasePosition
// Purpose   :
//
// Parameters: t_Channel *pChannel      :
//             t_RealPosition *basePos  :
//
// Return    : int                      :
//
// Remarks   :
//
int
getChannelBasePosition( t_Channel *pChannel, t_RealPosition *basePos)
{
  if(!pChannel)
    return FALSE;

  if(((t_RawData *)pChannel)->getBasePosition(basePos))
    return 1;
  return 0;
} // getChannelBasePosition

//-----------------------------------------------------------------------------
// Function  : getChannelBump
// Purpose   :
//
// Parameters: t_Channel *pChannel      :
//             t_RealPosition *bumpDelta:
//
// Return    : int                      :
//
// Remarks   :
//
int
getChannelBump( t_Channel *pChannel, t_RealPosition *bumpDelta)
{
  if(!pChannel)
    return FALSE;

  if(((t_RawData *)pChannel)->getBump(bumpDelta))
    return 1;
  return 0;
} // getChannelBump

//-----------------------------------------------------------------------------
// Function  : getChannelDistanceTraveled
// Purpose   :
//
// Parameters: t_Channel *pChannel      :
//             float *dist              :
//
// Return    : int                      :
//
// Remarks   :
//
int
getChannelDistanceTraveled( t_Channel *pChannel,float *dist)
{
  if(!pChannel)
    return FALSE;

  ((t_RawData *)pChannel)->getDistanceTraveled(*dist);
  return 1;
} // getChannelDistanceTraveled

//-----------------------------------------------------------------------------
// Function  : getChannelFrontLaser
// Purpose   :
//
// Parameters:  t_Channel *pChannel      :
//             t_Reading *reading       :
//
// Return    : int                      :
//
// Remarks   :
//
int
getChannelFrontLaser( t_Channel *pChannel, t_Reading *reading)
{
  if(!pChannel)
    return FALSE;

  if(((t_RawData *)pChannel)->getFrontLaser(reading))
    return 1;
  return 0;
} // getChannelLaser

//-----------------------------------------------------------------------------
// Function  : getChannelMarker
// Purpose   :
//
// Parameters:  t_Channel *pChannel      :
//             long *number             :
//
// Return    : int                      :
//
// Remarks   :
//
int
getChannelMarker( t_Channel *pChannel, long *number,
                  t_RealPosition *markerPos)
{
  if(!pChannel)
    return FALSE;

  if(((t_RawData *)pChannel)->getMarker( *number, markerPos))
    return 1;
  return 0;
} // getChannelMarker

//-----------------------------------------------------------------------------
// Function  : getChannelMovement
// Purpose   :
//
// Parameters:  t_Channel *pChannel      :
//             t_RealPosition *delta    :
//
// Return    : int                      :
//
// Remarks   :
//
int
getChannelMovement( t_Channel *pChannel, t_RealPosition *delta)
{
  if(!pChannel)
    return FALSE;

  if(((t_RawData *)pChannel)->getMovement(delta))
    return 1;
  return 0;
} // getChannelMovement

//-----------------------------------------------------------------------------
// Function  : getChannelRearLaser
// Purpose   :
//
// Parameters:  t_Channel *pChannel      :
//             t_Reading *reading       :
//
// Return    : int                      :
//
// Remarks   :
//
int
getChannelRearLaser( t_Channel *pChannel, t_Reading *reading)
{
  if(!pChannel)
    return FALSE;

  if(((t_RawData *)pChannel)->getRearLaser(reading))
    return 1;
  return 0;
} // getChannelLaser

//-----------------------------------------------------------------------------
// Function  : getChannelSonar
// Purpose   :
//
// Parameters:  t_Channel *pChannel      :
//             t_Reading *reading       :
//
// Return    : int                      :
//
// Remarks   :
//
int
getChannelSonar( t_Channel *pChannel, t_Reading *reading)
{
  if(!pChannel)
    return FALSE;

  if(((t_RawData *)pChannel)->getSonar(reading))
    return 1;
  return 0;
} // getChannelSonar

//-----------------------------------------------------------------------------
// Function  : logMessage
// Purpose   :
//
// Parameters: t_Channel *pChannel      :
//             char *printfParams       :
//             ...                      :
//
// Return    : void                     :
//
// Remarks   :
//
void
logMessage( t_Channel *pChannel , char *printfParams , ...)
{
  if(!pChannel)
    return;

  va_list ap;                   // wichtig, gehoert zu varargs.h
  va_start (ap,printfParams);   // wichtig, gehoert zu varargs.h

  ((t_RawData *)pChannel)->logMessage(printfParams,ap);

  va_end (ap);
} // logMessage

//-----------------------------------------------------------------------------
// Function  : *openChannel
// Purpose   :
//
// Parameters:  char *scriptFileName    :
//             float timeFactor         :
//             t_ErrorAction e          :
//             FILE *logFile            :
//             t_ChannelType type       :
//
// Return    : t_Channel *              :
//
// Remarks   :
//
t_Channel *
openChannel( char *scriptFileName,
             float timeFactor,
             t_ErrorAction e,
             FILE *logFile,
             t_ChannelType type)
{
  if(type != SCRIPT_CHANNEL) {
    cerr << "openChannel: channel type not yet supported!\n";
    return NULL;
  }

  if(sizeof(t_Channel *) > sizeof(t_RawData *)) {
    cerr << "openChannel: ERROR C++ to C WRAPPER not supported on this"
         << " architecture!\n"
         << "             use C++ only or write new wrapper!\n";
    exit(49);
  }

  switch (type) {
    case SCRIPT_CHANNEL:
      return (t_Channel *) new t_ScriptData(scriptFileName,
                                            timeFactor,
                                            e,
                                            logFile);
      break;
    case TCX_CHANNEL:
      cerr << "TCX_CHANNEL not yet implemented.\n";
      return NULL;
      break;
    default:
      cerr << "Unknown channel type.\n";
      return NULL;
  } // switch
} // openChannel

int openScript(char *fileName, script *s)
{
  if(localizeWrapperIsInitialized) {
    logMessage(localizeWrapperChannel,
               "openScript: -- WARNING -- localize wrapper "
               "reinitialization\n");
    closeScript(s);
    elapsedScriptTime=0.0;
    nonRelevantTime=0.0;
  }

  s->fileName=fileName;
  localizeWrapperChannel = openChannel(fileName,
                                       s->timeFactor,
                                       STOP_ON_ERROR,
                                       logFile,
                                       SCRIPT_CHANNEL);
  if(localizeWrapperIsInitialized = channelStatus(localizeWrapperChannel)) {
    if(s->odometryCorrection) {
      ((t_RawData *)localizeWrapperChannel)->setOdometryCorrection(CORRECTION_FACTOR);
    }
    // init script-struct
    s->newMarker=FALSE;
    s->sonarCount=0;
    s->positionCount=0;
    s->laserCount=0;
    s->markerCount=0;
    s->distanceOffset=0;
    s->bumpOccured=0;
    s->bumpForward=0;
    s->bumpSideward=0;
    s->bumpRot=0;
    s->noMoveCount=0;

    // fill in script reading
    updateScript(s);
    char versionStr[2000];
    //versionStr[0]=0;
    channelVersion(2000,versionStr,SCRIPT_CHANNEL);
    logMessage(localizeWrapperChannel,versionStr);
    logMessage(localizeWrapperChannel,"\n");
    return 1;
  }
  return 0;
} // openScript

//-----------------------------------------------------------------------------
// Function  : channelResetTimer
// Purpose   :
//
// Parameters: t_Channel *pChannel      :
//
// Return    : void                     :
//
// Remarks   :
//
void
channelResetTimer( t_Channel *pChannel)
{
  if(!pChannel)
    return;

  ((t_RawData *)pChannel)->resetTimer();

} // channelResetTimer



//-----------------------------------------------------------------------------
// Function  : setChannelWantedData
// Purpose   :
//
// Parameters: unsigned int which       : disjunction of
//                                        SONAR, LASER, MOVEMENT,
//                                        BASEPOSITION or MARKER. If you
//                                        want to set all, use ALL_DATA
//                                        instead.
//
// Return    : void                     :
//
// Remarks   :
//
void
setChannelWantedData( t_Channel *pChannel, unsigned int which)
{
  if(!pChannel)
    return;

  ((t_RawData *)pChannel)->setWantedData(which);
} // setChannelWantedData

void updateScript(script *s)
{
  t_RealPosition pos;
  unsigned long sC,lC,pC,mC,nmC;
  ((t_RawData *)localizeWrapperChannel)->getStartPosition(&(s->start));
  ((t_RawData *)localizeWrapperChannel)->getBasePosition(&pos);
  s->currentPos=pos;

  ((t_RawData *)localizeWrapperChannel)->getCounts(&sC,&lC,&pC,&mC,&nmC);
  s->sonarCount=sC;
  s->laserCount=lC;
  s->positionCount=pC;
  s->markerCount=mC;
  s->noMoveCount=nmC;

  ((t_RawData *)localizeWrapperChannel)->getDistanceTraveled(s->distanceOffset);
  s->bumpOccured=0;
  s->bumpForward=0;
  s->bumpSideward=0;
  s->bumpRot=0;

  long number;
  t_RealPosition markerPos;
  if(s->newMarker = getChannelMarker(localizeWrapperChannel,
                                                  &number,
                                                  &markerPos)) {
    (s->newMarker)++;
    if(number != s->markerCount) {
      char infoStr[2000];
      sprintf(infoStr,
              "# NEW MARKER -- WARNING -- found marker    %5ld\n"
              "#                          expected marker %5d\n",
              number,s->markerCount);
      logMessage(localizeWrapperChannel,infoStr);
    }
    s->markerCount = (int)number;
  }

  t_RealPosition bump;
  if(0 != (s->bumpOccured = getChannelBump(localizeWrapperChannel,&bump))) {
    s->bumpForward  = bump.x;
    s->bumpSideward = bump.y;
    s->bumpRot      = bump.rot;
  }
} // updateScript

void updateRawSensings(rawSensings* actualSensings)
{
  t_RealPosition pos;
  t_Reading reading;
  unsigned int i;
  getChannelBasePosition(localizeWrapperChannel,
                         &(actualSensings->basePosition));

  actualSensings->delta.isNew=getChannelMovement(localizeWrapperChannel,
                                                 &pos);
  if(actualSensings->delta.isNew) {
    actualSensings->delta.forward=pos.x;
    actualSensings->delta.sideward=pos.y;
    actualSensings->delta.rotation=pos.rot;
    actualSensings->delta.elapsedTime=-1.0;
  }

  reading.maxNumberOfReadings=NUMBER_OF_SONARS;
  reading.reading=actualSensings->sonar.reading;
  actualSensings->sonar.isNew=getChannelSonar(localizeWrapperChannel,&reading);

  if(actualSensings->sonar.isNew) {
    actualSensings->sonar.numberOfReadings=reading.actualNumberOfReadings;

    if(1) {
      if(NUMBER_OF_SONARS < reading.actualNumberOfReadings) {
        char badStr[2000];
        sprintf(badStr,
                "# updateRawSensings ERROR: new number of sonar\n"
                "                           readings (%d) large!\n"
                "#                          Allowed are %d readings!\n",
                reading.actualNumberOfReadings,
                NUMBER_OF_SONARS);

        logMessage(localizeWrapperChannel,badStr);
        channelChooseErrorAction(localizeWrapperChannel);
        return;
      }
    }
    for(i=0 ; i < reading.actualNumberOfReadings; i++)
      actualSensings->sonar.reading[i].dist=reading.reading[i].dist;
    //printf("# (%ld) actualNumberOfReadings (Sonar) %u, [0]: %f, %f\n",
    //       ((t_RawData *)localizeWrapperChannel)->getActualLine(),
    //       reading.actualNumberOfReadings,
    //       actualSensings->sonar.reading[0].dist,
    //       actualSensings->sonar.reading[0].rot);
  }

  reading.maxNumberOfReadings=NUMBER_OF_SCANS_PER_LASER;
  reading.reading=actualSensings->frontLaser.reading;
  actualSensings->frontLaser.isNew=getChannelFrontLaser(localizeWrapperChannel,
                                                        &reading);
  if(actualSensings->frontLaser.isNew) {
    actualSensings->frontLaser.numberOfReadings=reading.actualNumberOfReadings;
    if(1) {
      if(NUMBER_OF_SCANS_PER_LASER < reading.actualNumberOfReadings) {
        char badStr[2000];
        sprintf(badStr,
                "# updateRawSensings ERROR: new number of front laser\n"
                "                           readings (%d) large!\n"
                "#                          Allowed are %d readings!\n",
                reading.actualNumberOfReadings,
                NUMBER_OF_SCANS_PER_LASER);

        logMessage(localizeWrapperChannel,badStr);
        channelChooseErrorAction(localizeWrapperChannel);
        return;
      }
    }

    for(i=0 ; i < reading.actualNumberOfReadings; i++)
      actualSensings->frontLaser.reading[i].dist=reading.reading[i].dist;
  }

  reading.maxNumberOfReadings=NUMBER_OF_SCANS_PER_LASER;
  reading.reading=actualSensings->rearLaser.reading;
  actualSensings->rearLaser.isNew=getChannelRearLaser(localizeWrapperChannel,
                                                      &reading);

  if(actualSensings->rearLaser.isNew) {
    actualSensings->rearLaser.numberOfReadings=reading.actualNumberOfReadings;
    if(1) {
      if(NUMBER_OF_SCANS_PER_LASER < reading.actualNumberOfReadings) {
        char badStr[2000];
        sprintf(badStr,
                "# updateRawSensings ERROR: new number of rear laser\n"
                "                           readings (%d) large!\n"
                "#                          Allowed are %d readings!\n",
                reading.actualNumberOfReadings,
                NUMBER_OF_SCANS_PER_LASER);

        logMessage(localizeWrapperChannel,badStr);
        channelChooseErrorAction(localizeWrapperChannel);
        return;
      }
    }

    for(i=0 ; i < reading.actualNumberOfReadings; i++)
      actualSensings->rearLaser.reading[i].dist=reading.reading[i].dist;
  }

  getChannelDistanceTraveled(localizeWrapperChannel,
                             &(actualSensings->distanceTraveled));

  // set the global measured position of the robot
  measuredPosition = calculateEndPoint( measuredPosition,actualSensings->delta);

} // updateRawSensings

// End of ChannelData //
