/* Time-stamp: <1998-02-11 11:52:59 derr>
***********************************************************************
***********************************************************************
***********************************************************************
*****
***** File name:                   channelData.h
*****
***** Part of:                     RHINO SOFTWARE
*****
***** Creator:                     Andreas Derr, University of Bonn
*****
***** Date of creation:            Oct 1997
*****
***** Purpose:                     Wrapper for CLASS t_RawData
*****
*****
***** $Source: /usr/local/cvs/bee/src/localize/rawData/channelData.h,v $
*****
***** $Revision: 1.1 $
*****
***** $Date: 2002/09/14 17:06:31 $
*****
***** $Author: rstone $
*****
*****
*****
***** Contact derr@informatik.uni-bonn.de
*****
***********************************************************************
***********************************************************************
***********************************************************************
*****
***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED BY
***** APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING THE
***** COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM "AS IS"
***** WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED,
***** INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
***** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE ENTIRE
***** RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU.
***** SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL
***** NECESSARY SERVICING, REPAIR OR CORRECTION.
*****
***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO LOSS OF
***** DATA OR DATA BEING RENDERED INACCURATE OR LOSSES SUSTAINED BY YOU
***** OR THIRD PARTIES OR A FAILURE OF THE PROGRAM TO OPERATE WITH ANY
***** OTHER PROGRAMS), EVEN IF SUCH HOLDER OR OTHER PARTY HAS BEEN
***** ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
*****
***********************************************************************
***********************************************************************
***********************************************************************
**
**
**                  ----- REVISION HISTORY -----
**
** $Log: channelData.h,v $
** Revision 1.1  2002/09/14 17:06:31  rstone
** *** empty log message ***
**
** Revision 1.1  1998/02/12 18:13:58  derr
** Added library for reading laserint- sonarint- and 'new'-scripts.
**
** Revision 1.1  1998/02/12 15:59:47  derr
** New library librawData for reading laserint-, sonarint and 'new'-script.
** Will replace bee/src/localize/script.c.
**
**
**
***********************************************************************
***********************************************************************
**********************************************************************/
#ifndef CHANNEL_DATA_INCLUDE
#define CHANNEL_DATA_INCLUDE
/* load some general definitions */
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include "typedefRawData.h"



typedef struct {
  unsigned long empty;
} t_Channel;

typedef enum {
  SCRIPT_CHANNEL = 0,
  TCX_CHANNEL
} t_ChannelType;

#ifdef __cplusplus
extern "C" {
#endif

  void channelChooseErrorAction( t_Channel *pChannel);

  /* TRUE: everything is allright */
  bool channelStatus( t_Channel *pChannel);

  /* get version */
  void channelVersion( unsigned int length, char *versionStr,
                       t_ChannelType type);

  /* new data arrived? */
  t_Result channelCheckNext( t_Channel *pChannel, float nonRelevantTime);

  void closeChannel(t_Channel *pChannel);


  /* get new data
  ** TRUE:  parameter contains new data
  ** FALSE: parameter unchanged */
  int getChannelBasePosition( t_Channel *pChannel, t_RealPosition *basePos);
  int getChannelDistanceTraveled( t_Channel *pChannel, float *dist);
  int getChannelFrontLaser( t_Channel *pChannel, t_Reading *reading);
  int getChannelMarker( t_Channel *pChannel, long *number,
                         t_RealPosition *markerPos);
  int getChannelMovement( t_Channel *pChannel, t_RealPosition *delta);
  int getChannelRearLaser( t_Channel *pChannel, t_Reading *reading);
  int getChannelSonar( t_Channel *pChannel, t_Reading *reading);

  void logMessage( t_Channel *pChannel , char *printfParams , ...);

  /* use script file <scriptFileName>,
  ** logging output to stdout, stderr _and_ <logFile>
  ** <scriptFileName> may equal NULL if type equals TCX_CHANNEL */
  t_Channel *openChannel( char *scriptFileName,
                          float timeFactor,
                          t_ErrorAction e,
                          FILE *logFile,
                          t_ChannelType type);

  void channelResetTimer( t_Channel *pChannel);
  void setChannelWantedData( t_Channel *pChannel, unsigned int which);

char *
GetRCSVersionAndDateC(const char *str,const char *since,const char *prefix);


  /******************* scpecial wrapper feature for localize only */
  /* the localize wrapper */
  int openScript(char *fileName, script *s);
  int closeScript(script *s);
  int readScript(script* s, rawSensings* actualSensings);

  realPosition calculateEndPoint( realPosition start, movement move);
  void updateScript(script *s);
  void updateRawSensings(rawSensings* actualSensings);
  extern float elapsedScriptTime;
  extern float nonRelevantTime;
#define BUFFLEN 3000

#ifdef __cplusplus
};  /*extern "C" { */
#endif

#endif
/* End of CHANNEL_DATA_HEADER */

