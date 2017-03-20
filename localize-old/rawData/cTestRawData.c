/* time-stamp value _has to_ reside in the first 8 lines */
static char *TESTRawDataTimeStamp=" (C)TEST rawData     \
Time-stamp: <1998-02-07 12:04:31 derr>";

#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <sys/types.h>

#include "channelData.h"

FILE *logFile=NO_LOG_FILE;

int main(int argc, char **argv)
{
  char vStr[2000];
  t_Channel *script;
  float nonRelevantTime=0.0;
  t_Result res;
  bool bEnd=FALSE;
  t_RealPosition basePos;
  t_RealPosition markerPos;
  t_Reading laserReading;
  t_DistanceReading *laserDistance;
  t_DistanceReading *pLaserDistance=NULL;
  long number;
  t_RealPosition delta;
  t_Reading sonarReading;
  t_DistanceReading *sonarDistance;
  t_DistanceReading *pSonarDistance=NULL;
  char message[2048];

  /* get version */
  channelVersion(2000,vStr,SCRIPT_CHANNEL);
  printf("%s\n%s\n",TESTRawDataTimeStamp,vStr);

  /* open script */
  if( 1==argc ) {
    script=openChannel("test.script",1.0,CORE_ON_ERROR,NO_LOG_FILE,
                       SCRIPT_CHANNEL);
  } else {
    script=openChannel(argv[1],1.0,CORE_ON_ERROR,NO_LOG_FILE,
                       SCRIPT_CHANNEL);
  }
  if(!channelStatus(script)) {
    printf("script bad...\n");
    printf("usage:\n%s [script-file]\n",argv[0]);
    exit(1);
  }

  laserDistance = (t_DistanceReading *)
    malloc ( NUMBER_OF_SCANS_PER_LASER * sizeof( t_DistanceReading ) );
  pLaserDistance=laserDistance;

  sonarDistance = (t_DistanceReading *)
    malloc ( NUMBER_OF_SONARS * sizeof( t_DistanceReading ) );
  pSonarDistance=sonarDistance;

  if(NULL == laserDistance || NULL == sonarDistance) {
     fprintf(stderr,"Out of memory (sonarDist or laserDist)!\n");
     exit(1);
  }
  /* init readings */
  laserReading.reading=pLaserDistance;
  laserReading.actualNumberOfReadings=0;
  laserReading.maxNumberOfReadings=NUMBER_OF_SCANS_PER_LASER;

  sonarReading.reading=pSonarDistance;
  sonarReading.actualNumberOfReadings=0;
  sonarReading.maxNumberOfReadings=NUMBER_OF_SONARS;

  /*----------------------------------------------------------------------*/
  /*------------------------------  GO  ----------------------------------*/
  /*----------------------------------------------------------------------*/

  if(0) {
    /*
    setChannelWantedData(script,MARKER | MOVEMENT);
    */
    setChannelWantedData(script,LASER_DATA);
  }


  do {
    res=channelCheckNext(script,nonRelevantTime);
    nonRelevantTime=0.0;
    switch (res) {
      case ERROR:
        logMessage(script,"ERROR occured\n");
        logMessage(script,"choosen action: end program\n");
        bEnd=TRUE; /* you may also choose to read again! */
        break;
      case NO_NEW_DATA:
        /* nothing to do */
        break;
      case END_REACHED:
        logMessage(script,"END reached\n");
        bEnd=TRUE;
        break;
      case NEW_DATA:
        logMessage(script,"new data: ");
        if(1 == getChannelBasePosition(script,&basePos))
          logMessage(script,"BASE ");
        else
          logMessage(script,"     ");
        if(1 == getChannelFrontLaser(script,&laserReading))
          logMessage(script,"FRONT_LASER ");
        else
          logMessage(script,"            ");
        if(1 == getChannelRearLaser(script,&laserReading))
          logMessage(script,"REAR_LASER ");
        else
          logMessage(script,"           ");
        if(1 == getChannelMarker(script,&number, &markerPos))
          logMessage(script,"MARKER ");
        else
          logMessage(script,"       ");
        if(1 == getChannelMovement(script,&delta))
          logMessage(script,"MOVEMENT ");
        else
          logMessage(script,"         ");
        if(1 == getChannelSonar(script,&sonarReading))
          logMessage(script,"SONAR ");
        else
          logMessage(script,"      ");
        logMessage(script,"\n");

        /* you may read the same readings several times */
        if(1 == getChannelBasePosition(script,&basePos)) {
          sprintf(message, "BASE %4.1f %4.1f %4.1f\n",
                  basePos.x,basePos.y,basePos.rot);
          logMessage(script,message);
        }
        if(1 == getChannelFrontLaser(script,&laserReading)) {
          sprintf(message, "FrontLaser %4.1f %4.1f %4.1f %4.1f %4.1f...\n",
                  laserReading.reading[0].dist,
                  laserReading.reading[1].dist,
                  laserReading.reading[2].dist,
                  laserReading.reading[3].dist,
                  laserReading.reading[4].dist);
          logMessage(script,message);
        }
        if(1 == getChannelRearLaser(script,&laserReading)) {
          sprintf(message, "RearLaser %4.1f %4.1f %4.1f %4.1f %4.1f...\n",
                  laserReading.reading[0].dist,
                  laserReading.reading[1].dist,
                  laserReading.reading[2].dist,
                  laserReading.reading[3].dist,
                  laserReading.reading[4].dist);
          logMessage(script,message);
        }
        if(1 == getChannelMarker(script,&number, &markerPos)) {
          sprintf(message, "Marker %lu at %4.1f %4.1f %4.1f\n",
                  number,markerPos.x,markerPos.y,markerPos.rot);
          logMessage(script,message);
        }
        if(1 == getChannelMovement(script,&delta)) {
          sprintf(message, "DELTA %4.1f %4.1f %4.1f\n",
                  delta.x,delta.y,delta.rot);
          logMessage(script,message);
        }
        if(1 == getChannelSonar(script,&sonarReading)) {
          sprintf(message, "Sonar %4.1f %4.1f %4.1f %4.1f %4.1f...\n",
                  sonarReading.reading[0].dist,
                  sonarReading.reading[1].dist,
                  sonarReading.reading[2].dist,
                  sonarReading.reading[3].dist,
                  sonarReading.reading[4].dist);
          logMessage(script,message);

        }
        logMessage(script,
                   "---------------------------------------"
                   "---------------------------------------\n");
        break;
      default:
        logMessage(script,"unknown result!!!!!!!!!!!!!!!!\n");
    } /* switch */
  } while (!bEnd);

  closeChannel(script);

  free(laserDistance);
  laserDistance=NULL;
  free(sonarDistance);
  sonarDistance=NULL;

  return 1;
}
