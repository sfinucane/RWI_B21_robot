/* time-stamp value _has to_ reside in the first 8 lines */
static char *LocalizeTestRawDataTimeStamp=" (C)TEST rawData     \
Time-stamp: <98/02/12 15:33:29 derr>";

#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <sys/types.h>

#include "channelData.h"

FILE *logFile=NO_LOG_FILE;

int main(int argc, char **argv)
{
  char vStr[2000];
  float nonRelevantTime=0.0;
  bool res;
  bool bEnd=FALSE;
  t_DistanceReading *frontLaserDistance;
  t_DistanceReading *rearLaserDistance;
  t_DistanceReading *sonarDistance;
  char message[2048];

  rawSensings rawS;
  script s;
  char *fileName;

  if( 1==argc ) {
     s.fileName="test.script";
  } else {
     s.fileName=argv[1];
  }
  s.odometryCorrection=0;
  s.timeFactor=1.0;

  fileName=s.fileName;

  /* get version */
  channelVersion(2000,vStr,SCRIPT_CHANNEL);
  printf("%s\n%s\n",LocalizeTestRawDataTimeStamp,vStr);

  /* open script */
  if(!openScript(fileName,&s)) {
     fprintf(stderr, "Bad script!\n");
     fprintf(stderr,"usage:\n%s [script-file]\n",argv[0]);
     exit(1);
  }

  frontLaserDistance = (t_DistanceReading *)
    malloc ( NUMBER_OF_SCANS_PER_LASER * sizeof( t_DistanceReading ) );

  rearLaserDistance = (t_DistanceReading *)
    malloc ( NUMBER_OF_SCANS_PER_LASER * sizeof( t_DistanceReading ) );

  sonarDistance = (t_DistanceReading *)
    malloc ( NUMBER_OF_SONARS * sizeof( t_DistanceReading ) );

  if(NULL == frontLaserDistance ||
     NULL == rearLaserDistance ||
     NULL == sonarDistance) {
     fprintf(stderr,"Out of memory (sonarDist or laserDist)!\n");
     exit(1);
  }
  /* init readings */
  rawS.frontLaser.reading=frontLaserDistance;
  rawS.frontLaser.numberOfReadings=NUMBER_OF_SCANS_PER_LASER/2;
  rawS.rearLaser.reading=rearLaserDistance;
  rawS.rearLaser.numberOfReadings=NUMBER_OF_SCANS_PER_LASER/2;
  rawS.sonar.reading=sonarDistance;
  rawS.sonar.numberOfReadings=NUMBER_OF_SONARS;

  /*----------------------------------------------------------------------*/
  /*------------------------------  GO  ----------------------------------*/
  /*----------------------------------------------------------------------*/


  s.timeFactor=-1;

  do {
     s.nonRelevantTime=nonRelevantTime;
     res=readScript(&s,&rawS);
     nonRelevantTime=0.0;
     if(!res) {
        fprintf(stdout,"END reached.\n");
        bEnd=TRUE;
     } else {
        fprintf(stdout,"new data: ");

        if(rawS.frontLaser.isNew)
          fprintf(stdout,"FRONT_LASER ");
        else
          fprintf(stdout,"            ");

        if(rawS.rearLaser.isNew)
          fprintf(stdout,"REAR_LASER ");
        else
          fprintf(stdout,"           ");

        if(s.newMarker)
          fprintf(stdout,"MARKER ");
        else
          fprintf(stdout,"       ");

        if(rawS.delta.isNew)
          fprintf(stdout,"MOVEMENT ");
        else
          fprintf(stdout,"         ");

        if(rawS.sonar.isNew)
          fprintf(stdout,"SONAR ");
        else
          fprintf(stdout,"      ");

        fprintf(stdout,"\n");

        if(rawS.delta.isNew) {
          sprintf(message, "#ROBOT %4.1f %4.1f %4.1f\n",
                  rawS.basePosition.x,
                  rawS.basePosition.y,
                  rawS.basePosition.rot);
          fprintf(stdout,message);
        }

        if(rawS.frontLaser.isNew) {
          sprintf(message, "FrontLaser %4.1f %4.1f %4.1f %4.1f %4.1f...\n",
                  rawS.frontLaser.reading[0].dist,
                  rawS.frontLaser.reading[1].dist,
                  rawS.frontLaser.reading[2].dist,
                  rawS.frontLaser.reading[3].dist,
                  rawS.frontLaser.reading[4].dist);
          fprintf(stdout,message);
        }
        if(rawS.rearLaser.isNew) {
          sprintf(message, "RearLaser %4.1f %4.1f %4.1f %4.1f %4.1f...\n",
                  rawS.rearLaser.reading[0].dist,
                  rawS.rearLaser.reading[1].dist,
                  rawS.rearLaser.reading[2].dist,
                  rawS.rearLaser.reading[3].dist,
                  rawS.rearLaser.reading[4].dist);
          fprintf(stdout,message);
        }

        if(s.newMarker) {
          sprintf(message, "Marker %d\n", s.markerCount);
          fprintf(stdout,message);
        }

        if(rawS.delta.isNew) {
          sprintf(message, "DELTA %4.1f %4.1f %4.1f\n",
                  rawS.delta.forward,rawS.delta.sideward,rawS.delta.rotation);
          fprintf(stdout,message);
        }

        if(rawS.sonar.isNew) {
          sprintf(message, "Sonar %4.1f %4.1f %4.1f %4.1f %4.1f...\n",
                  rawS.sonar.reading[0].dist,
                  rawS.sonar.reading[1].dist,
                  rawS.sonar.reading[2].dist,
                  rawS.sonar.reading[3].dist,
                  rawS.sonar.reading[4].dist);
          fprintf(stdout,message);

        }
        fprintf(stdout,
                   "---------------------------------------"
                   "---------------------------------------\n");
    }
  } while (!bEnd);

  closeScript(&s);

  free(frontLaserDistance);
  frontLaserDistance=NULL;
  free(rearLaserDistance);
  rearLaserDistance=NULL;
  free(sonarDistance);
  sonarDistance=NULL;

  return 1;
}
