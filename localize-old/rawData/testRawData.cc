// time-stamp value _has to_ reside in the first 8 lines
static char *RawDataTimeStamp=" TEST    rawData     \
Time-stamp: <1998-02-07 12:04:39 derr>";
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
/////
///// File name:                   testRawData.cc
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
///// $Source: /usr/local/cvs/bee/src/localize/rawData/testRawData.cc,v $
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
// $Log: testRawData.cc,v $
// Revision 1.1  2002/09/14 17:06:31  rstone
// *** empty log message ***
//
// Revision 1.1  1998/02/12 18:14:03  derr
// Added library for reading laserint- sonarint- and 'new'-scripts.
//
// Revision 1.1  1998/02/12 15:59:52  derr
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

FILE *logFile=NO_LOG_FILE;

int main(int argc, char **argv)
{
  float nonRelevantTime=0.0;
  t_Result res;
  bool bEnd=FALSE;
  t_RealPosition basePos;
  t_RealPosition markerPos;
  t_Reading laserReading;
  t_DistanceReading laserDistance[NUMBER_OF_SCANS_PER_LASER];
  t_DistanceReading *pLaserDistance=laserDistance;
  long number;
  t_RealPosition delta;
  t_Reading sonarReading;
  t_DistanceReading sonarDistance[NUMBER_OF_SONARS];
  t_DistanceReading *pSonarDistance=sonarDistance;
  char message[2048];


  /* get version */
  cerr << RawDataTimeStamp << "\n";
  char vStr[2000];
  t_ScriptData vScript(2000,vStr);
  cerr << vStr << "\n";

  /* open script */
  t_ScriptData *script;
  if( 1==argc ) {
    script=new t_ScriptData("test.script",1.0,CORE_ON_ERROR,NO_LOG_FILE);
  } else {
    script=new t_ScriptData(argv[1],1.0,CORE_ON_ERROR,NO_LOG_FILE);
  }
  if(!script->getStatus()) {
    cerr << "script bad...\n"
         << "usage:\n" << argv[0] << " [script-file]\n";
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

  int downcount=0;
  if(1) {
    /*
    script->setWantedData(MARKER | MOVEMENT);
    */
    script->setWantedData(BASEPOSITION_DATA);
    downcount=50;
  }

  do {
    res=script->checkNext(nonRelevantTime);
    if(1 && (0 != downcount)) {
      downcount--;
      if(0 == downcount) {
        script->setWantedData(LASER_DATA | BASEPOSITION_DATA);
      }
    }
    nonRelevantTime=0.0;
    switch (res) {
      case ERROR:
        script->logMessage("ERROR occured\n");
        script->logMessage("choosen action: end program\n");
        bEnd=TRUE; /* you may also choose to read again! */
        break;
      case NO_NEW_DATA:
        script->logMessage("WARNING: NO_NEW_DATA! why?\n");
        /* nothing to do */
        break;
      case END_REACHED:
        script->logMessage("END reached\n");
        bEnd=TRUE;
        break;
      case NEW_DATA:
        script->logMessage("new data: ");
        if(script->getBasePosition(basePos))
          script->logMessage("BASE ");
        else
          script->logMessage("     ");
        if(script->getFrontLaser(laserReading))
          script->logMessage("FRONT_LASER ");
        else
          script->logMessage("            ");
        if(script->getRearLaser(laserReading))
          script->logMessage("REAR_LASER ");
        else
          script->logMessage("           ");
        if(script->getMarker(number,markerPos))
          script->logMessage("MARKER ");
        else
          script->logMessage("       ");
        if(script->getMovement(delta))
          script->logMessage("MOVEMENT ");
        else
          script->logMessage("         ");
        if(script->getSonar(sonarReading))
          script->logMessage("SONAR ");
        else
          script->logMessage("      ");
        script->logMessage("\n");

        /* you may read the same readings several times */
        if(script->getBasePosition(basePos)) {
          sprintf(message, "BASE %4.1f %4.1f %4.1f\n",
                  basePos.x,basePos.y,basePos.rot);
          script->logMessage(message);
        }
        if(script->getFrontLaser(laserReading)) {
          sprintf(message, "FrontLaser %4.1f %4.1f %4.1f %4.1f %4.1f...\n",
                  laserReading.reading[0].dist,
                  laserReading.reading[1].dist,
                  laserReading.reading[2].dist,
                  laserReading.reading[3].dist,
                  laserReading.reading[4].dist);
          script->logMessage(message);
        }
        if(script->getRearLaser(laserReading)) {
          sprintf(message, "RearLaser %4.1f %4.1f %4.1f %4.1f %4.1f...\n",
                  laserReading.reading[0].dist,
                  laserReading.reading[1].dist,
                  laserReading.reading[2].dist,
                  laserReading.reading[3].dist,
                  laserReading.reading[4].dist);
          script->logMessage(message);
        }
        if(script->getMarker(number,markerPos)) {
          sprintf(message, "Marker %lu at %4.1f %4.1f %4.1f\n",
                  number,markerPos.x,markerPos.y,markerPos.rot);
          script->logMessage(message);
        }
        if(script->getMovement(delta)) {
          sprintf(message, "DELTA %4.1f %4.1f %4.1f\n",
                  delta.x,delta.y,delta.rot);
          script->logMessage(message);
        }
        if(script->getSonar(sonarReading)) {
          sprintf(message, "Sonar %4.1f %4.1f %4.1f %4.1f %4.1f...\n",
                  sonarReading.reading[0].dist,
                  sonarReading.reading[1].dist,
                  sonarReading.reading[2].dist,
                  sonarReading.reading[3].dist,
                  sonarReading.reading[4].dist);
          script->logMessage(message);

        }
        script->logMessage(
                   "---------------------------------------"
                   "---------------------------------------\n");
        break;
      default:
        script->logMessage("unknown result!!!!!!!!!!!!!!!!\n");
    } /* switch */
  } while (!bEnd);

  delete script;

  return 1;
}


// End of testRawData //
