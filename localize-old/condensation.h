
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** Welcome!
 *****
 ***** This file is part of the robot control software provided
 ***** by Real World Interface Inc.
 *****
 ***** All copyrights are by Real World Interface Inc., Carnegie
 ***** Mellon University, and the University of Bonn, Germany.
 ***** Removing the copyright information is illegal. Please read
 ***** and make sure you understand the disclaimer below.
 *****
 ***** Contact tyson@rwii.com if you have questions or would
 ***** like to obtain further information.
 *****
 ***** We hope you enjoy working with this software package.
 *****
 *****                      Tyson D. Sawyer and Sebastian Thrun
 *****
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** 
 ***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED
 ***** BY APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING
 ***** THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM
 ***** "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR
 ***** IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 ***** OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE
 ***** ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM
 ***** IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME
 ***** THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
 ***** 
 ***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
 ***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
 ***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
 ***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
 ***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
 ***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO
 ***** LOSS OF DATA OR DATA BEING RENDERED INACCURATE OR LOSSES
 ***** SUSTAINED BY YOU OR THIRD PARTIES OR A FAILURE OF THE PROGRAM
 ***** TO OPERATE WITH ANY OTHER PROGRAMS OR FAILURE TO CONTROL A
 ***** PHYSICAL DEVICE OF ANY TYPE), EVEN IF SUCH HOLDER OR OTHER
 ***** PARTY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/condensation.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 17:06:29 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: condensation.h,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.13  1999/11/02 18:12:33  fox
 * Version of multi localize journal paper.
 *
 * Revision 1.12  1999/09/01 21:26:54  fox
 * Works almost perfectly :-)
 *
 * Revision 1.11  1999/09/01 00:02:56  fox
 * Getting closer. But still not close enough.
 *
 * Revision 1.10  1999/07/13 23:08:01  fox
 * Some changes.
 *
 * Revision 1.9  1999/04/21 14:06:00  fox
 * Just an intermediate version.
 *
 * Revision 1.8  1999/01/22 17:48:01  fox
 * Final version of sampling paper and denver demo.
 *
 * Revision 1.7  1999/01/11 19:47:47  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.6  1998/11/24 23:05:25  fox
 * Implemented furhter routines for condensation and vision.
 *
 * Revision 1.5  1998/11/17 23:26:17  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.4  1998/11/03 21:02:16  fox
 * Lasers will not be integrated if the robot rotates faster than
 * MAX_ROT_VEL_FOR_LASERS deg/sec.
 *
 * Revision 1.3  1998/10/02 15:16:36  fox
 * Several improvements of condensation. Updated the function calls to the
 * new version of the distancServer.
 *
 * Revision 1.2  1998/09/25 04:02:53  fox
 * First version of CONDENSATION!!!!
 *
 * Revision 1.1  1998/09/18 17:24:42  fox
 * Added skeleton files for condensation.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#ifndef CONDENSATION_INCLUDE
#define CONDENSATION_INCLUDE

#include "general.h"
#include "sensings.h"
#include "LOCALIZE-messages.h"


/**************************************************************************
 **************************************************************************
 * Mandatory functions.
 **************************************************************************
 **************************************************************************/

/**************************************************************************
 **************************************************************************
 * Structures.
 **************************************************************************
 **************************************************************************/

typedef struct {
  float distanceNoise;
  float angleNoise;
  float sideDrift;
  float rotDrift;
  int usePosition;
  int dumpSamples;
  float resetProbability;
  sampleSetHistory* samplesHistory;
} condensationParameters;



/**************************************************************************
 **************************************************************************
 * Other functions.
 **************************************************************************
 **************************************************************************/

void
initialize_CONDENSATION( char* fileName,
			 actionInformation* actionInfo,
			 sensingActionMask* actionMask, int argc, char** argv);

void
checkWhichActionsToPerform_CONDENSATION( actionInformation* info, sensingActionMask* mask);

void
performActions_CONDENSATION( actionInformation* info, sensingActionMask* mask);

void
setSamplePosition( realPosition center,
		   sampleSet* samples,
		   float deviation);

void
updateSampleMean( sampleSet* samples);

void
shiftSamples( sampleSet* samples, probabilityGrid* aPrioriProbs);

void
generateSample( sampleSet* sampleSrc, sampleSet* sampleDest);

sampleSet
initializedSamples( int numberOfSamples,
		   float minX, float minY,
		   float maxX, float maxY);

void
deallocateSamples( sampleSet* samples);

void
setSampleRegion( sampleSet* samples,
		 float minX, float minY,
		 float maxX, float maxY);

void
setUniformDistribution( sampleSet* samples, probabilityGrid* aPrioriProbs);

void
copySamples( sampleSet* src, sampleSet* dest);

#ifdef ONLINE_FILTER
featureStruct** preprocessedLaserFeatures;
#endif

#endif






