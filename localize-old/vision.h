
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/vision.h,v $
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
 * $Log: vision.h,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.7  1998/11/25 16:29:43  wolfram
 * Added higher resolution for vision maps. resolution is now read from file.
 * Couldn't integrate it consistently into graphic.c.
 *
 * Revision 1.6  1998/11/24 15:31:11  wolfram
 * Fixed a bug, added samples to vision
 *
 * Revision 1.5  1998/09/05 22:06:57  wolfram
 * Changes regarding vision!
 *
 * Revision 1.4  1998/08/23 00:01:06  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.3  1998/08/20 00:23:04  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.2  1998/04/19 10:40:39  wolfram
 * Added graphical display to multi localization
 *
 * Revision 1.1  1998/03/17 08:41:13  wolfram
 * First steps to add vision.
 *
 * Revision 1.5  1998/01/22 13:06:21  fox
 * First version after selection-submission.
 *
 * Revision 1.4  1997/11/28 14:11:52  fox
 * Minor changes.
 *
 * Revision 1.3  1997/11/28 13:34:37  fox
 * Added questions.
 *
 * Revision 1.2  1997/11/26 15:47:45  fox
 * Added some structures for questions.
 *
 * Revision 1.1  1997/11/21 15:36:07  fox
 * Modifications in graphic
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#ifndef VISION_INCLUDE
#define VISION_INCLUDE

#define INTEGRATE_VISION         0
#define NUMBER_OF_ACTIONS_VISION 1

#include "general.h"
#include "sensings.h"
#include "probGrid.h"


#define IMAGE_SIZE_X 25
#define IMAGE_SIZE_Y 25

/*********************************************************
 *********************************************************
 * The following functions and structures are mandatory for
 * each sensor.
 *********************************************************
 *********************************************************/


void
initialize_VISION( char* fileName,
		   actionInformation* actionInfo,
		   sensingActionMask* actionMask,
		   sensingFunctions* handlers);


void
checkIfConsider_VISION( actionInformation* actionInfo,
			  sensingActionMask* mask);

void
checkWhichActionsToPerform_VISION( actionInformation* actionInfo,
				   sensingActionMask* mask);

void
performActions_VISION( actionInformation* actionInfo,
		       sensingActionMask* mask);



/*********************************************************
 *********************************************************
 * Structures.
 *********************************************************
 *********************************************************/

typedef struct {
  float integrateThreshold;
  float maxQuotaOfPlanes;
  float crunchFactor;
  int resolution;
  bool useVision;
  int maxNumberOfIntegrationsPerStop;
} visionParameters;



typedef struct {
  int numberOfExpectedFeatures;
  int numberOfMeasuredFeatures;
  probability** prob;
} visionFeatureProbTable;



/* This struct stores general information to integrate question information. */
typedef struct {
  int*                     useProbGrid;
  bool                     useVision;
  positionProbabilityGrid* grid;
  sampleSet*               samples;
  probabilityGrid*         map;
  visionMap*               visMap;
  visionMap*               visVarMap;
  visionMap*               varianceMap;
  visionMap*               varianceVarMap;
  sensing_VISION*          image;
  visionFeatureProbTable   probTab;
} informationsFor_VISION;

/* This struct stores information needed to integrate the question information. */
typedef struct {
  int number;
  bool actualImage;
} infoForVisionFeature;




/*********************************************************
 *********************************************************
 * Functions.
 *********************************************************
 *********************************************************/

#endif




