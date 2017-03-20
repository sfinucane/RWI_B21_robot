
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** Source file:     $Source: /usr/local/cvs/bee/src/maploc/data.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 16:16:24 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: data.c,v $
 * Revision 1.1  2002/09/14 16:16:24  rstone
 * *** empty log message ***
 *
 * Revision 1.7  1998/08/14 03:51:31  thrun
 * intermediate version, don't use
 *
 * Revision 1.6  1997/10/06 19:17:00  thrun
 * Fixed two REALLY BAD BUGS, one of which had to do with the initialization
 * of the forward_probs table. As a result, this table was partially
 * undefined, and the forward convolution was random.
 *
 * Revision 1.5  1997/10/05 22:16:47  thrun
 * File interface, fixed bugs, improved XWindows interface.
 *
 * Revision 1.4  1997/10/04 16:16:56  thrun
 * .
 *
 * Revision 1.3  1997/09/19 03:39:05  thrun
 * .
 *
 * Revision 1.2  1997/09/18 12:57:09  thrun
 * Improved the interpolation. Wean data set can now be plotted.
 *
 * Revision 1.1  1997/09/17 01:25:05  thrun
 * Some intermediate version - I don't know.
 *
 *
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include <signal.h>


#include "EZX11.h"
#include "o-graphics.h"


#include "rai.h"
#include "raiClient.h"
#include "baseClient.h"
#include "sonarClient.h"
#include "tactileClient.h"
#include "rai.h"
#include "raiClient.h"
#include "cameraClient.h"
#include "buttonClient.h"
#include "pantiltClient.h"

#include "main3d.h"


/************************************************************************\
 ************************************************************************
\************************************************************************/



void
demo_data()
{

  /**** new demo run ***/

#define ERROR_FAC 0.0

  set_initial_position(10.0, 10.0, 90.0, 1);  
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(0.0, 40.0, 0.0, 0.0); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(0.0, 20.0, 0.0, 0.0); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(-90.0, 40.0, ERROR_FAC * (-9.0), ERROR_FAC * 2.0); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(180.0, 40.0, ERROR_FAC * 18.0, ERROR_FAC * -2.0); 
  saw_landmark_polar(0.0, 0.0);

  robot_motion_polar(90.0, 20.0, 0.0, 0.0); 
  saw_landmark_polar(0.0, 0.0);


  set_initial_position(50.0, 50.0, -90.0, 0);  
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(0.0, 40.0, 0.0, 0.0); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(90.0, 20.0, 0.0, 0.0); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(90.0, 60.0, 0.0, 0.0); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(90.0, 20.0, 0.0, 0.0); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(90.0, 20.0, 0.0, 0.0); 
  saw_landmark_polar(0.0, 0.0);

  robot_motion_polar(-90.0, 40.0, 0.0, 0.0); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(-90.0, 20.0, 0.0, 0.0); 
  saw_landmark_polar(0.0, 0.0);

  /*num_items = 6;*/
}


void
straight_line_data()
{
#ifdef change_soon


  robot_position_absolute(35.0, 35.0, 35.0, 35.0);
  saw_no_landmark();
  robot_position_absolute(30.0, 30.0, 30.0, 30.0);
  robot_position_absolute(25.0, 25.0, 25.0, 25.0);
  robot_position_absolute(20.0, 20.0, 20.0, 20.0);
  robot_position_absolute(15.0, 15.0, 15.0, 15.0);
  saw_no_landmark();
  robot_position_absolute(20.0, 20.0, 20.0, 20.0);
  robot_position_absolute(25.0, 25.0, 25.0, 25.0);
  robot_position_absolute(30.0, 30.0, 30.0, 30.0);
  robot_position_absolute(35.0, 35.0, 35.0, 35.0);
  saw_no_landmark();
  robot_position_absolute(30.0, 30.0, 30.0, 30.0);
  robot_position_absolute(25.0, 25.0, 25.0, 25.0);
  robot_position_absolute(20.0, 20.0, 20.0, 20.0);
  robot_position_absolute(15.0, 15.0, 15.0, 15.0);
  saw_landmark_absolute(15.0, 15.0);
  robot_position_absolute(20.0, 20.0, 20.0, 20.0);
  robot_position_absolute(25.0, 25.0, 25.0, 25.0);
  robot_position_absolute(30.0, 30.0, 30.0, 30.0);
  robot_position_absolute(35.0, 35.0, 35.0, 35.0);
  saw_no_landmark();
  robot_position_absolute(30.0, 30.0, 30.0, 30.0);
  robot_position_absolute(25.0, 25.0, 25.0, 25.0);
  robot_position_absolute(20.0, 20.0, 20.0, 20.0);
  robot_position_absolute(15.0, 15.0, 15.0, 15.0);
  saw_landmark_absolute(15.0, 15.0);
#endif
}

void
square_data()
{


  set_initial_position(15.0, 15.0, 180.0, 1);  
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(-90.0, 4.0, 1.0, 0.0); 
  robot_motion_polar(0.0, 4.0, 1.0, 0.0); 
  robot_motion_polar(0.0, 4.0, 1.0, 0.0); 
  robot_motion_polar(0.0, 4.0, 1.0, 0.0); 
  robot_motion_polar(0.0, 4.0, 1.0, 0.0); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(-90.0, 4.0, 1.0, 0.0); 
  robot_motion_polar(0.0, 4.0, 1.0, 0.0); 
  robot_motion_polar(0.0, 4.0, 1.0, 0.0); 
  robot_motion_polar(0.0, 4.0, 1.0, 0.0); 
  robot_motion_polar(0.0, 4.0, 1.0, 0.0); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(-90.0, 4.0, 1.0, 0.0); 
  robot_motion_polar(0.0, 4.0, 1.0, 0.0); 
  robot_motion_polar(0.0, 4.0, 1.0, 0.0); 
  robot_motion_polar(0.0, 4.0, 1.0, 0.0); 
  robot_motion_polar(0.0, 4.0, 1.0, 0.0); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(-90.0, 4.0, 1.0, 0.0); 
  robot_motion_polar(0.0, 4.0, 1.0, 0.0); 
  robot_motion_polar(0.0, 4.0, 1.0, 0.0); 
  robot_motion_polar(0.0, 4.0, 1.0, 0.0); 
  robot_motion_polar(0.0, 4.0, 1.0, 0.0); 
  saw_landmark_polar(0.0, 0.0);


}


void
square_data_short()
{


  set_initial_position(15.0, 15.0, 180.0, 1);  
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(-90.0, 20.0, 5.0, 0.0); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(-90.0, 20.0, 5.0, 0.0); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(-90.0, 20.0, 5.0, 0.0); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(-90.0, 20.0, 5.0, 0.0); 
  saw_landmark_polar(0.0, 0.0);


}

void
back_and_forth_data()
{

  set_initial_position(5.0, 15.0, 180.0, 1);  
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(180.0, 5.0, 0.5, 0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, 0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, 0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, 0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, 0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, 0.4); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(180.0, 5.0, 0.5, -0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, -0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, -0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, -0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, -0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, -0.4); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(180.0, 5.0, 0.5, 0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, 0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, 0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, 0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, 0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, 0.4); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(180.0, 5.0, 0.5, -0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, -0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, -0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, -0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, -0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, -0.4); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(180.0, 5.0, 0.5, 0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, 0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, 0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, 0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, 0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, 0.4); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(180.0, 5.0, 0.5, -0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, -0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, -0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, -0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, -0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, -0.4); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(180.0, 5.0, 0.5, 0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, 0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, 0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, 0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, 0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, 0.4); 
  saw_landmark_polar(0.0, 0.0);
  robot_motion_polar(180.0, 5.0, 0.5, -0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, -0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, -0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, -0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, -0.4); 
  robot_motion_polar(0.0, 5.0, 0.5, -0.4); 
  saw_landmark_polar(0.0, 0.0);

}

/*
  z={{33040.9, 31051.9}, {33785.2, 31439.5}, {34382.6, 30414.5},
  {35459.5, 30995}, {34827.1, 32002.9}, {33780.2, 31321.6}, {34386.6,
  30303.6}, {35473.5, 30860.1}, {34909, 31840.1}, {33795.2, 33356.6},
  {32151.8, 34781.2}, {30260.7, 35937}, {29214.7, 34043.9}, {30528.4,
  33319.6}, {33046.9, 31795.2}, {34459.5, 30750.2}, {35036.9, 31360.6},
  {35826.1, 32319.6}, {34447.5, 33519.4}, {32587.4, 34645.3}, {30563.4,
  35544.4}, {29780.2, 33567.4}, {31135.8, 32984}, {33876.1, 31940},
  {35513.4, 31292.7}, {35875.1, 32037.9}, {35516.5, 31287.9}, {33856.1,
  31874.1}}
  ListPlot[z,PlotJoined->True,AspectRatio->Automatic]
  
  x={33040.9, 33785.2, 34382.6, 35459.5, 34827.1, 33780.2, 34386.6,
  35473.5, 34909, 33795.2, 32151.8, 30260.7, 29214.7, 30528.4, 33046.9,
  34459.5, 35036.9, 35826.1, 34447.5, 32587.4, 30563.4, 29780.2,
  31135.8, 33876.1, 35513.4, 35875.1, 35516.5, 33856.1};
  
  y={31051.9, 31439.5, 30414.5, 30995, 32002.9, 31321.6, 30303.6,
  30860.1, 31840.1, 33356.6, 34781.2, 35937, 34043.9, 33319.6, 31795.2,
  30750.2, 31360.6, 32319.6, 33519.4, 34645.3, 35544.4, 33567.4, 32984,
  31940, 31292.7, 32037.9, 31287.9, 31874.1};
  
  o={-60.1172, 29.8828, -61.5234, 27.0703, 126.211, -147.305, -59.4141,
  23.9062, 124.102, 133.945, 151.523, -60.4688, -123.75, -28.8281,
  -33.0469, -30.5859, 46.7578, 49.2188, 154.336, 160.523, -111.445,
  -115.664, -25.6641, -13.7109, -19.3359, 66.7969, -107.227, 66.7969};
  
  Min[x]=29214.7
  Min[y]=30303.6
*/



