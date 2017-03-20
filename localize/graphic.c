
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/graphic.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:39 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: graphic.c,v $
 * Revision 1.1  2002/09/14 20:45:39  rstone
 * *** empty log message ***
 *
 * Revision 1.192  2000/10/25 18:56:15  fox
 * At least human selection (MODE 2) works with the samples again.
 *
 * Revision 1.191  2000/10/21 01:39:06  fox
 * Changed a parameter in the positioning tool.
 *
 * Revision 1.190  2000/08/09 23:40:45  wolfram
 * Fixed the pain problem with MIN_WINDOW_SCALE
 *
 * Revision 1.189  2000/03/06 20:00:44  wolfram
 * Changed the code of the vision routines.
 *
 * Revision 1.188  1999/12/28 16:17:24  fox
 * Fixed a bug.
 *
 * Revision 1.187  1999/12/16 16:13:59  fox
 * Several preparation changes for angles.
 *
 * Revision 1.186  1999/11/02 18:12:34  fox
 * Version of multi localize journal paper.
 *
 * Revision 1.185  1999/10/21 17:30:43  fox
 * Fixed some bugs in real time scripts.
 *
 * Revision 1.184  1999/09/29 16:06:10  fox
 * Should work.
 *
 * Revision 1.183  1999/09/26 21:20:57  fox
 * Nothing special.
 *
 * Revision 1.182  1999/08/30 05:48:42  fox
 * Doesn't work!!
 *
 * Revision 1.181  1999/08/27 22:22:32  fox
 * Better communication for multi robot localization.
 *
 * Revision 1.180  1999/07/12 15:24:53  fox
 * Minor changes in Texas.
 *
 * Revision 1.179  1999/06/25 19:48:12  fox
 * Minor changs for the urbie.
 *
 * Revision 1.178  1999/06/24 00:21:50  fox
 * Some changes for the urbies.
 *
 * Revision 1.177  1999/06/23 16:22:01  fox
 * Added robot type urban.
 *
 * Revision 1.176  1999/06/03 12:52:54  wolfram
 * Corrected sonar geometry for B21
 *
 * Revision 1.175  1999/05/18 15:15:19  fox
 * Added keywords to determine whether correction parameters should be sent to map and plan or not.
 *
 * Revision 1.174  1999/04/29 17:24:37  fox
 * Removed the EDIT_MAP stuff.
 *
 * Revision 1.173  1999/04/29 17:21:37  fox
 * Added feature to edit a map when setting SET_ROBOT_POSITION. At each
 * mouse press the corresponding pixel will be set to free space. When done,
 * the map is stored in testmap. Define EDIT_MAP.
 *
 * Revision 1.172  1999/04/26 18:55:38  fox
 * Communication with sampling seems to work (no more stuck situations).
 *
 * Revision 1.171  1999/03/19 16:04:39  wolfram
 * Added REAL_TIME simulation in script.c
 *
 * Revision 1.170  1999/03/12 00:41:49  fox
 * Minor changes.
 *
 * Revision 1.169  1999/03/10 15:30:34  schulz
 * Added message for querying samples
 *
 * Revision 1.168  1999/03/08 16:47:40  wolfram
 * Fixed problems with map, caused by the fact that it no longer accpets
 * correction parameters from LOCALIZE.  Improved the handling of sonars
 * with Pioneers.  Currently sonars do not work properly on a B21.
 *
 * Revision 1.167  1999/03/01 17:44:29  wolfram
 * Added support for Pioneer II.  Pioneer I will be added next
 *
 * Revision 1.166  1999/02/17 19:42:23  fox
 * Enhanced gif utilities.
 *
 * Revision 1.165  1999/02/05 23:02:42  fox
 * Minor changes for samples.
 *
 * Revision 1.164  1999/02/01 21:52:22  fox
 * Added support for dumping gif files.
 *
 * Revision 1.163  1999/02/01 15:47:00  wolfram
 * Small change so that robot window no longer flickers
 *
 * Revision 1.162  1999/01/29 09:31:43  wolfram
 * Small changes
 *
 * Revision 1.161  1999/01/22 18:10:40  fox
 * Removed some hacks done for denver and the sampling paper.
 *
 * Revision 1.160  1999/01/22 17:48:04  fox
 * Final version of sampling paper and denver demo.
 *
 * Revision 1.159  1999/01/14 00:33:00  wolfram
 * Changes for vision
 *
 * Revision 1.158  1999/01/11 19:47:49  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.156  1999/01/08 22:28:46  wolfram
 * Better integration of scanAlignment
 *
 * Revision 1.155  1999/01/07 01:07:07  wolfram
 * Changes to integrate scan matching
 *
 * Revision 1.153  1998/12/10 17:56:06  fox
 * Fixed a bug in displayPositions.
 *
 * Revision 1.152  1998/11/27 09:12:02  wolfram
 * Some changes for Frank
 *
 * Revision 1.151  1998/11/25 16:29:23  wolfram
 * Added higher resolution for vision maps. resolution is now read from file.
 * Couldn't integrate it consistently into graphic.c.
 *
 * Revision 1.150  1998/11/25 08:27:33  wolfram
 * Added slower display for overlay running on every XServer
 *
 * Revision 1.149  1998/11/24 23:05:26  fox
 * Implemented furhter routines for condensation and vision.
 *
 * Revision 1.148  1998/11/24 18:42:11  fox
 * First version of condensation with vision.
 *
 * Revision 1.147  1998/11/24 15:31:08  wolfram
 * Fixed a bug, added samples to vision
 *
 * Revision 1.146  1998/11/23 21:19:25  fox
 * Fixed some minor bugs.
 *
 * Revision 1.145  1998/11/23 19:45:07  fox
 * Latest version.
 *
 * Revision 1.144  1998/11/19 03:14:25  fox
 * Newest version with integrated mapping and localization. If MAP is connected,
 * then the correction parameters are NOT sent to PLAN.
 * Furthermore, the size of the onlinemap adapts to the partial maps sent by MAP.
 *
 * Revision 1.143  1998/11/17 23:26:19  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.142  1998/11/16 08:35:26  wolfram
 * Cleaned up vision.c. It seems to work give appropriate maps!
 *
 * Revision 1.141  1998/10/29 03:45:00  fox
 * Nothing special.
 *
 * Revision 1.140  1998/10/27 23:18:38  wolfram
 * Added bw display to robotDump as well as numbering of images
 *
 * Revision 1.139  1998/10/26 22:16:17  wolfram
 * Added logScale option for plotting
 *
 * Revision 1.138  1998/10/19 18:29:55  fox
 * *** empty log message ***
 *
 * Revision 1.137  1998/10/02 15:16:38  fox
 * Several improvements of condensation. Updated the function calls to the
 * new version of the distancServer.
 *
 * Revision 1.136  1998/09/25 17:53:30  fox
 * Improved version of condensation.
 *
 * Revision 1.135  1998/09/25 04:02:55  fox
 * First version of CONDENSATION!!!!
 *
 * Revision 1.134  1998/09/18 15:44:26  fox
 * First attempt to get a running system after museum.
 *
 * Revision 1.133  1998/09/05 22:06:56  wolfram
 * Changes regarding vision!
 *
 * Revision 1.132  1998/08/31 22:29:20  wolfram
 * Several changes
 *
 * Revision 1.131  1998/08/23 00:01:00  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.130  1998/08/20 00:22:58  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.129  1998/08/19 16:33:55  fox
 * Minor changes. Position is dumped upon exit of tcxServer.
 *
 * Revision 1.128  1998/08/11 23:05:36  wolfram
 * Changes for the Smithsonian museum.
 *
 * WARNING: LOCALIZE now uses only one stddev for Lasers. Laser beams have 25 m
 * Maxrange now.
 * Please use ~wolfram/bee/data/localize/params/laser-0.1stdev with this version
 * of LOCALIZE
 *
 * Revision 1.127  1998/06/12 10:16:30  fox
 * Implemented virutal sensor.
 *
 * Revision 1.126  1998/03/02 06:35:23  wolfram
 * Added output of sensor errror values. Small change in graphic.c
 *
 * Revision 1.125  1998/02/13 14:12:20  fox
 * Minor changes.
 *
 * Revision 1.124  1998/01/22 13:06:15  fox
 * First version after selection-submission.
 *
 * Revision 1.123  1998/01/15 00:47:17  thrun
 * New: correction parameters are now sent to MAP, if MAP exists.
 * MAP broadcasts those then on to other modules.
 *
 * Revision 1.122  1997/12/02 15:20:37  fox
 * Nothing remarkable.
 *
 * Revision 1.121  1997/11/28 14:11:50  fox
 * Minor changes.
 *
 * Revision 1.120  1997/11/28 13:34:35  fox
 * Added questions.
 *
 * Revision 1.119  1997/11/27 18:11:18  fox
 * Several changes to make angles work better.
 *
 * Revision 1.118  1997/11/25 17:12:49  fox
 * Should work.
 *
 * Revision 1.117  1997/11/23 16:30:37  wolfram
 * RobotPlot.log is not dumped by default
 *
 * Revision 1.116  1997/11/23 15:50:18  wolfram
 * Changes because of robotDump
 *
 * Revision 1.115  1997/11/23 15:29:59  fox
 * Changed dumpRobotWindow.
 *
 * Revision 1.114  1997/11/21 15:36:03  fox
 * Modifications in graphic
 *
 * Revision 1.113  1997/11/20 17:10:14  wolfram
 * Fixed some accesses to not initialized memory using purify
 *
 * Revision 1.112  1997/11/20 12:58:10  fox
 * Version with good sensor selection.
 *
 * Revision 1.111  1997/11/18 16:23:54  wolfram
 * Changes because of robotDump
 *
 * Revision 1.110  1997/11/18 08:38:06  wolfram
 * Added more information to the robot window dump
 *
 * Revision 1.109  1997/11/13 10:17:30  wolfram
 * Add procedures to dump the robot window
 *
 * Revision 1.108  1997/11/12 12:55:45  wolfram
 * Expected distances now have fixed width
 *
 * Revision 1.107  1997/11/07 12:39:39  fox
 * Added some graphic features.
 *
 * Revision 1.106  1997/10/31 13:11:40  fox
 * Version for active sensing.
 *
 * Revision 1.105  1997/10/28 12:34:18  wolfram
 * Removed unused variables and parameters
 *
 * Revision 1.104  1997/10/06 16:34:35  wolfram
 * Added logScale option to displayXYMAX
 *
 * Revision 1.103  1997/10/06 13:22:43  rhino
 * *** empty log message ***
 *
 * Revision 1.102  1997/10/05 19:42:44  wolfram
 * Speed-up for displayXYMaxWindow
 *
 * Revision 1.101  1997/10/01 11:29:58  fox
 * Minor changes.
 *
 * Revision 1.100  1997/09/30 13:38:51  wolfram
 * Integration of angles is stopped after integration of any proximity sensor
 *
 * Revision 1.99  1997/09/30 10:40:43  wolfram
 * Fixed a bug in initialization of graphic.c
 *
 * Revision 1.98  1997/09/29 10:45:23  wolfram
 * Added default values for all values in .ini file
 *
 * Revision 1.97  1997/09/26 17:02:09  wolfram
 * Intermediate version, do not use
 *
 * Revision 1.96  1997/09/25 16:37:17  wolfram
 * Fixed a bug in setMaxFactor
 *
 * Revision 1.95  1997/09/21 08:03:45  wolfram
 * Fixed bugs in graphic.c
 *
 * Revision 1.94  1997/09/09 19:45:12  fox
 * Fixed a bug in finding local maxima and did several changes to active
 * localization.
 *
 * Revision 1.93  1997/08/22 04:16:38  fox
 * Final version before IJCAI.
 *
 * Revision 1.92  1997/08/19 22:19:08  wolfram
 * Fixed a bug in display of robot window and in planeOfRotation
 *
 * Revision 1.91  1997/08/19 18:24:05  wolfram
 * Fixed a bug in computeWeightedPositionAndSum
 *
 * Revision 1.90  1997/08/18 19:34:19  wolfram
 * Small changes in graphic.c and proximityTools.c
 *
 * Revision 1.89  1997/08/16 22:59:49  fox
 * Last version before I change selsection.
 *
 * Revision 1.88  1997/08/12 01:55:37  wolfram
 * Better Computation of the cubeSum in displayXYMAX
 *
 * Revision 1.87  1997/08/10 14:40:23  wolfram
 * Small change in fieldColor(...)
 *
 * Revision 1.86  1997/08/09 18:23:31  wolfram
 * Improved computation of angles (aligned readings are weighted
 * according to their distance to the robot.
 *
 * Revision 1.85  1997/08/08 19:52:12  wolfram
 * Robot window now displays the simulator map as seen from the lasers.
 * Simulator map can be displayed at a certain ZRange.
 *
 * Revision 1.84  1997/08/02 17:38:39  wolfram
 * Windows are cleared by drawing a white rectangle
 *
 * Revision 1.83  1997/08/02 16:51:03  wolfram
 * 1. Changed the order of indexes of the grid (and only of the grid):
 * The order now is grid->prob[rot][x][y].  This results in a significant
 * speed-up for different operations such as normalization and
 * integration of new sensory data.  Reimplemented the ConvolveThirdDim
 * procedure for convolving over rot.
 *
 * 2. Changed the algorithm to detect linear alignments of readings.  Now
 * we use the approach of Lu.
 *
 * 3. Linear alignments of readings is also checked for laser readings.
 *
 * 4. Expected distances are now computed given the simulator map if
 * available.  For that purpose the library libGetDistance is included.
 *
 * 5. Graphic output now displays the simulator map (if available). This
 * concernes the map overlay as well as the robot window.
 *
 * 6. Fixed some minor bugs.
 *
 * 7. Added different parameters to the ini-file (see example.ini).
 *
 * Revision 1.82  1997/07/05 15:37:34  fox
 * Fixed a bug.
 *
 * Revision 1.81  1997/07/04 17:29:13  fox
 * Final version before holiday!!!
 *
 * Revision 1.80  1997/06/27 16:26:26  fox
 * New model of the proximity sensors.
 *
 * Revision 1.79  1997/06/25 14:16:38  fox
 * Changed laser incorporation.
 *
 * Revision 1.78  1997/06/20 07:36:09  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.77  1997/05/26 08:47:48  fox
 * Last version before major changes.
 *
 * Revision 1.76  1997/05/11 19:08:07  wolfram
 * Faster normalization, slight changes in graphic.c
 *
 * Revision 1.75  1997/05/05 15:43:10  wolfram
 * Map window can be ommited now
 *
 * Revision 1.74  1997/04/30 12:25:40  fox
 * Some minor changes.
 *
 * Revision 1.73  1997/04/11 06:16:51  wolfram
 * Display of the robot in the robot window has been corrected
 *
 * Revision 1.72  1997/04/08 14:56:23  fox
 * Changed the reset function for new planes and the prob function.
 *
 * Revision 1.71  1997/04/08 09:47:54  wolfram
 * DisplaySensings now uses the sensing offset to display expected distances
 *
 * Revision 1.70  1997/04/07 11:03:21  fox
 * Should be ok.
 *
 * Revision 1.69  1997/04/03 13:17:50  fox
 * Some minor changes.
 *
 * Revision 1.68  1997/04/03 10:25:18  wolfram
 * Selected sensings now are displayed in the same colors as all sensings
 * of the same sensor type
 *
 * Revision 1.67  1997/04/02 08:57:32  fox
 * Display the robot right after sensor selection.
 *
 * Revision 1.66  1997/03/24 21:51:42  wolfram
 * Minor changes
 *
 * Revision 1.65  1997/03/24 06:55:28  wolfram
 * Cleaned up graphic.c and added a graphic window as a global variable
 * in graphic.c
 *
 * Revision 1.64  1997/03/18 18:45:29  fox
 * First executable version with a global scan mask for different sensor types.
 *
 * Revision 1.63  1997/03/17 18:41:13  fox
 * First version with sensor selection in cluttered environments.
 *
 * Revision 1.62  1997/03/14 17:58:18  fox
 * This version should run quite stable now.
 *
 * Revision 1.61  1997/03/14 11:20:54  wolfram
 * Added windows for different maps
 *
 * Revision 1.60  1997/03/13 19:09:25  wolfram
 * Lasers are now correctly displayed
 *
 * Revision 1.59  1997/03/13 17:36:20  fox
 * Temporary version. Don't use!
 *
 * Revision 1.58  1997/03/03 12:59:21  wolfram
 * Initial position probabilities are computed out of the simulator map if
 * the simulator map is available
 *
 * Revision 1.57  1997/02/11 11:04:09  wolfram
 * Faster Computation of expected distances, height of sensors is considered
 *
 * Revision 1.56  1997/02/11 10:09:31  fox
 * No comment.
 *
 * Revision 1.55  1997/01/31 16:19:16  wolfram
 * Added start position to each reading of a scan
 *
 * Revision 1.54  1997/01/30 17:17:24  fox
 * New version with integrated laser.
 *
 * Revision 1.53  1997/01/30 10:50:03  fox
 * Minor changes.
 *
 * Revision 1.52  1997/01/29 12:23:07  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.51  1997/01/27 14:23:18  fox
 * Xfig format can be dumped.
 *
 * Revision 1.50  1997/01/23 14:07:22  fox
 * Version of ijcai-submission.
 *
 * Revision 1.49  1997/01/20 13:09:38  fox
 * Unbelievable, that this position estimation ever worked ..........
 *
 * Revision 1.48  1997/01/19 23:35:15  fox
 * Fixed a bug in convolve.
 *
 * Revision 1.47  1997/01/19 19:31:16  fox
 * yeah
 *
 * Revision 1.46  1997/01/18 14:07:54  fox
 * Test version.
 *
 * Revision 1.45  1997/01/17 13:21:06  fox
 * Added timer for non relevant functions.
 *
 * Revision 1.44  1997/01/16 12:42:49  fox
 * Fixed a bug in scan masks.
 *
 * Revision 1.43  1997/01/14 16:53:22  fox
 * Removed a very very nasty bug for sensor selection.
 *
 * Revision 1.42  1997/01/14 10:38:20  wolfram
 * Test Version
 *
 * Revision 1.41  1997/01/09 08:27:17  wolfram
 * Fixed a bug in displayRobotWindow
 *
 * Revision 1.40  1997/01/08 15:52:56  fox
 * Rotations are not any longer treated like movements.
 * Send parameters to PLAN.
 *
 * Revision 1.39  1997/01/07 13:14:28  wolfram
 * Movement decisions are made on the basis of a list of real positions
 *
 * Revision 1.38  1997/01/03 13:52:22  wolfram
 * Fixed a bug in update of robot and angle window
 *
 * Revision 1.37  1996/12/20 15:29:38  fox
 * Added four parameters.
 *
 * Revision 1.36  1996/12/19 14:33:28  wolfram
 * More sensitive computation of expected distance tables
 *
 * Revision 1.35  1996/12/09 10:12:00  wolfram
 * Fixed a bug in angle computation
 *
 * Revision 1.34  1996/12/05 08:32:27  wolfram
 * Fixed a bug in showAllColors
 *
 * Revision 1.33  1996/12/04 13:08:44  wolfram
 * Robot position is now correct in the robot window
 *
 * Revision 1.32  1996/12/03 15:40:25  fox
 * ok
 *
 * Revision 1.31  1996/12/03 12:27:40  fox
 * Seems to work with the thin corridor.
 *
 * Revision 1.30  1996/12/02 18:46:26  fox
 * First version with the new expected distances.
 *
 * Revision 1.29  1996/12/02 14:18:25  fox
 * Fixed a bug in proximityDecision.c
 *
 * Revision 1.28  1996/12/02 13:27:10  wolfram
 * better dist prob functions
 *
 * Revision 1.27  1996/12/02 10:25:35  wolfram
 * Added support for standard deviation of expected distances
 *
 * Revision 1.26  1996/11/29 15:33:38  fox
 * ok
 *
 * Revision 1.25  1996/11/29 08:55:57  wolfram
 * *** empty log message ***
 *
 * Revision 1.24  1996/11/28 17:56:22  fox
 * *** empty log message ***
 *
 * Revision 1.23  1996/11/27 15:51:17  fox
 * For Woflram.
 *
 * Revision 1.22  1996/11/27 12:40:25  wolfram
 * Displaying robots in movement field
 *
 * Revision 1.21  1996/11/27 12:18:17  fox
 * Nothing special.
 *
 * Revision 1.20  1996/11/27 10:47:17  wolfram
 * Added Function to display error map
 *
 * Revision 1.19  1996/11/27 08:33:50  wolfram
 * Better display of movement errors
 *
 * Revision 1.18  1996/11/26 16:24:24  wolfram
 * *** empty log message ***
 *
 * Revision 1.17  1996/11/26 16:08:25  fox
 * Nothing special.
 *
 * Revision 1.16  1996/11/26 14:00:46  wolfram
 * position of grid window is now computet correctly given a real position
 *
 * Revision 1.15  1996/11/26 11:08:12  fox
 * Improved version.
 *
 * Revision 1.14  1996/11/25 19:35:40  fox
 * Test version for decisions of movements.
 *
 * Revision 1.13  1996/11/25 09:47:24  wolfram
 * Added graphic support for movment errors
 * Improved procedure for computing the entropy of a grid
 *
 * Revision 1.12  1996/11/22 10:30:24  wolfram
 * Added graphic for expected errors of movements
 *
 * Revision 1.11  1996/11/21 12:40:25  fox
 * Tools for bayesian reasoning on the grids.
 *
 * Revision 1.10  1996/11/18 09:58:30  ws
 * Added support for selecting the optimal sensor
 * Changed some variables to improve compatibility with C++
 *
 * Revision 1.9  1996/11/15 17:44:06  ws
 * *** empty log message ***
 *
 * Revision 1.8  1996/10/25 13:05:22  ws
 * Added MIN_WINDOW_SCALE keyword for map and grid windows
 *
 * Revision 1.7  1996/10/25 12:07:35  fox
 * Changed size of the windows.
 *
 * Revision 1.6  1996/10/25 07:28:46  fox
 * Improved dump of graphic output.
 *
 * Revision 1.5  1996/10/24 14:16:48  ws
 * fixed wrong position of robot zoom window
 *
 * Revision 1.4  1996/10/24 12:07:10  fox
 * Fixed a bug.
 *
 * Revision 1.3  1996/10/24 09:56:53  fox
 * LOCALIZE also works in a write protected directory.
 *
 * Revision 1.2  1996/09/24 16:29:37  wolfram
 * Fixed a problem with the new Makefile
 *
 * Revision 1.1.1.1  1996/09/22 16:46:31  rhino
 * General reorganization of the directories/repository, fusion with the
 * RWI software.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <stdio.h>
#include <gd.h>

#include "graphic.h"
#include "general.h"
#include "probGrid.h"
#include "function.h"
#include "map.h"
#include "script.h"
#include "file.h"
#include "sonar.h"
#include "laser.h"
#include "proximityTools.h"
#include "communication.h"
#include "allocate.h"
#include "condensation.h"
#include "vision.h"

#define USE_GRAPHIC_TOKEN         0
#define START_POS_TOKEN           1
#define START_POS_MAP_TOKEN       2
#define START_POS_SCRIPT_TOKEN    3
#define SET_ROBOT_POSITION_TOKEN  4
#define SHOW_ROBOT_ZOOM_TOKEN     5
#define SHOW_MAP_TOKEN            6
#define SHOW_ANGLES_TOKEN         7
#define CREATE_MAP_OVERLAY_TOKEN  8
#define MIN_WINDOW_SCALE_TOKEN    9 
#define SHOW_SONAR_MAP_TOKEN      10
#define SHOW_LASER_MAP_TOKEN      11
#define SHOW_INITIAL_POSITION_PROBS_TOKEN 12
#define SHOW_SELECTED_SENSINGS_TOKEN    13
#define ROBOT_ZOOM_SCALE_TOKEN    14
#define DISPLAY_SKIP_TOKEN        15
#define DUMP_XY_GRAPHIC_TOKEN     16
#define DUMP_ROBOT_WINDOW_TOKEN   17
#define SHOW_EXPECTED_DISTANCES_TOKEN   18


/**************************************************************************
**************************************************************************
* Local variables and defines.
 **************************************************************************
 **************************************************************************/

/* #define LOGSCALE 1 */


#define MYMAXCOLORS     155
#define COL_BACK        0                  /* background of window       */
#define COL_UNKNOWN     C_LAWNGREEN           /* color of unknown area      */
#define COL_RED         C_RED

#ifdef MANYCOLORS
#define COL_BLACK       54
#define COL_WHITE       154
#else
#define COL_BLACK      C_GREY0 
#define COL_WHITE      C_GREY100
#endif

/* #define FADED_MAP */
#ifdef FADED_MAP
#undef COL_BLACK
#define COL_BLACK      C_GREY70
#endif

#define COL_ROBOT       C_YELLOW
#define WALL_LENGTH     100  
#define WALL_COLOR      C_YELLOW

#define FILL_ROBOT 1
#define DONT_FILL_ROBOT 0

typedef struct{
  int x;
  int y;
} windowPosition;


/* this is a global variable allowing the update of the robot Window
   from arbitrary points of the program */

static struct{
  bool initialized;
  informationsFor_GRAPHIC* graphicInfo;
  actionInformation* actionInfo;
  sensingActionMask* actionMask;
  bool showExpectedDistances;
} globalGraphicInfo;

/********************************************************************
 ********************************************************************
 * Forward declarations of internal functions.
 ********************************************************************
 ********************************************************************/

static int
cm2Pixel(gridWindow* win, float cm);
 
int
fieldColor(mapProbability f);

static int
gridWindowScale( int sizeX, int sizeY, int minScale);

static float
positionWindowScale( float sizeX, float sizeY, float maxCmPerPixel);

static windowPosition
windowPositionOfRealPosition(gridWindow* win, realPosition pos);

static windowPosition
robotWindowPosition(gridWindow* robwin, windowPosition center,
		    realPosition pos);

void
displayOverlay( probabilityGrid *map, gridWindow *mapwin);

static realPosition
realPositionOfWindowPosition(gridWindow* win, windowPosition winPos);

void
displayRealPosition( gridWindow* win,
		     float radius,
		     realPosition pos);

static void
displaySampleMean( positionWindow* win,
		   float radius, realPosition pos, int color);

gridWindow *
createPlaneWindow(positionProbabilityGrid *grid, char* text, int x, int y);

void
displayPlaneWindow(gridWindow *win, positionProbabilityGrid *grid);

void
drawSimulatorMapZRange( gridWindow *win,
			simulatorMap *simMap,
			int color,
			int deltaX,
			int deltaY,
			float fromZ,
			float toZ);

void
dumpRobotWindow( robot rob,
		 actionInformation* actionInfo,
		 sensingActionMask* actionMask);


static void
dumpPositionWindow( sampleSet* samples,
		    probabilityGrid *map,
		    visionMap* visMap,
		    float distanceTraveled);


/**************************************************************************
 **************************************************************************
 * Mandatory functions.
 **************************************************************************
 **************************************************************************/

void
initialize_GRAPHICS( char* fileName,
		     actionInformation* actionInfo,
		     sensingActionMask* actionMask,
		     informationsFor_GRAPHIC* graphicInfo)
{
  char positionFile[MAX_STRING_LENGTH], mapFile[MAX_STRING_LENGTH],
    scriptFile[MAX_STRING_LENGTH];
  int setRobotPosition;
  extern mapParameters globalMapParameters;
  extern informationsFor_COMMUNICATION communicationInfo;


  /*-------------------------------------------------------------------------
   * Get the parameters from the file. 
   *------------------------------------------------------------------------*/
  token tok[NUMBER_OF_GRAPHIC_PARAMETERS];
  
  /* Initialize global graphic info */
  globalGraphicInfo.initialized = FALSE;
  
  /*-------------------------------------------------------------------------
   * Initialize with default values
   *------------------------------------------------------------------------*/

  graphicInfo->useGraphic = TRUE;
  sprintf(positionFile, "%s", "../startpos.dat");
  graphicInfo->showMap = FALSE;
  graphicInfo->showSonarMap = FALSE;
  graphicInfo->showLaserMap = FALSE;
  graphicInfo->showInitialPositionProbs = FALSE;
  graphicInfo->showRobotZoom = TRUE;
  graphicInfo->showAngles = FALSE;
  graphicInfo->createMapOverlay = TRUE;
  graphicInfo->minWindowScale = 4;
  graphicInfo->showSelectedSensings = TRUE;
  graphicInfo->robotZoomScale = 4;
  graphicInfo->displaySkip = 1;
  graphicInfo->dumpXYMax = 0;
  graphicInfo->dumpRobWin = 0;

  globalGraphicInfo.showExpectedDistances = FALSE;

  setRobotPosition = FALSE;  
  sprintf(mapFile, "%s.map", globalMapParameters.mapFileName);
  if (!communicationInfo.useTcx)
    strcpy( scriptFile, communicationInfo.scr->fileName);
  
  setTokensInitialized(tok, NUMBER_OF_GRAPHIC_PARAMETERS);

  /*-------------------------------------------------------------------------
   * Now get the values
   *------------------------------------------------------------------------*/

  tok[USE_GRAPHIC_TOKEN].format   = INT_FORMAT;
  tok[USE_GRAPHIC_TOKEN].variable = &(graphicInfo->useGraphic);
  tok[USE_GRAPHIC_TOKEN].keyWord  = USE_GRAPHIC_KEYWORD;

  tok[START_POS_TOKEN].format   = STRING_FORMAT;
  tok[START_POS_TOKEN].variable = positionFile;
  tok[START_POS_TOKEN].keyWord  = START_POS_KEYWORD;

  tok[START_POS_MAP_TOKEN].format   = STRING_FORMAT;
  tok[START_POS_MAP_TOKEN].variable = mapFile;
  tok[START_POS_MAP_TOKEN].keyWord  = MAP_KEYWORD;

  tok[START_POS_SCRIPT_TOKEN].format   = STRING_FORMAT;
  tok[START_POS_SCRIPT_TOKEN].variable = scriptFile;
  tok[START_POS_SCRIPT_TOKEN].keyWord  = SCRIPT_KEYWORD;

  tok[SET_ROBOT_POSITION_TOKEN].format   = INT_FORMAT;
  tok[SET_ROBOT_POSITION_TOKEN].variable = &setRobotPosition;
  tok[SET_ROBOT_POSITION_TOKEN].keyWord  = SET_ROBOT_POSITION_KEYWORD;

  tok[SHOW_ROBOT_ZOOM_TOKEN].format   = INT_FORMAT;
  tok[SHOW_ROBOT_ZOOM_TOKEN].variable = &(graphicInfo->showRobotZoom);
  tok[SHOW_ROBOT_ZOOM_TOKEN].keyWord  = SHOW_ROBOT_ZOOM_KEYWORD;

  tok[SHOW_SELECTED_SENSINGS_TOKEN].format   = INT_FORMAT;
  tok[SHOW_SELECTED_SENSINGS_TOKEN].variable =
    &(graphicInfo->showSelectedSensings);
  tok[SHOW_SELECTED_SENSINGS_TOKEN].keyWord  = SHOW_SELECTED_SENSINGS_KEYWORD;

  tok[SHOW_MAP_TOKEN].format   = INT_FORMAT;
  tok[SHOW_MAP_TOKEN].variable = &(graphicInfo->showMap);
  tok[SHOW_MAP_TOKEN].keyWord  = SHOW_MAP_KEYWORD;

  tok[SHOW_SONAR_MAP_TOKEN].format   = INT_FORMAT;
  tok[SHOW_SONAR_MAP_TOKEN].variable = &(graphicInfo->showSonarMap);
  tok[SHOW_SONAR_MAP_TOKEN].keyWord  = SHOW_SONAR_MAP_KEYWORD;

  tok[SHOW_LASER_MAP_TOKEN].format   = INT_FORMAT;
  tok[SHOW_LASER_MAP_TOKEN].variable = &(graphicInfo->showLaserMap);
  tok[SHOW_LASER_MAP_TOKEN].keyWord  = SHOW_LASER_MAP_KEYWORD;

  tok[SHOW_INITIAL_POSITION_PROBS_TOKEN].format   = INT_FORMAT;
  tok[SHOW_INITIAL_POSITION_PROBS_TOKEN].variable =
     &(graphicInfo->showInitialPositionProbs);
  tok[SHOW_INITIAL_POSITION_PROBS_TOKEN].keyWord  =
     SHOW_INITIAL_POSITION_PROBS_KEYWORD;

  tok[SHOW_ANGLES_TOKEN].format   = INT_FORMAT;
  tok[SHOW_ANGLES_TOKEN].variable = &(graphicInfo->showAngles);
  tok[SHOW_ANGLES_TOKEN].keyWord  = SHOW_ANGLES_KEYWORD;

  tok[CREATE_MAP_OVERLAY_TOKEN].format   = INT_FORMAT;
  tok[CREATE_MAP_OVERLAY_TOKEN].variable = &(graphicInfo->createMapOverlay);
  tok[CREATE_MAP_OVERLAY_TOKEN].keyWord  = CREATE_MAP_OVERLAY_KEYWORD;

  tok[MIN_WINDOW_SCALE_TOKEN].format   = INT_FORMAT;
  tok[MIN_WINDOW_SCALE_TOKEN].variable = &(graphicInfo->minWindowScale);
  tok[MIN_WINDOW_SCALE_TOKEN].keyWord  = MIN_WINDOW_SCALE_KEYWORD;

  tok[ROBOT_ZOOM_SCALE_TOKEN].format   = INT_FORMAT;
  tok[ROBOT_ZOOM_SCALE_TOKEN].variable = &(graphicInfo->robotZoomScale);
  tok[ROBOT_ZOOM_SCALE_TOKEN].keyWord  = ROBOT_ZOOM_SCALE_KEYWORD;

  tok[DISPLAY_SKIP_TOKEN].format   = INT_FORMAT;
  tok[DISPLAY_SKIP_TOKEN].variable = &(graphicInfo->displaySkip);
  tok[DISPLAY_SKIP_TOKEN].keyWord  = DISPLAY_SKIP_KEYWORD;

  tok[DUMP_XY_GRAPHIC_TOKEN].format   = INT_FORMAT;
  tok[DUMP_XY_GRAPHIC_TOKEN].variable = &(graphicInfo->dumpXYMax);
  tok[DUMP_XY_GRAPHIC_TOKEN].keyWord  = DUMP_XY_GRAPHIC_KEYWORD;

  tok[DUMP_ROBOT_WINDOW_TOKEN].format   = INT_FORMAT;
  tok[DUMP_ROBOT_WINDOW_TOKEN].variable = &(graphicInfo->dumpRobWin);
  tok[DUMP_ROBOT_WINDOW_TOKEN].keyWord  = DUMP_ROBOT_WINDOW_KEYWORD;

  tok[SHOW_EXPECTED_DISTANCES_TOKEN].format   = INT_FORMAT;
  tok[SHOW_EXPECTED_DISTANCES_TOKEN].variable = &(globalGraphicInfo.showExpectedDistances);
  tok[SHOW_EXPECTED_DISTANCES_TOKEN].keyWord  = SHOW_EXPECTED_DISTANCES_KEYWORD;

  readTokens( fileName, tok, NUMBER_OF_GRAPHIC_PARAMETERS, FALSE); 

  /*-------------------------------------------------------------------------
   * done.
   *------------------------------------------------------------------------*/

  /* If the start position is known we display it in the map. */
  graphicInfo->robotInMap.radius = ROB_RADIUS;
  if (graphicInfo->displaySkip <= 0)
    graphicInfo->displaySkip = 1;

  if ( tok[START_POS_TOKEN].initialized) {
    if ( ! tok[START_POS_MAP_TOKEN].initialized
	 || ! tok[START_POS_SCRIPT_TOKEN].initialized) {
      fprintf( stderr, "Error: map- and script file needed to get the start position.\n");
      closeLogAndExit(1);
    }
    else {
      graphicInfo->robotInMap.pos.x = graphicInfo->robotInMap.pos.y =
	graphicInfo->robotInMap.pos.rot = 0.0;
      
      /* If the position is not found in the file all coordinates are set to zero. */
      if ( graphicInfo->robotInMap.pos.x == 0.0
	   && graphicInfo->robotInMap.pos.y == 0.0
	   && graphicInfo->robotInMap.pos.rot == 0.0)
	graphicInfo->showRobotInMap = FALSE;
      else
	graphicInfo->showRobotInMap = TRUE;
    }
  }
  else {
    graphicInfo->showRobotInMap = FALSE;
  }
  
  /* set the global graphic information structure */
  globalGraphicInfo.graphicInfo = graphicInfo;
  globalGraphicInfo.actionInfo = actionInfo;
  globalGraphicInfo.actionMask = actionMask; 
  globalGraphicInfo.initialized = TRUE;
    
    
  if ( graphicInfo->useGraphic) {

    int mapXOffset = 0;
    int mapYOffset = 0;
    
    if (1) putc( 7, stderr);
    fprintf(stderr, "# Hit <Return> to continue!\n");
    getchar(); 
	
    setTimer(9);
	     
    /* Displays the map. */
    if ( graphicInfo->showMap) {
      graphicInfo->mapWindow =
	createMapWindow( &(actionInfo->map), "Occupancy grid", 0, 0,
			 graphicInfo->minWindowScale);
      mapXOffset += graphicInfo->mapWindow->sizeX + 2;
      mapYOffset += graphicInfo->mapWindow->sizeY + 8;
    }
    else
      graphicInfo->mapWindow = NULL;

    /* Displays the sonar map. */
    if ( actionMask->use[SONAR] && graphicInfo->showSonarMap) {
      graphicInfo->sonarMapWindow =
	createMapWindow( &(actionInfo->sonarMap), "Sonar Map", 0, 0,
			 graphicInfo->minWindowScale);
    }
    else
      graphicInfo->sonarMapWindow = NULL;

    if ((actionMask->use[VISION] && actionInfo->visMap.initialized)){
      if (graphicInfo->showMap)
	graphicInfo->visionMapWindow =
	  createVisionMapWindow( &(actionInfo->visMap), "Vision Map",
				 0, 0,
				 graphicInfo->minWindowScale);
      if (graphicInfo->showMap)
	displayVisionMapWindow(&(actionInfo->visMap),
			       graphicInfo->visionMapWindow);
      if (graphicInfo->showMap && actionInfo->map.initialized)
	displayOverlay(&(actionInfo->map),
		       graphicInfo->visionMapWindow);
    }
    
    /* Displays the laser map. */
    if ( actionMask->use[LASER] && graphicInfo->showLaserMap) {
      graphicInfo->laserMapWindow =
	createMapWindow( &(actionInfo->laserMap), "Laser Map", 0, mapYOffset,
			 graphicInfo->minWindowScale);
    }
    else
      graphicInfo->laserMapWindow = NULL;
    
    
    /* Displays the initial positionProbs. */
    if ( actionMask->use[MOVEMENT] && graphicInfo->showInitialPositionProbs) {
      graphicInfo->initialPositionProbsWindow =
	createMapWindow( &(actionInfo->initialPositionProbs),
			 "Initial Position Probs", 0, mapYOffset,
			 graphicInfo->minWindowScale);
      
      mapYOffset += graphicInfo->initialPositionProbsWindow->sizeY + 8;
    }
    else
      graphicInfo->initialPositionProbsWindow = NULL;
    

    /* Displays the maximal probabilities for all angles of the estimated
     * positions of the robot.  */
    
    if ( actionInfo->useProbGrid) {
      graphicInfo->xyMaxWindow =
	createGridWindow( &(actionInfo->positionProbs),
			  XYPLANE,
			  "Maximum Probabilities",
			  0,
			  mapYOffset,
			  graphicInfo->minWindowScale);
      if ( ! graphicInfo->showMap) {
	mapXOffset += graphicInfo->xyMaxWindow->sizeX + 2;
	mapYOffset += graphicInfo->xyMaxWindow->sizeY + 8;
      }
    }
    else if ( graphicInfo->createMapOverlay) {
      char* title = actionInfo->robotName ? actionInfo->robotName : "Samples";
      graphicInfo->sampleWindow =
	createPositionWindow( &(actionInfo->samples),
			      title,
			      0,
			      mapYOffset,
			      graphicInfo->minWindowScale);
      
      if ( ! graphicInfo->showMap) {
	mapXOffset += graphicInfo->sampleWindow->sizeX + 2;
	mapYOffset += graphicInfo->sampleWindow->sizeY + 8;
      }
      /* #define DUMP_GIFS */
#ifdef DUMP_GIFS
      {
	if ( actionMask->use[LASER]) {
	  if ( actionMask->use[SONAR])
	    dumpPositionWindow( &(actionInfo->samples),
				&(actionInfo->map),
				&(actionInfo->visMap),
				0.0);
	  else
	    dumpPositionWindow( &(actionInfo->samples),
				&(actionInfo->laserMap),
				&(actionInfo->visMap),
				0.0);
	}
	else
	    dumpPositionWindow( &(actionInfo->samples),
				&(actionInfo->sonarMap),
				&(actionInfo->visMap),
				0.0);
      }
#endif
    }
    

    /* Shows the robot and the sensor beams. */
    if ( graphicInfo->showRobotZoom) {
      char* title = actionInfo->robotName ? actionInfo->robotName : "Robot window";
      graphicInfo->robotWindow =
	createRobotWindow( &(actionInfo->map), title,
			   2000,
			   mapXOffset+100,
			   0,
			   graphicInfo->robotZoomScale);
    }
    else
      graphicInfo->robotWindow = NULL;


    if (0) graphicInfo->planeWindow =
	     createPlaneWindow( &(actionInfo->positionProbs), "Plane window",
			 400, 400);

    /* Show the map. */
    if ( graphicInfo->showMap)
      displayMapWindow( &(actionInfo->map),
			graphicInfo->mapWindow);
    
    /* Show the sonar map. */
    if ( actionMask->use[SONAR] && graphicInfo->showSonarMap)
      displayMapWindow( &(actionInfo->sonarMap),
			graphicInfo->sonarMapWindow);
    
    /* Show the laser map. */
    if ( actionMask->use[LASER] && graphicInfo->showLaserMap)
      displayMapWindow( &(actionInfo->laserMap),
			graphicInfo->laserMapWindow);
    
    /* Show the initial position probs. */
    if ( actionMask->use[MOVEMENT] && graphicInfo->showInitialPositionProbs)
      displayMapWindow( &(actionInfo->initialPositionProbs),
			graphicInfo->initialPositionProbsWindow);


    
    /* Set the initial position of the robot if desired. */
    if ( tok[SET_ROBOT_POSITION_TOKEN].initialized && setRobotPosition) {

      realPosition robPos;
      
      /* Suggest an initial robot position. First try to read it from a file
       * dumped after tcx stopped. */
      if ( ! readStartPosition( &robPos)) {

	if ( actionInfo->useProbGrid) {
	  
	  robPos = positionRobotinWindow( SET_ROBOT_POSITION_TOKEN,
					  *(graphicInfo->xyMaxWindow),
					  &(actionInfo->map),
					  &(actionInfo->simMap),
					  graphicInfo->robotInMap,
					  mapXOffset,
					  0, TRUE);	  
	  
	  /* Display the robot. */
	  if ( graphicInfo->showMap) {
	    graphicInfo->robotInMap.pos = robPos;
	    graphicInfo->showRobotInMap = TRUE;
	  }
	  else
	    graphicInfo->showRobotInMap = FALSE;
	  
	  
	  robPos.x += actionInfo->positionProbs.offsetX;
	  robPos.y += actionInfo->positionProbs.offsetY;
	
	  if (setRobotPosition == 1)
	    setPosition( robPos,
			 &(actionInfo->positionProbs),
			 1.0);
	  else if (setRobotPosition == 2)
	    setPositionUniform( robPos,
				&(actionInfo->positionProbs),
				1.0);
	  
	}
	else {

	  /* We need a grid window. */	  
	  gridWindow* posWin;

	  posWin = createMapWindow( &(actionInfo->map),
				    "Positioning Tool", 0, mapYOffset,
				    actionInfo->map.resolution); 
	  
	  
	  robPos = positionRobotinWindow( SET_ROBOT_POSITION_TOKEN,
					  *posWin,
					  &(actionInfo->map),
					  &(actionInfo->simMap),
					  graphicInfo->robotInMap,
					  mapXOffset,
					  0, TRUE);	  
	  
	  EZX_EndWindow(posWin->window);
	  free( posWin);
	  
	  /* Display the robot. */
	  if ( graphicInfo->showMap) {
	    graphicInfo->robotInMap.pos = robPos;
	    graphicInfo->showRobotInMap = TRUE;
	  }
	  else
	    graphicInfo->showRobotInMap = FALSE;
	  
	  
	  robPos.x += actionInfo->positionProbs.offsetX;
	  robPos.y += actionInfo->positionProbs.offsetY;
	  
	  setSamplePosition( robPos,
			     &(actionInfo->samples),
			     40.0);
	  
	}
      }
    }

    /* Show the robot in the map. */
    if ( graphicInfo->showRobotInMap && graphicInfo->showMap)	

      displayRobot( graphicInfo->mapWindow, graphicInfo->robotInMap, COL_ROBOT, FILL_ROBOT);

    /* Display for each xy-position the best angle. */
    if ( actionInfo->useProbGrid) {
      displayXYMax( &(actionInfo->positionProbs),
		    &(actionInfo->map),
		    &(actionInfo->simMap),
		    graphicInfo->xyMaxWindow,
		    graphicInfo->createMapOverlay,
		    actionInfo->actualSensings.distanceTraveled);

      displayRealCellList( graphicInfo->xyMaxWindow,
			 &(actionInfo->localMaxima));
    }
    else {
      if ( graphicInfo->createMapOverlay) {
	probabilityGrid *map = &(actionInfo->map);
	if ( actionMask->use[LASER] && ! actionMask->use[SONAR])
	  map = &(actionInfo->laserMap);
	else if ( ! actionMask->use[LASER] && actionMask->use[SONAR])
	  map = &(actionInfo->sonarMap);
	displayPositions( &(actionInfo->samples),
			  map,
			  &(actionInfo->visMap),
			  graphicInfo->sampleWindow,
			  graphicInfo->createMapOverlay,
			  actionInfo->actualSensings.distanceTraveled);
      }
    }
    
    /* If the angles are used we show the histogram. */
    if ( actionMask->use[ANGLE]) {
      
      informationsFor_ANGLE* angleInfo =
	(informationsFor_ANGLE*) actionInfo->info[ANGLE];
      
      graphicInfo->angleProbs = &(angleInfo->angleProbs);
      if ( graphicInfo->showAngles)
	graphicInfo->angleWindow =
	  createAngleWindow( graphicInfo->angleProbs, "Angle histogram",
			     400, 480);
      else
	graphicInfo->angleWindow = NULL;      
    }
    else
      graphicInfo->angleProbs = NULL;

  }
}

/********************************************************************
 ********************************************************************
 * Global functions.
 ********************************************************************
 ********************************************************************/
void
accumulateMovements_GRAPHICS( movement delta,
			      informationsFor_GRAPHIC* graphicInfo)
{
  if ( graphicInfo->useGraphic && delta.isNew)
    graphicInfo->robotInMap.pos = endPoint( graphicInfo->robotInMap.pos,
					    delta);
}

/* Display the new information. */
void
update_SENSING_GRAPHICS(  actionInformation* actionInfo,
			  sensingActionMask* mask,
			  informationsFor_GRAPHIC* graphicInfo)
{
  /* We don't want to count the time needed for graphical  output. */
  setTimer( GRAPHICS_TIMER);

  if ( graphicInfo->useGraphic) {
    
    if ( actionInfo->useProbGrid) {
      
      /* Get the grid position of the estimated robot. */
      
      /* We only display the robot if it has moved. */
      if ( actionInfo->actualSensings.noMoveCnt == 0 ||
	   (actionInfo->useProbGrid && mask->normalizeGrid)) {
	
	/* Display the robot in the map and in the grid. */
	if ( graphicInfo->showRobotInMap) {
	  
	  if ( graphicInfo->showMap)
	    displayRobot( graphicInfo->mapWindow,
			  graphicInfo->robotInMap, COL_ROBOT, FILL_ROBOT);
	  
	}
	
	/* Show the estimated position of the robot. */
	displayRealPosition( graphicInfo->xyMaxWindow,
			     actionInfo->estimatedRobot.radius,
			     actionInfo->estimatedRobot.pos);
	displayRealCellList(  graphicInfo->xyMaxWindow,
			      &(actionInfo->localMaxima));
	
	if (0) displayPlaneWindow( graphicInfo->planeWindow,
				   &(actionInfo->positionProbs));
	
	/* Show the estimated position of the robot. */
	if ( graphicInfo->showMap)
	  displayRealPosition( graphicInfo->mapWindow,
			       actionInfo->estimatedRobot.radius,
			       actionInfo->estimatedRobot.pos);
	
      }
      /* Show the angles if they have changed. */
      if ( mask->perform[ANGLE][UPDATE_ANGLE_GRID] && graphicInfo->showAngles
	   && mask->consider[ANGLE])
	displayAngleWindow( graphicInfo->angleProbs, graphicInfo->angleWindow, 0.0);
      
    }
    else {
      /* Show the estimated position of the robot. */
       displaySampleMean( graphicInfo->sampleWindow,
			 actionInfo->estimatedRobot.radius,
 			 actionInfo->estimatedRobot.pos,
			 C_LAWNGREEN);
    }
  }

  nonRelevantTime += timeExpired( GRAPHICS_TIMER);
}



void
update_GRAPHICS( actionInformation* actionInfo,
		 sensingActionMask* mask,
		 informationsFor_GRAPHIC* graphicInfo)
{
  static int skipCnt =0;
  
  if ( (actionInfo->useProbGrid && mask->normalizeGrid) ||
       (! actionInfo->useProbGrid && mask->perform[MOVEMENT][INTEGRATE_MOVEMENT])) {
    if ( actionInfo->useProbGrid && graphicInfo->dumpXYMax)
      dumpXYMaxGraphic( &(actionInfo->positionProbs),
			&(actionInfo->map),
			actionInfo->estimatedRobot.pos,
			&(actionInfo->localMaxima));
  }
  

  if ( graphicInfo->useGraphic) {
    
    if ( (actionInfo->useProbGrid && mask->normalizeGrid) ||
	 (! actionInfo->useProbGrid && mask->perform[MOVEMENT][INTEGRATE_MOVEMENT])) {

      if ( (skipCnt++ % graphicInfo->displaySkip) == 0) {

	float secs;
	
	setTimer( GRAPHICS_TIMER);
	
	writeLog( "# Graphics output ... ");
	
	/* Refresh the map. */
	if ( graphicInfo->showMap)
	  displayMapWindow( &(actionInfo->map),
			    graphicInfo->mapWindow);
	
	if (0) displayPlaneWindow( graphicInfo->planeWindow,
				   &(actionInfo->positionProbs));
	
	/* Display for each xy-position the best angle. */
	if ( actionInfo->useProbGrid) {
	  displayXYMax( &(actionInfo->positionProbs),
			&(actionInfo->map),
			&(actionInfo->simMap),
			graphicInfo->xyMaxWindow,
			graphicInfo->createMapOverlay,
			actionInfo->actualSensings.distanceTraveled);
	  
	  displayRealCellList( graphicInfo->xyMaxWindow,
			       &(actionInfo->localMaxima));
	}
	else 
	  if ( graphicInfo->createMapOverlay)
	    if (0) displayPositions( &(actionInfo->samples),
				     &(actionInfo->map),
				     &(actionInfo->visMap),
				     graphicInfo->sampleWindow,
				     graphicInfo->createMapOverlay,
				     actionInfo->actualSensings.distanceTraveled);
	
	update_SENSING_GRAPHICS( actionInfo, mask, graphicInfo);
	
	secs = timeExpired( GRAPHICS_TIMER); 
	nonRelevantTime += secs;
	writeLog( "done in %f secs.\n", secs);
      }
    }
  }
}

void
checkPosition( actionInformation* actionInfo, sensingActionMask* mask)
{
  static int firstTime = TRUE;
  static float lastDistanceDump = 0.0;
  informationsFor_GRAPHIC* graphicInfo = globalGraphicInfo.graphicInfo;
  char line[MAX_STRING_LENGTH];
  int quality;
  static FILE* fp;

#define MIN_METER_DIST 8.0
      
  if ( graphicInfo->useGraphic) {
    
    if ( actionInfo->actualSensings.distanceTraveled - lastDistanceDump
	 > MIN_METER_DIST) {
      
      setTimer( GRAPHICS_TIMER);  
      
      if ( firstTime) {
	fp = fopen( "marker.log","w");
	firstTime = FALSE;
	
	if (fp == NULL) {
	  fprintf(stderr,"# Warning: Could not open file 'marker.log'!\n");
	  getchar();
	  exit(0);
	}
      }
      
      if ( graphicInfo->dumpRobWin) 
	
	dumpRobotWindow( actionInfo->estimatedRobot,
			 actionInfo,
			 mask);
      
      if (globalGraphicInfo.initialized
	  && graphicInfo->showRobotZoom
	  && graphicInfo->robotWindow != NULL) {
	
	displayRobotWindow( graphicInfo->robotWindow,
			    actionInfo->estimatedRobot,
			    actionInfo,
			    mask,
			    graphicInfo);

	putc(7, stderr);
	
	fgets(line, MAX_STRING_LENGTH, stdin);
	
	if ( ! sscanf( line, "%d", &quality))
	  quality = 0;
	
	fprintf( fp, "%f %f %f %f %f #MARKER%d\n", elapsedScriptTime,
		 actionInfo->estimatedRobot.pos.x, 
		 actionInfo->estimatedRobot.pos.y, 
		 rad2Deg(actionInfo->estimatedRobot.pos.rot),
		 actionInfo->actualSensings.distanceTraveled,
		 quality);
	fflush( fp);
      }
      
      if ( quality == 1)
	lastDistanceDump = actionInfo->actualSensings.distanceTraveled;
      nonRelevantTime += timeExpired( GRAPHICS_TIMER);
    }
  }
}

gridWindow *
createPlaneWindow(positionProbabilityGrid *grid, char* text, int x, int y) {
  /* creates a window under EZX depending on map_size_x, map_size_y and
   scale (size of a pixel) */

    char corner[80];
    gridWindow *win;
    
    win = (gridWindow *) malloc(sizeof(gridWindow));
    win->scale = 2;

    win->gridSizeX = grid->sizeZ;
    win->gridSizeY = 1;
    win->gridResolution =  win->gridOffsetX = win->gridOffsetY = 0;
    
    win->sizeX = win->scale * grid->sizeZ;
    win->sizeY = 5 * win->scale;

    win->firstTimePosition = TRUE;

    sprintf(corner,"+%d+%d",x,y);
    
    EZX_NoMotionEvents();
    win->window = EZX_MakeWindow(text,win->sizeX,win->sizeY,corner);
    EZX_SetWindowBackground (win->window,C_BLACK);
    EZX_Flush();

    win->startX = 0;
    win->startY = 0;
    return(win);
}

void
displayPlaneWindow(gridWindow *win, positionProbabilityGrid *grid)
{
  int i;

  for (i = 0; i < grid->sizeZ; i++){
    if (grid->updatePlane[i])
      EZX_SetColor(C_BLUE);
    else
      EZX_SetColor(C_RED);
    EZX_FillRectangle(win->window,win->startX+i*win->scale,win->startY,
		      win->startX+ (i+1)*win->scale -1,
		      win->startY + 5 * win->scale);
    
  }
  EZX_Flush();
}


void
displayOverlay( probabilityGrid *map, gridWindow *mapwin){
  int x,y;
  float clipThreshold = 0.4;
  float scale = ((float) map->resolution) / mapwin->gridResolution;
  int winX, winY;
  int endX = mapwin->startX + map->sizeX * mapwin->scale;
  int endY = mapwin->startY + map->sizeY * mapwin->scale;
	
  y=map->sizeY;
  EZX_SetColor( C_BLUE);
  for (winY=mapwin->startY; winY <= endY; winY += mapwin->scale){
    y--;
    x=0;
    for (winX=mapwin->startX; winX < endX; winX += mapwin->scale){
      if ( map->prob[x][y] < 0 ||
	   map->prob[x][y] > clipThreshold) {
	EZX_FillRectangle(mapwin->window,winX*scale,winY*scale,
			  mapwin->scale*scale,mapwin->scale*scale);
      }
      
      x++;
    }
  }
} 



gridWindow *
createVisionMapWindow(visionMap *m, char* text, int x, int y, int minScale) {
  probabilityGrid grid;

  grid.sizeX = m->sizeX;
  grid.sizeY = m->sizeY;
  grid.resolution = m->resolution;
  grid.offsetX = m->offsetX;
  grid.offsetY = m->offsetY;
  if ( globalGraphicInfo.graphicInfo->useGraphic)
    return createMapWindow(&grid, text, x, y, minScale);
  else
    return NULL;
}

gridWindow *
createMapWindow(probabilityGrid *m, char* text, int x, int y, int minScale) {
  /* creates a window under EZX depending on map_size_x, map_size_y and
   scale (size of a pixel) */

  char corner[80];
  gridWindow *mapwin;
  
  mapwin = (gridWindow *) malloc(sizeof(gridWindow));
  mapwin->scale = iMax(1, m->resolution/minScale);
  
  mapwin->gridSizeX = m->sizeX;
  mapwin->gridSizeY = m->sizeY;
  mapwin->gridResolution = m->resolution;
  mapwin->gridOffsetX = round(m->offsetX / m->resolution);
  mapwin->gridOffsetY = round(m->offsetY / m->resolution);

  mapwin->sizeX = m->sizeX * mapwin->scale;
  mapwin->sizeY = m->sizeY * mapwin->scale;

  mapwin->firstTimePosition = TRUE;
  
  sprintf(corner,"+%d+%d",x,y);
  
  EZX_NoMotionEvents();
  mapwin->window = EZX_MakeWindow(text,mapwin->sizeX,mapwin->sizeY,corner);
  EZX_SetWindowBackground (mapwin->window,C_BLACK);
  EZX_Flush();
  
  mapwin->startX = 0;
  mapwin->startY = 0;
  return(mapwin);
}


void
displayMapWindow(probabilityGrid *m, gridWindow *mapwin) {
/* plots map in window */

  int x,y;
  
  int winX, winY;
  int endX = mapwin->startX + m->sizeX * mapwin->scale;
  int endY = mapwin->startY + m->sizeY * mapwin->scale;

  /* Update the offset according to the new map. */
  mapwin->gridOffsetX = round(m->offsetX / m->resolution);
  mapwin->gridOffsetY = round(m->offsetY / m->resolution);
    
  y=m->sizeY;
  for (winY=mapwin->startY; winY <  endY; winY += mapwin->scale){
    y--;
    x=0;
    for (winX=mapwin->startX; winX < endX; winX += mapwin->scale){
	  EZX_SetColor(fieldColor( occupancyProbabilityMap(x,y,*m)));
	  EZX_FillRectangle(mapwin->window,winX,winY,
			    mapwin->scale,mapwin->scale);
	  x++;
    }
  }
  
  EZX_Flush();
  
}

void
displayMapWindowMax(probabilityGrid *m, gridWindow *mapwin) {
/* plots map in window */

  register probability** prob = m->prob;
  int x,y;
  
  int winX, winY;
  int endX = mapwin->startX + m->sizeX * mapwin->scale;
  int endY = mapwin->startY + m->sizeY * mapwin->scale;

  probability max = 0.0;
  
  /*     EZX_SetColor(COL_UNKNOWN); */
  /*     EZX_FillRectangle(mapwin->window,mapwin->startX,mapwin->startY,endX,endY); */

  for (x = 0; x < m->sizeX; x++){
    for (y= 0; y < m->sizeY; y++){
      if (prob[x][y] > max)
	max = prob[x][y];
    }
  }
  
  if (max == 0)
    max = 1.0;
  else
    max = 1.0/max;
  
  y=m->sizeY;
  for (winY=mapwin->startY; winY <= endY; winY += mapwin->scale){
    y--;
    x=0;
    for (winX=mapwin->startX; winX < endX; winX += mapwin->scale){
      EZX_SetColor(fieldColor( occupancyProbabilityMap(x,y,*m) * max));
      EZX_FillRectangle(mapwin->window,winX,winY,
			mapwin->scale,mapwin->scale);
      x++;
	}
  }
  
  EZX_Flush();
  
}


void
displayVisionMapWindow(visionMap *m, gridWindow *mapwin) {
/* plots map in window */

  if ( globalGraphicInfo.graphicInfo->useGraphic) {
    
    int x,y;
  
    int winX, winY;
    int endX = mapwin->startX + m->sizeX * mapwin->scale;
    int endY = mapwin->startY + m->sizeY * mapwin->scale;
  
    y=m->sizeY;
    for (winY=mapwin->startY; winY < endY; winY += mapwin->scale){
      y--;
      x=0;
      for (winX=mapwin->startX; winX < endX; winX += mapwin->scale){
	EZX_SetColor(fieldColor( 1.0 - (m->pix[x][y] / 255.0)));
	EZX_FillRectangle(mapwin->window,winX,winY,
			  mapwin->scale,mapwin->scale);
	x++;
      }
    }

    EZX_Flush();
  }  
}

void
displayVisionMapinPositionWindow(visionMap *visMap) {
/* plots map in window */
  int scale = 6;
  unsigned int x,y;
  int offsetX, offsetY;
  if ( globalGraphicInfo.graphicInfo->useGraphic) {
    positionWindow *posWin = globalGraphicInfo.graphicInfo->sampleWindow;

    offsetX = posWin->sizeX - visMap->sizeX*scale-4;
    offsetY = posWin->sizeY - visMap->sizeY*scale-4;
    
    EZX_SetColor(C_YELLOW);
    EZX_DrawRectangle(posWin->window,offsetX, offsetY,
		      scale*visMap->sizeX,scale*visMap->sizeY);
    
    for (x = 0; x < visMap->sizeX; x++){
      for (y = 0; y < visMap->sizeY; y++){
	EZX_SetColor(fieldColor( 1.0 - (visMap->pix[x][visMap->sizeY - y - 1] / 255.0)));
	EZX_FillRectangle(posWin->window,offsetX+scale*x,offsetY+scale*y, scale,scale);
      }
    }
    
    EZX_Flush();
  }  
}



void
clearMapWindow(gridWindow *win)
{
  EZX_SetColor(C_WHITE);
  EZX_FillRectangle(win->window, 0, 0, win->sizeX, win->sizeY);
}



static int
errorColor(float error, float minError, float maxError)
{
  probability prob;
  if (minError == maxError)
    prob = 0.5;
  else
    prob = (error - minError) / (maxError - minError);

  return fieldColor( prob);
}  



void
displayMovementField( gridWindow *mapwin,
		      probabilityGrid *m,
		      realCellList* cellList)
{
/* plots map in window */

  int x,y;
    
  int winX, winY;
  int endX = mapwin->startX + m->sizeX * mapwin->scale;
  int endY = mapwin->startY + m->sizeY * mapwin->scale;
  int maxX, maxY;

  float minError = 1e20;
  float maxError = -1e20;

  maxX = maxY = 0;
    
  /* compute maximum and minimum for nice scaling */
  for (x = 0; x < m->sizeX; x++) {
    for (y = 0; y < m->sizeY; y++) 
      if ( m->prob[x][y] != m->unknown) {
	if (m->prob[x][y] < minError)
	  minError = m->prob[x][y];
	if (m->prob[x][y] > maxError) {
	  maxX = x;
	  maxY = y;
	  maxError = m->prob[x][y];
	}
      }
  }
  
  /* Set the background. */
  EZX_SetColor(COL_UNKNOWN);
  EZX_FillRectangle(mapwin->window,mapwin->startX,mapwin->startY,endX,endY);

  /* Fill the window. */
  y=m->sizeY;
  for (winY=mapwin->startY; winY <= endY; winY += mapwin->scale){
    y--;
    if ( y > 0) {
      x=0;
      for (winX=mapwin->startX; winX < endX; winX += mapwin->scale){

	if ( m->prob[x][y] != m->unknown) {
	  if ( x == maxX && y == maxY)
	    EZX_SetColor( C_RED);
	  else
	    EZX_SetColor(errorColor(m->prob[x][y], minError, maxError));
	  EZX_FillRectangle(mapwin->window,winX,winY,
			    mapwin->scale,mapwin->scale);
	}
	x++;
      }
    }
  }

  /* Display the possible positions of the robot. */
  {
    robot rob1, rob2;
    float offsetRot;
    int offsetX, offsetY, i;

#define MAX_RADIUS (3.0 * ROB_RADIUS)
#define MIN_RADIUS (0.7 * ROB_RADIUS)

    offsetX   = (m->sizeX / 2 * m->resolution - cellList->cell[0].pos.x);
    offsetY   = (m->sizeY / 2 * m->resolution - cellList->cell[0].pos.y);
    offsetRot = DEG_90 - cellList->cell[0].pos.rot;

    rob1.pos.x   = cellList->cell[0].pos.x + offsetX;
    rob1.pos.y   = cellList->cell[0].pos.y + offsetY;
    rob1.pos.rot = DEG_90;
    rob1.radius  = MAX_RADIUS;
    
    displayRobot( mapwin, rob1, C_RED, DONT_FILL_ROBOT);

    {
      windowPosition pos1;
      
      pos1 = windowPositionOfRealPosition( mapwin, rob1.pos);
      
      EZX_SetColor(C_BLUE);
      EZX_DrawLine(mapwin->window, pos1.x - 3, pos1.y, pos1.x + 3, pos1.y);
      EZX_DrawLine(mapwin->window, pos1.x, pos1.y - 3, pos1.x, pos1.y + 3);
      
      EZX_Flush();
    }
    
    /* Display the other used positions as circles. */
    for ( i = 1; i < cellList->numberOfCells; i++) {
      
      movement move;
      
      rob1.pos.rot = cellList->cell[0].pos.rot;
      
      /* Set the current position into the window position. */
      rob2.pos.x = cellList->cell[i].pos.x + offsetX;
      rob2.pos.y = cellList->cell[i].pos.y + offsetY;
      rob2.pos.rot = cellList->cell[i].pos.rot;

      /* Compute the offest between this point and the reference position. */
      move = movementBetweenPoints( rob1.pos, rob2.pos);

      /* Rotate reference point and compute rotated current point. */
      rob1.pos.rot += offsetRot;
      rob2.pos = endPoint( rob1.pos, move);
      
      rob2.radius = fNorm( cellList->cell[i].prob,
			   cellList->cell[cellList->numberOfCells-1].prob,
			   cellList->cell[0].prob,
			   MIN_RADIUS,
			   MAX_RADIUS);
      
      displayRobot( mapwin, rob2, C_BLUE, DONT_FILL_ROBOT);
    }
  }
  EZX_Flush();
}



void
destroyMapWindow(gridWindow *win)
{
  EZX_EndWindow(win->window);
}


gridWindow *
createGridWindow(positionProbabilityGrid *grid, int planeType, char *title,
		 int x, int y, int minScale)
{
/* creates a window under EZX depending on map_size_x, map_size_y and the chosen type of the plane */

    gridWindow *mapwin;
    char temp[80];
    mapwin = (gridWindow *) malloc(sizeof(gridWindow));

    mapwin->scale = iMax(1, grid->positionResolution/minScale);
    mapwin->sizeX = grid->sizeX * mapwin->scale;
    mapwin->sizeY = grid->sizeY * mapwin->scale;
    mapwin->gridSizeX = grid->sizeX;
    mapwin->gridSizeY = grid->sizeY;
    mapwin->gridOffsetX = round(grid->offsetX / grid->positionResolution);
    mapwin->gridOffsetY = round(grid->offsetY / grid->positionResolution);

    sprintf(temp,"+%d+%d",x,y);
    EZX_NoMotionEvents();
    mapwin->window = EZX_MakeWindow(title,mapwin->sizeX,mapwin->sizeY,temp);
    EZX_SetWindowBackground (mapwin->window,C_BLACK);

    
    mapwin->startX = 0;
    mapwin->startY = 0;
    mapwin->gridResolution = grid->positionResolution;
    mapwin->planeType = planeType;

    return(mapwin);
}

/* creates a window under EZX depending on map_size_x, map_size_y
 * and the chosen type of the plane */
positionWindow *
createPositionWindow( sampleSet* samples, char *title,
		      int x, int y, float maxCmPerPixel)
{
  positionWindow* mapwin;
  char temp[80];
  extern visionParameters globalVisionParameters;
  maxCmPerPixel = fMax(1, maxCmPerPixel);
  
  if ( samples->minX == samples->maxX ||
       samples->minY == samples->maxY) {
    fprintf( stderr, "Error: Cannot display samples with zero size.\n");
    exit(0);
  }
 
  mapwin = (positionWindow *) malloc(sizeof(positionWindow));
  
  mapwin->cmPerPixel = positionWindowScale( samples->maxX - samples->minX,
					    samples->maxY - samples->minY,
					    maxCmPerPixel);
  if (globalVisionParameters.useVision) {
    mapwin->cmPerPixel = globalVisionParameters.resolution;
    fprintf(stderr, "# WARNING: cmPerPixel: %f in graphic.c\n",  (float) mapwin->cmPerPixel);
  }
  
  mapwin->sizeX = (samples->maxX - samples->minX) / mapwin->cmPerPixel;
  mapwin->sizeY = (samples->maxY - samples->minY) / mapwin->cmPerPixel;

  sprintf(temp,"+%d+%d",x,y);
  EZX_NoMotionEvents();
  mapwin->window = EZX_MakeWindow(title, mapwin->sizeX, mapwin->sizeY, temp);
  EZX_SetWindowBackground (mapwin->window,C_BLACK);

  return(mapwin);
}

void
windowStartRectangle(gridWindow *win, gridPosition pos,
		     windowPosition *winPos)
{
  winPos->x = pos.x * win->scale;
  winPos->y = (win->gridSizeY - pos.y - 1) * win->scale;
}


void
displayGridCellList( gridWindow *win, gridCellList *maxima)
{
  int i;
  windowPosition winPos;

  EZX_SetColor(C_LAWNGREEN);
  
  for (i=0; i < maxima->numberOfCells; i++){
    windowStartRectangle(win, maxima->cell[i].pos, &winPos);
    EZX_DrawRectangle(win->window, winPos.x, winPos.y,
		    win->scale-1, win->scale-1);
  }
  EZX_Flush();
}


void
displayRealCellList( gridWindow *win,
		     realCellList *maxima)
{ 
  int i;
  windowPosition winPos, endWinPos;
  movement move;
  realPosition endPos;
  
  move.forward = ROB_RADIUS;
  move.sideward = move.rotation = 0.0;
  EZX_SetColor(C_RED);
  
  for (i=0; i < maxima->numberOfCells; i++){
    winPos = windowPositionOfRealPosition( win, maxima->cell[i].pos);
    endPos = endPoint(maxima->cell[i].pos, move);
    endWinPos = windowPositionOfRealPosition( win, endPos);
    EZX_FillCircle(win->window, winPos.x, winPos.y, 2);
    EZX_DrawLine(win->window, winPos.x, winPos.y, endWinPos.x, endWinPos.y);
  }
  EZX_Flush();
}

void
drawSimulatorMap( gridWindow *win,
		  simulatorMap *simMap,
		  int color,
		  int deltaX,
		  int deltaY){
  drawSimulatorMapZRange( win, simMap, color, deltaX, deltaY, 0.0, 1000.0);
}


void
drawSimulatorMapZRange( gridWindow *win,
			simulatorMap *simMap,
			int color,
			int deltaX,
			int deltaY,
			float fromZ,
			float toZ){

  int i;
  windowPosition pos1, pos2;
  realPosition realPos;
  simulatorObject *object = simMap->object;
  
  realPos.rot = 0;
  EZX_SetColor(color);
  for (i = 0; i < simMap->numberOfObjects; i++, object++){
    if (intersection( object->posZ-object->height * 0.5,
		      object->posZ+object->height * 0.5,
		      fromZ,
		      toZ))
      switch (object->type){
      case CYLINDER:
	realPos.x = object->posX;
	realPos.y = object->posY;
	pos1 = windowPositionOfRealPosition( win, realPos);
	realPos.x += object->width;
	pos2 = windowPositionOfRealPosition( win, realPos);
	EZX_FillCircle(win->window, pos1.x + deltaX,
		       pos1.y + deltaY, pos2.x - pos1.x);
	break;
      case CUBE:
	if (object->rot == 0.0){
	  realPos.x = object->posX - object->width * 0.5;
	  realPos.y = object->posY + object->depth * 0.5;
	  pos1 = windowPositionOfRealPosition( win, realPos);
	  realPos.x += object->width;
	  realPos.y -= object->depth;
	  pos2 = windowPositionOfRealPosition( win, realPos);
	  pos2.x -= pos1.x;
	  if (pos2.x <= 0) pos2.x = 1;
	  pos2.y -= pos1.y;
	  if (pos2.y <= 0) pos2.y = 1;
	  EZX_FillRectangle(win->window, pos1.x+deltaX, pos1.y+deltaY,
			    pos2.x, pos2.y);
	}
	else {
	  float cosRot = cos(object->rot); 
	  float sinRot = sin(object->rot); 
	  XPoint point[4];
	  float dx1 = object->width * cosRot;
	  float dx2 = object->depth * sinRot;
	  float dy1 = object->width * sinRot;
	  float dy2 = object->depth * cosRot;

	  /* center */
	  realPos.x = object->posX + deltaX;
	  realPos.y = object->posY + deltaY;
	  /* first point */
	  realPos.x = object->posX + 0.5 * (dx1 + dx2);
	  realPos.y = object->posY + 0.5 * (dy1 - dy2);
	  pos1 = windowPositionOfRealPosition( win, realPos);
	  point[0].x = (short) round(pos1.x + deltaX);
	  point[0].y = (short) round(pos1.y + deltaY);
	  /* third point */
	  point[2].x = - (short) cm2Pixel(win, dx1);
	  point[2].y = (short) cm2Pixel(win, dy1);
	  /* forth point */
	  point[3].x = (short) cm2Pixel(win, dx2);
	  point[3].y = (short) cm2Pixel(win, dy2);
	  /* second point */
	  point[1].x = - point[3].x;
	  point[1].y = - point[3].y;
	  
	  EZX_FillPolygon(win->window, 4, point);
	}
	break;
      }	  
  }
  EZX_Flush();
}

void
computeLogScaleMap(probability **map, int sizeX, int sizeY,
		   probability min, probability *max){
  
  int x, y;
  
  if (*max > min){
    min = fastLog(min);
    *max = fastLog(*max);
    *max -= min;
    
    for (x = 0; x < sizeX; x++)
      for (y = 0; y < sizeY; y++)
	if (map[x][y] > 0){
	  map[x][y] = (fastLog(map[x][y]) - min) / *max;
	  if (map[x][y] < 0.0)
	    map[x][y] = 0.0;
	}
    *max = 1.0;
  }

}


gridPosition
displayXYMax( positionProbabilityGrid *m,
	      probabilityGrid *map,
	      simulatorMap *simMap,
	      gridWindow *mapwin,
	      bool createMapOverlay,
	      float distanceTraveled)
{
  register int x,y;
  int z;
  probability recmax, max=0.0;
  probability cubeSum = 0.0, dummy;
  gridPosition bestPos;
  realPosition realPos;
  extern probGridParameters globalProbGridParameters;
  register probability *xyMaxP,*p;
  static probability **xyMaxMap;
  static bool firstTime = TRUE;
  float clipThreshold = 0.1;
  
  int winX=0, winY=0;
  int endX, endY;
  
  int maxCnt = 0;

  if (firstTime){
    xyMaxMap = (probability **) allocate2D(m->sizeX, m->sizeY, PROBABILITY);

    if ( createMapOverlay) {
      if (simMap->initialized)
	drawSimulatorMap(mapwin, simMap, C_BLUE, 0, 0);
      else {
	int x,y;
	
	int winX, winY;
	int endX = mapwin->startX + map->sizeX * mapwin->scale;
	int endY = mapwin->startY + map->sizeY * mapwin->scale;
	
	y=map->sizeY;
	EZX_SetColor( C_BLUE);
	for (winY=mapwin->startY; winY <= endY; winY += mapwin->scale){
	  y--;
	x=0;
	for (winX=mapwin->startX; winX < endX; winX += mapwin->scale){
	  if ( map->prob[x][y] < 0 ||
	       map->prob[x][y] > clipThreshold) {
	    EZX_FillRectangle(mapwin->window,winX,winY,
			      mapwin->scale,mapwin->scale);
	  }
	  
	  x++;
	}
	}
      }
    }
    
    firstTime = FALSE;
  }
  
  for (x = 0; x<m->sizeX;x++){
    xyMaxP = xyMaxMap[x];
    for (y = 0; y<m->sizeY;y++)
      *xyMaxP++ = 0.0;
  }
  
  for(z=0;z<m->sizeZ;z++)
    if ( m->updatePlane[z])
      for(x=0;x<m->sizeX;x++){
	xyMaxP = xyMaxMap[x];
	p = m->prob[z][x];
	for(y=0;y<m->sizeY;y++,p++,xyMaxP++){
	  if (*p > *xyMaxP){
	    *xyMaxP = *p;
	    if (*p >= max) {
	      if (*p == max) 
		maxCnt++;
	      else {
		maxCnt = 1;
		max=*p;
		bestPos.x = x;
		bestPos.y = y;
		bestPos.rot = z;
	      }
	    }
	  }
	}
      }
  
  endX = mapwin->startX + m->sizeX * mapwin->scale;
  endY = mapwin->startY + m->sizeY * mapwin->scale;

#ifdef LOGSCALE
  computeLogScaleMap(xyMaxMap,m->sizeX,m->sizeY,m->minimumProbability,&max);
#endif

  if (max == 0.0)
    recmax = 1.0;
  else
    recmax = 1/max;

  

  computeWeightedPositionAndSum( bestPos, m, &realPos,
				 &cubeSum, &dummy,
				 globalProbGridParameters.deltaPosForLocalMax,
				 globalProbGridParameters.deltaRotForLocalMax);
  
  for (winX=mapwin->startX,x=0; winX < endX; winX += mapwin->scale, x++){
    xyMaxP = &(xyMaxMap[x][m->sizeY-1]);
    for (winY=mapwin->startY, y=map->sizeY-1;
	 winY < endY && y >= 0; xyMaxP--, winY += mapwin->scale, y--){
      if (coordinateInMap(x,y, *map))
	if (map->prob[x][y] > 0 && map->prob[x][y] <= clipThreshold){
	  EZX_SetColor( fieldColor((*xyMaxP)*recmax));
	  EZX_FillRectangle(mapwin->window,winX,winY,mapwin->scale,mapwin->scale);
	}
    }
  }
  /* Plot the map above the probabilities. */
  /* Write cube sum and traveled distance. */
  {
    char s[80];
    sprintf(s, "dist: %4.1f  prob: %.5g", distanceTraveled, cubeSum);
    EZX_SetColor(C_RED);
    EZX_SetBackgroundColor(C_WHITE);
    EZX_DrawTextAt(mapwin->window, endX, endY,
		   "                              ", 'r');
    EZX_DrawTextAt(mapwin->window, endX, endY, s, 'r');
  }
  
  EZX_Flush();
  
  return bestPos;
}


void
displaySamples()
{
  if ( globalGraphicInfo.graphicInfo->useGraphic &&
       globalGraphicInfo.graphicInfo->createMapOverlay) {
    
    actionInformation* actionInfo = globalGraphicInfo.actionInfo;
    sensingActionMask* actionMask = globalGraphicInfo.actionMask;
    probabilityGrid *map = &(actionInfo->map);

    if ( actionMask->use[LASER] && ! actionMask->use[SONAR])
      map = &(actionInfo->laserMap);
    else if ( ! actionMask->use[LASER] && actionMask->use[SONAR])
      map = &(actionInfo->sonarMap);
    displayPositions( &(actionInfo->samples),
		      map,
		      &(actionInfo->visMap),
		      globalGraphicInfo.graphicInfo->sampleWindow,
		      globalGraphicInfo.graphicInfo->createMapOverlay,
		      actionInfo->actualSensings.distanceTraveled);
  }
}


void
displayPositionsFast( sampleSet* samples,
		      probabilityGrid *map,
		      visionMap* visMap,
		      positionWindow *mapwin,
		      bool createMapOverlay,
		      float distanceTraveled)
{
  static XImage *ximage = NULL;
  static char   *ximage_buf = NULL;

  int s;

/* #define DUMP_GIFS   */

#ifdef DUMP_GIFS
  dumpPositionWindow( samples, map, visMap, distanceTraveled);
#endif
  /* The area of the samples changes frequently, so adapt scaling and region. */
  if ( globalGraphicInfo.actionInfo->onlineMapping) {

    mapwin->minX = samples->minX;
    mapwin->minY = samples->minY;
    
    if ( createMapOverlay) {
      
      /* Determine the next scale. */
      float cmPerPixelX = (samples->maxX - samples->minX) / mapwin->sizeX;
      float cmPerPixelY = (samples->maxY - samples->minY) / mapwin->sizeY;

      mapwin->cmPerPixel = fMax( cmPerPixelX, cmPerPixelY);
    }
  }

  /*-----------------------------------------------------------------
   * Display a map overlay. 
   *-----------------------------------------------------------------*/
  if ( map->initialized && createMapOverlay) {
    
    if ( ximage_buf == NULL) {
      
#define CLIP_THRESHOLD 0.1
      
      int     x,y,i;
      int     color;
      int     depthFactor = 1;
      int     tlookup;

      int useVisionMap = FALSE;
      float resolution;
      int mapSizeX, mapSizeY;
      
      if ( visMap) {
	if (visMap->initialized) 
	  useVisionMap = TRUE;
      }

      if ( useVisionMap) {
	resolution = visMap->resolution;
	mapSizeX = visMap->sizeX;
	mapSizeY = visMap->sizeY;
      }
      else {
	resolution = map->resolution;
	mapSizeX = map->sizeX;
	mapSizeY = map->sizeY;
      }

      if(theDepth > 8) {
	if(theDepth == 24)
	  depthFactor = 4;
	else
	  depthFactor = theDepth / 8;
      }

      fprintf(stderr, "MapSizeX: %d MapSizeY: %d Depth %d\n", mapSizeX, mapSizeY, depthFactor);
      

      ximage_buf = (char *) malloc( sizeof(char) * depthFactor *
				    mapwin->sizeX * mapwin->sizeY);
      
      if (ximage_buf == NULL) {
	fprintf(stderr, 
		"ERROR: Out of memory (1)\n");
	exit(1);
      }


      
      XSetForeground(theDisplay, theGC, theBlackPixel);
      XSetBackground(theDisplay, theGC, theWhitePixel);

      for ( y = 0; y < mapwin->sizeY; y++) {
	
	int mapY = y * mapwin->cmPerPixel / resolution;
	
	tlookup = (mapwin->sizeY - y - 1)  * mapwin->sizeX * depthFactor;
	
	for ( x = 0; x < mapwin->sizeX; x++) {
	  
	  int mapX = x * mapwin->cmPerPixel / resolution;

	  if ( mapX < 0 || mapY < 0 || mapX >= mapSizeX || mapY >= mapSizeY) {
	    color = C_LAWNGREEN;
	  }
	  else {
	    if ( ! useVisionMap) {

	      if (( map->prob[mapX][mapY] != map->unknown)
		  && map->prob[mapX][mapY] < CLIP_THRESHOLD) {
		color = C_WHITE;
	      }
	      else {
		color = C_BLUE;
	      }
	    }
	    else {
	      color = fieldColor( 1.0 - (visMap->pix[mapX][mapY] / 255.0));
	    }
	  }
	  
	  for(i=0;i<depthFactor;i++)
	    ximage_buf[tlookup+(x*depthFactor+i)] = 
	      thePixels[color] >> (i*8);
	}
      }

      ximage = XCreateImage(theDisplay, theVisual,
			    theDepth, ZPixmap,
			    0, 
			    ximage_buf,
			    mapwin->sizeX, 
			    mapwin->sizeY,
			    8*depthFactor, 
			    mapwin->sizeX*depthFactor);
      
      if (ximage == NULL){
	fprintf(stderr, 
		"ERROR: Out of memory (2)\n");
	exit(1);
      }
      
      ximage->byte_order = XImageByteOrder(theDisplay);
    }

    /* During online mapping we have to update the map display each time. */
    if ( globalGraphicInfo.actionInfo->onlineMapping) {
      
      /* Show the map. */
      XPutImage( theDisplay, mapwin->window->w, theGC, ximage, 
		 0, 0, 
		 0, 0,
		 mapwin->sizeX, mapwin->sizeY);
      
      free( ximage_buf );
      
      ximage_buf = NULL;
      
      ximage->data = NULL;
      
      XDestroyImage( ximage );
    }
  }
  
  /* Show the map. */
  if (! globalGraphicInfo.actionInfo->onlineMapping)
    XPutImage( theDisplay, mapwin->window->w, theGC, ximage, 
	       0, 0, 
	       0, 0,
	       mapwin->sizeX, mapwin->sizeY);
  
#ifdef FADED_MAP
  EZX_SetColor( C_GREY0);
#else
  EZX_SetColor( C_RED);
#endif

  for ( s = 0; s < samples->numberOfSamples; s++) {
    
    realPosition samplePos = samples->sample[s].pos;
    int winPosX = round( (samplePos.x - samples->minX) / mapwin->cmPerPixel);
    int winPosY = mapwin->sizeY - round((samplePos.y - samples->minY) / mapwin->cmPerPixel);
    EZX_DrawPoint(mapwin->window, winPosX, winPosY);
  }  

  {
    char s[80];
    
#define DISPLAY_MEASURED_POS 1
    
    if (DISPLAY_MEASURED_POS){
      realPosition measuredPos;
      extern float elapsedScriptTime;
      bool success;
      
      measuredPos = measuredRobotPosition(elapsedScriptTime, &success);
      
      if (success){
	int winPosX = round( (measuredPos.x - samples->minX) / mapwin->cmPerPixel);
	int winPosY = mapwin->sizeY - round((measuredPos.y - mapwin->minY)
					    / mapwin->cmPerPixel);
	EZX_SetColor(C_YELLOW2);
	EZX_DrawCircle(mapwin->window, winPosX, winPosY, 2);
      }
      {
	realPosition pos = globalGraphicInfo.actionInfo->estimatedRobot.pos;
	sprintf(s, "dist: %4.1f  samples: %d, position: x=%.1f y=%.1f rot=%.1f",
		distanceTraveled, samples->numberOfSamples,
		pos.x *0.01,
		pos.y *0.01,
		rad2Deg(normalizedAngle(pos.rot)));
	EZX_SetColor(C_RED);
	EZX_SetBackgroundColor(C_BLACK);
	EZX_DrawTextAt(mapwin->window, 5, mapwin->sizeY, s, 'l');
      }
    }
    displaySampleMean( mapwin, 30, samples->mean, C_BLACK);
  }
  
  EZX_Flush();
}

void
displayPositionsSlow( sampleSet* samples,
		      probabilityGrid *map,
		      visionMap* visMap,
		      positionWindow *mapwin,
		      bool createMapOverlay,
		      float distanceTraveled)
{

  int s;
  static int cnt=0;
  map = map;
  distanceTraveled = distanceTraveled;
  /* The area of the samples changes frequently, so adapt scaling and region. */
  if ( globalGraphicInfo.actionInfo->onlineMapping) {
    
    mapwin->minX = samples->minX;
    mapwin->minY = samples->minY;
    
    if ( createMapOverlay) {
      
      /* Determine the next scale. */
      float cmPerPixelX = mapwin->sizeX / (samples->maxX - samples->minX);
      float cmPerPixelY = mapwin->sizeY / (samples->maxY - samples->minY);
      
      mapwin->cmPerPixel = fMin( cmPerPixelX, cmPerPixelY);
    }
  }
  
  /*-----------------------------------------------------------------
   * Display a map overlay. 
   *-----------------------------------------------------------------*/
  
  /* Show the map. */
  if (! globalGraphicInfo.actionInfo->onlineMapping){
    gridWindow win;
    
    win.window = mapwin->window;
    win.sizeX = mapwin->sizeX;
    win.sizeY = mapwin->sizeY;
    win.gridSizeX = visMap->sizeX;
    win.gridSizeY = visMap->sizeY;
    win.gridOffsetX = win.gridOffsetY = 0;
    win.scale = 1;
    win.planeType = XYPLANE;
    win.startX = win.startY = 0;
    if (cnt == 0){
      displayVisionMapWindow(visMap, &win);
    }
  }
  
  
  EZX_SetColor( C_RED+((cnt)%6));
  cnt++;
  if (cnt > 100) cnt = 0;
  
  for ( s = 0; s < samples->numberOfSamples; s++) {
    
    realPosition samplePos = samples->sample[s].pos;
    
    int winPosX = round( (samplePos.x - mapwin->minX) / mapwin->cmPerPixel);
    int winPosY = mapwin->sizeY - round((samplePos.y - mapwin->minY) / mapwin->cmPerPixel);
    
    EZX_DrawPoint(mapwin->window, winPosX, winPosY);
  }  


  
  EZX_Flush();
}



#define FAST 1

void
displayPositions( sampleSet* samples,
		  probabilityGrid *map,
		  visionMap* visMap,
		  positionWindow *mapwin,
		  bool createMapOverlay,
		  float distanceTraveled)
{
  if (FAST)
    displayPositionsFast( samples, map, visMap, mapwin, createMapOverlay, distanceTraveled);
  else
    displayPositionsSlow( samples, map, visMap, mapwin, createMapOverlay, distanceTraveled);
}    
    


void
dumpXYMaxGraphic( positionProbabilityGrid *m,
		  probabilityGrid *map,
		  realPosition scriptPos,
		  realCellList* localMaxima)
{
  static int plotCnt = 0;
  char s[80];
  FILE* fp;
  int x,y,z;
  int maxZ;
/*   if ( plotCnt++ < 80) */
/*     return; */

  writeLog( "dump probs no %d\n", plotCnt);
  sprintf(s, "plot.%d", plotCnt);
  fp = fopen( s, "w");

  fprintf( fp, "%d %d %d %g\n", m->sizeX, m->sizeY,
	   m->positionResolution, m->minimumProbability);
  fprintf( fp, "%f %f %f\n", scriptPos.x,
	   scriptPos.y, scriptPos.rot);
  
  fprintf( fp, "%d\n", localMaxima->numberOfCells);
  for ( x = 0; x < localMaxima->numberOfCells; x++)
    fprintf( fp, "%f %f %f %f\n", localMaxima->cell[x].pos.x,
	     localMaxima->cell[x].pos.y, localMaxima->cell[x].pos.rot,
	     localMaxima->cell[x].prob);

  if ( plotCnt < 2000) {
#ifdef LOGSCALE
    probability max = 0.0, min = fastLog(m->minimumProbability);
    
    fprintf(stderr, "# Generating log scale plot!\n");
    for (z = 0; z < m->sizeZ; z++)
      if (m->updatePlane[z])
	for (x = 0; x < m->sizeX; x++)
	  for (y = 0; y < m-> sizeY; y++){
	    if (m->prob[z][x][y] > max)
	      max = m->prob[z][x][y];
	  }
    if (max > min){
      max = fastLog(max) - min;
    }
    else {
      max = 1;
    }

#endif
    for ( y=m->sizeY-1; y >= 0; y--)  {
      for ( x = 0; x < m->sizeX; x++) {
	maxZ = 0;
	for (z = 0; z < m->sizeZ; z++) 
	  if (m->prob[z][x][y] > m->prob[maxZ][x][y]) 
	    maxZ = z;
	/* Now dump the maximal value. */
	if (m->prob[maxZ][x][y] > m->minimumProbability)
#ifdef LOGSCALE
	    fprintf( fp, "%g ", fMax(0.0, fastLog(m->prob[maxZ][x][y]) - min)
		     / max);
#else
	  fprintf( fp, "%g ", m->prob[maxZ][x][y]);
#endif
	else
	  fprintf( fp, "-1.0 ");
      }
      fprintf( fp, "\n");
    }
  }
  
  if ( plotCnt == 0) {
    for ( y=m->sizeY-1; y >= 0; y--)  {
      for ( x = 0; x < m->sizeX; x++) {	
	if ( occupancyProbabilityMap(x,y,*map) > 0.1) 
	  fprintf( fp, "%d ", OVERLAY_VALUE);
	else 
	  fprintf( fp, "%d ", NON_OVERLAY_VALUE);
      }
      fprintf( fp, "\n");
    }
  }
  fclose ( fp);
  plotCnt++;
}
  
void
writeSensings( FILE *fp,
	       realPosition pos,
	       int sensorTag,
	       distanceScan *sensings,
	       abstractSensorVector abstractSensings)
{
  int i,j;
  realPosition sensingStart, sensingEnd;
  movement tmpReading;
  bool selected[MAX_SIZE_OF_SCAN], maxRange;
  
  sensingStart = endPoint( pos, sensings->offset);

  for (i = 0; i < sensings->numberOfReadings; i++)
    selected[i] = FALSE;
  
  for (j = 0; j < abstractSensings.numberOfSensorsToBeUsed; j++) 
    selected[abstractSensings.mask[j]] = TRUE;

  for (i = 0; i < sensings->numberOfReadings; i++){
    sensingStart.rot = normalizedAngle(pos.rot + sensings->reading[i].rot);
    tmpReading.forward = fMin(1000.0, sensings->reading[i].dist);
    tmpReading.sideward = tmpReading.rotation = 0.0;
    sensingEnd = endPoint( sensingStart, tmpReading);

    maxRange = abstractSensings.sensor[i].measuredFeature
      == (abstractSensings.sensor[i].numberOfFeatures -1);
    fprintf(fp, "%f %f %f %f %d %d %d\n",
	    (float) sensingStart.x,
	    (float) sensingStart.y,
	    (float) sensingEnd.x,
	    (float) sensingEnd.y,
	    sensorTag, selected[i], maxRange);
  }
  
}

  

void
dumpRobotWindow( robot rob,
		 actionInformation* actionInfo,
		 sensingActionMask* actionMask)
{
  FILE* fp;
  static bool firstTime = TRUE;
  static char fileName[MAX_STRING_LENGTH];
  realPosition pos = rob.pos;

  /* #define DUMP_SCRIPT_DATA 1 */
  
#ifdef DUMP_SCRIPT_DATA
  pos = measuredRobotPositionInMap();
#endif

  if (( actionMask->use[LASER]  &&
	actionInfo->actualSensings.frontLaser.isNew
	&& actionMask->perform[LASER][INTEGRATE_LASER])
      ||
      ( actionMask->use[LASER] &&
	actionInfo->actualSensings.rearLaser.isNew
	&& actionMask->perform[LASER][INTEGRATE_LASER])
	||
	( actionMask->use[SONAR] &&
	  actionInfo->actualSensings.sonar.isNew
	  && actionMask->perform[SONAR][INTEGRATE_SONAR])
      ) {
    
    if (firstTime){
      sprintf( fileName, "%s.plot", actionInfo->logFileName);
      fp = fopen( fileName, "w");
      firstTime = FALSE;
    }
    else
      fp = fopen( fileName, "a");
    
    if (fp == NULL)
      return;
    
    fprintf( fp, "%f %f %f %f\n", pos.x, pos.y, pos.rot,
	     elapsedScriptTime);
    
    if ( actionMask->use[LASER]  &&
	 actionInfo->actualSensings.frontLaser.isNew
	 && actionMask->perform[LASER][INTEGRATE_LASER]){
      writeSensings( fp, pos, LASER_TAG,
		     &(actionInfo->actualSensings.frontLaser),
		     actionInfo->abstractSensors[ABSTRACT_FRONT_LASER]);
    }
    
    if (  actionMask->use[LASER] &&
	  actionInfo->actualSensings.rearLaser.isNew
	  && actionMask->perform[LASER][INTEGRATE_LASER]){
      writeSensings( fp, pos, LASER_TAG,
		     &(actionInfo->actualSensings.rearLaser),
		     actionInfo->abstractSensors[ABSTRACT_REAR_LASER]);
    }
    if (  actionMask->use[SONAR] &&
	  actionInfo->actualSensings.sonar.isNew
	  && actionMask->perform[SONAR][INTEGRATE_SONAR]){
      writeSensings( fp, pos, SONAR_TAG,
		     &(actionInfo->actualSensings.sonar),
		   actionInfo->abstractSensors[ABSTRACT_SONAR]);
    }
    fprintf(fp, "\n");
    
    fclose ( fp);
  }
}


void
displayRealPosition( gridWindow* win,
		     float radius,
		     realPosition pos)
{
  windowPosition winPos;
  float rot = pos.rot;
  int x2,y2;
  
  /*  pos.x -= 0.5 * grid->positionResolution;
  pos.y -= 0.5 * grid->positionResolution; */
  
  winPos = windowPositionOfRealPosition( win, pos);
  
  x2 = round(winPos.x + (radius/win->gridResolution * win->scale) * cos(rot));
  y2 = round(winPos.y - (radius/win->gridResolution * win->scale) * sin(rot));
  
  EZX_SetColor(C_RED);  
  EZX_DrawCircle(win->window, winPos.x, winPos.y,
		 radius / win->gridResolution * win->scale );
  EZX_DrawLine(win->window, winPos.x, winPos.y, x2, y2);
  EZX_Flush();
}


void
displayGridPosition( gridWindow* win,
		     positionProbabilityGrid* grid,
		     float radius,
		     gridPosition pos)
{
  displayRealPosition( win, radius,
		       realPositionOfGridPosition(pos, grid));
}



#define COL_ROBOT      C_YELLOW
#define COL_ROBODIR    C_BLACK
#define COL_BUTTONBACK C_RED
#define COL_BUTTONTEXT C_YELLOW 
#define COL_TEXT       C_YELLOW
#define COL_RECTANGLE  C_RED
#define MYWINDOW_X     88
#define MYWINDOW_Y     136
#define COL_BACKGROUND C_BLUE

realPosition
positionRobotinWindow(int mode, gridWindow mapwindow,
		      probabilityGrid *map,
		      simulatorMap *simMap,
		      robot rob, int x, int y,
		      int destroyPositionTool) {
  /* Parameters:
   * mode=SET_ROBOT_POSITION: set robot to single location and get orientation
   *   else: select two coordinates as the area of interest and get orientation
   * gridWindow: structure of window to be used to modify position in
   */
  
  int xr, yr, function = 0; 
  int x1 = 0, y1 = 0, rot = 0;
  char corner[80];
  char data[10];

  EZX_EventType theEvent;
  struct timeval block_waiting_time = { 999,0 };
  static EZXW_p mywindow = NULL;
  windowPosition winPos;
  
  if (simMap->initialized){
    clearMapWindow( &mapwindow);
    drawSimulatorMap( &mapwindow, simMap, C_BLACK, 0, 0);
  }
  else{
    displayMapWindow( map, &mapwindow);
  }
      
  sprintf(corner,"+%d+%d",x,y);
  
  /* Open own window to rotate robot */
  if ( mywindow == NULL) {
    mywindow = EZX_MakeWindow("Place Robot", MYWINDOW_X, MYWINDOW_Y, corner);
    EZX_SetColor(COL_BACKGROUND);
    EZX_FillRectangle(mywindow, 0,0, MYWINDOW_X, MYWINDOW_Y);
    /* plot window, gadgets */
    EZX_SetColor(COL_BUTTONBACK);
    EZX_FillRectangle(mywindow, 5, MYWINDOW_Y-24, MYWINDOW_X-10, 19);
    EZX_FillRectangle(mywindow, 5, MYWINDOW_Y-48, 19, 19);
    EZX_FillRectangle(mywindow, MYWINDOW_X-24, MYWINDOW_Y-48, 19, 19);
    EZX_SetColor(COL_BUTTONTEXT);
    EZX_SetBackgroundColor(COL_BUTTONBACK);
    EZX_DrawText(mywindow, 13, MYWINDOW_Y-9,"D O N E");
    EZX_DrawText(mywindow, 9, MYWINDOW_Y-33,"<");
    EZX_DrawText(mywindow, MYWINDOW_X-18, MYWINDOW_Y-33,">");
  }
  
  winPos.x = 0;
  winPos.y = 0;
  
  do {
    
    /* plot BIG robot */
    EZX_SetColor(COL_ROBOT);
    EZX_FillCircle(mywindow, MYWINDOW_X/2, MYWINDOW_X/2, MYWINDOW_X/2-5);
    EZX_SetColor(COL_ROBODIR);
    xr = MYWINDOW_X/2 + cos(rot * M_PI/180.0) * (MYWINDOW_X/2-5.5) - 0.5;
    yr = MYWINDOW_X/2 - sin(rot * M_PI/180.0) * (MYWINDOW_X/2-5.5) - 0.5;
    EZX_DrawLine(mywindow, MYWINDOW_X/2, MYWINDOW_X/2, xr, yr);
    sprintf(data,"%3i°",rot);
    EZX_SetColor(COL_TEXT);
    EZX_SetBackgroundColor(COL_BACKGROUND);
    EZX_DrawText(mywindow, 26, MYWINDOW_Y-33, data);
    
    if (mode == SET_ROBOT_POSITION_TOKEN) {
      /* plot small robot */
      rob.pos = realPositionOfWindowPosition(&mapwindow, winPos);
      rob.pos.rot = deg2Rad((float) rot);

      fprintf(stderr,"# Location: x=%.2f y=%.2f rot=%d. Grid position x=%d y=%d\n", 
	      rob.pos.x,
	      rob.pos.y,
	      rot,
	      mapCoordinateOfRealCoordinate(rob.pos.x, 
					    map->offsetX, 
					    map->resolution),
	      mapCoordinateOfRealCoordinate(rob.pos.y, 
					    map->offsetY, 
					    map->resolution)
	      );

      displayRobot(&mapwindow, rob, COL_ROBOT, FILL_ROBOT);
    }
    else {
      /* draw frame */
    }
    
    /* wait for mouse activity */
    
    /* wait for a mouse action, sleep if none present */
    EZX_Flush();
    EZX_GetEvent (&theEvent); 
    while (theEvent.type != EZX_Event_was_Button_Press) {
      if (theEvent.type == EZX_Event_was_Nothing)
	EZX_block_and_wait ( &block_waiting_time ); 
      EZX_GetEvent (&theEvent); 
    }
    
    /* analyze mouse action */
    if (EZX_TestCursor(mywindow)) {
      /* mouse was in controlling window */
      if ((theEvent.PointerX >= 5) 
	  && (theEvent.PointerX < (MYWINDOW_X-5))
	  && (theEvent.PointerY < (MYWINDOW_Y-5))
	  && (theEvent.PointerY >= (MYWINDOW_Y-48))
	  && ((theEvent.PointerY < (MYWINDOW_Y-29))
	      || (theEvent.PointerY >= (MYWINDOW_Y-24)))
	  ) {
	/* was in button area... */
	if (theEvent.PointerY >= (MYWINDOW_Y-24))
	  function = 1;
	else {
	  switch(theEvent.Button) {
	  case 1: function = 1;
	    break;
	  case 2: function = 10;
	    break;
	  default: function = 45;
	    break;
	  }
	  if (theEvent.PointerX < 24) {
	    rot += function;
	    if (rot > 359) rot -= 360;
	  }
	  else if (theEvent.PointerX >= MYWINDOW_X-24) {
	    rot -= function;
	    if (rot < 0) rot += 360;
	  }
	  function = 0;
	}
      }
      
    }
    else if( EZX_TestCursor(mapwindow.window)) {
      
      /* mouse was on map window */
      switch(theEvent.Button) {
      case 1: /* left button */
	winPos.x = x1 = theEvent.PointerX;
	winPos.y = y1 = theEvent.PointerY;
	if (mode == SET_ROBOT_POSITION_TOKEN){
	}
	break;
	
      case 3: /* right button */
	if (simMap->initialized){
	  clearMapWindow( &mapwindow);
	  drawSimulatorMap( &mapwindow, simMap, C_BLACK, 0, 0);
	}
	else{
	  displayMapWindow( map, &mapwindow);
	}
	break;
      default: break;
      }
    }
    if (function == 1) {
      /* clear map if done or if new position chosen */
    }
    
  } while (function != 1);
  
#ifdef EDIT_MAP
  writeGridMap( "test", "map", map);
  exit(0);
#endif
  
  if ( destroyPositionTool)
    EZX_EndWindow(mywindow);
  
  return rob.pos;
}


static windowPosition
windowPositionOfRealPosition(gridWindow* win, realPosition pos) {

  windowPosition winPos;

  pos.x -= win->gridResolution * win->gridOffsetX;
  pos.y -= win->gridResolution * win->gridOffsetY;
  
  winPos.x = win->startX
    +  round((pos.x / win->gridResolution) * win->scale);
  winPos.y = win->startY
    +  round((win->gridSizeY - (pos.y / win->gridResolution))
		  * win->scale);
  return winPos;
}


static realPosition
realPositionOfWindowPosition(gridWindow* win, windowPosition winPos){
  realPosition pos;
  pos.x= (((winPos.x - win->startX) / (float) win->scale) + win->gridOffsetX)
    * win->gridResolution;
  pos.y = ((win->gridSizeY - ((winPos.y - win->startY)
			     / (float) win->scale)) + win->gridOffsetY)
    * win->gridResolution;
  pos.rot = 0.0;
  
  return pos;
}


void
displayRobotAtWindowPositions(gridWindow *win, windowPosition pos1,
			      windowPosition pos2, float robRadius, int col, bool fillRobot){
  EZX_SetLineWidth(1);
  if (fillRobot) {
    EZX_SetColor(col);
    EZX_FillCircle(win->window, pos1.x, pos1.y,
		   (float) robRadius / (float) win->gridResolution * (float) win->scale);
    EZX_SetColor(C_BLACK);
  }
  else {
      EZX_SetColor(col);
      EZX_DrawCircle(win->window, pos1.x, pos1.y,
		     (float) robRadius / (float) win->gridResolution * (float) win->scale);
  }
  
  
  EZX_DrawLine(win->window, pos1.x, pos1.y, pos2.x, pos2.y);
  
  EZX_Flush();
}


void
displayRobotInRobotWindow(gridWindow *robwin, robot rob, windowPosition center,
			  int col, bool fill){
  windowPosition pos1, pos2;
  realPosition pos;
  pos1 = robotWindowPosition( robwin, center, rob.pos);
  pos.x = rob.pos.x + (rob.radius-3) * cos(rob.pos.rot);
  pos.y = rob.pos.y + (rob.radius-3) * sin(rob.pos.rot);
  pos.rot = rob.pos.rot;
  
  pos2 = robotWindowPosition( robwin, center, pos);

  displayRobotAtWindowPositions(robwin, pos1, pos2, rob.radius, col, fill);
}

  





void
displayRobot(gridWindow *win, robot rob, int col, bool fillRobot)
{
  windowPosition pos1, pos2;
  realPosition pos;
  
  pos1 = windowPositionOfRealPosition( win, rob.pos);
  pos.x = rob.pos.x + (rob.radius-3) * cos(rob.pos.rot);
  pos.y = rob.pos.y + (rob.radius-3) * sin(rob.pos.rot);
  pos.rot = rob.pos.rot;
  
  pos2 = windowPositionOfRealPosition( win, pos);
  displayRobotAtWindowPositions(win, pos1, pos2, rob.radius, col, fillRobot);
}



void
showAllColors()
{
  EZXW_p win;
  int x, y, scale;

  scale = 3;
  
  EZX_NoMotionEvents();
  win = EZX_MakeWindow("Map window",
		       MYMAXCOLORS * scale,
		       MYMAXCOLORS * scale,
		       "+0+0");

  for (x = 0; x < MYMAXCOLORS; x++) {
    for (y = 0; y < MYMAXCOLORS; y++) {
      EZX_SetColor( x);
      EZX_FillRectangle(win, x * scale, y * scale, scale, scale);
    }
  }
  EZX_Flush();
}

gridWindow *
createRobotWindow( probabilityGrid *map, char* text,
		   int size, int x, int y, int scale)
{
  /* creates a window under EZX for the robot and sensor information */

  char corner[80];
  gridWindow *robwin;
  
  robwin = (gridWindow *) malloc(sizeof(gridWindow));
  
  robwin->gridSizeX = size / map->resolution;
  if (robwin->gridSizeX % 2 == 0)
    robwin->gridSizeX ++;

  robwin->gridSizeY = robwin->gridSizeX;
  robwin->gridOffsetX = robwin->gridOffsetY = 0;
  robwin->scale = scale;

  robwin->sizeX = robwin->gridSizeX * robwin->scale;
  robwin->sizeY = robwin->gridSizeY * robwin->scale;
  robwin->gridResolution = map->resolution;

  robwin->startX = 0;
  robwin->startY = 0;
  robwin->planeType = XYPLANE;

  sprintf(corner,"+%d+%d",x,y);
    
  EZX_NoMotionEvents();
  robwin->window = EZX_MakeWindow(text,robwin->sizeX,robwin->sizeY,corner);
  EZX_SetWindowBackground (robwin->window,C_BLACK);
  EZX_Flush();

  return(robwin);
}



windowPosition
robotWindowPosition( gridWindow* robwin, windowPosition center,
		     realPosition pos)
{
  windowPosition winPos;

  pos.x -= robwin->gridResolution * robwin->gridOffsetX;
  pos.y -= robwin->gridResolution * robwin->gridOffsetY;

  winPos.x = 
    robwin->startX
    + (int) round(((pos.x -
		    ((center.x - robwin->gridSizeX/2) )
		    * robwin->gridResolution) / robwin->gridResolution
		   )
		  * robwin->scale);

  winPos.y =
    robwin->startY
    + (int) round(((robwin->gridSizeY -
		    (pos.y - ((center.y - robwin->gridSizeY/2))
		     * robwin->gridResolution) / robwin->gridResolution
		    - 1))
		  * robwin->scale);
  return winPos;
}


int
stdDevIndexColor(int stdDevIndex)
{
    switch (stdDevIndex) {
    case 0:
	stdDevIndex = C_LAWNGREEN;
	break;
    case 1: 
	stdDevIndex = C_YELLOW;
	break;
    case 2: 
	stdDevIndex = C_VIOLET;
	break;
    case 3: 
	stdDevIndex = C_RED;
	break;
    }
    return stdDevIndex;
}


static void
displaySensings( gridWindow *robwin,
		 distanceScan sensings,
		 robot rob,
		 windowPosition center,
		 expectedDistTable *distTab,
		 probabilityGrid* map,
		 int maxRangeColor,
		 int beamColor)
{
  int i;

  realPosition sensingStart, sensingEnd;

  windowPosition pos1, pos2, pos3;

  EZX_SetLineStyle(FillSolid);
  EZX_SetLineWidth(1);
  
  /* compute start point of reading */

  for (i = 0; i < sensings.numberOfReadings; i++) {
    
    movement tmpReading;
    sensingStart = endPoint( rob.pos, sensings.readingOffset[i]);
	
    /* compute end point for reading */
    tmpReading.forward = sensings.reading[i].dist;
    tmpReading.sideward = tmpReading.rotation = 0.0;
    sensingEnd = endPoint( sensingStart, tmpReading);
    
    /* compute window positions for reading points */
    pos1 = robotWindowPosition(robwin, center, sensingStart);
    pos2 = robotWindowPosition(robwin, center, sensingEnd);

    /* Draw the beam. */
    if (sensings.reading[i].dist < sensings.maxDistance) 
      EZX_SetColor(beamColor);
    else
      EZX_SetColor(maxRangeColor);
    EZX_DrawLine(robwin->window, pos1.x, pos1.y, pos2.x, pos2.y);

    /* Show the expected distances. */
    if ( globalGraphicInfo.showExpectedDistances) {
      
      gridPosition gridPos;
      distance dist;
      int stdDevIndex;
      
      gridPos.x = round(center.x + (sensingStart.x - rob.pos.x) /
			robwin->gridResolution);
      gridPos.y = round(center.y + (sensingStart.y - rob.pos.y) /
			robwin->gridResolution);
      gridPos.rot = distTabPlaneOfRotation( sensingStart.rot, distTab);

      /* Use the expected distance table. */
      if ( ! globalGraphicInfo.actionInfo->onlineMapping) {
	dist = expectedDist( gridPos, distTab);
	stdDevIndex = numberOfExpectedStdDev( gridPos, distTab);
      }
      else {
/*  	dist = expectedFeatureGridMap( map, */
/*  				       gridPos.x, gridPos.y, gridPos.rot, */
/*  				       distTab->distanceResolution, */
/*  				       distTab->numberOfExpectedDistances); */

/*  	dist *= distTab->distanceResolution; */
	dist = expectedDistanceGridMap( map,
					gridPos.x, gridPos.y, gridPos.rot,
					distTab->distanceResolution,
					distTab->numberOfExpectedDistances);

	stdDevIndex = 0;
      }
      
      tmpReading.forward = dist;
      sensingEnd = endPoint( sensingStart, tmpReading);
      pos3 = robotWindowPosition(robwin, center, sensingEnd);
      
      /* #define MARK_EXPECTED_DISTANCE */
      EZX_SetColor(stdDevIndexColor(stdDevIndex));
      EZX_FillCircle(robwin->window, pos3.x, pos3.y, 3);
    }
  }
}


static void
displaySelectedSensings( gridWindow *robwin,
			 distanceScan sensings,
			 abstractSensorVector abstractSensings,
			 robot rob,
			 windowPosition center,
			 expectedDistTable *distTab,
			 probabilityGrid* map,
			 int maxRangeColor,
			 int beamColor)
{
  distanceScan usedSensings;
  int i,j;


  usedSensings = sensings;

  usedSensings.numberOfReadings = abstractSensings.numberOfSensorsToBeUsed;

  usedSensings.reading =
    (distanceReading *) malloc( usedSensings.numberOfReadings
				* sizeof(distanceReading));

  usedSensings.readingOffset =
    (movement*) malloc( usedSensings.numberOfReadings
			* sizeof(movement));

  
  for (j = 0; j < abstractSensings.numberOfSensorsToBeUsed; j++) {
    i = abstractSensings.mask[j];

    usedSensings.reading[j] = sensings.reading[i];
    usedSensings.readingOffset[j] = sensings.readingOffset[i];
  }
  
  displaySensings( robwin, usedSensings, rob, center,
		   distTab, map, maxRangeColor, beamColor);

  free(usedSensings.reading);
  free(usedSensings.readingOffset);
}


void
updateGlobalRobotWindow( actionInformation* actionInfo,
			 sensingActionMask* mask)
{
  informationsFor_GRAPHIC* graphicInfo = globalGraphicInfo.graphicInfo;
  static int skipCnt = 0;
  
  if ( skipCnt % graphicInfo->displaySkip == 0
       ||
       (skipCnt-1) % graphicInfo->displaySkip == 0) {

    if ( graphicInfo->dumpRobWin) 
      
      dumpRobotWindow( actionInfo->estimatedRobot,
		       actionInfo,
		       mask);
  }
  
  if (globalGraphicInfo.initialized) {
    
    if (graphicInfo->useGraphic	&& graphicInfo->showRobotZoom)
      
      if ( skipCnt++ % graphicInfo->displaySkip == 0) {
	
	if ( graphicInfo->robotWindow != NULL)
	  displayRobotWindow( graphicInfo->robotWindow,
			      actionInfo->estimatedRobot,
			      actionInfo,
			      mask,
			      graphicInfo);
      }
  }

  if (mask->perform[MOVEMENT][INTEGRATE_MOVEMENT]
      || mask->perform[SONAR][INTEGRATE_SONAR]
      || mask->perform[ANGLE][UPDATE_ANGLE_GRID]
      || mask->perform[LASER][INTEGRATE_LASER]
      || communicationInfo.scr->newMarker)
    skipCnt++;
}


void
displayWalls( gridWindow* robwin,
	      robot rob,
	      windowPosition center,
	      informationsFor_ANGLE* angleInfo){
  realPosition wallStart, wallEnd;
  windowPosition pos1, pos2;
  wall tmpWall, actualWall;

  EZX_SetLineWidth(5);
  EZX_SetLineStyle(FillSolid);
  EZX_SetLineStyle(FillTiled); 
  EZX_SetColor(WALL_COLOR);
	
  /*  for ( i = 0; i < angleInfo->angles.numberOfWalls; i++) {
    actualWall = angleInfo->angles.wall[i];
    */
  {
    actualWall = angleInfo->angles.bestWall;
    if (0) fprintf(stderr, "i: %d, alpha: %f, x: %f, y: %f, cert %f\n",
	    angleInfo->angles.bestWallIndex,
	    rad2Deg(actualWall.pos.rot),
	    actualWall.pos.x,
	    actualWall.pos.y,
	    actualWall.certainty);

    if ( actualWall.isNew) {
      wallStart.rot = wallEnd.rot = tmpWall.pos.rot =
	actualWall.pos.rot + rob.pos.rot;
      
      tmpWall.pos.x = rob.pos.x + actualWall.pos.x * cos(rob.pos.rot)
	- actualWall.pos.y * sin(rob.pos.rot);
      tmpWall.pos.y = rob.pos.y + actualWall.pos.x * sin(rob.pos.rot)
	+ actualWall.pos.y * cos(rob.pos.rot);
      
      wallStart.x = tmpWall.pos.x - WALL_LENGTH * cos(wallStart.rot);
      wallStart.y = tmpWall.pos.y - WALL_LENGTH * sin(wallStart.rot);
      wallEnd.x = tmpWall.pos.x + WALL_LENGTH * cos(wallEnd.rot);
      wallEnd.y = tmpWall.pos.y + WALL_LENGTH * sin(wallEnd.rot);
      
      pos1 = robotWindowPosition(robwin, center, wallStart);
      pos2 = robotWindowPosition(robwin, center, wallEnd);
      
      EZX_DrawLine(robwin->window, pos1.x, pos1.y, pos2.x, pos2.y);
    }
  }
  EZX_SetLineWidth(1);
  if (0) {
    displayRobotInRobotWindow(robwin, rob, center, COL_ROBOT, FILL_ROBOT);
    EZX_Flush();
    getchar();
  }
}



/* updates the RobotWindow */
void
displayRobotWindow( gridWindow* robwin,
		    robot rob,
		    actionInformation* actionInfo,
		    sensingActionMask* actionMask,
		    informationsFor_GRAPHIC* graphicInfo)
{
  if ( robwin != NULL) {

    /* All this only has to be done if anything was intgrated. */
    if ( actionMask->perform[MOVEMENT][INTEGRATE_MOVEMENT]
	 || actionMask->perform[SONAR][INTEGRATE_SONAR]
	 || actionMask->perform[ANGLE][UPDATE_ANGLE_GRID]
	 || actionMask->perform[LASER][INTEGRATE_LASER]
	 || communicationInfo.scr->newMarker) {
      
      int x,y;
      
      bool sonarDisplayed = FALSE, laserDisplayed = FALSE;
      int winX, winY;
      int endX = robwin->startX + robwin->gridSizeX * robwin->scale;
      int endY = robwin->startY + robwin->gridSizeY * robwin->scale;
      
      windowPosition center;
      extern laserParameters globalLaserParameters;
      
      /*---------------------------------------------------------------
       * Get some special information of the different sensings. 
       *---------------------------------------------------------------*/
      
      /* This structure contains all relevant information about the sensors. */
      informationsFor_SONAR* sonarInfo =
	(informationsFor_SONAR*) actionInfo->info[SONAR];
      informationsFor_LASER* laserInfo =
	(informationsFor_LASER*) actionInfo->info[LASER];
      informationsFor_ANGLE* angleInfo =
	(informationsFor_ANGLE*) actionInfo->info[ANGLE];
      
      /* Just for easier access. */
      probabilityGrid* map;
      distanceScan* sonars = &(actionInfo->actualSensings.sonar);
      expectedDistTable* sonarExpDist =
	&(sonarInfo->general.expectedDistances);
      expectedDistTable* laserExpDist =
	&(laserInfo->general.expectedDistances);
      distanceScan* frontLaser = &(actionInfo->actualSensings.frontLaser);
      distanceScan* rearLaser = &(actionInfo->actualSensings.rearLaser);
      
      if ( actionMask->use[LASER]) {
	map = &(actionInfo->laserMap);
      }
      else
	map = &(actionInfo->sonarMap);

      if ( globalGraphicInfo.actionInfo->onlineMapping) {
	/* Update the offset according to the new map. */
	robwin->gridOffsetX = round(map->offsetX / map->resolution);
	robwin->gridOffsetY = round(map->offsetY / map->resolution);
      }
      
      center.x = mapCoordinateOfRealCoordinate( rob.pos.x,
						map->offsetX,
						map->resolution);
      
      center.y = mapCoordinateOfRealCoordinate( rob.pos.y,
						map->offsetY,
						map->resolution);

      
      if (actionInfo->simMap.initialized){
	realPosition realOrigin;
	windowPosition origin;

	clearMapWindow(robwin);
	realOrigin.x = realOrigin.y = realOrigin.rot = 0;
	origin = robotWindowPosition(robwin, center, realOrigin);
	origin.y = robwin->sizeY - origin.y - 1;
	if ( 0 && actionMask->use[LASER])
	  drawSimulatorMapZRange( robwin, &(actionInfo->simMap),
				  C_BLACK, origin.x, -origin.y,
				  globalLaserParameters.parameters.sensorHeight,
				  globalLaserParameters.parameters.sensorHeight);
	else
	  drawSimulatorMap( robwin, &(actionInfo->simMap),
			    C_BLACK, origin.x, -origin.y);
      }
      else {
	y=center.y + robwin->gridSizeY/2;
	for (winY=robwin->startY; winY <= endY; winY += robwin->scale){
	  y--;
	  x = center.x - robwin->gridSizeX/2;	  
	  for (winX=robwin->startX; winX < endX; winX += robwin->scale){
	    if ( x < 0 || y < 0 || x >= map->sizeX || y >= map->sizeY)
	      EZX_SetColor(C_LAWNGREEN);
	    else if ( map->prob[x][y] == map->unknown)
	      EZX_SetColor(C_BLUE);
	    else
	      EZX_SetColor(fieldColor(occupancyProbabilityMap(x,y, *map)));
	    EZX_FillRectangle(robwin->window,winX,winY,
			      robwin->scale,robwin->scale);
	    x++;
	  }
	}
      }

      /* display the lasers */

      if ( actionMask->use[LASER] &&
	   ((frontLaser->isNew && actionMask->perform[LASER][INTEGRATE_LASER])
	   || communicationInfo.scr->newMarker) )
	   {

	     displaySensings( robwin, *frontLaser, rob, center,
			      laserExpDist, map,
			      C_YELLOW2, C_GREY65);
	     
	if (graphicInfo->showSelectedSensings)
	  displaySelectedSensings( robwin, *frontLaser,
				   actionInfo->abstractSensors[ABSTRACT_FRONT_LASER],
				   rob, center,
				   laserExpDist,  map,
				   C_PINK, C_DEEPPINK);
	laserDisplayed = TRUE;
      }
      
      if ( actionMask->use[LASER] &&  ((rearLaser->isNew 
	   && actionMask->perform[LASER][INTEGRATE_LASER]) ||
	   communicationInfo.scr->newMarker))

	   {
	
	displaySensings( robwin, *rearLaser, rob, center,
			 laserExpDist, map,
			 C_YELLOW2, C_GREY65);
	if (graphicInfo->showSelectedSensings)
	  displaySelectedSensings( robwin, *rearLaser,
				   actionInfo->abstractSensors[ABSTRACT_REAR_LASER],
				   rob, center,
				   laserExpDist,  map,
				   C_PINK, C_DEEPPINK);
	laserDisplayed = TRUE;
      }

      /* display the sonars */
      if ( sonars->isNew && actionMask->perform[SONAR][INTEGRATE_SONAR]){
	displaySensings( robwin, *sonars, rob, center,
			 sonarExpDist,  map,
			 C_BLUE, C_GREY70);
	if (graphicInfo->showSelectedSensings)
	  displaySelectedSensings( robwin, *sonars,
				   actionInfo->abstractSensors[ABSTRACT_SONAR],
				   rob, center,
				   sonarExpDist,  map,
				   C_PINK, C_DEEPPINK);
	sonarDisplayed = TRUE;
      }
	
      /* display the angles */
      if (actionMask->use[ANGLE]){
	if ( angleInfo->angles.isNew &&
	     actionMask->perform[ANGLE][UPDATE_ANGLE_GRID] &&
	     actionMask->consider[ANGLE]) {
	  if (angleInfo->useSonar && sonars->isNew && !sonarDisplayed)
	    displaySensings( robwin, *sonars, rob, center,
			     sonarExpDist,  map,
			     C_BLUE, C_LAWNGREEN);
	  if (angleInfo->useLaser && frontLaser->isNew && !laserDisplayed)
	    displaySensings( robwin, *frontLaser, rob, center,
			     laserExpDist,  map,
			     C_BLACK, C_RED);
	  if (angleInfo->useLaser && rearLaser->isNew && !laserDisplayed)
	    displaySensings( robwin, *rearLaser, rob, center,
			     laserExpDist,  map,
			     C_BLACK, C_RED);
	  displayWalls(robwin, rob, center, angleInfo);
	}
      }
	
      displayRobotInRobotWindow(robwin, rob, center, COL_ROBOT, FILL_ROBOT);

      if ( 0) {
	char s[80];
	sprintf(s, "x=%.2fm y=%.2fm rot=%.1f",
		rob.pos.x*0.01,rob.pos.y*0.01,rad2Deg(rob.pos.rot));
	EZX_SetColor(C_RED);
	EZX_SetBackgroundColor(C_WHITE);
	EZX_DrawTextAt(robwin->window, 4, robwin->sizeY-4, s, 'l');
      }

      EZX_Flush();
    }
  }
}



gridWindow *
createAngleWindow(angleProbTable *grid, char* text, int x, int y) {
  /* creates an angle grid window */
  
  char corner[80];
  gridWindow *mapwin;
  
  mapwin = (gridWindow *) malloc(sizeof(gridWindow));
  mapwin->scale = 1;
  
  mapwin->gridSizeX = 5;
  mapwin->gridSizeY = 80;
  mapwin->gridResolution = 2;
  
  mapwin->sizeX = grid->numberOfAngles*mapwin->scale;
  mapwin->sizeY = mapwin->gridSizeY*mapwin->scale;

  mapwin->firstTimePosition = TRUE;

  sprintf(corner,"+%d+%d",x,y);
    
  EZX_NoMotionEvents();
  mapwin->window = EZX_MakeWindow(text,mapwin->sizeX,mapwin->sizeY,corner);
  EZX_SetWindowBackground (mapwin->window,C_BLACK);
  EZX_Flush();

  mapwin->startX = 0;
  mapwin->startY = 0;
  return(mapwin);
}



void
displayAngleWindow(angleProbTable *grid, gridWindow *anglewin,
		   double referenceValue) {
  /* plots map in window */

  register int x;
    
  probability max=referenceValue, min = 1e40;
    
  int winX, winY, lastWinY;

  EZX_SetColor(C_WHITE);
  EZX_FillRectangle(anglewin->window, 0,0, anglewin->sizeX, anglewin->sizeY);

  EZX_SetColor(C_BLACK);
  EZX_SetLineStyle(FillTiled);

  for (x=0; x < 360; x+=90) 		    
    EZX_DrawLine( anglewin->window,
		  x*anglewin->scale,
		  0,
		  x*anglewin->scale,
		  anglewin->sizeY);

  EZX_SetLineStyle(FillSolid);
    
  for (x = 0; x < grid->numberOfAngles; x++) {
    if (grid->prob[x] > max)
      max = grid->prob[x];
    if (grid->prob[x] < min)
      min = grid->prob[x];
  }

  winX = 0;
  lastWinY = 0; 
  for (x = 0; x < grid->numberOfAngles; x++) {
    if (max == min)
      winY = 0;
    else
      winY = anglewin->scale*
	anglewin->gridSizeY*(1-(grid->prob[x]-min)/(max-min));

    EZX_SetColor(C_BLACK);
    if (x > 0) {

      EZX_FillCircle( anglewin->window, winX, winY, 2);
			  
      EZX_DrawLine(anglewin->window, winX, winY, winX-anglewin->scale,
		   lastWinY);
    }
    lastWinY = winY;

    winX += anglewin->scale;
  }

  /* Display the reference value. */
  EZX_SetColor(C_RED);
  winY = anglewin->scale*
    anglewin->gridSizeY*(1-(referenceValue-min)/(max-min));
  EZX_DrawLine( anglewin->window, 0, winY, anglewin->sizeX,
		winY);
  EZX_Flush();

}


/********************************************************************
 ********************************************************************
 * Local functions.
 ********************************************************************
 ********************************************************************/


static int
gridWindowScale(int sizeX, int sizeY, int minScale) {
  int scaleX = (int) (400 / sizeX);
  int scaleY = (int) (300 / sizeY);
  
  if (scaleX < minScale)
    scaleX = minScale;
  if (scaleY < minScale)
    scaleY = minScale;
  
  if (scaleY < scaleX)
    return(scaleY);
  else
    return(scaleX);
}


static float
positionWindowScale(float sizeX, float sizeY, float maxCmPerPixel)
{
  float cmPerPixelX = fMin( sizeX / 100, maxCmPerPixel);
  float cmPerPixelY = fMin( sizeY / 100, maxCmPerPixel);

  return fMin( cmPerPixelX, cmPerPixelY);
}

static void
displaySampleMean( positionWindow* mapwin,
		   float radius,
		   realPosition pos,
		   int color)
{
  writeLog( "%f %f grm\n", pos.x, pos.y);
  if ( mapwin == NULL)
    return;
  else {
    int winPosX = round( (pos.x - mapwin->minX) / mapwin->cmPerPixel);
    int winPosY = mapwin->sizeY - round((pos.y - mapwin->minY) / mapwin->cmPerPixel);
    
#ifndef SHOW_SAMPLE_EVOLUTION
    EZX_SetColor(color);
    EZX_SetLineWidth( 3);
    EZX_DrawCircle(mapwin->window, winPosX, winPosY, radius / mapwin->cmPerPixel); 
#else
    EZX_SetColor(C_BLACK);
    EZX_FillCircle(mapwin->window, winPosX, winPosY, radius / mapwin->cmPerPixel);
#endif
    EZX_Flush();
  }
}
  

static int
cm2Pixel(gridWindow* win, float cm){
  return round(cm / win->gridResolution * win->scale);
}



int
fieldColor(mapProbability f){
#  ifdef DIM
  f /= 10;
#endif
  if (f < 0.00001){
    if ( f < 0.0)
      return COL_UNKNOWN;
    else
      return COL_WHITE;
  }
  else if (f > 0.9999)
    return COL_BLACK;
  else
    return COL_WHITE - (int) round((double) f*(COL_WHITE-COL_BLACK));
}


realPosition
measuredRobotPositionInMap()
{
  return globalGraphicInfo.graphicInfo->robotInMap.pos;
}


void
plotGifSensors( gdImagePtr GifPic,
		int picSizeY,
		float cmPerPixel,
		float minX, float minY,
		realPosition robPos, 
		distanceScan *sensings,
		abstractSensorVector abstractSensings,
		int beamCol)
{
  int i,j;
  realPosition sensingEnd;
  movement tmpReading;
  int endX, endY;
  int sensorSkip;
  bool selected[MAX_SIZE_OF_SCAN], maxRange;
  
  realPosition sensingStart = endPoint( robPos, sensings->offset);
  int startX = (sensingStart.x - minX) / cmPerPixel;
  int startY = (sensingStart.y - minY) / cmPerPixel;

  if ( sensings->numberOfReadings > 24)
    sensorSkip= 3;
  else
    sensorSkip = 1;

  for (i = 0; i < sensings->numberOfReadings; i++)
    selected[i] = FALSE;
  
  for (j = 0; j < abstractSensings.numberOfSensorsToBeUsed; j++) 
    selected[abstractSensings.mask[j]] = TRUE;
  
  for (i = 0; i < sensings->numberOfReadings; i++){
    if ( i % sensorSkip == 0) {
      sensingStart.rot = normalizedAngle(robPos.rot + sensings->reading[i].rot);
      tmpReading.forward = fMin(1000.0, sensings->reading[i].dist);
      tmpReading.sideward = tmpReading.rotation = 0.0;
      sensingEnd = endPoint( sensingStart, tmpReading);
      
      endX = (sensingEnd.x - minX) / cmPerPixel;
      endY = (sensingEnd.y - minY) / cmPerPixel;
      
      maxRange = abstractSensings.sensor[i].measuredFeature
	== (abstractSensings.sensor[i].numberOfFeatures -1);
      
      
      gdImageLine( GifPic, startX, picSizeY - startY, endX, picSizeY - endY, beamCol);
    }
  }
}

static void
insertRobotIntoGif( gdImagePtr GifPic,
		     int picSizeX, int picSizeY,
		     float cmPerPixel,
		     float minRealX, float minRealY,
		     realPosition center, float radius, int circleColor)
{
  register int x, y;
  int minX, maxX, minY, maxY;

  int centerX = (center.x - minRealX) / cmPerPixel;
  int centerY = (center.y - minRealY) / cmPerPixel;
  int pixelRadius = radius / cmPerPixel;
  
  fprintf(stderr, "%d %f %f\n", pixelRadius, radius, cmPerPixel);
  
  minX = centerX - pixelRadius;
  minY = centerY - pixelRadius;
  maxX = centerX + pixelRadius + 1;
  maxY = centerY + pixelRadius + 1;
  
  minX = iMax(0, minX);
  minX = iMin(minX, picSizeX - 1);
  maxX = iMax(0, maxX);
  maxX = iMin(maxX, picSizeX - 1);
  minY = iMax(0, minY);
  minY = iMin(minY, picSizeY - 1);
  maxY = iMax(0, maxY);
  maxY = iMin(maxY, picSizeY - 1);

  for (x = minX; x <= maxX; x++) {
    float sqrX = fSqr(centerX - x);
    for (y = minY; y <= maxY; y++) {
      if (sqrt(sqrX + fSqr(centerY - y)) < pixelRadius)
	{
	  gdImageSetPixel( GifPic, x, picSizeY - y - 1, circleColor);
	}
    }
  }
  
  {
    movement nose;
    int noseX, noseY;
    int noseCol = gdImageColorAllocate( GifPic, 0, 0, 0);
    realPosition noseOfRobot;
    nose.forward  = radius;
    nose.rotation = nose.sideward = 0.0;
    noseOfRobot = endPoint( center, nose);
    
    noseX = (noseOfRobot.x - minRealX) / cmPerPixel;
    noseY = (noseOfRobot.y - minRealY) / cmPerPixel;

    gdImageLine( GifPic, centerX, picSizeY - centerY, noseX, picSizeY - noseY, noseCol);
  }

}

static void
dumpPositionWindow( sampleSet* samples,
		    probabilityGrid *map,
		    visionMap* visMap,
		    float distanceTraveled)
{
  static int plotCnt = 0;

#define NUMBER_OF_GREY_LEVELS 100
#define NUMBER_OF_RED_LEVELS 20
  
  int showSensors = FALSE;
  int showSamples = TRUE;
  int showRobot   = FALSE;
  gdImagePtr GifPic;
  int mapColor[NUMBER_OF_GREY_LEVELS], col;
  int unknownColor, robotColor, whiteColor;
  int beamColor=0;
  int i, s, x, y;
  float cmPerPixel = fMax( (samples->maxX - samples->minX) / 1000, 1.0);
  int gifSizeX = (samples->maxX - samples->minX) / cmPerPixel;
  int gifSizeY = (samples->maxY - samples->minY) / cmPerPixel;

  /* Open the file. */
  char plotFileName[80], *header;
  FILE* fp;
  
  static FILE* infoFile;
  static char previousPlotFileName[80];

  realPosition robPos = samples->mean;
  robPos = globalGraphicInfo.actionInfo->measuredRobotPosition;
  
  fprintf(stderr, "DUMP %d\n", plotCnt);
  fprintf(stderr, "SIZE : %d %d %f %f %f \n", gifSizeX, gifSizeY, cmPerPixel, samples->maxX, samples->minX);
  setTimer(0);
  
  writeLog( "# Dump position window no %d\n", plotCnt);
  if ( samples->alreadySampled == 2) 
    header = "moved";
  else if ( samples->alreadySampled == 1)
    header = "resampled";
  else if ( samples->alreadySampled == 3) {
    showSensors = TRUE;
    showRobot   = TRUE;
    /*     showSamples = FALSE; */
    header = "scan";
  }
  else
    header = "weighted";

  if ( plotCnt < 10)
    sprintf(plotFileName, "00%d.%s.gif", plotCnt, header);
  else if ( plotCnt < 100)
    sprintf(plotFileName, "0%d.%s.gif", plotCnt, header);
  else
    sprintf(plotFileName, "%d.%s.gif", plotCnt, header);
  if ((fp = fopen(plotFileName,"wt")) == NULL) {
    fprintf(stderr,"ERROR: Could not open gif file '%s'!\n",plotFileName);
      exit(0);
  }

  /* Open the file for automatic gif animation. */
  if ( plotCnt == 0) {
    if ((infoFile = fopen( "_animate", "wt")) == NULL) {
      fprintf(stderr, "ERROR: Could not open gif info file!\n");
      exit(0);
    }
    fprintf( infoFile, "~/tools/animateGifs/whirlgif -o scan.gif ");
  }
  else {
    fprintf( infoFile, "-time %d %s ", (int) (timeExpired(GIF_TIMER) * 100),
	     previousPlotFileName);
  }
    
  /* Gif structure and colors. */
  GifPic = gdImageCreate( gifSizeX, gifSizeY);
  unknownColor = gdImageColorAllocate( GifPic, 150, 150, 150);
  robotColor   = gdImageColorAllocate( GifPic, 0, 255, 0);
  whiteColor   = gdImageColorAllocate( GifPic, 255, 255, 255);
  
  /* Initialize the color tables. */
  for (i = 0; i < NUMBER_OF_GREY_LEVELS; i++){
    int greyValue = i * (255.0 / NUMBER_OF_GREY_LEVELS) + 0.5;
    mapColor[i] = gdImageColorAllocate( GifPic, greyValue, greyValue, greyValue );
  }

  if ( map->initialized) {
    
    int useVisionMap = FALSE;
    float resolution;
    int mapSizeX, mapSizeY;
    
    if ( visMap->initialized) {
      useVisionMap = TRUE;
      resolution = visMap->resolution;
      mapSizeX = visMap->sizeX;
      mapSizeY = visMap->sizeY;
    }
    else {
      resolution = map->resolution;
      mapSizeX = map->sizeX;
      mapSizeY = map->sizeY;
    }
    
    for ( x = 0; x < gifSizeX; x++) {
      int mapX = x * cmPerPixel / resolution;
      
      for ( y = 0; y < gifSizeY; y++) {
	int mapY = y * cmPerPixel / resolution;
	
	if ( mapX < 0 || mapY < 0 || mapX >= mapSizeX || mapY >= mapSizeY) {
	  col = unknownColor;
	}
	else if ( !useVisionMap && map->prob[mapX][mapY] == map->unknown) {
	  col = unknownColor;
	}
#define CLIP_MAP
#ifdef CLIP_MAP
	else if ( !useVisionMap && map->prob[mapX][mapY] > CLIP_THRESHOLD) {
	  col = unknownColor;
	}
	else
	  col = whiteColor;
#else
	else {
	  if (useVisionMap)
	    col = round(visMap->pix[mapX][mapY] / 256.0 * (NUMBER_OF_GREY_LEVELS - 1));
	  else
	    col = mapColor[(int) ((1.0 - map->prob[mapX][mapY]) * (NUMBER_OF_GREY_LEVELS-1) + 0.5)];
	}
#endif
	gdImageSetPixel( GifPic, x, gifSizeY - y - 1, col);
      }
    }
  }

  /* The sensors. */
  if (showSensors) {

    if ( showSamples) {
      if ( globalGraphicInfo.actionMask->use[LASER])
	beamColor = gdImageColorAllocate( GifPic, 0, 0, 255);
      else
	beamColor = gdImageColorAllocate( GifPic, 0, 0, 255);
    }
    else
      beamColor = gdImageColorAllocate( GifPic, 255, 0, 0);
    
    if ( globalGraphicInfo.actionMask->use[LASER]  &&
	 globalGraphicInfo.actionInfo->actualSensings.frontLaser.isNew
	 && globalGraphicInfo.actionMask->perform[LASER][INTEGRATE_LASER]){
      plotGifSensors( GifPic, gifSizeY,
		      cmPerPixel, samples->minX, samples->minY,
		      robPos, 
		      &(globalGraphicInfo.actionInfo->actualSensings.frontLaser),
		      globalGraphicInfo.actionInfo->abstractSensors[ABSTRACT_FRONT_LASER],
		      beamColor);
  }
    
    if (  globalGraphicInfo.actionMask->use[LASER] &&
	  globalGraphicInfo.actionInfo->actualSensings.rearLaser.isNew
	  && globalGraphicInfo.actionMask->perform[LASER][INTEGRATE_LASER]){
      plotGifSensors( GifPic, gifSizeY,
		      cmPerPixel, samples->minX, samples->minY,
		      robPos, 
		      &(globalGraphicInfo.actionInfo->actualSensings.rearLaser),
		      globalGraphicInfo.actionInfo->abstractSensors[ABSTRACT_REAR_LASER],
		      beamColor);
    }
    if (  globalGraphicInfo.actionMask->use[SONAR] &&
	  globalGraphicInfo.actionInfo->actualSensings.sonar.isNew
	  && globalGraphicInfo.actionMask->perform[SONAR][INTEGRATE_SONAR]){
      plotGifSensors( GifPic, gifSizeY,
		      cmPerPixel, samples->minX, samples->minY,
		      robPos, 
		      &(globalGraphicInfo.actionInfo->actualSensings.sonar),
		      globalGraphicInfo.actionInfo->abstractSensors[ABSTRACT_SONAR],
		      beamColor);
    }
  }
  
  /* Now the samples. */
  if ( showSamples) {
#ifdef SAMPLE_ONE_COLOR
    int sampleColor = gdImageColorAllocate( GifPic, 255, 0, 0); 

    for ( s = 0; s < samples->numberOfSamples; s++) {
      realPosition samplePos = samples->sample[s].pos;
      x = round( (samplePos.x - samples->minX) / cmPerPixel);
      y = gifSizeY - round((samplePos.y - samples->minY) / cmPerPixel);
      
      gdImageSetPixel( GifPic, x, y, sampleColor);
    }
#else

#define MIN_WEIGHT_THRESHOLD 0.00001 / samples->numberOfSamples
    
    int sampleColor[NUMBER_OF_RED_LEVELS], prevCol;
    float minWeight = 1e6, maxWeight = 0.0;
    float minLogWeight, maxLogWeight;
    
    /* Check for minimal and maximal value of the weights. */
    for ( s = 0; s < samples->numberOfSamples; s++) {
      if (samples->sample[s].weight > maxWeight)
	maxWeight = samples->sample[s].weight;
      if (samples->sample[s].weight < minWeight)
	minWeight = samples->sample[s].weight;
    }

    minWeight = fMax( minWeight, MIN_WEIGHT_THRESHOLD);

    if ( samples->alreadySampled && samples->alreadySampled != 3) {
      minWeight = maxWeight;
      fprintf(stderr, "Set all same\n");
    }
    
    minLogWeight = fastLog( minWeight);
    maxLogWeight = fastLog( maxWeight);

    if ( minLogWeight == maxLogWeight) {
      sampleColor[0] = gdImageColorAllocate( GifPic, 255, 0, 0 );
    }
    else {
      int redValue, greenValue, blueValue;
      int sizeOfFirstRegion = NUMBER_OF_RED_LEVELS / 3;

      for (i = 0; i < NUMBER_OF_RED_LEVELS; i++){
	if ( i < sizeOfFirstRegion) {
	  redValue   = (int) fNorm( i, 0, sizeOfFirstRegion-1, 100, 255);
	  greenValue = 0;
	  blueValue  = 0;
	}
	else {
	  redValue   = 255;
	  greenValue = (int) fNorm( i, sizeOfFirstRegion, NUMBER_OF_RED_LEVELS, 0, 200);
	  blueValue  = greenValue;
	}

	/* Invert sort. */
	sampleColor[NUMBER_OF_RED_LEVELS - i - 1] =
	  gdImageColorAllocate( GifPic, redValue, greenValue, blueValue);
      }
    }

    fprintf(stderr, "min %f %f\n", minWeight, maxWeight);
    
    for ( s = 0; s < samples->numberOfSamples; s++) {
      realPosition samplePos = samples->sample[s].pos;
      x = round( (samplePos.x - samples->minX) / cmPerPixel);
      y = gifSizeY - round((samplePos.y - samples->minY) / cmPerPixel);

      if ( minLogWeight == maxLogWeight)
	col = sampleColor[0];
      else {
	float weight = fMax( samples->sample[s].weight, MIN_WEIGHT_THRESHOLD);
#define LOG_SCALE
#ifdef LOG_SCALE
	col = sampleColor[(int) fNorm( fastLog(weight),
				       minLogWeight, maxLogWeight,
				       0, NUMBER_OF_RED_LEVELS-1)];
#else
	col = sampleColor[(int) fNorm( weight, minWeight, maxWeight,
				       0, NUMBER_OF_RED_LEVELS-1)];
#endif
      }

      /* Set several pixels instead of one. */
      {
#define PIXEL_AREA 1
	int sx, sy;
	
	for ( sx = x - PIXEL_AREA; sx <= x + PIXEL_AREA; sx++) 
	  if ( sx >= 0 && sx < gifSizeX) {
	    for ( sy = y - PIXEL_AREA; sy <= y + PIXEL_AREA; sy++) 
	      if ( sy >= 0 && sy < gifSizeY) {
		prevCol = gdImageGetPixel( GifPic, sx, sy);
		
		if ( prevCol == whiteColor || ( showSensors && prevCol == beamColor)
		     || col < prevCol)
		  gdImageSetPixel( GifPic, sx, sy, col);
	      }
	  }
      }
#endif
    }
  }
  
  /* Sample mean. */
  if (showRobot)
    insertRobotIntoGif( GifPic, gifSizeX, gifSizeY,
			cmPerPixel, samples->minX, samples->minY,
			robPos, 1.0 * ROB_RADIUS, robotColor);
  
  gdImageGif( GifPic, fp );
  fclose(fp);
  gdImageDestroy( GifPic);
  
  plotCnt++;

  nonRelevantTime += timeExpired(0);
  setTimer( GIF_TIMER);
  strcpy( previousPlotFileName, plotFileName);
}




