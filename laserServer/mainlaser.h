/*
 * $Id: mainlaser.h,v 1.1 2002/09/14 16:33:01 rstone Exp $
 *
 */

#ifndef MAINLASER_H
#define MAINLASER_H

char *tcxMachine;
int laserDef[2];

#define B21_ROBOT    0
#define B18_ROBOT    1
#define PIONEER_ATRV 2
#define PIONEER_II   3
#define URBAN_ROBOT  4
#define PIONEER_I    5
#define PIONEER_IF   6
#define Scout        7

extern int robotType;

#endif
/*
 * $Log: mainlaser.h,v $
 * Revision 1.1  2002/09/14 16:33:01  rstone
 * *** empty log message ***
 *
 * Revision 1.10  1999/09/24 14:35:33  fox
 * Added support for scout.
 *
 * Revision 1.9  1999/07/20 14:25:46  schneid1
 * laserServer now uses beeSoft.ini & -display works again
 *
 * Revision 1.8  1999/06/25 19:49:56  fox
 * Added urban robot.
 *
 * Revision 1.7  1999/04/19 09:19:35  rhino
 * Fixed conflicts with dieter's last commit.
 *
 * Revision 1.6  1998/10/30 18:58:04  fox
 * Added support for pioneers and multiple robots.
 *
 * Revision 1.5  1998/08/06 03:22:56  thrun
 * supports 2 lasers.
 *
 * Revision 1.4  1997/11/06 17:56:35  swa
 * Fixed a bug with TCXHOST. If the environment variable TCXHOST is set,
 * take it. If not, take the TCXHOST from beeSoft.ini. If this doesn't
 * work, take localhost. If this doesn't work: bummer.
 *
 * Revision 1.3  1997/08/07 15:42:54  swa
 * First working version of the laserServer. Seems to run fine. Has support for
 * a second laser clientwise, yet the device stuff for the second one is not
 * implemented.
 *
 * Revision 1.2  1997/08/07 02:45:52  swa
 * First version of a laserServer. Compiles nicely, yet produces nothing useful. ;)
 * Just to copy things to a safe place ...
 *
 * Revision 1.1  1997/08/06 15:12:33  swa
 * Very first and incomplete version of a laserServer. No TCX comm yet.
 *
 *
 */
