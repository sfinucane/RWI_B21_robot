/*
 * $Id: laserHandlers.h,v 1.1 2002/09/14 16:33:01 rstone Exp $
 *
 */
#ifndef LASERHANDLERS_H
#define LASERHANDLERS_H

int n_auto_sweep0_update_modules;

int n_auto_sweep1_update_modules;

void LaserCloseHandler( char *name, TCX_MODULE_PTR module );

int LaserInitializeTCX( char* robotName);

void send_automatic_sweep_update( int numLaser, float *values );

void LaserRegisterAutoUpdateHandler( TCX_REF_PTR                    ref,
				     LASER_SERVER_register_auto_update_ptr data);

void connect_to_BaseServer(void);
/* ----------------------------------------------------------------------- */

void LaserSweepQueryHandler( TCX_REF_PTR           ref,
			     LASER_SERVER_sweep_query_ptr data );

void LaserSendSweepTo( TCX_MODULE_PTR module, int numLaser, float *values );

/* ----------------------------------------------------------------------- */

#endif

/*
 * $Log: laserHandlers.h,v $
 * Revision 1.1  2002/09/14 16:33:01  rstone
 * *** empty log message ***
 *
 * Revision 1.6  1998/10/30 18:58:04  fox
 * Added support for pioneers and multiple robots.
 *
 * Revision 1.5  1998/08/16 20:22:15  thrun
 * Can now receive status information, but doesn't do anything with it
 * yes. Only activated with "-status=1" option.
 *
 * Revision 1.4  1998/01/13 22:41:41  thrun
 * resolved a naming conflict. CHanged LASER to LASER_SERVER.
 *
 * Revision 1.3  1997/08/07 15:42:53  swa
 * First working version of the laserServer. Seems to run fine. Has support for
 * a second laser clientwise, yet the device stuff for the second one is not
 * implemented.
 *
 * Revision 1.2  1997/08/07 03:50:21  swa
 * Fixed a bunch of bugs. Still not working.
 *
 * Revision 1.1  1997/08/07 02:45:52  swa
 * First version of a laserServer. Compiles nicely, yet produces nothing useful. ;)
 * Just to copy things to a safe place ...
 *
 *
 */
