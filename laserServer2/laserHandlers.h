#ifndef LASERHANDLERS_H
#define LASERHANDLERS_H

extern int n_auto_sweep0_update_modules;
extern int n_auto_sweep1_update_modules;

void LaserCloseHandler              ( char *name, TCX_MODULE_PTR module );
int  LaserInitializeTCX             ( char *robotName);
void send_automatic_sweep_update    ( int numLaser, int numValues, float *values, 
				      struct timeval timestamp );
void LaserRegisterAutoUpdateHandler ( TCX_REF_PTR ref, 
                                      LASER_SERVER_register_auto_update_ptr data);
void LaserSweepQueryHandler         ( TCX_REF_PTR ref, 
                                      LASER_SERVER_sweep_query_ptr data );
void LaserSendSweepTo               ( TCX_MODULE_PTR module, int numLaser,
				      int numValues, float *values, 
				      struct timeval timestamp );

#endif
