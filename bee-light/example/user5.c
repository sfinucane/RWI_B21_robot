#include "bee.h"
#include <stdio.h>
#define PI 3.1415926f
float compu_distance(float x1, float y1, float x2, float y2)
{
    float temp_x, temp_y ;
    temp_x = (x2 - x1) * (x2 - x1) ; 
    temp_y = (y2 - y1) * (y2 - y1) ;
    return (sqrt(temp_x + temp_y) ) ;  
}


void my_laser_callback(int laser_f_numberOfReadings,float laser_f_angleResolution, float laser_f_startAngle, int * laser_f_reading, int laser_r_numberOfReadings, float laser_r_angleResolution, float laser_r_startAngle, int * laser_r_reading)
{ 
    static int * flaser_val ;
    static int known = 0 ;
    static float last_des_x ;
    static float last_des_y ;
    int movecount = 0 ;
    int discardcount = 0 ;
    int max = 0 ;
    int indexOfMax = 0 ;
    int i ;
    if(known==0) 
    {
        flaser_val = (int *) malloc(laser_f_numberOfReadings * sizeof(int)) ;
        known = 1 ;
        for (i = 0 ; i < laser_f_numberOfReadings ; i++)
             flaser_val[i] = 0 ;
        robot_position * position ;
        position = beeGetCurrentPosition() ;
        last_des_x = position->pos_x ;
        last_des_y = position->pos_y ;
    }
    for (i = 0 ; i < laser_f_numberOfReadings ; i++) 
        flaser_val[i] = laser_f_reading[i] ;
    if(discardcount%10000==0) 
    {
        for (i = 0 ; i < laser_f_numberOfReadings ; i++)
        {
            if (flaser_val[i] > max) {
                 max = flaser_val[i] ;
                 indexOfMax = i ;
            }
        }
        float distance = (float ) flaser_val[indexOfMax]  ;
        float des_x = distance * cos((float)indexOfMax * PI / 180) ;
        float des_y = distance * sin((float)indexOfMax * PI / 180) ;      
//        printf("\n largest distance %d %d", indexOfMax, flaser_val[indexOfMax]) ; 
        robot_position * position ;
        position = beeGetCurrentPosition() ;
         
        if(compu_distance (position->pos_x, position->pos_y, last_des_x, last_des_y) < 50) 
        { 
             beeApproachRelative(des_x, des_y) ;
             last_des_x = position->pos_x 
                                 + des_x * cos(position->orientation)
                                 + des_y * sin(position->orientation) ;
             last_des_y = position->pos_y
                                 - des_x * sin(position->orientation)
                                 + des_y * cos(position->orientation) ;
             printf("\n MOVE to relative position x=%f y=%f", des_x, des_y) ;
             discardcount == 0 ;
        }
    }
    discardcount ++ ;
}

int main(int argc, char *argv[])
{
    beeInitialize("USER");
    bool returnval ;
    returnval = beeLaserStartRegularUpdate() ;
    beeLaserRegisterCallback(my_laser_callback) ;
//sleep(10) ;
    while(1) ; 
}

