#include "bee.h"
#include <stdio.h>
void my_laser_callback(int laser_f_numberOfReadings,float laser_f_angleResolution, float laser_f_startAngle, int * laser_f_reading, int laser_r_numberOfReadings, float laser_r_angleResolution, float laser_r_startAngle, int * laser_r_reading)
{ 
    static int * flaser_val ;
    static int * rlaser_val ;
    static int known = 0 ;
    static int count  = 0 ; 
    int i ;
    count ++ ;
    if(known==0) 
    {
        flaser_val = (int *) malloc(laser_f_numberOfReadings * sizeof(int)) ;
        rlaser_val = (int *) malloc(laser_r_numberOfReadings * sizeof(int)) ;
        known = 1 ;
        for (i = 0 ; i < laser_f_numberOfReadings ; i++)
             flaser_val[i] = 0 ;
        for (i = 0 ; i < laser_r_numberOfReadings ; i++)
             rlaser_val[i] = 0 ;
    }
    for (i = 0 ; i < laser_f_numberOfReadings ; i++) 
            flaser_val[i] += laser_f_reading[i] ;
    for (i = 0 ; i < laser_r_numberOfReadings ; i++)
            rlaser_val[i] += laser_f_reading[i] ;
    if(count == 100) 
    {
        printf("\n Average of 100 values:   ") ;
        for (i = 0 ; i < laser_f_numberOfReadings ; i++)
        {
            printf("%d ", flaser_val[i] / 100) ; 
            flaser_val[i] = 0 ; 
        }
        for (i = 0 ; i <  laser_r_numberOfReadings ; i++)
        {
            printf("%d ", rlaser_val[i] / 100) ;
            rlaser_val[i] = 0 ;
        }
        printf("\n") ;
        count = 0 ;
    }
}

int main(int argc, char *argv[])
{
    beeInitialize("USER");
    bool returnval ;
    int i ;
    int * f_reading ;
    int * r_reading ;
    int f_numberofreading ;
    int r_numberofreading ;
    int count = 0 ;
    int * flaser_val = NULL ;
    int * rlaser_val = NULL ;
    returnval = beeLaserStartRegularUpdate() ;
    sleep(10) ;
    printf("\n BeeSoft defined callback test") ;
    if(returnval)
    {
        while(count <= 200 )
        {
            f_reading = beeGetLaserValue(0, &f_numberofreading) ;
            r_reading = beeGetLaserValue(1, &r_numberofreading) ; 
            if(flaser_val == NULL)  {
                flaser_val = (int *) malloc(f_numberofreading * sizeof(int)) ;
                rlaser_val = (int *) malloc(r_numberofreading * sizeof(int)) ;
                for (i = 0 ; i < f_numberofreading ; i++)
                      flaser_val[i] = 0 ;
                for (i = 0 ; i < r_numberofreading ; i++)
                      rlaser_val[i] = 0 ;    
            }
            count ++ ;
            for (i = 0 ; i < f_numberofreading ; i++)
                flaser_val[i] += f_reading[i] ; 
            for (i = 0 ; i < r_numberofreading ; i++)
                rlaser_val[i] += r_reading[i] ; 
            if(count%100==0) {
              printf("\n Get laser Value list %d freading %d  ", count, f_numberofreading) ;
            for (int i = 0 ; i < f_numberofreading ; i ++)
               printf("%d ", f_reading[i]/count) ;
            for (int i = 0 ; i < r_numberofreading ; i ++)
               printf("%d ", r_reading[i]/count) ;   
            printf("\n") ; 
 }
        } 
    }
    printf("\n my own callback test") ;
    beeLaserRegisterCallback(my_laser_callback) ;
    sleep(10) ;
    printf("\n Back to BeeSoft defined callback test") ;
    beeLaserRegisterCallback() ;
    sleep(10) ;
    beeLaserStopRegularUpdate() ;
    beeTranslateBy(500.0 ) ;
    sleep(10) ;
    
}

