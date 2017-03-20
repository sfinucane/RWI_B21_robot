#include "bee.h"
#include <stdio.h>
void my_sonar_callback(float * sonar)
{
    int i ;
    static float Sonar_val[24] ;
    int count = 0 ; 
    for (i = 0 ; i < 24 ; i++)
        Sonar_val[i] = 0 ;
    for (i = 0 ; i < 24 ; i++)
        Sonar_val[i] += sonar[i] ; 
    printf("\n invoke !") ;
    count ++ ;
    if(count %1000) 
    {
        printf("\n Average of 1000 values:   ") ;
        for (i = 0 ; i < 24 ; i++)
        {
            printf("%f ", Sonar_val[i] / count) ; 
            Sonar_val[i] = 0 ; 
        }
        printf("\n") ;
        count = 0 ;
    }
}

int main(int argc, char *argv[])
{
    beeInitialize("USER");
    float *SonarVal ;
    int number ;
    bool returnval ;
    int count = 0 ;
    returnval = beeSonarStartRegularUpdate() ;
    printf("\n BeeSoft defined callback test") ;
    if(returnval)
    {
        while(count < 2000 )
        {
            SonarVal = beeGetSonarValue(&number) ;
            fprintf(stderr, "\n location %p", SonarVal) ;
            count ++ ;
            if(count%1000==0) {
              printf("\n Get Sonar Value list %d   ", count) ;
            for (int i = 0 ; i < number ; i ++)
               printf("%f ", SonarVal[i]) ;
            printf("\n") ; }
        } 
    }
    printf("\n my own callback test") ;
    beeSonarRegisterCallback(my_sonar_callback) ;
    sleep(10) ;
    printf("\n Back to BeeSoft defined callback test") ;
    beeSonarRegisterCallback() ;
    sleep(10) ;
    beeSonarStopRegularUpdate() ;
    beeTranslateBy(500.0 ) ;
     
}

