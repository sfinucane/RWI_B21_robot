#include "bee.h"
#include <stdio.h>

void my_ir_callback(int * irupperrow, int * irlowerrow, int * irdrow)
{
fprintf(stderr, "\n my ir callback") ;
    static int Ir_upperval[24] ;
    static int Ir_lowerval[24] ;
    static int Ir_dval[8] ;
    int i ;
    int count = 0 ; 
    for (i = 0 ; i < 24 ; i++)
    {
        Ir_upperval[i] = 0 ;
        Ir_lowerval[i] = 0 ;
    }
    for (i=0 ; i< 8 ; i++) 
        Ir_dval[i] = 0 ;
    for (i = 0 ; i < 24 ; i++)
    {
        Ir_lowerval[i] += irlowerrow[i] ; 
        Ir_upperval[i] += irupperrow[i] ;
    }
    for (i=0 ; i<8 ; i++) 
        Ir_dval[i] += irdrow[i] ;
    count ++ ;
    if(count %1000) 
    {
        printf("\n Average of 1000 values:   ") ;
        for (i = 0 ; i < 24 ; i++)
        {
            printf("%f ", Ir_lowerval[i] / count) ; 
            Ir_lowerval[i] = 0 ; 
        }
        for (i = 0 ; i < 24 ; i++)
        {
            printf("%f ", Ir_upperval[i] / count) ;
            Ir_upperval[i] = 0 ;
        }
        for (i = 0 ; i < 8 ; i++)
        {
            printf("%f ", Ir_dval[i] / count) ;
            Ir_dval[i] = 0 ;
        }
        printf("\n") ;
        count = 0 ;
    }
}

int main(int argc, char *argv[])
{
    beeInitialize("USER");
    int ir_upperval[24] ;
    int ir_lowerval[24] ;
    int ir_dval[8] ;
    bool returnval ;
    int count = 0 ;
    returnval = beeIRStartRegularUpdate() ;
    printf("\n BeeSoft defined callback test") ;
    if(returnval)
    {
        while(count < 2000 )
        {
            returnval = beeGetIRValue(UPPERROW, ir_upperval) ;
            if(returnval!=0) printf("\n Get nonfalse value") ;
            beeGetIRValue(LOWERROW, ir_lowerval) ;
            beeGetIRValue(DROW, ir_dval) ;
            count ++ ;
            if(count%1000==0) {
              printf("\n Get IR Value list %d   ", count) ;
            for (int i = 0 ; i < 24 ; i ++) {
               printf("%f ", ir_upperval[i]) ;
               printf("%f ", ir_lowerval[i]) ;
            }
            for (int i = 0 ; i < 8 ; i ++) 
               printf("%f ", ir_dval[i] ) ;
            printf("\n") ; }
        } 
    }
    printf("\n my own callback test") ;
    beeIRRegisterCallback(my_ir_callback) ;
    sleep(20) ;
    printf("\n Back to BeeSoft defined callback test") ;
    beeIRRegisterCallback() ;
    sleep(10) ;
    beeIRStopRegularUpdate() ;
    beeTranslateBy(500.0 ) ;
    sleep(10) ;
     
}

