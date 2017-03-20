/*******************************************************************************
 * 
 *   BeeSoft Light 
 *   Example program
 *   This program uses beeGotoAbsolute()
 *   The map is "floor.map"
 *
 **********************************************************************************
 */ 
#include "bee.h"
#include <stdio.h>


int main(int argc, char *argv[])
{
    beeInitialize("ROBOT");
    int room_no;
    bool returnval ;
    int go_on = 1 ;
    while (go_on)
    {
        /*  Those destination are points in "floor.map" */        
        printf("\n Which room do you want to enter :" ) ;
        fflush(stdout) ;
        fscanf(stdin, "%d", &room_no) ;
        switch (room_no) {
           case 1 : returnval = beeGotoAbsolute(760.0, 1480.0) ;
                    break ;
           case 2 : returnval = beeGotoAbsolute(1065.0, 1630.0) ;
                    break ;
           case 3 : returnval = beeGotoAbsolute(1490.0, 1850.0) ;
                    break ; 
           case 4 : returnval = beeGotoAbsolute(1875.0, 1575.0) ;
                    break;
           case 5 : returnval = beeGotoAbsolute(2385.0, 1690.0) ;
                    break;
           case 6 : returnval = beeGotoAbsolute(725.0, 380.0) ;
                    break;
           case 7 : returnval = beeGotoAbsolute(1055.0, 410.0) ;
                    break;
           case 8 : returnval = beeGotoAbsolute(1530.0, 645.0) ;
                    break;
           case 9 : returnval = beeGotoAbsolute(1885.0, 340.0) ;
                    break;
           case 10: returnval = beeGotoAbsolute(2310.0, 620.0) ;
                    break;
           default : go_on = 0 ;
        }
    }
    printf("Thank you for using stupid program\n") ;
}

