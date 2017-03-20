/**************************************************************************
 *
 *    BeeSoft Light API 
 *    Example program
 *    This program uses beeGotoAbsolute(). The map is the fifth floor of
 *    Wean Hall
 *
 **************************************************************************/

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
        /* Those destinations in Wean Hall fifth floor */        
        printf("\n Where do you want the robot to go :" ) ;
        printf("\n 1. Just go out of the door of room 5310") ;
        printf("\n 2. Go to the corner of corrider 5300") ;
        printf("\n 3. Go to the door faced to Porter Hall") ;
        printf("\n 4. Go to the front of elevators") ; 
        printf("\n 5. Go back to the room 5310") ;
        printf("\n 6. Stop! Stop! Stop!") ;
        printf("\n 7. Approach the corner of corrider 5300") ;
        printf("\n 8. Approach outside of the room 5310") ;
        printf("\n 9. Approach the inside of the room 5310") ;
        printf("\n 0. Terminate the program") ;
        printf("\n") ;
        fflush(stdout) ;
        fscanf(stdin, "%d", &room_no) ;
        switch (room_no) {
           case 5 : returnval = beeGotoAbsolute(3620.0, 1310.0 );
                    break ;
           case 1 : returnval = beeGotoAbsolute(3570.0, 930.0) ;
                    break ;
           case 2 : returnval = beeGotoAbsolute(5070.0, 820.0) ;
                    break ;
           case 3 : returnval = beeGotoAbsolute(6110.0, 3300.0) ;
                    break ; 
           case 4 : returnval = beeGotoAbsolute(6080.0, 2650.0) ;
                    break;
           case 6 : returnval = beeStop() ;
                    break ;
           case 7 : returnval = beeApproachAbsolute(5070.0, 820.0) ;
                    break ;
           case 8 : returnval = beeApproachAbsolute(3570.0, 930.0) ;
                    break ;
           case 9:  returnval = beeApproachAbsolute(3620.0, 1310.0) ;
                    break ;
                    
           default : go_on = 0 ;
        }
    }
    printf("Thank you for using stupid program\n") ;
}

