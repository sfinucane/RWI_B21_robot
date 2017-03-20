#include "bee.h"
#include <stdio.h>
#include <math.h>

#define bool int

int main(int argc, char *argv[])
{
    int command ;
    float param[3] ;
    int paramnum ;
    bool returnval ;
    int go_on = 1 ;
    robot_position * robotPosition ;
    robot_speed * robotSpeed ;
    beeInitialize("ROBOT");
    while (go_on)
    {
        
        printf("\n   1. beeTranslateBy(float distance)") ;
        printf("\n   2. beeTranslatePositive(void).") ;  
        printf("\n   3. beeTranslateNegative(void).")  ;
        printf("\n   4. beeRotateBy(float degree)") ;
        printf("\n   5. beeRotatePositive(void).") ;
        printf("\n   6. beeRotateNegative(void).") ;
        printf("\n   7. beeSetTransVelocity(float velocity)")  ;
        printf("\n   8. beeSetRotVelocity(float velocity)") ;
        printf("\n   9. beeSetTransAcceleration(float acceleration)") ;
        printf("\n   10. beeSetRotAcceleration(float acceleration)") ;
        printf("\n   11. beeStop()")  ;
        printf("\n   12. beeApproachRelative(float rel_target_x, float rel_target_y)") ;
        printf("\n   13. beeApproachAbsolute(float target_x, float target_y)")  ;
        printf("\n   14. beeGetCurrentPosition()") ;
        printf("\n   15. beeGetCurrentSpeed()") ;
        printf("\n   16. beeGotoRelative(float rel_target_x, float rel_target_y)")
; 
        printf("\n   17. beeGotoAbsolute(float target_x, float targett_y)") ;
        printf("\n   18. quit ") ;
        printf("\n Please input a command :" ) ;
        fflush(stdout) ;
        fscanf(stdin, "%d", &command) ;
        switch (command)
        {
           case 2 :
           case 3 :
           case 5 :
           case 6 :
           case 11:
           case 14 :
           case 15 : paramnum = 0 ;
                     break ;
           case 1 :
           case 4 :
           case 7 :
           case 8 :
           case 9 :
           case 10 :
                     paramnum = 1 ;
                     break ;
           case 12 :
           case 13 :
           case 16 :
           case 17 :
                     paramnum = 2 ;
                     break ;
           default : paramnum = 0 ;       
                     break ;
        }
        // printf("I get %d", command) ;
        if(paramnum) 
        {
             int i;
             printf("\n Please input %d parameters: ", paramnum ) ;
             fflush(stdout) ;
             for (i=0 ; i < paramnum ; i++)
                 fscanf(stdin, "%f", &param[i]) ;
        }
        switch (command) {
           case 1 : returnval = beeTranslateBy(param[0]) ;
                    break ;
           case 2 : returnval = beeTranslatePositive() ;
                    break ;
           case 3 : returnval = beeTranslateNegative() ;
                    break ;
           case 4 : returnval = beeRotateBy(param[0]) ;
                    break ;
           case 5 : returnval = beeRotatePositive() ;
                    break ;
           case 6 : returnval = beeRotateNegative() ;
                    break ;
           case 7 : returnval = beeSetTransVelocity(param[0]) ;
                    break ;
           case 8 : returnval = beeSetRotVelocity(param[0]) ;
                    break ;
           case 9 : returnval = beeSetTransAcceleration(param[0]) ;
                    break ;
           case 10 : returnval = beeSetRotAcceleration(param[0]) ;
                    break ;
           case 11 : returnval = beeStop() ;
                    break ;
           case 12 : returnval = beeApproachRelative(param[0], param[1]) ;
                    break ;
           case 13 : returnval = beeApproachAbsolute(param[0], param[1]) ;
                    break;
           case 14 : robotPosition = beeGetCurrentPosition() ;
                     if(robotPosition)
                     printf("\nGet current position: %f %f %f", robotPosition->pos_x, robotPosition->pos_y, robotPosition->orientation) ;
                     else printf("\n Current Position is not available now") ;
                    break ;
           case 15 : robotSpeed = beeGetCurrentSpeed() ;
                     if(robotSpeed)
                     printf("\nGet current speed: %f %f", robotSpeed->trans_speed, robotSpeed->rot_speed) ;
                     else printf("\n Current Speed is not available now") ;
                     break ;
           case 16 : returnval = beeGotoRelative(param[0], param[1]) ;
                    break ; 
           case 17 : returnval = beeGotoAbsolute(param[0], param[1]) ;
                    break;
           
           case 18 :  go_on = 0 ;
                    break ;
           default : returnval = 0 ;
        }
        if (returnval) 
        printf("return value is TRUE") ;
        else printf("return value is FALSE") ;
    }
}

