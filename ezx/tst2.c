
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** Welcome!
 *****
 ***** This file is part of the robot control software provided
 ***** by Real World Interface Inc.
 *****
 ***** All copyrights are by Real World Interface Inc., Carnegie
 ***** Mellon University, and the University of Bonn, Germany.
 ***** Removing the copyright information is illegal. Please read
 ***** and make sure you understand the disclaimer below.
 *****
 ***** Contact tyson@rwii.com if you have questions or would
 ***** like to obtain further information.
 *****
 ***** We hope you enjoy working with this software package.
 *****
 *****                      Tyson D. Sawyer and Sebastian Thrun
 *****
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** 
 ***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED
 ***** BY APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING
 ***** THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM
 ***** "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR
 ***** IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 ***** OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE
 ***** ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM
 ***** IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME
 ***** THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
 ***** 
 ***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
 ***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
 ***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
 ***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
 ***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
 ***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO
 ***** LOSS OF DATA OR DATA BEING RENDERED INACCURATE OR LOSSES
 ***** SUSTAINED BY YOU OR THIRD PARTIES OR A FAILURE OF THE PROGRAM
 ***** TO OPERATE WITH ANY OTHER PROGRAMS OR FAILURE TO CONTROL A
 ***** PHYSICAL DEVICE OF ANY TYPE), EVEN IF SUCH HOLDER OR OTHER
 ***** PARTY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 ***** Source file:     $Source: /usr/local/cvs/bee/src/ezx/tst2.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:27:28 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: tst2.c,v $
 * Revision 1.1  2002/09/14 15:27:28  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:35  rhino
 * General reorganization of the directories/repository, fusion with the
 * RWI software.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/



#ifdef VMS
#include "vms.h"
struct timeval {
        long    tv_sec;         /* seconds */
        long    tv_usec;        /* and microseconds */
};
#endif

#include "EZX11.h"

main()
{
   EZXW_p w1, w2, w3, w4;
   int i,x,y,x1,y1,x2,y2,x3=0,y3=0,x4,y4,x5=0,y5=0,flag=0;

   char **fontList;
   int fontListNum;

   /* EZX_InitX(); */
   w1 = EZX_MakeWindow("demo",     300, 300, "+600+140");  /* "+200+200" */
   w2 = EZX_MakeWindow("action",   300, 300, "+200+80");  /* "+200+200" */
   w3 = EZX_MakeWindow("graphics", 500, 400, "+500+480");  /* "+200+200" */
   w4 = EZX_MakeWindow("graphics", 400, 300, "+50+480");  /* "+200+200" */

   EZX_SetColor(C_RED);

   for(i=10; i<90; i++)
     EZX_DrawRectangle(w2,i,i/2,i,i);

   EZX_Flush();

   EZX_SetColor(C_FORESTGREEN);

   EZX_DrawLine(w1,10,10,100,100);
   EZX_SetLineStyle(LineOnOffDash);
   EZX_DrawLine(w1,20,10,110,100);
   EZX_SetDashes(2,6);
   EZX_SetLineStyle(LineDoubleDash);
   EZX_DrawLine(w1,30,10,120,100);
   EZX_SetLineStyle(LineSolid);

   EZX_SetLineWidth(5);
   EZX_DrawLine(w1,110,110,200,200);

   EZX_SetLineStyle(LineDoubleDash);
   EZX_DrawLine(w1,110,10,200,100);


   EZX_SetLineWidth(15);
   EZX_SetLineStyle(LineSolid);
   EZX_SetColor(C_BLUE);
   EZX_DrawLine(w1,110,30,200,10);

   EZX_SetMode(GXxor);
   EZX_DrawLine(w1,110,50,200,30);
   EZX_SetMode(GXor);
   EZX_DrawLine(w1,110,70,200,50);
   EZX_SetColor(C_YELLOW);
   EZX_DrawLine(w1,110,90,200,70);

   EZX_UseFont(theGC, "lucidasans-18");
   EZX_SetColor(C_RED);
   EZX_DrawTextAt(w1, 150, 280-EZX_GetFontHeight(), "Press any button",'C'); 
   EZX_DrawTextAt(w1, 150, 280, "in top left window to quit",'C');

   EZX_UseFont(theGC, "8x16");
   EZX_SetColor(C_FORESTGREEN);
   EZX_DrawTextAt(w3, 250, 380-2*EZX_GetFontHeight(), 
    "Try pressing",'C'); 
   EZX_DrawTextAt(w3, 250, 380-EZX_GetFontHeight(), 
    "left, middle and right button",'C');
   EZX_DrawTextAt(w3, 250, 380, 
    "in here",'C');

   EZX_UseFont(theGC, "7x13");
   EZX_SetColor(C_BLUE);
   EZX_DrawTextAt(w4, 200, 280-2*EZX_GetFontHeight(), 
    "Try pressing",'C'); 
   EZX_DrawTextAt(w4, 200, 280-EZX_GetFontHeight(), 
    "left, middle and right button",'C');
   EZX_DrawTextAt(w4, 200, 280, 
    "in here",'C');

   EZX_SetLineWidth(5);
   EZX_SetLineStyle(LineSolid);
   EZX_SetMode(GXcopy);

   x=random()%300;
   y=random()%300;
   while (! EZX_TestCursor(w2))
     {
     x1=(x-15+(random()%31)); 
     if (x1>299) x1=299-(x1-299);
     if (x1<0) x1=-x1;
     y1=(y-15+(random()%31));
     if (y1>299) y1=299-(y1-299);
     if (y1<0) y1=-y1;
     switch (random()%5)
       {
       case 0: EZX_SetColor(C_RED); break;
       case 1: EZX_SetColor(C_BLUE); break;
       case 2: EZX_SetColor(C_FORESTGREEN); break;
       case 3: EZX_SetColor(C_YELLOW); break;
       case 4: EZX_SetColor(C_PINK); break;
       }
     EZX_DrawLine(w2,x,y,x1,y1); x=x1; y=y1;

     if (EZX_TestCursor(w3))
       {
       switch (EZX_TestGetCursor(w3,&x2,&y2))
	  {
	  case LEFT_BUTTON:   x3=x2; y3=y2; 
			      break;
          case MIDDLE_BUTTON: {
         		      int save = EZX_SetColor(C_BLACK);

 		              EZX_DrawLine(w3,x3,y3,x2,y2);
  
  		              EZX_SetColor(save);

		              flag=0;
         		      }
			      break;
	  case RIGHT_BUTTON:  EZX_ClearWindow(w3);
			      break;
	  default:	      break;
          }
       }

     if (EZX_TestCursor(w4))
       {
       switch (EZX_TestGetCursor(w4,&x4,&y4))
	  {
	  case LEFT_BUTTON:   x5=x4; y5=y4; 
			      break;
          case MIDDLE_BUTTON: {
         		      int save = EZX_SetColor(C_BLACK);

 		              EZX_DrawLine(w4,x5,y5,x4,y4);
  
  		              EZX_SetColor(save);

		              flag=0;
         		      }
			      break;
	  case RIGHT_BUTTON:  EZX_ClearWindow(w4);
			      break;
	  default:	      break;
          }
       }

     if (EZX_TestCursor(w1))
       {
       EZX_TestGetCursor(w1,&x2,&y2);
       EZX_bell();
       fprintf(stderr,"don't press button in that window!\n");
       }

     EZX_Flush();	
     }

  EZX_EndWindow(w1);
  EZX_EndWindow(w2);
  EZX_EndWindow(w3);

}

