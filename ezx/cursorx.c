
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/ezx/cursorx.c,v $
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
 * $Log: cursorx.c,v $
 * Revision 1.1  2002/09/14 15:27:28  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1996/10/25 12:03:50  ws
 * Initx now includes stdlib.h and
 * XLookupString in cursorx.c now uses &theKeyEvent
 *
 * Revision 1.1.1.1  1996/09/22 16:46:34  rhino
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





/*
 *  cursorx.c
 *
 *  Created by Goang-Tay Hsu (gth).
 *  Modified by Long-Ji Lin (ljl) at Aug 1, 1990.
 *
 */

#ifdef VMS
#include "vms.h"
struct timeval {
        long    tv_sec;         /* seconds */
        long    tv_usec;        /* and microseconds */
};
#endif

#include "EZX11.h"

static XButtonEvent theButtonEvent;
static XEvent theEvent;
static XMotionEvent theMotionEvent;
static XKeyEvent theKeyEvent;

                                    /* For holding the button event */
extern GetMotionEvent;

static struct timeval Event_timeout;

void EZX_bell(void)
{
putc(7,stderr);
}

/* get cursor position when button pressed */

int EZX_GetCursor(int *xp, int *yp)
{
return EZX_GetCursorw(xp,yp,NULL);
}

int EZX_GetCursorw(int *xp, int *yp, Window *win)
{
    do {
      XNextEvent(theDisplay, &theEvent);
      theButtonEvent=theEvent.xbutton;
    } while (theButtonEvent.type != ButtonPress);

    if (win!=NULL)  *win  = theButtonEvent.window;
    *xp = theButtonEvent.x;
    *yp = theButtonEvent.y;

    if (theButtonEvent.button == Button1) return(LEFT_BUTTON);
    else if (theButtonEvent.button == Button2) return(MIDDLE_BUTTON);
    else if (theButtonEvent.button == Button3) return(RIGHT_BUTTON);
    else return(OTHER_BUTTON);
}





/* get cursor position when button pressed */
int EZX_TestGetCursorw(int *xp, int *yp, Window *win)
{
  *xp = -1;
  *yp = -1;

  if (XCheckMaskEvent(theDisplay, ButtonPressMask, &theEvent)){
    
    theButtonEvent=theEvent.xbutton;
    if (win != NULL)
      *win = theButtonEvent.window; /* this is the window the button
				     * was pressed in */
    *xp = theButtonEvent.x;
    *yp = theButtonEvent.y;
    
    if (theButtonEvent.button == Button1) return(LEFT_BUTTON);
    else if (theButtonEvent.button == Button2) return(MIDDLE_BUTTON);
    else if (theButtonEvent.button == Button3) return(RIGHT_BUTTON);
    else return(OTHER_BUTTON);
  }
  
  else if (XCheckTypedEvent(theDisplay, MotionNotify, &theEvent)){

    do{
      theMotionEvent=theEvent.xmotion;
      if (win != NULL)
	*win = theMotionEvent.window; /* this is the window pointer
				       * was moved in */
      *xp = theMotionEvent.x;
      *yp = theMotionEvent.y;
    }
    while(XCheckTypedEvent(theDisplay, MotionNotify, &theEvent));

    return -1;
  }

  else{				/* ...nothing found here */
    *xp = -1;
    *yp = -1;
    return -1;
  } 
}



/* get cursor position when button pressed */
int EZX_TestGetCursor(EZXW_p w, int *xp, int *yp)
{
  Window win;
  int ret_value;

  ret_value = EZX_TestGetCursorw(xp, yp, &win);
  if (w != NULL && win != w->w)
    *xp = *yp = ret_value = -1;
  return  ret_value;
}





/* set flag when button pressed */
int EZX_TestCursor(EZXW_p w)
{
      

    XCheckMaskEvent(theDisplay, ButtonPressMask, &theEvent);
					
    if (w==NULL)     
      return (theEvent.xbutton.type == ButtonPress);
    else
      return ((theEvent.xbutton.type == ButtonPress) &&
              (theEvent.xbutton.window==w->w));
}


/* get cursor positions when button pressed and released */
int EZX_GetCursors(int *xp, int *yp, int *xr, int *yr)
{
return EZX_GetCursorsw(xp, yp, xr, yr, NULL);
}

int EZX_GetCursorsw(int *xp, int *yp, int *xr, int *yr, Window *win)
{
    do {
      XNextEvent(theDisplay, &theEvent);
      theButtonEvent=theEvent.xbutton;
    } while (theButtonEvent.type != ButtonPress);

    if (win!=NULL) *win=theButtonEvent.window;
    *xp = theButtonEvent.x;
    *yp = theButtonEvent.y;

    do {
	XNextEvent(theDisplay, &theEvent);
	theButtonEvent=theEvent.xbutton;
     } while (theButtonEvent.type != ButtonRelease);

    *xr = theButtonEvent.x;
    *yr = theButtonEvent.y;

    if (theButtonEvent.button == Button1) return(LEFT_BUTTON);
    else if (theButtonEvent.button == Button2) return(MIDDLE_BUTTON);
    else if (theButtonEvent.button == Button3) return(RIGHT_BUTTON);
    else return(OTHER_BUTTON);
}


int EZX_block_and_wait(struct timeval *timeout)
{
  int stat, xfd;
  fd_set readMask;
  
  xfd = XConnectionNumber(theDisplay);
  
  FD_ZERO(&readMask);

  FD_SET(xfd,&readMask);
      
  stat = select(FD_SETSIZE, &readMask, NULL, NULL, timeout);

  if (stat > 0) 
    {
      if (FD_ISSET(xfd,&readMask))
	return 1;
      else
	return 2;
    }
  else return 0;
}


/**************************************************************************
 * EZX_GetEvent                            Boris AT Trouvain              *
 **************************************************************************/
int EZX_GetEvent ( EZX_EventPtr EZX_Event )
{
  Window root,child;
  unsigned int keys_buttons;
  int pos_x,pos_y,root_x,root_y;
  char buffer[20];
  int bufsize = 20;
  KeySym keysym;
  XComposeStatus compose;
  int charcount;
  
  if ( EZX_Event != NULL )
    {
      EZX_Event->type = EZX_Event_was_Nothing;

      if ( XCheckMaskEvent ( theDisplay,ButtonPressMask | ButtonReleaseMask
			    | KeyPressMask | PointerMotionHintMask |
			    Button1MotionMask | Button2MotionMask |
			    Button3MotionMask | Button4MotionMask | 
			    Button5MotionMask , &theEvent ) )
	{
	  
	  switch ( theEvent.type )
	    {
	    case KeyPress:
	      theKeyEvent = theEvent.xkey;
	      EZX_Event->type = EZX_Event_was_Key_Press;
	      EZX_Event->win = theKeyEvent.window;
	      EZX_Event->PointerX = theKeyEvent.x;
	      EZX_Event->PointerY = theKeyEvent.y;
	      
	      charcount = XLookupString ( &theKeyEvent,buffer,bufsize,&keysym,
					 &compose );
	      EZX_Event->Key = keysym;  /* hopefully an int */
	      /* There is yet no access to composed characters such as
		 ctrl-a alt-t and so on but this will be solved soon */
	      break;
	    case ButtonPress:
	      theButtonEvent = theEvent.xbutton;
	      EZX_Event->type = EZX_Event_was_Button_Press;
	      EZX_Event->win = theButtonEvent.window;
	      EZX_Event->PointerX = theButtonEvent.x;
	      EZX_Event->PointerY = theButtonEvent.y;
	      EZX_Event->Button = theButtonEvent.button;
	      break;
	    case ButtonRelease:
	      theButtonEvent = theEvent.xbutton;
	      EZX_Event->type = EZX_Event_was_Button_Release;
	      EZX_Event->win = theButtonEvent.window;
	      EZX_Event->PointerX = theButtonEvent.x;
	      EZX_Event->PointerY = theButtonEvent.y;
	      EZX_Event->Button = theButtonEvent.button;
	      break;
	    case MotionNotify:
	      theMotionEvent = theEvent.xmotion;
	      EZX_Event->type = EZX_Event_was_Motion;
	      EZX_Event->win = theMotionEvent.window;
	      EZX_Event->PointerX = theMotionEvent.x;
	      EZX_Event->PointerY = theMotionEvent.y;
	      if ( XQueryPointer ( theDisplay,theMotionEvent.window,
				  &root,&child,&root_x,&root_y,
				  &pos_x,&pos_y,&keys_buttons ))
		{
		  EZX_Event->type = EZX_Event_was_Nothing;
		  switch ( keys_buttons )
		    {
		    case 256: 
		      EZX_Event->Button = 1;
		      EZX_Event->type = EZX_Event_was_Motion;
		      break;
		    case 512:
		      EZX_Event->Button = 2;
		      EZX_Event->type = EZX_Event_was_Motion;
		      break;
		    case 1024:
		      EZX_Event->Button = 3;
		      EZX_Event->type = EZX_Event_was_Motion;
		      break;
		    }
		}
	      break;
	      
	    }
	  
	}
    }
    return 0;
}

/**************************************************************************
 * EZX_GetEventT                           Boris AT Trouvain              *
 **************************************************************************/
int EZX_GetEventT ( EZX_EventPtr EZX_Event )
{

  while ( EZX_block_and_wait ( NULL ) == 0 )
    {
    }
  
  EZX_GetEvent ( EZX_Event );
    
}

/**************************************************************************
 * EZX_GetEventT2                           Boris AT Trouvain             *
 **************************************************************************/
int EZX_GetEventT2 ( EZX_EventPtr EZX_Event , long microseconds )
{
  Event_timeout.tv_usec = microseconds;
  Event_timeout.tv_sec = 0;
  while ( EZX_block_and_wait ( &Event_timeout ) == 0 )
    {
    }
  
  EZX_GetEvent ( EZX_Event );
    
}

