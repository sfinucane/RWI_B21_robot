/*
 * $Id: button.h,v 1.1 2002/09/14 15:44:40 rstone Exp $
 */
#ifndef BUTTON_H_LOADED
#define BUTTON_H_LOADED

BUTTONS_status_reply_type status;

char *tcxMachine;

int buttons_effect;

int resetButtons();

void setButton(int button, unsigned char position);

void interrupt_handler();

int check_buttons();

void check_and_set_buttons();

#endif
/*
 * $Log: button.h,v $
 * Revision 1.1  2002/09/14 15:44:40  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1997/11/06 17:53:04  swa
 * Fixed a bug with TCXHOST. If the environment variable TCXHOST is set,
 * take it. If not, take the TCXHOST from beeSoft.ini. If this doesn't
 * work, take localhost. If this doesn't work: bummer.
 *
 * Revision 1.2  1997/07/04 16:15:19  swa
 * Added lib support for buttons. No more TCX.
 *
 * Revision 1.1  1997/06/29 21:58:27  thrun
 * Added a ton of files for a client/server button thing.                (swa)
 *
 */
