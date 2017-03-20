/*
 * $Id: buttonHandlers.h,v 1.1 2002/09/14 15:44:40 rstone Exp $
 */

void BUTTON_close_handler( char *name, TCX_MODULE_PTR module );

int BUTTONS_initialize_tcx();

void send_automatic_status_update();

void BUTTONS_register_auto_update_handler( TCX_REF_PTR                      ref,
					  BUTTONS_register_auto_update_ptr data);

void BUTTONS_status_query_handler( TCX_REF_PTR ref );

void BUTTONS_effect_handler( TCX_REF_PTR ref );

void BUTTONS_set_lights_handler( TCX_REF_PTR           ref,
				 BUTTONS_set_lights_ptr data );

void BUTTONS_setButton_handler( TCX_REF_PTR          ref,
				BUTTONS_setButton_ptr data );

void BUTTONS_simulate_button_press_handler(TCX_REF_PTR          ref,
				      int *data);

/*
 * $Log: buttonHandlers.h,v $
 * Revision 1.1  2002/09/14 15:44:40  rstone
 * *** empty log message ***
 *
 * Revision 1.4  1997/07/30 02:51:22  thrun
 * Oops. Something went wrong.
 * This version has more consistent status update, and a message
 * for simulating button presses
 *
 * Revision 1.3  1997/07/04 17:26:03  swa
 * Added lib support for buttons. Renamed the executable buttonServer to be
 * more consistent. No root permissions are necessary to run the buttonServer.
 * Renamed tons of things to be backward-compatible.
 *
 * Revision 1.2  1997/06/29 22:13:26  thrun
 * Compiles nicely with "-Wall" (swa)
 *
 * Revision 1.1  1997/06/29 21:58:28  thrun
 * Added a ton of files for a client/server button thing.                (swa)
 *
 */
