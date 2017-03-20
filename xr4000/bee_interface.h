


void
bee_init( char* robotName);

void
bee_check();

void 
bee_send_status_update(float pos_x, float pos_y, float pos_o,
		       float tvel, float rvel);


void 
bee_send_sonar_update(float values[MAX_NUM_SONARS]);

void
bee_check_trans_motion_terminator(float pos_x, float pos_y, float pos_o);

void
bee_set_trans_motion_terminator(float dist);

void
bee_check_rot_motion_terminator(float pos_x, float pos_y, float pos_o);

void
bee_set_rot_motion_terminator(float dist);

void
periodically_unlock_brake();
