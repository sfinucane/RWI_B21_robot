
#define WORLD_SIZE_X 50
#define WORLD_SIZE_Y 50
#define NUMBER_ANGLES 36

#define MAX_NUM_STEPS 100
#define ANGULAR_RESOLUTION (360.0 / ((float) (NUMBER_ANGLES))


#define NUM_MATRICES_PER_ROW 9	/* change this, if you want */


#define MATRIX_SIZE_X        2.0
#define MATRIX_SIZE_Y        (MATRIX_SIZE_X)
#define BUTTON_SIZE_Y        0.4
#define MATRIX_SEPARATOR     0.1
#define MAX_X                ((NUM_MATRICES_PER_ROW) * ((MATRIX_SIZE_X) + (MATRIX_SEPARATOR)))


#define NUM_WINDOWS_PER_DATA_ITEM 3
#define VISUAL_RANGE 5.0


/************************************************************************\
 ************************************************************************
\************************************************************************/


void
exit_proc(int garbage);


void
init_graphics();


void
register_density_window(float probs[NUM_WINDOWS_PER_DATA_ITEM]
			[WORLD_SIZE_X][WORLD_SIZE_Y],
			int *button_window_id,
			int probs_window_id[NUM_WINDOWS_PER_DATA_ITEM],
			int *markers_window_id,
			int bounding_box_window_id[NUM_WINDOWS_PER_DATA_ITEM]);


void
add_data_item(int delta_x, int delta_y,
	      int error_x, int error_y);


void
set_initial_position(int pos_x, int pos_y);


void
saw_landmark(int rel_x, int rel_y);


int 
mouse_test_loop();


void
update_density();


int
main(int argc, char *argv[]);

void
normalize_density(int n, int k);

void
display_density(int n, int k);

void
internal_add_data_item(int delta_x, int delta_y,
		       int error_x, int error_y, int new_episode);
