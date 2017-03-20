#include <stdio.h>
#include "libsof.h"

int
main( int argc, char *argv[] )
{
  SOF_map_type map;

  if (argc != 3) {
    fprintf(stderr, "usage: %s soffile simfile\n", argv[0]);
    exit(0);
  }      

  if (SOF_read_map( argv[1], &map ))
    fprintf( stderr, "SOF: stop!\n" );
  else {
    fprintf( stderr, "SOF: MAP:\n\n" );
    SOF_print_map( map );
    fprintf( stderr, "\n\nSOF: DEFINES:\n" );
    SOF_print_defines( map );
    SOF_save_elements( argv[2], map );
  }
  exit(0);
}
