#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define SOF_TYPE_CYLINDER   1
#define SOF_TYPE_CUBE       2
#define SOF_TYPE_OBJECT     3

#define TYPE_SIM_V10        1
#define TYPE_SOF_V10        1
#define TYPE_UNKNOWN       -1

/*********************************************************/

typedef struct {

  float                     x;
  float                     y;
  float                     z;
  float                     width;
  float                     length;
  float                     height;
  float                     o;
  char                      *text;

} SOF_cube_type, *SOF_cube_type_ptr;

/*********************************************************/

typedef struct {

  float                     x;
  float                     y;
  float                     z;
  float                     radius;
  float                     height;
  float                     o;
  char                      *text;

} SOF_cylinder_type, *SOF_cylinder_type_ptr;

/*********************************************************/

typedef struct {

  char                      *name;
  float                     x;
  float                     y;
  float                     z;
  float                     o;

} SOF_object_type, *SOF_object_type_ptr;

/*********************************************************/

union SOF_element {

  SOF_cube_type             cube;
  SOF_cylinder_type         cylinder;
  SOF_object_type           object;

};

/*********************************************************/

typedef struct SOF_elements_type {

  int                      obj_type;
  union SOF_element        obj;
  struct SOF_elements_type *next;

} SOF_elements_type, *SOF_elements_type_ptr;

/*********************************************************/

typedef struct SOF_object_def_type {

  char                       *name;
  int                        idnr;      /* for internal use */
  float                      center_x;
  float                      center_y;
  SOF_elements_type          *objs;
  SOF_elements_type          *flat_objs;
  struct SOF_object_def_type *next;

} SOF_object_def_type, *SOF_object_def_type_ptr;

/*********************************************************/

typedef struct {

  char                      *filename;
  FILE                      *fp;
  float                     start_x;
  float                     start_y;
  float                     end_x;
  float                     end_y;
  float                     robot_x;
  float                     robot_y;
  float                     robot_o;
  int                       type;
  SOF_elements_type         *elements;
  SOF_elements_type         *flat_elements;
  SOF_object_def_type       *defines;
} SOF_map_type, *SOF_map_type_ptr;

/*********************************************************/

extern int SOF_read_map( char *filename, SOF_map_type *smap );
extern int SOF_save_elements( char *filename, SOF_map_type map );
extern int SOF_print_defines( SOF_map_type smap );
extern int SOF_print_map( SOF_map_type smap );
