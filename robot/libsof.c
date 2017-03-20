#include <stdio.h>
#include <math.h>
#include "libsof.h"

typedef struct listelement {

  int                  val;
  struct listelement   *next;

} listelement;

int SOF_check_defines( SOF_map_type smap );
int SOF_flatten_map( SOF_map_type *smap );
int SOF_flatten_defines( SOF_map_type *smap );

listelement * 
AddItem ( listelement * listpointer, int data) {
  listelement * lp = listpointer;
  if (listpointer != NULL) {
    while (listpointer->next != NULL)
      listpointer = listpointer->next;
    listpointer->next = (struct listelement *) 
      malloc (sizeof (listelement));
    listpointer = listpointer->next;
    listpointer->next = NULL;
    listpointer->val = data;
    return lp;
  } else {
    listpointer = (struct listelement  *)
      malloc (sizeof (listelement));
    listpointer->next = NULL;
    listpointer->val = data;
    return listpointer;
  }
}

listelement *
RemoveItem (listelement * listpointer) {
  listelement * tempp;
  tempp = listpointer->next;
  free (listpointer);
  return tempp;
}

listelement *
ClearQueue (listelement * listpointer) {
  while (listpointer != NULL) {
    listpointer = RemoveItem (listpointer);
  }
  return NULL;
}

void 
PrintQueue (listelement * listpointer) {
  fprintf( stderr, "list: " );
  if (listpointer == NULL)
    fprintf( stderr, "queue is empty!\n" );
  else
    while (listpointer != NULL) {
      fprintf( stderr, "%d  ", listpointer->val );
      listpointer = listpointer->next;
    }
  fprintf( stderr, "\n" );
}

int 
FindInQueue (listelement * listpointer, int val) {
  int retval=1;
  if (listpointer != NULL)
    while (listpointer != NULL) {
      if (val==listpointer->val) {
	retval=0;
	break;
      }
      listpointer = listpointer->next;
    }
  return(retval);
}

SOF_elements_type *
SOF_add_cube( SOF_elements_type *elements, 
	      float x, float y, float z, 
	      float w, float l, float h,
	      float o, char *id )
{
  elements->next = (SOF_elements_type *) 
    malloc( sizeof(SOF_elements_type) );
  elements->obj_type = SOF_TYPE_CUBE;
  elements->obj.cube.x      = x;
  elements->obj.cube.y      = y;
  elements->obj.cube.z      = z;
  elements->obj.cube.width  = w;
  elements->obj.cube.length = l;
  elements->obj.cube.height = h;
  elements->obj.cube.o      = o;
  if (id!=NULL) {
    elements->obj.cube.text = (char *) malloc((strlen(id)+1)*sizeof(char));
    strcpy( elements->obj.cube.text, id );
  } else
    elements->obj.cube.text = id;
  return(elements->next);
}

SOF_elements_type *
SOF_add_cylinder( SOF_elements_type *elements, 
		  float x, float y, float z, 
		  float r, float h, char *id )
{
  elements->next = (SOF_elements_type *) 
    malloc( sizeof(SOF_elements_type) );
  elements->obj_type = SOF_TYPE_CYLINDER;
  elements->obj.cylinder.x      = x;
  elements->obj.cylinder.y      = y;
  elements->obj.cylinder.z      = z;
  elements->obj.cylinder.radius = r;
  elements->obj.cylinder.height = h;
  if (id!=NULL) {
    elements->obj.cylinder.text = (char *) malloc((strlen(id)+1)*sizeof(char));
    strcpy( elements->obj.cylinder.text, id );
  } else
    elements->obj.cylinder.text = id;
  return(elements->next);
}

SOF_elements_type *
SOF_add_object( SOF_elements_type *elements, 
		float x, float y, float z, 
		float o, char *name )
{
  elements->next = (SOF_elements_type *) 
    malloc( sizeof(SOF_elements_type) );
  elements->obj_type = SOF_TYPE_OBJECT;
  elements->obj.object.x      = x;
  elements->obj.object.y      = y;
  elements->obj.object.z      = z;
  elements->obj.object.o      = o;
  if (name!=NULL) {
    elements->obj.object.name = (char *) malloc((strlen(name)+1)*sizeof(char));
    strcpy( elements->obj.object.name, name );
  } else
    elements->obj.object.name = name;
  return(elements->next);
}

int
SOF_read_cube( char *line, SOF_elements_type *element )
{
  char com[255] ="";
  float x,y,z,h,l,w,o;
  int retval;
  retval = sscanf( line, "%f %f %f %f %f %f %f %s", 
		   &x, &y, &z, &w, &l, &h, &o, com );
  if ( retval != EOF) {
    if ((retval==7) || (retval==8)) {
      SOF_add_cube( element, x, y, z, w, l, h, o, com );
      return(0);
    } else 
      return(1);
  } else
    return(1);
}

int
SOF_read_cylinder( char *line, SOF_elements_type *element )
{
  char com[255];
  float x,y,z,h,r;
  int retval;
  retval = sscanf( line, "%f %f %f %f %f %s", 
		   &x, &y, &z, &r, &h, com );
  if ( retval != EOF) {
    if ((retval==5) || (retval==6)) {
      SOF_add_cylinder( element, x, y, z, r, h, com );
      return(0);
    } else 
      return(1);
  } else
    return(1);
}

int
SOF_read_object( char *line, SOF_elements_type *element )
{
  int retval;
  float x,y,z,o;
  char oname[255];
  retval = sscanf( line, "%s %f %f %f %f", 
		   oname, &x, &y, &z, &o );
  if ( (retval!=EOF) && (retval==5) ) {
    SOF_add_object( element, x, y, z, o, oname );
    return(0);
  } else
    return(1);
}

int
SOF_read_center( char *line, SOF_object_def_type *def )
{
  int retval;
  float x,y;
  retval = sscanf( line, "%f %f", &x, &y );
  if ( (retval!=EOF) && (retval==2) ) {
    def->center_x = x;
    def->center_y = y;
  } else {
    fprintf( stderr, "READ: error in CENTER line - %s\n", line );
    return(1);
  }
  return(0);
}

int
SOF_read_object_define( char *line, SOF_object_def_type *def )
{
  int retval;
  char oname[255];
  retval = sscanf( line, "%s", oname );
  if ( (retval!=EOF) && (retval==1) ) {
    def->name  = (char *) malloc((strlen(oname)+1)*sizeof(char));
    strcpy( def->name, oname );
  } else
    return(1);
  return(0);
}

int
SOF_read_map( char *filename, SOF_map_type *smap ) 
{
  int FEnd;
  FILE *iop;
  char command[256];
  char tcommand[256];
  char line[256];
  int RElem = 0;
  int dummy;
  int objdef = 0;
  int defnr = 0;
  int numl = 0;
  SOF_elements_type *element, *defelement;
  SOF_object_def_type *odefine;
  
  smap->elements = (SOF_elements_type *)  malloc(sizeof(SOF_elements_type));
  element = smap->elements;

  smap->defines = 
    (SOF_object_def_type *)  malloc(sizeof(SOF_object_def_type));
  odefine = smap->defines;

  smap->type = TYPE_UNKNOWN;

  if ((iop = fopen( filename, "r")) == 0){
    fprintf(stderr, "READ: could not open input file %s\n", filename );
    return(-1);
  } else {
    fprintf(stderr, "READ: read file %s\n", filename );
    smap->fp = iop;
    smap->filename = (char *) malloc((strlen(filename)+1)*sizeof(char) );
    strcpy( smap->filename, filename );

    FEnd=0;
    do{
      if (fscanf(iop, "%s", command) == EOF)
	FEnd=1;
      else{
	numl++;
	if ((numl==2) && (smap->type == TYPE_UNKNOWN)) {
	  fprintf(stderr, "READ: WARNING - no format type\n" );
	}
	if (!strcmp( command, "MAP") && !objdef ){
	  if (fscanf(iop, "%f %f %f %f", 
		     &smap->start_x, &smap->start_y,
		     &smap->end_x,   &smap->end_y ) == EOF)
	    FEnd=1;
	} else if (!strcmp( command, "ROBOT") && !objdef ){
	  if (fscanf(iop, "%f %f %f", 
		     &smap->robot_x, 
		     &smap->robot_y, 
		     &smap->robot_o ) == EOF)
	    FEnd=1;
	} else if (!strcasecmp( command, "#format:")){
	  if (numl!=1) {
	    fprintf(stderr, "READ: FORMAT-DEF not in line 1\n" );
	    return(1);
	  } else {
	    if (fscanf(iop, "%s", &tcommand ) == EOF) {
	      FEnd=1;
	    } else {
	      if (!strcasecmp( tcommand, "sim-v1.0")) {
		fprintf(stderr, "READ: FORMAT: SIM-V1.0\n" );
		smap->type = TYPE_SIM_V10;
	      } else if (!strcasecmp( tcommand, "sof-v1.0")) {
		fprintf(stderr, "READ: FORMAT: SOF-V1.0\n" );
		smap->type = TYPE_SOF_V10;
	      } else {
		fprintf(stderr, "READ: UNKNOWN FORMAT\n" );
		smap->type = TYPE_UNKNOWN;
	      }
	    }
	  }
	} else if (!strcmp( command, "CYLINDER") ) {
	  fgets( line, 256, iop );
	  if (!objdef) {
	    FEnd = SOF_read_cylinder( line, element );
	    element = element->next;
	  } else {
	    FEnd = SOF_read_cylinder( line, defelement );
	    defelement = defelement->next;
	  }
	} else if (!strcmp( command, "CUBE") ) {
	  fgets( line, 256, iop );
	  if (!objdef) {
	    FEnd = SOF_read_cube( line, element );
	    element = element->next;
	  } else {
	    FEnd = SOF_read_cube( line, defelement );
	    defelement = defelement->next;
	  }
	} else if (!strcmp( command, "OBJECT") ){
	  fgets( line, 256, iop );
	  if (!objdef) {
	    FEnd = SOF_read_object( line, element );
	    element = element->next;
	  } else {
	    FEnd = SOF_read_object( line, defelement );
	    defelement = defelement->next;
	  }
	} else if (!strcmp( command, "CENTER") ) {
	  if (!objdef) {
	    fprintf( stderr, "READ: CENTER not in an OBJECT-define \n" );
	    FEnd=1;
	  } else {
	    fgets( line, 256, iop );
	    FEnd = SOF_read_center( line, odefine  );
	  }
	} else if (!strcmp( command, "DEFINE") ){
	  if (objdef) {
	    fprintf( stderr, "READ: recursive OBJECT-define!\n" );
	    FEnd=1;
	  } else {
	    objdef=1;
	    fgets( line, 256, iop );
	    odefine->next = (SOF_object_def_type *) 
	      malloc( sizeof(SOF_object_def_type) );
	    odefine->objs = 
	      (SOF_elements_type *)  malloc(sizeof(SOF_elements_type));
	    defelement = odefine->objs;
	    odefine->idnr = defnr++;
	    FEnd = SOF_read_object_define( line, odefine );
	  }
	} else if (!strcmp( command, "ENDDEF") ) {
	  if (!objdef) {
	    fprintf( stderr, "READ: ENDDEF not in an OBJECT-define \n" );
	    FEnd=1;
	  } else {
	    defelement = NULL;
	    odefine = odefine->next;
	    objdef=0;
	  }
	} else {
	  if (!(command[0]=='#')){
	    fprintf( stderr, "READ: unknown keyword %s\n", command );
	    fclose(iop);
	    return(0);
	  } else 
	    fgets(command,sizeof(command),iop);
	}
      }
    } while (!FEnd);
    fclose(iop);
  }
  element    = NULL;
  odefine    = NULL;
  defelement = NULL;
  if (!SOF_check_defines( *smap )) {
    fprintf( stderr, "READ: error...\n" );
    return(1);
  } else {
    SOF_flatten_map( smap );
    SOF_flatten_defines( smap );
    return(0);
  }
}

SOF_object_def_type *
SOF_find_define( char *name, SOF_object_def_type * def ) 
{
  SOF_object_def_type *pos;
  pos = def;
  while( pos->next!=NULL ) {
    if (!strcmp(name,pos->name))
      return(pos);
    pos = pos->next;
  } 
  return(NULL);
}

int
SOF_check_defines( SOF_map_type smap ) 
{
  int error;
  listelement *listpointer;
  SOF_object_def_type *dpos;
  listpointer = NULL;
  dpos = smap.defines;
  while( dpos->next!=NULL ) {
    listpointer = AddItem( listpointer, dpos->idnr );
    if (!SOF_check_define(dpos,smap.defines,listpointer)) return(0);
    dpos = dpos->next;
    listpointer = ClearQueue (listpointer);
  }
  return(1);
}

int
SOF_check_define( SOF_object_def_type *def,  SOF_object_def_type *alldefs,
		  listelement *listpointer ) 
{
  SOF_object_def_type *tempp;
  SOF_elements_type *pos;
  int retval = 1;
  pos = def->objs;
  while( pos->next!=NULL ) {
    if (pos->obj_type==SOF_TYPE_OBJECT) {
      tempp = SOF_find_define(pos->obj.object.name,alldefs);
      if (tempp!=NULL) {
	if (FindInQueue(listpointer,tempp->idnr)) {
	  listpointer = AddItem( listpointer, tempp->idnr );
	  retval = SOF_check_define( tempp, alldefs, listpointer );
	} else {
	  fprintf( stderr, "READ: found loop in definition!\n" );
	  return(0); /* loop! */
	}
      } else {
	fprintf( stderr, "READ: object %s not found!\n", pos->obj.object.name );
	return(0); /* not found */
      }
    }
    pos = pos->next;
  }
  return(retval);
}

/*********************************************************/


/********************************************************

 For a two-dimensional coordinate system, the rotation
 matrix is as follows:

 | X1 |   | cos(A)  -sin(A) |
 |    | * |                 | = | X2  Y2 |
 | Y1 |   | sin(A)   cos(A) |

 X2 = X1*cos(A)-Y1*sin(A)
 Y2 = X1*sin(A)+Y1*cos(A)

 X1 = Xin - CenterX
 Y1 = Yin - CenterY

 Xout = X2 + CenterX = X1*cos(A) - Y1*sin(A) + CenterX 
      = (Xin-CenterX)*cos(A) - (Yin-CenterY)*sin(A) + CenterX

 Yout = Y2 + CenterY = X1*sin(A) + Y2*sin(A) + CenterY 
      = (Xin-CenterX)*sin(A) + (Yin-CenterY)*cos(A) + CenterY

*********************************************************/

double
rotationX( double x, double y, double centerX, double centerY, double angle )
{
  return( ((x-centerX)*(cos(angle)) - ((y-centerX)*sin(angle)) + centerX) );
}

double
rotationY( double x, double y, double centerX, double centerY, double angle )
{
  return( ((x-centerX)*(sin(angle)) + ((y-centerX)*cos(angle)) + centerY) );
}


SOF_elements_type *
SOF_flatten_object( SOF_elements_type *elements, 
		    SOF_elements_type *curobj,
		    SOF_object_def_type *alldefs )
{
  float nx, ny;

  SOF_elements_type newobj, *pos;
  SOF_object_def_type *tempp;

  tempp = SOF_find_define(curobj->obj.object.name,alldefs);
  pos = tempp->objs;
  while( pos->next!=NULL ) {
    switch (pos->obj_type) {
    case SOF_TYPE_CYLINDER:
      nx = (float) rotationX( (double) pos->obj.cylinder.x, 
			      (double) pos->obj.cylinder.y, 
			      (double) tempp->center_x,
			      (double) tempp->center_y,
			      (double) curobj->obj.object.o );
      ny = (float) rotationY( (double) pos->obj.cylinder.x, 
			      (double) pos->obj.cylinder.y, 
			      (double) tempp->center_x,
			      (double) tempp->center_y,
			      (double) curobj->obj.object.o );
      elements = 
	SOF_add_cylinder( elements, 
			  curobj->obj.object.x+nx,
			  curobj->obj.object.y+ny,
			  curobj->obj.object.z+pos->obj.cylinder.z,
			  pos->obj.cylinder.radius,
			  pos->obj.cylinder.height,
			  pos->obj.cube.text );
      break;
    case SOF_TYPE_CUBE:
      nx = (float) rotationX( (double) pos->obj.cube.x, 
			      (double) pos->obj.cube.y, 
			      (double) tempp->center_x,
			      (double) tempp->center_y,
			      (double) curobj->obj.object.o );
      ny = (float) rotationY( (double) pos->obj.cube.x, 
			      (double) pos->obj.cube.y, 
			      (double) tempp->center_x,
			      (double) tempp->center_y,
			      (double) curobj->obj.object.o );
      elements = 
	SOF_add_cube( elements, 
		      curobj->obj.object.x+nx,
		      curobj->obj.object.y+ny,
		      curobj->obj.object.z+pos->obj.cube.z,
		      pos->obj.cube.width,
		      pos->obj.cube.length,
		      pos->obj.cube.height,
		      curobj->obj.object.o+pos->obj.cube.o,
		      pos->obj.cube.text );
      break;
    case SOF_TYPE_OBJECT:
      nx = (float) rotationX( (double) pos->obj.object.x, 
			      (double) pos->obj.object.y, 
			      (double) tempp->center_x,
			      (double) tempp->center_y,
			      (double) curobj->obj.object.o );
      ny = (float) rotationY( (double) pos->obj.object.x, 
			      (double) pos->obj.object.y, 
			      (double) tempp->center_x,
			      (double) tempp->center_y,
			      (double) curobj->obj.object.o );
      newobj.obj_type = SOF_TYPE_OBJECT;
      newobj.obj.object.x = curobj->obj.object.x+nx;
      newobj.obj.object.y = curobj->obj.object.y+ny;
      newobj.obj.object.z = curobj->obj.object.z+pos->obj.object.z;
      newobj.obj.object.o = curobj->obj.object.o+pos->obj.object.o;
      newobj.obj.object.name = 
	(char *) malloc((strlen(pos->obj.object.name)+1)*sizeof(char));
      strcpy(newobj.obj.object.name,pos->obj.object.name);
      newobj.next = NULL;
      elements = 
	SOF_flatten_object( elements, &newobj, alldefs );
      free(newobj.obj.object.name);
      break;
    }
    pos = pos->next;
  }
  return(elements);
}

/*********************************************************/

int
SOF_flatten_map( SOF_map_type *smap ) 
{
  SOF_object_def_type *dpos;
  SOF_elements_type *pos, *elements;

  dpos = smap->defines;
  pos = smap->elements;

  elements = (SOF_elements_type *)  malloc(sizeof(SOF_elements_type));
  smap->flat_elements = elements;

  while( pos->next!=NULL ) {
    switch (pos->obj_type) {
    case SOF_TYPE_CYLINDER:
      elements = 
	SOF_add_cylinder( elements, 
			  pos->obj.cylinder.x,      pos->obj.cylinder.y,
			  pos->obj.cylinder.z,      pos->obj.cylinder.radius,
			  pos->obj.cylinder.height, pos->obj.cylinder.text );
      break;
    case SOF_TYPE_CUBE:
      elements = 
	SOF_add_cube( elements, 
		      pos->obj.cube.x,       pos->obj.cube.y,
		      pos->obj.cube.z,       pos->obj.cube.width,
		      pos->obj.cube.length,  pos->obj.cube.height,
		      pos->obj.cube.o,       pos->obj.cube.text );
      break;
    case SOF_TYPE_OBJECT:
      elements = 
	SOF_flatten_object( elements, pos, dpos );
      break;
    default:
      fprintf( stderr, "READ: UNKNOWN TYPE(%d)\n", pos->obj_type );
    }
    pos = pos->next;
  }
  elements->next=NULL;
}

int
SOF_flatten_defines( SOF_map_type *smap )
{
  SOF_object_def_type *dpos,*pos;
  SOF_elements_type *opos,*elements;

  pos = smap->defines;
  dpos = smap->defines;

  while( pos->next!=NULL ) {
    
    opos = pos->objs;
    elements = (SOF_elements_type *) malloc(sizeof(SOF_elements_type));
    pos->flat_objs = elements;
    while( opos->next!=NULL ) {
      switch (opos->obj_type) {
      case SOF_TYPE_CYLINDER:
	elements = 
	  SOF_add_cylinder( elements, 
			    opos->obj.cylinder.x,      
			    opos->obj.cylinder.y,
			    opos->obj.cylinder.z,
			    opos->obj.cylinder.radius,
			    opos->obj.cylinder.height,
			    opos->obj.cylinder.text );
	break;
      case SOF_TYPE_CUBE:
	elements = 
	  SOF_add_cube( elements, 
			opos->obj.cube.x,
			opos->obj.cube.y,
			opos->obj.cube.z,  
			opos->obj.cube.width,
			opos->obj.cube.length,
			opos->obj.cube.height,
			opos->obj.cube.o, 
			opos->obj.cube.text );
	break;
      case SOF_TYPE_OBJECT:
	elements = 
	  SOF_flatten_object( elements, opos, dpos );
	break;
      default:
	fprintf( stderr, "READ: UNKNOWN TYPE(%d)\n", opos->obj_type );
      }
      opos=opos->next;
    }
    elements->next = NULL;
    pos = pos->next;
  }
}

int
SOF_print_elements( SOF_elements_type *pos ) 
{
  while( pos->next!=NULL ) {
    switch (pos->obj_type) {
    case SOF_TYPE_CYLINDER:
      fprintf( stderr, "READ: CYLINDER %f %f %f %f %f %s\n",
	       pos->obj.cylinder.x,       pos->obj.cylinder.y,
	       pos->obj.cylinder.z,       pos->obj.cylinder.radius,
	       pos->obj.cylinder.height,  pos->obj.cylinder.text );
      break;
    case SOF_TYPE_CUBE:
      fprintf( stderr, "READ: CUBE %f %f %f %f %f %f %f %s\n",
	       pos->obj.cube.x,       pos->obj.cube.y,
	       pos->obj.cube.z,       pos->obj.cube.width,
	       pos->obj.cube.length,  pos->obj.cube.height,
	       pos->obj.cube.o,       pos->obj.cube.text );
      break;
    case SOF_TYPE_OBJECT:
      fprintf( stderr, "READ: OBJECT %s %f %f %f %f\n",
	       pos->obj.object.name,
	       pos->obj.object.x,       pos->obj.object.y,
	       pos->obj.object.z,       pos->obj.object.o );
      break;
    default:
      fprintf( stderr, "READ: UNKNOWN TYPE(%d)\n", pos->obj_type );
    }
    pos = pos->next;
  }
}

int
SOF_print_defines( SOF_map_type smap ) 
{
  int i = 1;
  SOF_object_def_type *dpos;
  SOF_elements_type *pos;
  dpos = smap.defines;
  while( dpos->next!=NULL ) {
    fprintf( stderr, "\n\nREAD: DEFINE(%d) %s\n", dpos->idnr, dpos->name );
    pos = dpos->objs;
    while( pos->next!=NULL ) {
      switch (pos->obj_type) {
      case SOF_TYPE_CYLINDER:
	fprintf( stderr, "READ: CYLINDER %f %f %f %f %f %s\n",
		 pos->obj.cylinder.x,       pos->obj.cylinder.y,
		 pos->obj.cylinder.z,       pos->obj.cylinder.radius,
		 pos->obj.cylinder.height,  pos->obj.cylinder.text );
	break;
      case SOF_TYPE_CUBE:
	fprintf( stderr, "READ: CUBE %f %f %f %f %f %f %f %s\n",
		 pos->obj.cube.x,       pos->obj.cube.y,
		 pos->obj.cube.z,       pos->obj.cube.width,
		 pos->obj.cube.length,  pos->obj.cube.height,
		 pos->obj.cube.o,       pos->obj.cube.text );
	break;
      case SOF_TYPE_OBJECT:
	fprintf( stderr, "READ: OBJECT %s %f %f %f %f\n",
		 pos->obj.object.name,
		 pos->obj.object.x,       pos->obj.object.y,
		 pos->obj.object.z,       pos->obj.object.o );
	break;
      default:
	fprintf( stderr, "READ: UNKNOWN TYPE(%d)\n", pos->obj_type );
      }
      pos = pos->next;
    }

    fprintf( stderr, "\nREAD: FLATTEN DEFINE(%d) %s\n", dpos->idnr, dpos->name );
    pos = dpos->flat_objs;
    while( pos->next!=NULL ) {
      switch (pos->obj_type) {
      case SOF_TYPE_CYLINDER:
	fprintf( stderr, "READ: CYLINDER %f %f %f %f %f %s\n",
		 pos->obj.cylinder.x,       pos->obj.cylinder.y,
		 pos->obj.cylinder.z,       pos->obj.cylinder.radius,
		 pos->obj.cylinder.height,  pos->obj.cylinder.text );
	break;
      case SOF_TYPE_CUBE:
	fprintf( stderr, "READ: CUBE %f %f %f %f %f %f %f %s\n",
		 pos->obj.cube.x,       pos->obj.cube.y,
		 pos->obj.cube.z,       pos->obj.cube.width,
		 pos->obj.cube.length,  pos->obj.cube.height,
		 pos->obj.cube.o,       pos->obj.cube.text );
	break;
      case SOF_TYPE_OBJECT:
	fprintf( stderr, "READ: OBJECT %s %f %f %f %f\n",
		 pos->obj.object.name,
		 pos->obj.object.x,       pos->obj.object.y,
		 pos->obj.object.z,       pos->obj.object.o );
	break;
      default:
	fprintf( stderr, "READ: UNKNOWN TYPE(%d)\n", pos->obj_type );
      }
      pos = pos->next;
    }
    dpos = dpos->next;
  }
}

int
SOF_print_map( SOF_map_type smap ) 
{
  int i = 1;
  SOF_elements_type *pos;
  pos = smap.elements;
  SOF_print_elements( pos );
}

int
SOF_save_elements( char *filename, SOF_map_type map ) 
{
  FILE *iop;
  SOF_elements_type *pos;
  if ((iop = fopen( filename, "w")) == 0){
    fprintf(stderr, "SAVE: could not open output file %s\n", filename );
    return(-1);
  } else {
    fprintf( stderr, "SAVE: save file %s\n", filename );
    fprintf( iop, "MAP %f %f %f %f\n",
	     map.start_x, map.start_y, map.end_x, map.end_y );
    fprintf( iop, "ROBOT %f %f %f\n",
	     map.robot_x, map.robot_y, map.robot_o );
    pos = map.flat_elements;
    while( pos->next!=NULL ) {
      switch (pos->obj_type) {
      case SOF_TYPE_CYLINDER:
	fprintf( iop, "CYLINDER %f %f %f %f %f\n",
		 pos->obj.cylinder.x,
		 pos->obj.cylinder.y,
		 pos->obj.cylinder.z,
		 pos->obj.cylinder.radius,
		 pos->obj.cylinder.height );
	break;
      case SOF_TYPE_CUBE:
	fprintf( iop, "CUBE %f %f %f %f %f %f %f\n",
		 pos->obj.cube.x,
		 pos->obj.cube.y,
		 pos->obj.cube.z,
		 pos->obj.cube.width,
		 pos->obj.cube.length,
		 pos->obj.cube.height,
		 pos->obj.cube.o );
	break;
      }
      pos = pos->next;
    }
    fclose(iop);
  }
}
