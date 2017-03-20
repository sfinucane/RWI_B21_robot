// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/options.hh,v $
//
//  Created by:      $Author: rstone $
//
//  Revision:        $Revision: 1.1 $
//
//  Date of revision $Date: 2002/09/14 16:52:31 $
//
// ============================================================================
//
//                 ------------- HISTORY -----------------
//
//  $Log: options.hh,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.1  1999/06/29 08:38:39  schulz
//  Moved handling of command line options into these extra files.
//
//  Revision 1.2  1998/10/05 09:51:44  schulz
//  Added option to include the simulator into the RTLClient
//
//  Revision 1.1  1998/06/19 13:57:54  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.2  1997/03/04 17:28:35  schulz
//  Added file header to *.hh files
//  Removed some old "Zombie" files
//
//
//
// ----------------------------------------------------------------------------


#define TOGGLE 0
#define VALUE 1

typedef char Boolean;

class option {
public:
  option(char *tkn, int tp, char *val, char *dsc);
  char *token;
  int type;
  union {
    Boolean *vbool;
    char *vstring;
  } value;
  char *descr;
};

void read_options(int argc, char**argv);
void add_option(char *tkn, int tp, char *val, char *dsc);
