#include <X11/Intrinsic.h>
/* #include "obstacles.h" XXX These where not found by make dep */
/* #include "store.h"     XXX These where not found by make dep */
#include "Drawing.h"

class t_compobst : public t_obst {
  private:
    float x1,y1,x2,y2;
    float cx,cy;
    float ori;
    Figure myfig;
    t_obstlist* components;
    virtual void redraw();
  public:
    t_compobst(int, float, float, float);
    virtual Boolean distance(float, float, float, float, float*, float *);
    virtual float min_distance(float, float);
    virtual void bounds(float*,float*,float*,float*);
    virtual Boolean inside(float, float);
    virtual void expose();
};
