#ifndef T_CIRCLE_HH
#define T_CIRCLE_HH

#include "obstacles.hh"
#include "surface.hh"

class t_circle : public t_obst {
private:
    float r;      /* circles radius */
    float h;
public:
    t_circle(char*, float, float, float, float, float);
    virtual Boolean distance(float, float, float, float, float, float, float*, float *, t_surface**);
    virtual float min_distance(float, float);
    virtual void bounds(float*,float*,float*,float*);
    virtual Boolean inside(float, float);
    virtual void expose();
    virtual void toggle();    
    virtual void redraw();    
    virtual void save(FILE*);
};

#endif
