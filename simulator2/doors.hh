#include <X11/Intrinsic.h>
#include "rectangle.hh"
#include "Drawing.h"

class t_door : public t_rectangle {
  private:
    float apx,apy,apz;
    float angle;
    float last_angle;
    void absolute_points();
    virtual void move(float, float);
  public:
    t_door(char*, float, float, float, float, float, float, float);
    virtual void save(FILE*);
    virtual void mouse_action(float,float);
};
