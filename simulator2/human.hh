#include "obstacles.hh"
#include "circle.hh"
#include "surface.hh"

#define HUMAN_RADIUS 20

class t_human : public t_circle {
public:
    t_human(char* tag, float x, float y, float speed, float angle);
    virtual void update(struct timeval *now);
    virtual void save(FILE*);
private:
  struct timeval last_update;
  int stopped;
  float speed;
  float angle;
  float start_pos_x;
  float start_pos_y;  
};
