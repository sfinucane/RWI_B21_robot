#include <X11/Intrinsic.h>

#ifdef __cplusplus

#include "obstacles.hh"
#include "surface.hh"
    
class t_playground {
public:
    Boolean enabled;
    float x1,y1;
    float x2,y2;
    t_playground(char*);
    t_playground(char*, float, float, float);
    void InitPlayground();
    Boolean _SavePlayground(char*);
    void _LoadPlayground(char*);    
    Boolean _get_distance(float, float, float, float, float, float, float*, float*, t_surface**);
    void set_active_obstacle(t_obst*);
    t_obst* get_active_obstacle();
private:
  int num_obstacles;
  char filename[256];
  t_obst* activated_obstacle;    
  void initialize(char*);
};

extern t_playground* map;
#endif

#ifdef __cplusplus
extern "C" {
#endif
    
Boolean get_distance(int, float, float, float, float, float, float, float*);
Boolean InsideObstacle(float,float);
Boolean InsideAnyObstacle(float,float);
void RemoveObstacle(float, float);
void ToggleObstacle(float, float);
Boolean SavePlayground(char*);
void LoadPlayground(char*);
void ObstaclePushAction(float,float);
void ObstacleDragAction(float,float);
float playground_x(float);
float playground_y(float);
float base_x(float);
float base_y(float);
Boolean map_enabled();
extern void new_rectangle(float, float, float, float);
extern void new_door(float, float, float, float);
extern void new_circle(float, float, float);
extern void new_human(float, float, float, float);
extern float obstacles_min_distance(float, float);
#ifdef __cplusplus
}
#endif

