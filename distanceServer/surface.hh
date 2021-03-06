
#ifndef SURFACE_HH 
#define SURFACE_HH

#define SONAR_SENSOR 0
#define LASER_SENSOR 1
#define EXACT_SENSOR 2
#define IR_SENSOR    3

#ifdef __cplusplus

class t_surface {
public:
  t_surface(char*, char*);
  ~t_surface();
  float add_error(int, float, float) const;
  void init_serror(float, float, float);
private:
  char *name;
  char *color;    
  float S_MIN_DIST;
  float S_MAX_DIST; 
  float S_MINDIST_EANGLE;
  float S_MAXDIST_EANGLE;
  float S_ANGLERANGE;
  float ran() const;
  float sonar_error(float, float) const;    
  float ir_error(float, float)const ;    
};

class t_surflist_entry {
public:
  t_surflist_entry();
  ~t_surflist_entry();
  t_surface *surface;
  t_surflist_entry *next;
};

class t_surflist {
public:
  t_surflist();
  t_surflist::~t_surflist();
  void new_entry(char*, char*, float, float, float);
private:
    t_surflist_entry *first;
    t_surflist_entry *last;    
};


#endif

#ifdef __cplusplus
extern "C" {
#endif

void new_surf_entry(char*, char*, float, float, float);    
    
#ifdef __cplusplus
}
#endif

#endif

