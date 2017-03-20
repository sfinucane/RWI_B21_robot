#ifndef CAMERA_HH
#define CAMERA_HH

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define _XOPEN_SOURCE
#include <math.h>
#undef _XOPEN_SOURCE

#include "vector.hh"
#include "obstacles.hh"
#include "frameBuffer.hh"

class t_Camera : public t_Obstacle {
public:
  t_Camera() {};
  t_Camera(float, float, float open_angle,
	   float x, float y, float z, 
	   float yrot, float ori, t_ObstacleGrid* scene);

  //  virtual void render();

  virtual void update();
  bool distance(const t_Vector3f &origin, 
		const t_Vector3f &ori,
		float maxLength,
		float &dist, 
		float &angle,
		int *face_id);
  virtual Boolean inside(const t_Vector3f &point);
  virtual float min_distance(float, float);


  t_Obstacle*
  t_Camera::obstacleAtPixel(float px, float py, float *dist);

  bool projectPixel(float px, float py,
		   const t_Vector3f &planePt, const t_Vector3f &planeNormal,
		   t_Vector3f *quad);

  virtual Boolean bounds(float *, float *, float *, float *);
  virtual void save(FILE*);
 
  virtual void netWrite();
  virtual ~t_Camera() {};

  t_Vector3f pos;
  t_Vector3f pan_tilt_rot;
  
  t_Vector2f getImageDimensions() const;
  float open_angle;
  void renderPolygons(const t_PolygonList& surfaces, t_Obstacle* obst) const; 
protected:
  t_ObstacleGrid* scene;
  t_TypedPtr type_info[13];
  virtual t_AttribArray netAttrs();
  void initialize_typeinfo_array();
  int last_time;
  int width, height;
  float aspect;
  float netAngle;
  float cam_x, cam_y, cam_z, cam_zrot;
  float lookat_x, lookat_y, lookat_z;
  float upvector_x, upvector_y, upvector_z;    
  float hfov, vfov;

  t_FrameBuffer<t_Obstacle*>* fb;

  // stuff used for 3D projections modified by update method
  t_Vector3f ccd_normal;
  t_Vector3f ccd_root;
  float focus;

  // stuff used for lens correction modified by update method
  t_Vector3f corr_root;
  t_Vector3f corr_normal;  
  inline float lens_forward_correction(const t_Vector3f &pos,
			       const t_Vector3f &dir,
			       float pxySqr);
  inline float lens_backward_correction(const t_Vector3f &pos,
					const t_Vector3f &dir,
					float pxySqr);

};



/*
3dvector EyePint
3dVector lookAt
3dVector upVector
double   hfov
double   vfov
*/

#endif
