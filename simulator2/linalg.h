#include <X11/Intrinsic.h>

#ifdef __cplusplus
extern "C" {
#endif
int
cut_line_and_line(float, float, 
		  float, float, 
		  float, float, 
		  float, float, 
		  float*, float*);
int 
cut_circle_and_line(float, float, float,
		    float, float,
		    float, float, 
		    float*, float*);
int
point_on_left_side(float px, float py,
		   float x1, float y1,
		   float x2, float y2);
extern void intersect_circleline(float, float, float, Boolean,
				 float, float, float,
				 Boolean*, float*, float*);
extern void intersect_rectline(float,float, float, float,
			       float, float, float, Boolean,
			       Boolean*, float*, float*);
extern float line_min_distance(float, float, float, float,
			       float, float);
extern void line_min_dist_point(float, float,
				float, float,
				float, float,
				float*, float*);
extern Boolean inhalfplane(float, float,
			float, float,
			float, float);
#ifdef __cplusplus
}
#endif
