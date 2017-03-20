#ifndef VECTOR3F_HH
#define VECTOR3F_HH

#define _XOPEN_SOURCE
#include <math.h>
#undef _XOPEN_SOURCE
#include <stdlib.h>
#ifdef __GNUG__
#include <typeinfo>
#else
#include <stl_config.h> 
#endif
#include <iostream.h>


class t_Vector3f;

class t_Vector2f
{
public:
  t_Vector2f::t_Vector2f();
  t_Vector2f(float x, float y);
  t_Vector2f rotate(float angle) const;
  float x() const;
  float y() const;
  float angle() const;
  float length() const;
  t_Vector2f& operator+=(const t_Vector2f &v);

  friend float dist(const t_Vector2f& a, const t_Vector2f& b);
  friend float scalarProd(const t_Vector2f &v1, const t_Vector2f &v2);
  friend t_Vector2f operator+(const t_Vector2f& a, const t_Vector2f& b);
  friend t_Vector2f operator-(const t_Vector2f& a, const t_Vector2f& b);
  friend t_Vector2f operator*(float a, const t_Vector2f& b);
  friend bool operator==(const t_Vector2f& a, const t_Vector2f& b);
  friend ostream& operator<<(ostream &s, const t_Vector2f &v);
protected:
  float p[2];
  friend class t_Vector3f;
};

class t_Vector3f
{
public:
  t_Vector3f();
  t_Vector3f(float x, float y, float z);
  t_Vector3f(const float *v3f);
  t_Vector3f(const t_Vector2f& v, float z);
  float x() const;
  float y() const;
  float z() const;
  inline const float* vec() const
  {
    return p;
  }
  t_Vector3f&  operator=(const t_Vector2f &b); 
  t_Vector3f& operator+=(const t_Vector3f &v);
  t_Vector3f rotate(const t_Vector3f &rot) const;
  t_Vector3f irotate(const t_Vector3f &rot) const;  
  friend t_Vector3f operator*(const t_Vector3f &v1, const t_Vector3f &v2);
  friend t_Vector3f operator*(const t_Vector3f &v, float f);
  friend t_Vector3f operator*(float f, const t_Vector3f &v);
  friend t_Vector3f operator-(const t_Vector3f &v1, const t_Vector3f &v2);
  friend t_Vector3f operator+(const t_Vector3f &v1, const t_Vector3f &v2);
  friend t_Vector3f operator/(const t_Vector3f &v, float f);
  friend ostream& operator<<(ostream &s, const t_Vector3f &v);
  friend float scalarProd(const t_Vector3f &v1, const t_Vector3f &v2);
  inline float length() const
  {
    return sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]);
  }
  static t_Vector3f unitX;
  static t_Vector3f unitY;
  static t_Vector3f unitZ;
  static t_Vector3f root;
  float p[3];
private:
  friend class t_Vector2f;
};

class t_Vector4f
{
public:
  t_Vector4f();
  t_Vector4f(const t_Vector3f &xyz, float w);
  t_Vector4f(float x, float y, float z, float w);
  float x() const;
  float y() const;
  float z() const;
  float w() const;
  inline float *vec() {
    return p;
  }
  t_Vector3f Vector3f() const {
    return t_Vector3f(p);
  };
  friend t_Vector4f operator/(const t_Vector4f &v, float f);
  friend ostream& operator<<(ostream &s, const t_Vector4f &v);
protected:
  float p[4];
};

struct t_Dimensions {
  float w;
  float d;
  float h;
};

/****************************************************************************/
// some inlined mathematical helper functions 
/****************************************************************************/

extern float scalarProd(const t_Vector3f &v1, const t_Vector3f &v2);

inline int
signOf(float a)
{
  return a > 0 ? 1 : -1;
}

inline float
minimumOf(float a, float b)
{
  if(a < b) return a;
  else return b;
}

inline float
maximumOf(float a, float b)
{
  if(a > b) return a;
  else return b;
}

inline bool
sameHalfspace(const t_Vector4f &planeNormal,
	      const t_Vector3f &planePt, 
	      const t_Vector3f &testPt,
	      const t_Vector3f &centerPt)
{
  float a = scalarProd(planeNormal.Vector3f(), (planePt-testPt));
  float b = scalarProd(planeNormal.Vector3f(), (planePt-centerPt));

  return ( fabs(a) < 1E-4 || fabs(b) < 1E-4 || signOf(a) == signOf(b));
}


static inline t_Vector4f
planeNormal(const t_Vector3f &v1, 
	    const t_Vector3f &v2,
	    const t_Vector3f &v3)
{
  t_Vector3f w1 = v2 - v1;
  t_Vector3f w2 = v3 - v2;
  t_Vector3f N  = w1 * w2;
  t_Vector4f normal(N, scalarProd(N,v1));
  return normal / normal.Vector3f().length();
}

inline float
linePlaneDistance(const t_Vector3f &linePt1, 
		  const t_Vector3f &lineOrientation, 
		  const t_Vector3f &planePt, 
		  const t_Vector3f &planeNormal)
{
  float temp1 = scalarProd( lineOrientation, planeNormal);
  if(temp1 != 0.0) {
    // line and plane intersect; return distance
    return scalarProd( (planePt-linePt1), planeNormal) / temp1;
  }
  else return -1.0; 
  // Attention!
  // returns no intersection
  // in case of the line lying on the plane
}

inline bool
linePlaneIntersection(const t_Vector3f &start,
		      const t_Vector3f &direction,
		      const t_Vector3f &planePt,
		      const t_Vector3f &planeNormal,
		      t_Vector3f &intersection)
{
  float length;
  length = linePlaneDistance(start, direction,
			     planePt,
			     planeNormal);
  if(length < 0) return 0;
  intersection = start + length * direction;
  return 1;
}

#endif
