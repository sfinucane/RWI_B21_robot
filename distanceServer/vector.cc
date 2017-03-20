
#include "vector.hh"



t_Vector2f::t_Vector2f()
{
  p[0] = 0;
  p[1] = 0;
}

t_Vector2f::t_Vector2f(float x, float y) 
{
  p[0] = x;
  p[1] = y;
}


t_Vector2f
t_Vector2f::rotate(float angle) const
{
  return t_Vector2f(p[0]*cos(angle)-p[1]*sin(angle),
		    p[0]*sin(angle)+p[1]*cos(angle));
}

float
t_Vector2f::length() const
{
  return sqrt(p[0]*p[0]+p[1]*p[1]);
}

float
t_Vector2f::angle() const
{
  if(p[1] >= 0) 
    return acos(p[0]/length());
  else
    return 2 * M_PI - acos(p[0]/length());
}

float
t_Vector2f::x() const
{
  return p[0];
}

float
t_Vector2f::y() const
{
  return p[1];
}

t_Vector2f
operator+(const t_Vector2f& a, const t_Vector2f& b)
{
  return t_Vector2f(a.p[0]+b.p[0],a.p[1]+b.p[1]);
}

t_Vector2f
operator-(const t_Vector2f& a, const t_Vector2f& b)
{
  return t_Vector2f(a.p[0]-b.p[0],a.p[1]-b.p[1]);
}

t_Vector2f
operator*(float a, const t_Vector2f& b)
{
  return t_Vector2f(a*b.p[0],a*b.p[1]);
}

t_Vector2f&
t_Vector2f::operator+=(const t_Vector2f &v)
{
  p[0] += v.p[0];
  p[1] += v.p[1];
  return *this;
}

float
dist(const t_Vector2f& a, const t_Vector2f& b)
{
  float dx = a.p[0]-b.p[0];
  float dy = a.p[1]-b.p[1];
  return sqrt(dx*dx+dy*dy);
}

float
scalarProd(const t_Vector2f &v1, const t_Vector2f &v2)
{
  return v1.p[0]*v2.p[0] + v1.p[1]*v2.p[1];
}


bool operator==(const t_Vector2f& a, const t_Vector2f& b)
{
  return (a.p[0] == b.p[0]) && (a.p[1] == b.p[1]);
}

ostream& operator<<(ostream &s, const t_Vector2f &v)
{
  s << '(' << v.p[0] << ',' << v.p[1] << ')';
  return s;
} 

// ----------------------- 3D vector stuff --------------------------------

t_Vector3f::t_Vector3f()
{
  p[0] = 0;
  p[1] = 0;
  p[2] = 0;
}

t_Vector3f::t_Vector3f(float xi, float yi, float zi)
{
  p[0] = xi;
  p[1] = yi;
  p[2] = zi;
}

t_Vector3f::t_Vector3f(const float *v3f)
{
  p[0] = v3f[0];
  p[1] = v3f[1];
  p[2] = v3f[2];
}


t_Vector3f::t_Vector3f(const t_Vector2f& v, float z)
{
  p[0] = v.p[0];
  p[1] = v.p[1];
  p[2] = z;
}

float
t_Vector3f::x() const
{
  return this->p[0];
}

float
t_Vector3f::y() const
{
  return this->p[1];
}

float
t_Vector3f::z() const
{
  return this->p[2];
}


t_Vector3f&
t_Vector3f::operator=(const t_Vector2f &v2d)
{
  p[0] = v2d.p[0];
  p[1] = v2d.p[1];
  return *this;
}

t_Vector3f
operator*(const t_Vector3f &v1, const t_Vector3f &v2)
{
  return t_Vector3f(v1.p[1]*v2.p[2]-v1.p[2]*v2.p[1],
		    v1.p[2]*v2.p[0]-v1.p[0]*v2.p[2],
		    v1.p[0]*v2.p[1]-v1.p[1]*v2.p[0]);
}

t_Vector3f
operator*(float f, const t_Vector3f &v)
{
  return t_Vector3f(f*v.p[0],f*v.p[1],f*v.p[2]);
}

t_Vector3f
operator*(const t_Vector3f &v, float f)
{
  return t_Vector3f(f*v.p[0],f*v.p[1],f*v.p[2]);
}

t_Vector3f
operator-(const t_Vector3f &v1, const t_Vector3f &v2)
{
  return t_Vector3f(v1.p[0]-v2.p[0], v1.p[1]-v2.p[1], v1.p[2]-v2.p[2]);
}

t_Vector3f
operator+(const t_Vector3f &v1, const t_Vector3f &v2)
{
  return t_Vector3f(v1.p[0]+v2.p[0], v1.p[1]+v2.p[1], v1.p[2]+v2.p[2]);
}

t_Vector3f&
t_Vector3f::operator+=(const t_Vector3f &v)
{
  p[0] += v.p[0];
  p[1] += v.p[1];
  p[2] += v.p[2];
  return *this;
}

t_Vector3f
operator/(const t_Vector3f &v, float f)
{
  return t_Vector3f(v.p[0]/f, v.p[1]/f, v.p[2]/f);
}

float
scalarProd(const t_Vector3f &v1, const t_Vector3f &v2)
{
  return v1.p[0]*v2.p[0] + v1.p[1]*v2.p[1] + v1.p[2]*v2.p[2];
}

t_Vector3f
t_Vector3f::rotate(const t_Vector3f& rot) const
{
  float t, sa, ca;
  t_Vector3f retVec = *this; 
  if(rot.p[0] != 0.0) {
    t = retVec.p[1];
    sa = sin(rot.p[0]);
    ca = cos(rot.p[0]);
    retVec.p[1] = t*ca - retVec.p[2]*sa;
    retVec.p[2] = t*sa + retVec.p[2]*ca;
  }
  if(rot.p[1] != 0.0) {
    t = retVec.p[0];
    sa = sin(rot.p[1]);
    ca = cos(rot.p[1]);
    retVec.p[0] = t*ca - retVec.p[2]*sa;
    retVec.p[2] = t*sa + retVec.p[2]*ca;
  }
  if(rot.p[2] != 0.0) {
    t = retVec.p[0];
    sa = sin(rot.p[2]);
    ca = cos(rot.p[2]);
    retVec.p[0] = t*ca - retVec.p[1]*sa;
    retVec.p[1] = t*sa + retVec.p[1]*ca;
  }
  return retVec;
}

t_Vector3f
t_Vector3f::irotate(const t_Vector3f& rot) const
{
  float t, sa, ca;
  t_Vector3f retVec = *this;
  if(rot.p[2] != 0.0) {
    t = retVec.p[0];
    sa = sin(-rot.p[2]);
    ca = cos(-rot.p[2]);
    retVec.p[0] = t*ca - retVec.p[1]*sa;
    retVec.p[1] = t*sa + retVec.p[1]*ca;
  }
  if(rot.p[1] != 0.0) {
    t = retVec.p[0];
    sa = sin(-rot.p[1]);
    ca = cos(-rot.p[1]);
    retVec.p[0] = t*ca - retVec.p[2]*sa;
    retVec.p[2] = t*sa + retVec.p[2]*ca;
  }
  if(rot.p[0] != 0.0) {
    t = retVec.p[1];
    sa = sin(-rot.p[0]);
    ca = cos(-rot.p[0]);
    retVec.p[1] = t*ca - retVec.p[2]*sa;
    retVec.p[2] = t*sa + retVec.p[2]*ca;
  }
  return retVec;
}

ostream& operator<<(ostream &s, const t_Vector3f &v)
{
  s << '(' << v.p[0] << ',' << v.p[1] << ',' << v.p[2] << ')';
  return s;
} 

t_Vector3f t_Vector3f::root(0,0,0);
t_Vector3f t_Vector3f::unitX(1,0,0);
t_Vector3f t_Vector3f::unitY(0,1,0);
t_Vector3f t_Vector3f::unitZ(0,0,1);

t_Vector4f::t_Vector4f()
{
  p[0] = 0;
  p[1] = 0;
  p[2] = 0;
  p[3] = 0;
}

t_Vector4f::t_Vector4f(float x, float y, float z, float w)
{
  p[0] = x;
  p[1] = y;
  p[2] = z;
  p[3] = w;
}

t_Vector4f::t_Vector4f(const t_Vector3f &xyz, float w)
{
  p[0] = xyz.x();
  p[1] = xyz.y();
  p[2] = xyz.z();
  p[3] = w;
}

float
t_Vector4f::x() const
{
  return p[0];
}
float
t_Vector4f::y() const
{
  return p[1];
}
float
t_Vector4f::z() const
{
  return p[2];
}
float
t_Vector4f::w() const
{
  return p[3];
}

t_Vector4f
operator/(const t_Vector4f &v, float f)
{
  return t_Vector4f(v.p[0]/f, v.p[1]/f, v.p[2]/f, v.p[3]/f);
}

ostream& operator<<(ostream &s, const t_Vector4f &v)
{
  s << '(' << v.p[0] << ',' << v.p[1] << ',' << v.p[2] << ',' << v.p[3] << ')'; return s;
} 
