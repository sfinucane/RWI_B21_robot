#include <iostream.h>
#include <stdio.h>
#include<values.h>

typedef struct {
  t_Vector3f pt;
  t_Vector4f normal;
} t_clipPlane;


template <class valType>
class t_FrameBuffer {
public:
  t_FrameBuffer(const t_Vector3f& pos, 
		const t_Vector3f& ori, 
		float focus, int width, int height);
  void renderPolygon(const t_Polygon& poly, const valType v);
  void writePixel(int u, int v, int w);
  valType readPixel(int x, int y, float *w) const;
  void setPixel(int x, int y, float z, valType val);
  void dump(FILE *file);
  ~t_FrameBuffer();
private:
  const int width, height;
  t_Vector3f position;
  t_Vector3f orientation;
  const float focus;
  valType **frame;
  valType level[256];
  valType val;
  int maxLevel;
  int **z_buffer;
  t_clipPlane clip_plane[6];
  t_Vector3f vertex[4];
  t_Vector3f *clip_vertex;

  int setup_levels();
  int greylevel(valType val);

  void renderTriangle(const t_Vector3f& v0,
		      const t_Vector3f& v1,
		      const t_Vector3f& v2);
  void renderUpperTriangle(const t_Vector3f& v0,
			   const t_Vector3f& v1,
			   const t_Vector3f& v2);
  void renderLowerTriangle(const t_Vector3f& v0,
			   const t_Vector3f& v1,
			   const t_Vector3f& v2);
  void scanLine(const t_Vector3f& v0, 
		const t_Vector3f& v1);
  void sort_vertices_according_y(t_Vector3f*);
  t_Vector3f vertex_to_eyeSpace(const t_Vector3f &pt) const;
  void projective_devide(int n);
  void renderTriangleList(int n);
  void setValue(valType v);
  int clipPolygon(int cnt);
};

template <class valType>
void
t_FrameBuffer<valType>::setValue(valType v)
{
  val = v;
}

template <class valType>
t_FrameBuffer<valType>::t_FrameBuffer(const t_Vector3f& pos,
				      const t_Vector3f& ori,
				      float f, int w, int h) 
  :  width(w),
     height(h),
     focus(f),
     position(pos),
     orientation(ori)
{
  frame = new valType* [width];
  for(int i = 0; i < width; i++)
    frame[i] = new valType [height];
  z_buffer = new int* [width];
  for(int i = 0; i < width; i++)
    z_buffer[i] = new int [height];
  for(int i = 0; i < width; i++)
    for(int j = 0; j < height; j++) {
      frame[i][j] = 0;
      z_buffer[i][j] = MAXINT;
    }
  maxLevel = 0;
  for(int i = 0; i < 256; i++) level[i] = NULL;

  double f10 = 100000*focus;
  t_Vector3f cpt0(focus, 1, 1);
  t_Vector3f cpt1(focus, -1, 1);
  t_Vector3f cpt2(focus, 1, -1);
  t_Vector3f cpt3(focus, -1, -1);
  t_Vector3f cpt4(f10,  100000,  100000);
  t_Vector3f cpt5(f10, -100000,  100000);
  t_Vector3f cpt6(f10,  100000, -100000);
  t_Vector3f cpt7(f10, -100000, -100000);

  clip_plane[0].pt = cpt0;
  clip_plane[0].normal = planeNormal(cpt0, cpt2, cpt4);  
  clip_plane[1].pt = cpt1;
  clip_plane[1].normal = planeNormal(cpt1, cpt3, cpt5);
  clip_plane[2].pt = cpt3;
  clip_plane[2].normal = planeNormal(cpt3, cpt2, cpt7);
  clip_plane[3].pt = cpt0;
  clip_plane[3].normal = planeNormal(cpt0, cpt1, cpt4);
  clip_plane[4].pt = cpt0;
  clip_plane[4].normal = planeNormal(cpt0, cpt1, cpt2);
  clip_plane[5].pt = cpt4;
  clip_plane[5].normal = planeNormal(cpt4, cpt6, cpt5);
  
}

template <class valType>
t_FrameBuffer<valType>::~t_FrameBuffer()
{
  for(int i = 0; i < width; i++) {
    delete [] frame[i];
    delete [] z_buffer[i];
  }
  delete [] frame;
  delete [] z_buffer;
}
template <class valType>
void
t_FrameBuffer<valType>::setPixel(int x, int y, float z, const valType val)
{
  frame[x][y] = val;
  z_buffer[x][y] = (int) (z*MAXINT);
}
template <class valType>
void
t_FrameBuffer<valType>::writePixel(int u, int v, int z)
{

  if(u >= width || u < 0 || v >= height || v < 0) return;

  if(z_buffer[u][v] >= z) {
    frame[u][v] = val;
    z_buffer[u][v] = z;
  }
}
template <class valType>
valType
t_FrameBuffer<valType>::readPixel(int x, int y, float *z) const
{
  *z = ((double) z_buffer[x][y]) / MAXINT;
  return frame[x][y];
}

template <class valType>
void
t_FrameBuffer<valType>::sort_vertices_according_y(t_Vector3f* v)
{
  if(v[0].y() > v[1].y()) {
    t_Vector3f t = v[0];
    v[0] = v[1];
    v[1] = t;
  }
  if(v[1].y() > v[2].y()) {
    t_Vector3f t = v[1];
    v[1] = v[2];
    v[2] = t;
  }
  if(v[0].y() > v[1].y()) {
    t_Vector3f t = v[0];
    v[0] = v[1];
    v[1] = t;
  }
}

template <class valType>
void
t_FrameBuffer<valType>::scanLine(const t_Vector3f& v0,
				 const t_Vector3f& v1)
{
  int u1 = (int)(width*(0.5*v0.x() + 0.5));
  int u2 = (int)(width*(0.5*v1.x() + 0.5));
  int v = (int)(width*(0.5*v0.y() + 0.5));

  int w = (int)(v0.z() * MAXINT);

  if(u2 ==  u1) {
    writePixel(u1, v, w);
    return;
  }

  double dz = (v1.z() - v0.z());
  int dw = (int)((dz * MAXINT) / (u2-u1));

  for(int u = u1; u < u2; u++, w += dw) {
    writePixel(u, v, w);
  } 
}

template <class valType>
void
t_FrameBuffer<valType>::renderUpperTriangle(const t_Vector3f& v0,
					    const t_Vector3f& v1,
					    const t_Vector3f& v2)
{
  cerr << "fb " << (void*)this << " renderUpperTriangle" << endl;
  double dy_0 = (v1.y() - v0.y());
  double dy_1 = (v2.y() - v0.y());
  
  t_Vector3f p0 = v0;
  t_Vector3f p1 = v0;

  double inc_0 = 1.0 / (width*dy_0);
  double inc_1 = 1.0 / (width*dy_1);
  t_Vector3f dv0 = inc_0 * (v1 - p0);
  t_Vector3f dv1 = inc_1 * (v2 - p1);

  while(p0.y() <= v1.y() && p0.y() < 1.0) {
    scanLine(p0, p1);
    p0 += dv0;
    p1 += dv1;
  }
}

template <class valType>
void
t_FrameBuffer<valType>::renderLowerTriangle(const t_Vector3f& v0,
					    const t_Vector3f& v1,
					    const t_Vector3f& v2)
{
  cerr << "fb " << (void*)this << " renderLowerTriangle" << endl;
  double dy_0 = (v2.y() - v0.y());
  double dy_1 = (v2.y() - v1.y());
  
  //  cerr << "lowerTriangle: " << v0 << " " << v1 << " " << v2 << endl;

  t_Vector3f p0 = v0;
  t_Vector3f p1 = v1;

  double inc_0 = 1.0 / (width * dy_0);
  double inc_1 = 1.0 / (width * dy_1);
  t_Vector3f dv0 = inc_0 * (v2 - p0);
  t_Vector3f dv1 = inc_1 * (v2 - p1);

  while(p0.y() <= v2.y() && p0.y() < 1.0) {    
    scanLine(p0, p1);
    p0 += dv0;
    p1 += dv1;
  }
}

template <class valType>
void
t_FrameBuffer<valType>::renderTriangle(const t_Vector3f& p0, 
				       const t_Vector3f& p1,
				       const t_Vector3f& p2)
{
  cerr << "fb " << (void*)this << " renderTriangle" << endl;
  /* sort according to y coordinate */
  t_Vector3f v[4];
  v[0] = p0;
  v[1] = p1;
  v[2] = p2;
  
  sort_vertices_according_y(v);

  double l = (v[1].y() - v[0].y()) / (v[2].y() - v[0].y());
  v[3] = v[0] + l * (v[2]-v[0]);

  if(v[3].x() < v[1].x()) {
    renderUpperTriangle(v[0],v[3],v[1]);
    renderLowerTriangle(v[3], v[1], v[2]);
  }
  else {
    renderUpperTriangle(v[0], v[1], v[3]);
    renderLowerTriangle(v[1], v[3], v[2]);
  }
  //  cerr << "done" << endl;
}

template<class valType>
void
t_FrameBuffer<valType>::projective_devide(int n)
{
  for(int i = 0; i < n; i++) {
    double z_val = clip_vertex[i].x();
    double proj = focus / z_val;
    clip_vertex[i] = t_Vector3f(proj*clip_vertex[i].y(),
				proj*clip_vertex[i].z(),
				1 - proj);
  }
}

template<class valType>
void
t_FrameBuffer<valType>::renderTriangleList(int n)
{
  cerr << "fb " << (void*)this << " renderTriangleList" << endl;
  cerr << "n " << n << endl;
  for(int i = 1; i < n-1; i++) {
    renderTriangle(clip_vertex[0], 
		   clip_vertex[i], 
		   clip_vertex[i+1]);
  }
}

template<class valType>
t_Vector3f
t_FrameBuffer<valType>::vertex_to_eyeSpace(const t_Vector3f &pt) const
{
  t_Vector3f direction = pt-position;
  return direction.irotate(orientation);
}

template <class valType>
int 
t_FrameBuffer<valType>::clipPolygon(int cnt)
{
  int vertex_cnt = cnt;
  t_Vector3f inPnt(2*focus, 0,0);
  bool in_list[8];
  static t_Vector3f clip_vertex1[8];
  static t_Vector3f clip_vertex2[8];
  t_Vector3f* in_vertex = clip_vertex1;
  t_Vector3f* out_vertex = clip_vertex2;
  int in_vertex_cnt;
  int out_vertex_cnt;

  /* copy vertices to temporary list */
  for(int i = 0; i <= vertex_cnt; i++) {
    out_vertex[i] = vertex[i];
  }
  out_vertex_cnt = vertex_cnt;

  for(int plane = 0; plane < 6; plane++) {

    t_clipPlane current_plane = clip_plane[plane];
    t_Vector3f *tmp = in_vertex;
    in_vertex = out_vertex;
    out_vertex = tmp;
    in_vertex_cnt = out_vertex_cnt;
    out_vertex_cnt = 0;

    for(int i = 0; i < in_vertex_cnt; i++) {
      in_list[i] = sameHalfspace(current_plane.normal, 
				 current_plane.pt, 
				 in_vertex[i],
				 inPnt);
    }
    if(in_list[0]) 
      out_vertex[out_vertex_cnt++] = in_vertex[0];
    for(int i = 0; i < in_vertex_cnt-1; i++) {
      if(in_list[i])
	if(in_list[(i+1)]) {
	  out_vertex[out_vertex_cnt++] = in_vertex[(i+1)];
	}
	else {
	  linePlaneIntersection(in_vertex[i],
				in_vertex[(i+1)]-in_vertex[i],
				current_plane.pt,
				current_plane.normal.Vector3f(),
				out_vertex[out_vertex_cnt++]);
	}
      else if(in_list[(i+1)%in_vertex_cnt]) {
	linePlaneIntersection(in_vertex[i],
			      in_vertex[(i+1)]-in_vertex[i],
			      current_plane.pt,
			      current_plane.normal.Vector3f(),
			      out_vertex[out_vertex_cnt++]);
	out_vertex[out_vertex_cnt++] = in_vertex[(i+1)];
      }
    }
    if(in_list[in_vertex_cnt-1] != in_list[0]) { 
      linePlaneIntersection(in_vertex[in_vertex_cnt-1],
			    in_vertex[0]-in_vertex[in_vertex_cnt-1],
			    current_plane.pt,
			    current_plane.normal.Vector3f(),
			    out_vertex[out_vertex_cnt++]);
    }
    
  }
  clip_vertex = out_vertex;
  return out_vertex_cnt;
}

template <class valType>
void
t_FrameBuffer<valType>::renderPolygon(const t_Polygon& poly, 
				      const valType obst)
{
  t_Vector3f p[4];
  int vtx_cnt = 0;
  cerr << "fb " << (void*)this << " renderPolygon" << endl;
  setValue(obst);
  switch(poly.type) {
  case t_Polygon::TRIANGLE: vtx_cnt = 3; break;
  case t_Polygon::RECTANGLE: vtx_cnt = 4; break;
  }
  for(int i = 0; i < vtx_cnt; i++) {
    vertex[i] = vertex_to_eyeSpace(poly.vertices[i]);
  }
  int vtcs = clipPolygon(vtx_cnt);
  projective_devide(vtcs);
  renderTriangleList(vtcs);
}
  
template <class valType>
int
t_FrameBuffer<valType>::greylevel(valType val)
{
  int i = 0;
  while(level[i] != val && i < 255) i++;
  return i;
}

template <class valType>
int
t_FrameBuffer<valType>::setup_levels()
{
  maxLevel = 0;
  for(int i = 0; i < 256; i++) 
    level[i] = NULL;
  for(int j = 0; j < height; j++) {
    for(int i = 0 ; i < width; i++) {
      if(frame[i][j] != NULL) {
	if(greylevel(frame[i][j]) == 255) {
	  level[maxLevel] = frame[i][j];
	  maxLevel++;
	}
      }
    } 
  }
  //  cerr << "greylevels: " << maxLevel << endl;
}

template <class valType>
void
t_FrameBuffer<valType>::dump(FILE* file)
{
  setup_levels();
  fprintf(file, "P2\n");
  fprintf(file, "#Create by RTLClient\n");
  fprintf(file, "%d %d\n", width, height);
  fprintf(file, "%d\n", 255);
  for(int j = 0; j < height; j++) {
    for(int i = 0 ; i < width; i++) {
      if(frame[i][j] == NULL) 
	fprintf(file, "%3d ", 255);
      else
	fprintf(file, "%3d ", greylevel(frame[i][j]));
    } 
    fprintf(file, "\n");
  }
  fprintf(file, "\n");
}

