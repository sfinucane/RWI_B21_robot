
#include "vector.hh"

class t_Polygon {
public:
  enum polyTypes {TRIANGLE, RECTANGLE};
  t_Vector3f* vertices;
  t_Vector4f*   normal;
  polyTypes type;
};


class t_PolygonList {
public:
  t_PolygonList(int count) {
    cnt = count;
    polygons = new t_Polygon[cnt];
  };
  ~t_PolygonList() {
    delete [] polygons;
  };
  t_Polygon& operator[](int index) const {
    return polygons[index];
  };
  int size() const {
    return cnt;
  }
private:
  int cnt;
  t_Polygon* polygons;
};
