#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
  // std::cout << "ray: " << r.o << " --> " << r.d << std::endl;
  // std::cout << "bbox: " << *this << std::endl;
  if (t0 > t1) {
    std::swap(t0, t1);
  }
  double tmin_x, tmax_x, tmin_y, tmax_y, tmin_z, tmax_z;
  tmin_x = (this->min.x - r.o.x) / r.d.x;
  tmax_x = (this->max.x - r.o.x) / r.d.x;
  if (tmax_x < tmin_x) {
    std::swap(tmin_x, tmax_x);
  }
  tmin_y = (this->min.y - r.o.y) / r.d.y;
  tmax_y = (this->max.y - r.o.y) / r.d.y;
  if (tmax_y < tmin_y) {
    std::swap(tmin_y, tmax_y);
  }
  tmin_z = (this->min.z - r.o.z) / r.d.z;
  tmax_z = (this->max.z - r.o.z) / r.d.z;
  if (tmax_z < tmin_z) {
    std::swap(tmin_z, tmax_z);
  }
  double tmin, tmax;
  tmin = std::max(tmin_x, std::max(tmin_y, tmin_z));
  tmax = std::min(tmax_x, std::min(tmax_y, tmax_z));
  // std::cout << tmin << " " << tmax << std::endl;
  if (tmax < tmin) {
    return false;
  }
  // as long as a segment of the ray is inside the bbox,
  // we need to check all the primitives inside the bbox
  // for intersection
  if (tmin >= t0 && tmin <= t1) {
    t0 = tmin;
  }
  if (tmax >= t0 && tmax <= t1) {
    t1 = tmax;
  }

  return true;

}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
