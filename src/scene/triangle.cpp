#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.
  // use Möller TrumboreAlgorithm
  Vector3D o, d, p0, p1, p2, e1, e2, s, s1, s2;
  double t, b1, b2;
  o = r.o;
  d = r.d;
  p0 = this->p1;
  p1 = this->p2;
  p2 = this->p3;
  e1 = p1 - p0;
  e2 = p2 - p0;
  s = o - p0;
  s1 = cross(d, e2);
  s2 = cross(s, e1);
  double denom;
  denom = dot(s1, e1);
  t = dot(s2, e2) / denom;
  b1 = dot(s1, s) / denom;
  b2 = dot(s2, d) / denom;
  if (t >= r.min_t && t <= r.max_t && b1 >= 0 && b2 >= 0 && b1+b2 <= 1) {
    r.max_t = t;
    return true;
  }

  return false;

}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
  // use Möller Trumbore Algorithm
  Vector3D o, d, p0, p1, p2, e1, e2, s, s1, s2;
  double t, b1, b2;
  o = r.o;
  d = r.d;
  p0 = this->p1;
  p1 = this->p2;
  p2 = this->p3;
  e1 = p1 - p0;
  e2 = p2 - p0;
  s = o - p0;
  s1 = cross(d, e2);
  s2 = cross(s, e1);
  double denom;
  denom = dot(s1, e1);
  t = dot(s2, e2) / denom;
  b1 = dot(s1, s) / denom;
  b2 = dot(s2, d) / denom;
  if (t >= r.min_t && t <= r.max_t && b1 >= 0 && b2 >= 0 && b1+b2 <= 1) {
    Vector3D normal;
    normal = n1*(1 - b1 - b2) + b1*n2 + b2*n3;
    normal.normalize();
    r.max_t = t;
    isect->t = t;
    isect->n = normal;
    isect->primitive = this;
    isect->bsdf = get_bsdf();
    // printf("Intersection\n");
    return true;
  }

  return false;


}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
