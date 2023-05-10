#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  Vector3D o, d, vc;
  o = r.o;
  d = r.d;
  vc = this->o;
  double a, b, c;
  a = d.norm2();
  b = 2 * dot(o - vc, d);
  c = (o - vc).norm2() - r2;
  double delta = b*b - 4*a*c;
  if (delta < 0) {
    return false;
  }
  double root_delta = sqrt(delta);
  t1 = (-b - root_delta) / (2*a);
  t2 = (-b + root_delta) / (2*a);

  return true;

}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  double t1, t2;
  bool intersect;
  intersect = test(r, t1, t2);
  if (!intersect) {
    t1 = -1;
    t2 = -1;
    return false;
  }
  if (t1 >= r.min_t && t1 <= r.max_t) {
    r.max_t = t1;
    return true;
  } else if (t2 >= r.min_t && t2 <= r.max_t) {
    r.max_t = t2;
    return true;
  }

  return false;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  double t1, t2, t = -1;
  bool intersect;
  intersect = test(r, t1, t2);
  if (!intersect) {
    return false;
  }
  if (t1 >= r.min_t && t1 <= r.max_t) {
    t = t1;
  } else if (t2 >= r.min_t && t2 <= r.max_t) {
    t = t2;
  }
  if (t > 0) {
    r.max_t = t;
    Vector3D normal, p;
    p = r.o + t*r.d;
    normal = p - this->o;
    normal.normalize();
    i->t = t;
    i->n = normal;
    i->primitive = this;
    i->bsdf = get_bsdf();
    return true;
  }

  return false;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
