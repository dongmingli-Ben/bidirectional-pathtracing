#include "light.h"

#include <iostream>

#include "pathtracer/sampler.h"

namespace CGL { namespace SceneObjects {

// Directional Light //

DirectionalLight::DirectionalLight(const Vector3D rad,
                                   const Vector3D lightDir)
    : radiance(rad) {
  dirToLight = -lightDir.unit();
}

Vector3D DirectionalLight::sample_L(const Vector3D p, Vector3D* wi,
                                    double* distToLight, double* pdf) const {
  *wi = dirToLight;
  *distToLight = INF_D;
  *pdf = 1.0;
  return radiance;
}

Vector3D DirectionalLight::sample_Le(Ray *ray,
                                    double* point_pdf, double* dir_pdf) const {
  cout << "sample_Le not ready for DirectionalLight" << endl;
  assert(0);
  return Vector3D();
}

// Infinite Hemisphere Light //

InfiniteHemisphereLight::InfiniteHemisphereLight(const Vector3D rad)
    : radiance(rad) {
  sampleToWorld[0] = Vector3D(1,  0,  0);
  sampleToWorld[1] = Vector3D(0,  0, -1);
  sampleToWorld[2] = Vector3D(0,  1,  0);
}

Vector3D InfiniteHemisphereLight::sample_L(const Vector3D p, Vector3D* wi,
                                           double* distToLight,
                                           double* pdf) const {
  Vector3D dir = sampler.get_sample();
  *wi = sampleToWorld* dir;
  *distToLight = INF_D;
  *pdf = 1.0 / (2.0 * PI);
  return radiance;
}

Vector3D InfiniteHemisphereLight::sample_Le(Ray *ray,
                                    double* point_pdf, double* dir_pdf) const {
  cout << "sample_Le not ready for InfiniteHemisphereLight" << endl;
  assert(0);
  return Vector3D();
}

// Point Light //

PointLight::PointLight(const Vector3D rad, const Vector3D pos) : 
  radiance(rad), position(pos) { }

Vector3D PointLight::sample_L(const Vector3D p, Vector3D* wi,
                             double* distToLight,
                             double* pdf) const {
  Vector3D d = position - p;
  *wi = d.unit();
  *distToLight = d.norm();
  *pdf = 1.0;
  return radiance;
}

Vector3D PointLight::sample_Le(Ray *ray,
                                    double* point_pdf, double* dir_pdf) const {
  Ray r(position, sampler.get_sample());
  *ray = r;
  *point_pdf = 1;
  *dir_pdf = 0.25 / PI;
  return radiance;
}


// Spot Light //

SpotLight::SpotLight(const Vector3D rad, const Vector3D pos,
                     const Vector3D dir, double angle) {

}

Vector3D SpotLight::sample_L(const Vector3D p, Vector3D* wi,
                             double* distToLight, double* pdf) const {
  return Vector3D();
}

Vector3D SpotLight::sample_Le(Ray *ray,
                                    double* point_pdf, double* dir_pdf) const {
  cout << "sample_Le not ready for SpotLight" << endl;
  assert(0);
  return Vector3D();
}


// Area Light //

AreaLight::AreaLight(const Vector3D rad, 
                     const Vector3D pos,   const Vector3D dir, 
                     const Vector3D dim_x, const Vector3D dim_y)
  : radiance(rad), position(pos), direction(dir),
    dim_x(dim_x), dim_y(dim_y), area(dim_x.norm() * dim_y.norm()) { }

Vector3D AreaLight::sample_L(const Vector3D p, Vector3D* wi, 
                             double* distToLight, double* pdf) const {

  Vector2D sample = sampler.get_sample() - Vector2D(0.5f, 0.5f);
  Vector3D d = position + sample.x * dim_x + sample.y * dim_y - p;
  double cosTheta = dot(d, direction);
  double sqDist = d.norm2();
  double dist = sqrt(sqDist);
  *wi = d / dist;
  *distToLight = dist;
  *pdf = sqDist / (area * fabs(cosTheta));
  return cosTheta < 0 ? radiance : Vector3D();
};

Vector3D AreaLight::sample_Le(Ray *ray,
                                    double* point_pdf, double* dir_pdf) const {
  Vector2D sample = sampler.get_sample() - Vector2D(0.5f, 0.5f);
  Vector3D o = position + sample.x * dim_x;
  Vector3D d = dir_sampler.get_sample(dir_pdf);
  // transform local ray direction to global ray dir
  Matrix3x3 o2w;
  make_coord_space(o2w, direction);
  Ray r(o, o2w * d);
  *ray = r;

  *point_pdf = 1. / area;
  return radiance;
}


// Sphere Light //

SphereLight::SphereLight(const Vector3D rad, const SphereObject* sphere) {

}

Vector3D SphereLight::sample_L(const Vector3D p, Vector3D* wi, 
                               double* distToLight, double* pdf) const {

  return Vector3D();
}

Vector3D SphereLight::sample_Le(Ray *ray,
                                    double* point_pdf, double* dir_pdf) const {
  cout << "sample_Le not ready for SphereLight" << endl;
  assert(0);
  return Vector3D();
}

// Mesh Light

MeshLight::MeshLight(const Vector3D rad, const Mesh* mesh) {

}

Vector3D MeshLight::sample_L(const Vector3D p, Vector3D* wi, 
                             double* distToLight, double* pdf) const {
  return Vector3D();
}

Vector3D MeshLight::sample_Le(Ray *ray,
                                    double* point_pdf, double* dir_pdf) const {
  cout << "sample_Le not ready for MeshLight" << endl;
  assert(0);
  return Vector3D();
}

} // namespace SceneObjects
} // namespace CGL
