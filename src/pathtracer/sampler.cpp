#include "sampler.h"

namespace CGL {

/**
 * A Sampler2D implementation with uniform distribution on unit square
 */
Vector2D UniformGridSampler2D::get_sample() const {

  return Vector2D(random_uniform(), random_uniform());

}


// Uniform Sphere Sampler3D Implementation //

Vector3D UniformSphereSampler3D::get_sample() const {
  double z = random_uniform() * 2 - 1;
  double sinTheta = sqrt(std::max(0.0, 1.0f - z * z));

  double phi = 2.0f * PI * random_uniform();

  return Vector3D(cos(phi) * sinTheta, sin(phi) * sinTheta, z);
}

double UniformSphereSampler3D::pdf(const Vector3D& v) const {
  double pdf = 0.25 / PI;

  return pdf;
}


/**
 * A Sampler3D implementation with uniform distribution on unit hemisphere
 */
Vector3D UniformHemisphereSampler3D::get_sample() const {

  double Xi1 = random_uniform();
  double Xi2 = random_uniform();

  double theta = acos(Xi1);
  double phi = 2.0 * PI * Xi2;

  double xs = sinf(theta) * cosf(phi);
  double ys = sinf(theta) * sinf(phi);
  double zs = cosf(theta);

  return Vector3D(xs, ys, zs);

}

/**
 * A Sampler3D implementation with uniform distribution on unit hemisphere
 */
double UniformHemisphereSampler3D::pdf(const Vector3D& v) const {

  double pdf = 0.5 / PI;

  return v.z > 0 ? pdf : 0.;

}

/**
 * A Sampler3D implementation with cosine-weighted distribution on unit
 * hemisphere. This function does not return the pdf.
 */
Vector3D CosineWeightedHemisphereSampler3D::get_sample() const {
  double f;
  return get_sample(&f);
}

/**
 * A Sampler3D implementation with cosine-weighted distribution on unit
 * hemisphere. This functions also sets the pdf to the proper probability
 */
Vector3D CosineWeightedHemisphereSampler3D::get_sample(double *pdf) const {

  double Xi1 = random_uniform();
  double Xi2 = random_uniform();

  double r = sqrt(Xi1);
  double theta = 2. * PI * Xi2;
  *pdf = sqrt(1-Xi1) / PI;
  return Vector3D(r*cos(theta), r*sin(theta), sqrt(1-Xi1));
}

/**
 * A Sampler3D implementation with cosine-weighted distribution on unit
 * hemisphere. This functions gets the pdf evaluated at the given sample
 */
double CosineWeightedHemisphereSampler3D::pdf(const Vector3D& v) const {
  double pdf = v.z > 0 ? v.z / PI : 0.;
  return pdf;
}


} // namespace CGL
