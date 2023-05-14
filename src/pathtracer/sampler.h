#ifndef CGL_SAMPLER_H
#define CGL_SAMPLER_H

#include "CGL/vector2D.h"
#include "CGL/vector3D.h"
#include "CGL/misc.h"
#include "util/random_util.h"

namespace CGL {

/**
 * Interface for generating 1D vector samples
 */
class Sampler1D {
 public:

  /**
   * Virtual destructor.
   */
  virtual ~Sampler1D() { }

  /**
   * Use the Sampler1D to obtain a discrete uniform sample
   */
  virtual int get_sample(int low, int high) {
    int random_int = low + std::rand() % (high - low + 1);
    return random_int;
  }

}; // class Sampler2D

/**
 * Interface for generating 2D vector samples
 */
class Sampler2D {
 public:

  /**
   * Virtual destructor.
   */
  virtual ~Sampler2D() { }

  /**
   * Use the Sampler2D to obtain a Vector2D sample
   * according to the particular sampler's distribution.
   */
  virtual Vector2D get_sample() const = 0;

}; // class Sampler2D

/**
 * Interface for generating 3D vector samples
 */
class Sampler3D {
 public:

  /**
   * Virtual destructor.
   */
  virtual ~Sampler3D() { }

  /**
   * Use the Sampler3D to obtain a Vector3D sample
   * according to the particular sampler's distribution.
   */
  virtual Vector3D get_sample() const = 0;

  // pdf, for multiple importance sampling in bdpt
  virtual double pdf(const Vector3D& v) const = 0;

}; // class Sampler3D

/**
 * A Sampler3D implementation with uniform distribution on unit sphere
 */
class UniformSphereSampler3D : public Sampler3D {
public:

  Vector3D get_sample() const;
  double pdf(const Vector3D& v) const;

}; // class UniformHemisphereSampler3D


/**
 * A Sampler2D implementation with uniform distribution on unit square
 */
class UniformGridSampler2D : public Sampler2D {
 public:

  Vector2D get_sample() const;

}; // class UniformSampler2D

/**
 * A Sampler3D implementation with uniform distribution on unit hemisphere
 */
class UniformHemisphereSampler3D : public Sampler3D {
 public:

  Vector3D get_sample() const;
  double pdf(const Vector3D& v) const;

}; // class UniformHemisphereSampler3D

/**
 * A Sampler3D implementation with cosine-weighted distribution on unit
 * hemisphere.
 */
class CosineWeightedHemisphereSampler3D : public Sampler3D {
 public:

  Vector3D get_sample() const;
  // Also returns the pdf at the sample point for use in importance sampling.
  Vector3D get_sample(double* pdf) const;
  double pdf(const Vector3D& v) const;

}; // class UniformHemisphereSampler3D

/**
 * TODO (extra credit) :
 * Jittered sampler implementations
 */

} // namespace CGL

#endif //CGL_SAMPLER_H
