#include "environment_light.h"
#include "util/lodepng.h"

namespace CGL { namespace SceneObjects {

  EnvironmentLight::EnvironmentLight(const HDRImageBuffer* envMap)
    : envMap(envMap) {
    init();
  }

  EnvironmentLight::~EnvironmentLight() {
    delete[] pdf_envmap;
    delete[] conds_y;
    delete[] marginal_y;
  }


  void EnvironmentLight::init() {
    uint32_t w = envMap->w, h = envMap->h;
    pdf_envmap = new double[w * h];
    conds_y = new double[w * h];
    marginal_y = new double[h];

    std::cout << "[PathTracer] Initializing environment light...";

    // TODO Assignment 7 Part 3 Task 3 Steps 1,2,3
    // Store the environment map pdf to pdf_envmap
    // Store the marginal distribution for y to marginal_y
    // Store the conditional distribution for x given y to conds_y

    double sum = 0;
    for (int j = 0; j < h; ++j) {
      for (int i = 0; i < w; ++i) {
        pdf_envmap[w * j + i] = envMap->data[w * j + i].illum() * sin(PI * (j + .5) / h);
        sum += pdf_envmap[w * j + i];
      }
    }
    for (int j = 0; j < h; ++j) {
      double prev_y_marginal_cdf;
      prev_y_marginal_cdf = j == 0 ? 0 : marginal_y[j-1];
      marginal_y[j] = prev_y_marginal_cdf;
      for (int i = 0; i < w; ++i) {
        pdf_envmap[w * j + i] /= sum;
        marginal_y[j] += pdf_envmap[w * j + i];
      }
      // for conditional cdf F(x|y)
      double py = marginal_y[j] - prev_y_marginal_cdf;
      for (int i = 0; i < w; i++) {
        conds_y[w * j + i] = i == 0 ? 0 : conds_y[w * j + i-1];
        conds_y[w * j + i] += pdf_envmap[w * j + i] / py;
      }
    }




    if (true)
      std::cout << "Saving out probability_debug image for debug." << std::endl;
    save_probability_debug();

    std::cout << "done." << std::endl;
  }

  // Helper functions

  void EnvironmentLight::save_probability_debug() {
    uint32_t w = envMap->w, h = envMap->h;
    uint8_t* img = new uint8_t[4 * w * h];

    for (int j = 0; j < h; ++j) {
      for (int i = 0; i < w; ++i) {
        img[4 * (j * w + i) + 3] = 255;
        img[4 * (j * w + i) + 0] = 255 * marginal_y[j];
        img[4 * (j * w + i) + 1] = 255 * conds_y[j * w + i];
        img[4 * (j * w + i) + 2] = 0;
      }
    }

    lodepng::encode("probability_debug.png", img, w, h);
    delete[] img;
  }

  Vector2D EnvironmentLight::theta_phi_to_xy(const Vector2D& theta_phi) const {
    uint32_t w = envMap->w, h = envMap->h;
    double x = theta_phi.y / 2. / PI * w;
    double y = theta_phi.x / PI * h;
    return Vector2D(x, y);
  }

  Vector2D EnvironmentLight::xy_to_theta_phi(const Vector2D& xy) const {
    uint32_t w = envMap->w, h = envMap->h;
    double x = xy.x;
    double y = xy.y;
    double phi = x / w * 2.0 * PI;
    double theta = y / h * PI;
    return Vector2D(theta, phi);
  }

  Vector2D EnvironmentLight::dir_to_theta_phi(const Vector3D dir) const {
    Vector3D unit_dir = dir.unit();
    double theta = acos(unit_dir.y);
    double phi = atan2(-unit_dir.z, unit_dir.x) + PI;
    return Vector2D(theta, phi);
  }

  Vector3D EnvironmentLight::theta_phi_to_dir(const Vector2D& theta_phi) const {
    double theta = theta_phi.x;
    double phi = theta_phi.y;

    double y = cos(theta);
    double x = cos(phi - PI) * sin(theta);
    double z = -sin(phi - PI) * sin(theta);

    return Vector3D(x, y, z);
  }

  // Credits to Luowen Qian from Spring 2018 for this more robust bilerp
  Vector3D EnvironmentLight::bilerp(const Vector2D& xy) const {
    long right = lround(xy.x), left, v = lround(xy.y);
    double u1 = right - xy.x + .5, v1;
    if (right == 0 || right == envMap->w) {
      left = envMap->w - 1;
      right = 0;
    }
    else left = right - 1;
    if (v == 0) v1 = v = 1; else if (v == envMap->h) {
      v = envMap->h - 1;
      v1 = 0;
    }
    else v1 = v - xy.y + .5;
    auto bottom = envMap->w * v, top = bottom - envMap->w;
    auto u0 = 1 - u1;
    return (envMap->data[top + left] * u1 + envMap->data[top + right] * u0) * v1 +
      (envMap->data[bottom + left] * u1 + envMap->data[bottom + right] * u0) * (1 - v1);
  }


  Vector3D EnvironmentLight::sample_L(const Vector3D p, Vector3D* wi,
    double* distToLight,
    double* pdf) const {
    // TODO: Assignment 7 Part 3 Tasks 2 and 3 (step 4)
    // First implement uniform sphere sampling for the environment light
    // Later implement full importance sampling

    // // Uniform
    // *wi = sampler_uniform_sphere.get_sample();
    // *distToLight = INF_D;
    // *pdf = 1.0 / (4.0 * PI);
    // Vector2D theta_phi = dir_to_theta_phi(*wi);
    // Vector2D xy = theta_phi_to_xy(theta_phi);
    // Importance
    int32_t w = envMap->w, h = envMap->h;
    Vector2D uv, theta_phi, xy;
    int x, y;
    uv = sampler_uniform2d.get_sample();
    y = std::upper_bound(marginal_y, marginal_y + h, uv.y) - marginal_y;
    x = std::upper_bound(&conds_y[w * y], 
                         &conds_y[w * y] + w, uv.x)
           - &conds_y[w * y];
    xy.x = x + random_uniform();
    xy.y = y + random_uniform();
    // wi already in world coordinate
    theta_phi = xy_to_theta_phi(xy);
    *wi = theta_phi_to_dir(theta_phi);
    *distToLight = INF_D;
    *pdf = pdf_envmap[w * y + x] * (w*h) / (2*PI*PI*sin(theta_phi.x));

    return bilerp(xy);
  }

  Vector3D EnvironmentLight::sample_dir(const Ray& r) const {
    // TODO: Assignment 7 Part 3 Task 1
    // Use the helper functions to convert r.d into (x,y)
    // then bilerp the return value
    Vector2D theta_phi = dir_to_theta_phi(r.d);
    Vector2D xy = theta_phi_to_xy(theta_phi);

    return bilerp(xy);

  }

  Vector3D EnvironmentLight::sample_Le(Ray *ray, double* point_pdf, double* dir_pdf,
                             Vector3D *normal) const {
    std::cout << "sample_Le not ready for EnvironmentLight" << std::endl;
    assert(0);
    return Vector3D();
  }

} // namespace SceneObjects
} // namespace CGL
