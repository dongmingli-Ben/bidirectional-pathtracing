#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out(0, 0, 0);

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 
  for (int i = 0; i < num_samples; i++) {
    double pdf;
    Vector3D wi, f, wi_world;
    // wi = hemisphereSampler->get_sample();
    // f = isect.bsdf->f(w_out, wi);
    // pdf = double(1) / (2*PI);
    f = isect.bsdf->sample_f(w_out, &wi, &pdf);
    wi_world = o2w * wi;
    wi_world.normalize();
    Ray ray(hit_p, wi_world);
    ray.min_t = EPS_F;
    Intersection intersect;
    if (!bvh->intersect(ray, &intersect)) {
      continue;
    }
    double costhetha = fabs(dot(wi_world, isect.n));
    Vector3D L_in = intersect.bsdf->get_emission();
    L_out += L_in * f * costhetha / pdf;
  }
  L_out /= num_samples;
  // L_out += isect.bsdf->get_emission();

  return L_out;

}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out(0, 0, 0);

  for (auto light: scene->lights) {
    int n_samples = ns_area_light;
    if (light->is_delta_light()) {
      n_samples = 1;
    }
    double pdf, distToLight;
    Vector3D wi, f, wi_world, emit_radiance, L_o(0, 0, 0);
    for (int i = 0; i < n_samples; i++) {
      emit_radiance = light->sample_L(hit_p, &wi_world, &distToLight, &pdf);
      wi = w2o * wi_world;
      f = isect.bsdf->f(w_out, wi);
      Ray ray(hit_p, wi_world);
      ray.min_t = EPS_F;
      ray.max_t = distToLight - EPS_F;
      Intersection intersect;
      // cout << "light: " << hit_p + wi_world*distToLight << endl;
      // cout << "ray" << ray.o << " --> " << ray.d << endl;
      if (bvh->intersect(ray, &intersect)) {
        // cout << "shadow ray! " << ray.o + ray.d*intersect.t << endl;
        continue;
      }
      double costhetha = fabs(dot(wi_world, isect.n));
      Vector3D L_in = distToLight >= INF_D ? emit_radiance : emit_radiance / (distToLight*distToLight);
      // Vector3D L_o_debug = L_in * f * costhetha / pdf;
      L_o += L_in * f * costhetha / pdf;
      // if (L_o_debug.x < 0 || L_o_debug.y < 0 || L_o_debug.z < 0 || L_o_debug.x >= INF_D || L_o_debug.y >= INF_D || L_o_debug.z >= INF_D) {
      //   std::cout << "[one-bounce]: Invalid L_o_debug: " << L_o_debug << ", L_in: " << L_in << ", f: " << f << ", costheta: " << costhetha << ", pdf: " << pdf << std::endl;
      //   if (L_o_debug.x < -10000) {
      //     asm("int3");
      //     isect.bsdf->f(w_out, wi);
      //   }
      // }
    }
    L_o /= n_samples;
    L_out += L_o;
  }
  // L_out += isect.bsdf->get_emission();
  // if (L_out.x < 0 || L_out.y < 0 || L_out.z < 0 || L_out.x >= INF_D || L_out.y >= INF_D || L_out.z >= INF_D) {
  //   std::cout << "[one-bounce]: Invalid L_out: " << L_out << std::endl;
  // }


  return L_out;

}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
  return isect.bsdf->get_emission();


}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`
  if (direct_hemisphere_sample) {
    return estimate_direct_lighting_hemisphere(r, isect);
  } else {
    return estimate_direct_lighting_importance(r, isect);
  }


}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out(0, 0, 0), L_o(0, 0, 0);

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.
  // if (r.depth > 0)  // for indirect illumination
  // if it is delta bsdf, direct lighting is not useful
  if (!isect.bsdf->is_delta()) {
    L_out += one_bounce_radiance(r, isect);
  }
  // L_out += isect.bsdf->get_emission();

  bool trace = true, roulette = false;
  double cpdf = 0.3;
  if (max_ray_depth == 0) {
    roulette = true;
    if (!coin_flip(cpdf) || r.depth >= 20) {
      trace = false;
    }
  } else if (r.depth >= max_ray_depth-1) {
    trace = false;
  } else {
    trace = true;
  }
  if (!trace) {
    return L_out;
  }
  Vector3D wi, f, wi_world;
  double pdf;
  f = isect.bsdf->sample_f(w_out, &wi, &pdf);
  wi_world = o2w * wi;
  wi_world.normalize();
  Ray ray(hit_p, wi_world, (int)r.depth+1);
  ray.min_t = EPS_F;
  Intersection intersect;
  if (!bvh->intersect(ray, &intersect)) {
    return L_out;
  }
  double costhetha = fabs(dot(wi_world, isect.n));
  Vector3D L_in = at_least_one_bounce_radiance(ray, intersect);
  // Vector3D L_in_debug = L_in, L_out_debug = L_out;
  if (isect.bsdf->is_delta()) {
    L_in += zero_bounce_radiance(ray, intersect);
  }
  if (roulette) {
    L_o += L_in * f * costhetha / pdf / cpdf;
  } else {
    L_o += L_in * f * costhetha / pdf;
  }
    
  L_out += L_o;
  // if (L_out.x < 0 || L_out.y < 0 || L_out.z < 0 || L_out.x >= INF_D || L_out.y >= INF_D || L_out.z >= INF_D) {
  //   std::cout << "Invalid L_out: " << L_out << ", L_o: " << L_o << ", L_out_debug: " << L_out_debug << ", L_in: " << L_in << ", L_in_debug: " << L_in_debug << std::endl;
  // }

  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.
  
  if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;


  L_out = (isect.t == INF_D) ? debug_shading(r.d) : zero_bounce_radiance(r, isect);

  // TODO (Part 3): Return the direct illumination.
  // L_out += one_bounce_radiance(r, isect);
  L_out += at_least_one_bounce_radiance(r, isect);

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"

  int num_samples = 0;          // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
  Vector3D illumination(0, 0, 0);
  double s1 = 0, s2 = 0;
  for (int i = 0; i < ns_aa; i = i + samplesPerBatch) {
    for (int j = 0; j < samplesPerBatch; j++) {
      Vector2D pixel_pos = gridSampler->get_sample();
      pixel_pos.x = pixel_pos.x + origin.x;
      pixel_pos.y = pixel_pos.y + origin.y;
      Ray ray;
      double dx, dy;
      dx = pixel_pos.x / sampleBuffer.w;
      dy = pixel_pos.y / sampleBuffer.h;
      // ray = camera->generate_ray(dx, dy);
      Vector2D samplesForLens = gridSampler->get_sample();
      ray = camera->generate_ray_for_thin_lens(dx, dy, samplesForLens.x, samplesForLens.y * 2.0 * PI);
      Vector3D ill;
      ill = est_radiance_global_illumination(ray);
      // if (ill.x < 0 || ill.y < 0 || ill.z < 0 || ill.x >= INF_D || ill.y >= INF_D || ill.z >= INF_D) {
      //   std::cout << "Invalid ill: " << ill << std::endl;
      // }
      illumination += ill;
      double illum = ill.illum();
      s1 += illum;
      s2 += illum*illum;
    }
    double mu, sigma;
    num_samples = i+samplesPerBatch;
    mu = s1 / num_samples;
    sigma = sqrt((s2 - s1*s1/num_samples) / (num_samples-1));
    double ci;
    ci = 1.96 * sigma / sqrt(num_samples);
    if (ci <= maxTolerance*mu && mu > EPS_F) {
      // cout << "mu: " << mu << " sigma: " << sigma << " ci: " << ci << endl;
      break;
    }
  }
  illumination /= num_samples;

  sampleBuffer.update_pixel(illumination, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;


}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
