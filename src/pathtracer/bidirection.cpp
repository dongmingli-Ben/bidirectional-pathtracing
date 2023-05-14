#include "pathtracer.h"
#include "bidirection.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {


void BidirectionalPathTracer::prepare_bidirectional_subpath(
    Ray r, const double point_pdf, 
    const double dir_pdf, vector<PathVertex>& path, 
    const Vector3D& init_radiance,
    const Vector3D& init_normal) {
  PathVertex v;
  path.push_back(v); // v0 (a pseudo vertex, not actually in the scene)
  // actual vertex starts from v1
  v.p = point_pdf;
  v.alpha = init_radiance / point_pdf;
  v.isect.n = init_normal;
  v.position = r.o;
  path.push_back(v);

  int i = 2;
  Intersection isect;
  double prev_pdf = dir_pdf;
  Vector3D prev_f(1., 1., 1.), prev_n(init_normal);
  // determine length with russian roulette
  while (bvh->intersect(r, &isect)) {
    double p_keep;
    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);
    Matrix3x3 w2o = o2w.T();

    // w_out points towards the source of the ray (e.g.,
    // toward the camera if this is a primary ray)
    const Vector3D hit_p = r.o + r.d * isect.t;
    const Vector3D w_out = w2o * (-r.d);

    Vector3D wi, f, wi_world;
    double pdf;
    f = isect.bsdf->sample_f(w_out, &wi, &pdf);
    wi_world = o2w * wi;
    wi_world.normalize();
    // update next ray
    Ray ray(hit_p, wi_world, (int)r.depth+1);
    ray.min_t = EPS_F;
    r = ray;

    // update vertex
    v.isect = isect;
    double g; // geometry term
    g = fabs(dot(prev_n, r.d) * dot(isect.n, r.d)) / (isect.t * isect.t);
    v.p = path[i-1].p * prev_pdf * g;
    v.alpha = path[i-1].alpha * prev_f / prev_pdf;
    v.position = r.o;

    // check path continuation
    p_keep = i > min_subpath_length ? min(1., f.norm() / pdf) : 1;
    v.q = p_keep;
    path.push_back(v);
    if (!coin_flip(p_keep)) {
      break;
    }

    // update prev vertex info
    prev_f = f;
    prev_n = isect.n;
    prev_pdf = pdf * p_keep;
    i++;

  }
}


Ray BidirectionalPathTracer::sample_light_ray(double &point_pdf,
    double &dir_pdf, Vector3D &init_radiance) {
  // randomly sample a light source
  int light_id = discrete_sampler.get_sample(0, scene->lights.size()-1);
  SceneLight *light = scene->lights[light_id];
  Ray r;
  Vector3D rad;
  rad = light->sample_Le(&r, &point_pdf, &dir_pdf);
  point_pdf /= scene->lights.size();
  init_radiance = rad;
  return r;
}


double BidirectionalPathTracer::multiple_importance_sampling_weight(int i_eye, int i_light,
            const vector<PathVertex>& eye_path, const vector<PathVertex>& light_path,
            double light_pdf) {
  // no MIS
  return 1. / (i_eye + i_light);
  // double w_inv = 0., ratio = 1.;
  // w_inv += ratio;
  // // travel along the eye path
  // PathVertex cur_v, next_v, prev_v;
  // for (int i = i_eye; i > 1; i--) {
  //   cur_v = eye_path[i];
  //   prev_v = i == i_eye ? light_path[i_light] : eye_path[i+1];
  //   next_v = eye_path[i-1];
  //   double nom, denom;
  //   double p, g;

  //   Matrix3x3 o2w;
  //   make_coord_space(o2w, prev_v.isect.n);
  //   Matrix3x3 w2o = o2w.T();

  //   Vector3D wi, wo, wi_world;
  //   double dist;
  //   wo = Vector3D(); // filled with empty for now (TODO)
  //   wi_world = cur_v.position - prev_v.position;
  //   dist = wi_world.norm();
  //   wi_world.normalize();
  //   wi = w2o * wi_world;
  //   p = prev_v.isect.bsdf->sample_pdf(wo, wi) * prev_v.q;
  //   g = fabs(cos_theta(wi) * dot(wi_world, cur_v.isect.n)) / (dist * dist);
  //   nom = p * g;

  //   make_coord_space(o2w, next_v.isect.n);
  //   w2o = o2w.T();

  //   wi_world = cur_v.position - next_v.position;
  //   dist = wi_world.norm();
  //   wi_world.normalize();
  //   wi = w2o * wi_world;
  //   p = next_v.isect.bsdf->sample_pdf(wo, wi) * next_v.q;
  //   g = fabs(cos_theta(wi) * dot(wi_world, cur_v.isect.n)) / (dist * dist);
  //   denom = p * g;

  //   ratio *= nom / denom;
  //   w_inv += ratio * ratio;
  // }
  // // TODO: travel along the light path
  // ratio = 1.;
  // for (int i = i_light; i > 0; i--) {
  //   cur_v = light_path[i];
  //   prev_v = i == i_light? eye_path[i_eye] : light_path[i+1];
  //   next_v = light_path[i-1];
  //   double nom, denom;
  //   double p, g;

  //   Matrix3x3 o2w;
  //   make_coord_space(o2w, prev_v.isect.n);
  //   Matrix3x3 w2o = o2w.T();

  //   Vector3D wi, wo, wi_world;
  //   double dist;
  //   wo = Vector3D(); // filled with empty for now (TODO)
  //   wi_world = cur_v.position - prev_v.position;
  //   dist = wi_world.norm();
  //   wi_world.normalize();
  //   wi = w2o * wi_world;
  //   p = prev_v.isect.bsdf->sample_pdf(wo, wi) * prev_v.q;
  //   g = fabs(cos_theta(wi) * dot(wi_world, cur_v.isect.n)) / (dist * dist);
  //   nom = p * g;

  //   if (i > 1) {
  //     make_coord_space(o2w, next_v.isect.n);
  //     w2o = o2w.T();

  //     wi_world = cur_v.position - next_v.position;
  //     dist = wi_world.norm();
  //     wi_world.normalize();
  //     wi = w2o * wi_world;
  //     p = next_v.isect.bsdf->sample_pdf(wo, wi) * next_v.q;
  //     g = fabs(cos_theta(wi) * dot(wi_world, cur_v.isect.n)) / (dist * dist);
  //     denom = p * g;

  //   } else {
  //     // one vertex is on the light source
  //     denom = light_pdf;
  //   }
  //   ratio *= nom / denom;
  //   w_inv += ratio * ratio;
  // }
  // return 1. / w_inv;
}


Vector3D BidirectionalPathTracer::estimate_bidirection_radiance(
  int i_eye, int i_light, const vector<PathVertex>& eye_path, 
  const vector<PathVertex>& light_path, double light_pdf
) {
  // do not handle special cases for now
  PathVertex ve, vl;
  ve = eye_path[i_eye];
  vl = light_path[i_light];
  Vector3D c;
  if (i_light == 0) {
    // c_{0,t} = L_e(z_{t-1} -> z_{t-2})
    c = light_path[1].alpha * light_path[1].p;
  } else {
    Vector3D eye_ray, light_ray, connect_ray;
    Vector3D f_eye, f_light;
    if (i_eye == 1) {
      // light path directly connects to camera lens, which does not
      // contribute to the current pixel
      f_eye = Vector3D(0., 0., 0.);
    } else {
      eye_ray = eye_path[i_eye-1].position - eye_path[i_eye].position;
      eye_ray.normalize();
      connect_ray = light_path[i_light].position - eye_path[i_eye].position;
      connect_ray.normalize();
      f_eye = eye_path[i_eye].isect.bsdf->f(eye_ray, connect_ray);
    }
    if (i_light == 1) {
      f_light = Vector3D(1., 1., 1.);
    } else {
      light_ray = light_path[i_light-1].position - eye_path[i_light].position;
      light_ray.normalize();
      connect_ray = light_path[i_eye].position - eye_path[i_light].position;
      connect_ray.normalize();
      f_light = light_path[i_eye].isect.bsdf->f(light_ray, connect_ray);
    }
    connect_ray = light_path[i_eye].position - eye_path[i_light].position;
    connect_ray.normalize();
    double g;
    Ray r(light_path[i_light].position, connect_ray);
    Intersection isect;
    if (!bvh->intersect(r, &isect)) {
      return Vector3D();
    } else {
      g = fabs(dot(light_path[i_light].isect.n, connect_ray) * dot(eye_path[i_eye].isect.n, connect_ray))
        / (isect.t * isect.t);
    }
    c = f_eye * g * f_light;
  }
  // multiple importance sampling
  double w = multiple_importance_sampling_weight(i_eye, i_light, eye_path, light_path, light_pdf);
  
  Vector3D ill, contrib;
  contrib = eye_path[i_eye].alpha * light_path[i_light].alpha * c;
  ill = contrib * w;
  return ill;
}


Vector3D BidirectionalPathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // main workflow of bidirectional path tracing
  vector<PathVertex> eye_path, light_path;
  prepare_bidirectional_subpath(r, 1., 1., eye_path, Vector3D(1, 1, 1), r.d);
  Ray light_init_ray;
  Vector3D light_init_radiance;
  double light_pdf, light_dir_pdf;
  light_init_ray = sample_light_ray(light_pdf, light_dir_pdf, light_init_radiance);
  prepare_bidirectional_subpath(light_init_ray, light_pdf, light_dir_pdf, 
                                light_path, light_init_radiance,
                                light_init_ray.d);

  // connect different paths
  for (int i = 1; i < eye_path.size(); i++) {
    for (int j = 0; j < light_path.size(); j++) {

      Vector3D L_in = estimate_bidirection_radiance(i, j, eye_path, light_path, light_pdf);
      L_out += L_in;
    }
  }

  return L_out;
}


void BidirectionalPathTracer::raytrace_pixel(size_t x, size_t y) {
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
      ray = camera->generate_ray(dx, dy);  // TODO: density = 1 ???
      // Vector2D samplesForLens = gridSampler->get_sample();
      // ray = camera->generate_ray(dx, dy, samplesForLens.x, samplesForLens.y * 2.0 * PI);
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

} // namespace CGL
