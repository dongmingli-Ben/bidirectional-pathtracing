#include "pathtracer.h"
#include "bidirection.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

std::ostream& operator<<(std::ostream& os, const PathVertex& v) {
    os << "position: " << v.position << " alpha: " << v.alpha
         << " p: " << v.p << " q: " << v.q;
    return os;
}


void BidirectionalPathTracer::prepare_bidirectional_subpath(
    Ray r, const double point_pdf, 
    const double dir_pdf, vector<PathVertex>& path, 
    const Vector3D& init_radiance,
    const Vector3D& init_normal,
    bool is_light, SceneLight *light = nullptr) {
  PathVertex v;
  path.push_back(v); // v0 (a pseudo vertex, not actually in the scene)
  // actual vertex starts from v1
  v.p = point_pdf;
  v.alpha = init_radiance / point_pdf;
  v.isect.n = init_normal;
  v.position = r.o;
  v.q = 1.;
  v.is_light = is_light;
  v.light = light;
  v.dir_pdf = dir_pdf;
  path.push_back(v);

  int i = 2;
  Intersection isect;
  double prev_pdf = dir_pdf;
  Vector3D prev_f(1., 1., 1.), prev_n(init_normal);  // TODO: multiply cos theta for f ???
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

    // update vertex
    v.isect = isect;
    double g; // geometry term
    g = fabs(dot(prev_n, r.d) * dot(isect.n, r.d)) / (isect.t * isect.t);
    // std::cout << "g: " << g << " prev_pdf: " << prev_pdf << " p_{i-1}: " << path[i-1].p << std::endl;
    v.p = path[i-1].p * prev_pdf * g;
    // std::cout << "v.p: "  << path[i-1].p * prev_pdf * g << std::endl;
    // * add costheta according to: https://www.pbr-book.org/3ed-2018/Light_Transport_III_Bidirectional_Methods/Bidirectional_Path_Tracing#RandomWalk
    v.alpha = path[i-1].alpha * fabs(dot(prev_n, r.d)) * prev_f / prev_pdf;  // todo: costheta is needed ??? !!! (THIS SEEMS CRUCIAL, although it does not follow veach's paper, p. 329)
    // std::cout << "v.alpha: "  << v.alpha << " prev_f: " << prev_f << " prev_pdf: " << prev_pdf << std::endl;
    v.position = hit_p;
    v.is_light = false;
    v.light = nullptr;

    // update next ray
    Ray ray(hit_p, wi_world, (int)r.depth+1);
    ray.min_t = EPS_F;
    r = ray;

    p_keep = 1;
    v.q = p_keep;
    path.push_back(v);
    // std::cout << i << " " << v << std::endl;
    if (i >= max_ray_depth + 1) {
      break;
    }
    // // check path continuation (with roulette)
    // p_keep = i > min_subpath_length ? min(1., f.norm() / pdf) : 1;
    // v.q = p_keep;
    // path.push_back(v);
    // if (!coin_flip(p_keep)) {
    //   break;
    // }

    // update prev vertex info
    prev_f = f;
    prev_n = isect.n;
    prev_pdf = pdf * p_keep;
    i++;

  }
}


Ray BidirectionalPathTracer::sample_light_ray(double &point_pdf,
    double &dir_pdf, Vector3D &init_radiance, Vector3D& light_init_normal,
    SceneLight *light) {
  // randomly sample a light source
  int light_id = discrete_sampler.get_sample(0, scene->lights.size()-1);
  light = scene->lights[light_id];
  Ray r;
  Vector3D rad;
  rad = light->sample_Le(&r, &point_pdf, &dir_pdf, &light_init_normal);
  point_pdf /= scene->lights.size();
  init_radiance = rad;
  r.min_t = EPS_F;  // avoid intersecting with itself (when generating light path)
  return r;
}


double BidirectionalPathTracer::multiple_importance_sampling_weight(int i_eye, int i_light,
            const vector<PathVertex>& eye_path, const vector<PathVertex>& light_path,
            const PathVertex& light_sample, const PathVertex& eye_sample) {
  // // no MIS
  // return 1.;
  // // return 1. / (i_eye + i_light);
  // MIS
  double w_inv = 0., ratio = 1.;
  w_inv += ratio;
  // travel along the eye path
  PathVertex cur_v, next_v, prev_v;
  SceneLight *eye_light = NULL; // the light that the eye path endpoint intersects with directly
  for (int i = i_eye; i > 1; i--) {  // set to 2: do not consider directly connect light path to eye for now
    cur_v = eye_path[i];
    if (i == i_eye) {
      prev_v = i_light == 1 ? light_sample : light_path[i_light];
    } else {
      prev_v = eye_path[i+1];
    }
    next_v = eye_path[i-1];
    double nom, denom;
    double p, g;

    Matrix3x3 o2w;
    make_coord_space(o2w, prev_v.isect.n);
    Matrix3x3 w2o = o2w.T();

    Vector3D wi, wo, wi_world;
    double dist;
    wo = Vector3D(); // filled with empty for now (TODO)
    wi_world = cur_v.position - prev_v.position;
    // cout << "wi_world: " << wi_world << endl;
    dist = wi_world.norm();
    wi_world.normalize();
    // cout << "wi_world: " << wi_world << endl;
    wi = w2o * wi_world;

    g = fabs(cos_theta(wi) * dot(wi_world, cur_v.isect.n)) / (dist * dist);
    if (i_light == 0 && i == i_eye) {
      // eye path intersects lights
      // get the probability of sampling the intersection point on the light source
      bool intersect_light = false;
      for (int j = 0; j < scene->lights.size(); j++) {
        if (scene->lights[j]->contain_point(cur_v.position)) {
          intersect_light = true;
          eye_light = scene->lights[j];
          g = 1.;
          double point_pdf, dir_pdf;
          scene->lights[j]->sample_pdf(cur_v.position, Vector3D(), &point_pdf, &dir_pdf);
          p = point_pdf;
          break;
        }
      }
      if (!intersect_light) {
        return 0.;
      }
    } else if (i_light == 1 && i == i_eye) {
      // eye path directly connects to the lights, use alternative light sample
      assert(light_sample.new_sample);
      p = light_sample.dir_pdf * light_sample.q;
    } else if (i_light == 0 && i == i_eye-1) {
      // eye path directly connects to the lights, need to obtain dir_pdf
      double dir_pdf, point_pdf;
      // wi_world points outward the light source
      Vector3D w = -wi_world;
      eye_light->sample_pdf(prev_v.position, w, &point_pdf, &dir_pdf);
      p = dir_pdf * light_path[1].q;
    } else {
      p = prev_v.isect.bsdf->sample_pdf(wo, wi) * prev_v.q;
    }
    nom = p * g;
    // std::cout << "eye path: p: " << p << " g: " << g << " dist: " << dist << " nom: " << nom << std::endl;

    make_coord_space(o2w, next_v.isect.n);
    w2o = o2w.T();

    wi_world = cur_v.position - next_v.position;
    dist = wi_world.norm();
    wi_world.normalize();
    wi = w2o * wi_world;

    g = fabs(cos_theta(wi) * dot(wi_world, cur_v.isect.n)) / (dist * dist);
    if (i == 2) {
      // next_v is on the camera lens
      // denom = p_ray (since the ray is sampled uniformly from the image plane, it is 1)
      p = 1.;
      g = 1.;
    } else {
      p = next_v.isect.bsdf->sample_pdf(wo, wi) * next_v.q;
    }
    denom = p * g;
    // std::cout << "eye path: p: " << p << " g: " << g << " dist: " << dist << " denom: " << denom << std::endl;

    ratio *= nom / denom;

    if ((cur_v.isect.bsdf != NULL && cur_v.isect.bsdf->is_delta())
        || (next_v.isect.bsdf != NULL && next_v.isect.bsdf->is_delta())) {
      continue;
    }
    w_inv += ratio * ratio;
  }
  // TODO: travel along the light path
  ratio = 1.;
  for (int i = i_light; i > 0; i--) {
    cur_v = light_path[i];
    if (i == i_light) {
      prev_v = i_eye == 1 ? eye_sample : eye_path[i_eye];
    } else {
      prev_v = light_path[i+1];
    }
    next_v = light_path[i-1];
    double nom, denom;
    double p, g;

    Matrix3x3 o2w;
    make_coord_space(o2w, prev_v.isect.n);
    Matrix3x3 w2o = o2w.T();

    Vector3D wi, wo, wi_world;
    double dist;
    wo = Vector3D(); // filled with empty for now (TODO)
    wi_world = cur_v.position - prev_v.position;
    dist = wi_world.norm();
    wi_world.normalize();
    wi = w2o * wi_world;
    if (i_eye <= 1 && i == i_light) {
      // light vertex directly connects to eye
      assert(eye_sample.new_sample);
      p = eye_sample.dir_pdf * eye_sample.q;
    } else {
      p = prev_v.isect.bsdf->sample_pdf(wo, wi) * prev_v.q;
    }
    g = fabs(cos_theta(wi) * dot(wi_world, cur_v.isect.n)) / (dist * dist);
    nom = p * g;
    // std::cout << "light path: p: " << p << " g: " << g << " nom: " << nom << std::endl;

    if (i > 1) {
      make_coord_space(o2w, next_v.isect.n);
      w2o = o2w.T();

      wi_world = cur_v.position - next_v.position;
      dist = wi_world.norm();
      wi_world.normalize();
      // cout << "wi_world: " << wi_world << endl;
      wi = w2o * wi_world;
      // cout << "wi: " << wi << endl;
      if (i == 2) {
        // next_v is on the light source
        p = next_v.dir_pdf;
      } else {
        p = next_v.isect.bsdf->sample_pdf(wo, wi) * next_v.q;
      }
      g = fabs(cos_theta(wi) * dot(wi_world, cur_v.isect.n)) / (dist * dist);
      denom = p * g;
      // std::cout << "light path: p: " << p << " g: " << g << " denom: " << denom << std::endl;

    } else {
      // one vertex is on the light source
      denom = cur_v.p; // for initial vertex on light: p = p_area
      // std::cout << "light path: denom: " << denom << std::endl;
    }
    ratio *= nom / denom;

    if ((cur_v.isect.bsdf != NULL && cur_v.isect.bsdf->is_delta())
        || (next_v.isect.bsdf != NULL && next_v.isect.bsdf->is_delta())) {
      continue;
    }
    w_inv += ratio * ratio;
  }
  // if (1. / w_inv > 0.01)
  // std::cout << "i_eye: " << i_eye << " i_light: " << i_light << " w: " << 1. / w_inv << std::endl;
  return 1. / w_inv;
}


Vector3D BidirectionalPathTracer::estimate_bidirection_radiance(
  int i_eye, int i_light, const vector<PathVertex>& eye_path, 
  const vector<PathVertex>& light_path
) {
  // do not handle special cases for now
  PathVertex ve, vl, light_sample;  // i_light = 1 --> use another light sample
  PathVertex eye_sample; // i_eye = 1 --> use another eye sample that is valid for connection
  int eye_x = -1, eye_y = -1; // the coordinate on the image plane for i_eye = 1 case
  ve = eye_path[i_eye];
  vl = light_path[i_light];
  Vector3D c;
  if (i_light == 0) {
    // c_{0,t} = L_e(z_{t-1} -> z_{t-2})
    if (i_eye > 1) {  // eye path intersect with light source
      c = eye_path[i_eye].isect.bsdf->get_emission();
      if (c.norm() > EPS_F) {
        // redo the calculation because bsdf does not distinguish the direction for area light
        bool intersect_light = false;
        for (int j = 0; j < scene->lights.size(); j++) {
          if (scene->lights[j]->contain_point(eye_path[i_eye].position)) {
            intersect_light = true;
            double point_pdf, dir_pdf;
            Vector3D wi = eye_path[i_eye].position - eye_path[i_eye-1].position;
            wi.normalize();
            c = scene->lights[j]->sample_pdf(eye_path[i_eye].position, wi, &point_pdf, &dir_pdf);
            break;
          }
        }
        if (!intersect_light) {
          c = Vector3D();
        }
      }
    }
  } else {
    Vector3D eye_ray, light_ray, connect_ray;
    Vector3D f_eye, f_light;
    if (i_light == 1) {
      Vector3D light_init_radiance, light_init_normal, light_dir, light_point;
      double light_point_pdf, light_dir_pdf, dist;
      SceneLight *light;
      // randomly sample a light source
      int light_id = discrete_sampler.get_sample(0, scene->lights.size()-1);
      light = scene->lights[light_id];
      Ray r;
      Vector3D rad;
      rad = light->sample_Le_point(eye_path[i_eye].position, &light_dir, &light_point, &dist, 
                                   &light_point_pdf, &light_dir_pdf, &light_init_normal);
      light_point_pdf /= scene->lights.size();
      light_init_radiance = rad;
      
      light_sample.p = light_point_pdf;
      light_sample.alpha = light_init_radiance / light_point_pdf;
      light_sample.q = 1.;
      light_sample.position = light_point;
      light_sample.isect.n = light_init_normal;
      light_sample.is_light = true;
      light_sample.light = light;
      light_sample.new_sample = true;
      light_sample.dir_pdf = light_dir_pdf;

      f_light = Vector3D(1., 1., 1.);
      vl = light_sample;
    }

    if (i_eye == 1) {
      // light path directly connects to camera lens, which does not
      // contribute to the current pixel
      // todo: this is a special case, the illuminance contributes to the `light` image
      // sample the corresponding pixel
      Vector3D eye_init_radiance, eye_init_normal, eye_dir, eye_point;
      double eye_point_pdf, eye_dir_pdf, dist;
      eye_init_radiance = camera->sample_ray_pdf(vl.position, &eye_dir, &eye_point, &dist, 
                            &eye_point_pdf, &eye_dir_pdf, &eye_init_normal,
                            &eye_x, &eye_y, sampleBuffer.w, sampleBuffer.h);
      // eye_init_radiance = Vector3D(1., 1., 1.);
      
      eye_sample.p = eye_point_pdf;
      eye_sample.alpha = eye_init_radiance / eye_point_pdf;
      eye_sample.q = 1.;
      eye_sample.position = eye_point;
      eye_sample.isect.n = eye_init_normal;
      eye_sample.is_light = false;
      eye_sample.new_sample = true;
      eye_sample.dir_pdf = eye_dir_pdf;

      f_eye = Vector3D(1., 1., 1.);
      ve = eye_sample;
    }

    if (i_eye > 1) {
      Matrix3x3 o2w;
      make_coord_space(o2w, eye_path[i_eye].isect.n);
      Matrix3x3 w2o = o2w.T();

      eye_ray = eye_path[i_eye-1].position - eye_path[i_eye].position;
      eye_ray.normalize();
      eye_ray = w2o * eye_ray;

      connect_ray = vl.position - eye_path[i_eye].position;
      connect_ray.normalize();
      connect_ray = w2o * connect_ray;

      f_eye = eye_path[i_eye].isect.bsdf->f(eye_ray, connect_ray);
    }
    // std::cout << "f_eye: " << f_eye << std::endl;

    if (i_light > 1) {
      Matrix3x3 o2w;
      make_coord_space(o2w, light_path[i_light].isect.n);
      Matrix3x3 w2o = o2w.T();

      light_ray = light_path[i_light-1].position - light_path[i_light].position;
      light_ray.normalize();
      light_ray = w2o * light_ray;

      connect_ray = ve.position - light_path[i_light].position;
      connect_ray.normalize();
      connect_ray = w2o * connect_ray;

      f_light = light_path[i_light].isect.bsdf->f(connect_ray, light_ray);
    }
    // std::cout << "f_light: " << f_light << std::endl;
    double dist;
    connect_ray = vl.position - ve.position;
    dist = connect_ray.norm();
    connect_ray.normalize();
    double g;
    Ray r(ve.position, connect_ray);
    r.min_t = EPS_F;
    r.max_t = dist - EPS_F;
    Intersection isect;
    // todo: potential problem: connecting paths through interior area of objects
    if (bvh->intersect(r, &isect)) {
      return Vector3D();
    } else {
      g = fabs(dot(vl.isect.n, connect_ray) * dot(ve.isect.n, connect_ray))
        / (dist * dist);
    }
    // std::cout << "g: " << g << " dist: " << dist << std::endl;
    c = f_eye * g * f_light;
    // std::cout << "c: " << c << std::endl;
  }
  
  Vector3D ill, contrib;
  Vector3D light_alpha = i_light == 1 ? light_sample.alpha : light_path[i_light].alpha;
  Vector3D eye_alpha = i_eye == 1 ? eye_sample.alpha : eye_path[i_eye].alpha;
  double w = 0.;
  // cout << "contrib: " << contrib << " eye alpha: " << eye_alpha << " light alpha: " << light_alpha << endl;
  contrib = eye_alpha * light_alpha * c;
  if (contrib.norm() > EPS_F) {
    // multiple importance sampling
    w = multiple_importance_sampling_weight(
      i_eye, i_light, eye_path, light_path, 
      light_sample, eye_sample);
    // if (w > 0.9) {
    //   w = multiple_importance_sampling_weight(
    //     i_eye, i_light, eye_path, light_path, 
    //     light_sample, eye_sample);
    // }
  }
  ill = contrib * w;
  if (i_eye == 1) {
    // update to light image
    if (eye_x >= 0 && eye_y >= 0 && eye_x < sampleBuffer.w && eye_y < sampleBuffer.h) {
      update_pixel(lightBuffer, eye_x, eye_y, ill / ns_aa);
      update_pixel(sampleBuffer, eye_x, eye_y, ill / ns_aa);
    } else {
      // cout << "out of image | x: " << eye_x << " y: " << eye_y << endl;
    }
    // do not contribute to eye image
    return Vector3D();
  }
  return ill;
}


Vector3D BidirectionalPathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // main workflow of bidirectional path tracing
  vector<PathVertex> eye_path, light_path;
  prepare_bidirectional_subpath(r, 1., 1., eye_path, Vector3D(1., 1., 1.), r.d, false, nullptr);
  Ray light_init_ray;
  Vector3D light_init_radiance, light_init_normal;
  double light_point_pdf, light_dir_pdf;
  SceneLight *light;
  light_init_ray = sample_light_ray(light_point_pdf, light_dir_pdf, light_init_radiance,
                                    light_init_normal, light);
  prepare_bidirectional_subpath(light_init_ray, light_point_pdf, light_dir_pdf, 
                                light_path, light_init_radiance,
                                light_init_normal, true, light);

  // connect different paths
  for (int i = 1; i < eye_path.size(); i++) {
    for (int j = 0; j < light_path.size(); j++) {
      // if (i + j != 4) continue;  // do not consider bounces more than 1 for now
      // if (i != 4) continue;  // do not consider bounces more than 1 for now
      Vector3D L_in = estimate_bidirection_radiance(i, j, eye_path, light_path);
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

  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
  Vector3D illumination(0, 0, 0);
  for (int i = 0; i < ns_aa; i++) {
    // * note: do not use adaptive sampling, it introduces bias for BDPT
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
  }
  illumination /= ns_aa;

  eyeBuffer.update_pixel(illumination, x, y);
  update_pixel(sampleBuffer, x, y, illumination);
  sampleCountBuffer[x + y * sampleBuffer.w] = ns_aa;


}

void BidirectionalPathTracer::update_pixel(HDRImageBuffer &target, size_t x, size_t y, const Vector3D& s) {
  Vector3D w;
  update_lock.lock();
  w = target.get_pixel(x, y);
  w += s;
  target.update_pixel(w, x, y);
  update_lock.unlock();
}

void BidirectionalPathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
  eyeBuffer.resize(width, height);
  lightBuffer.resize(width, height);
}

void BidirectionalPathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  eyeBuffer.clear();
  lightBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
  eyeBuffer.resize(0, 0);
  lightBuffer.resize(0, 0);
}

} // namespace CGL
