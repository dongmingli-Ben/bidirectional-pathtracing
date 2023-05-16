#ifndef CGL_SCENE_H
#define CGL_SCENE_H

#include "CGL/CGL.h"
#include "primitive.h"

#include <vector>

namespace CGL { namespace SceneObjects {

/**
 * Interface for objects in the scene.
 */
class SceneObject {
 public:

  /**
   * Get all the primitives in the scene object.
   * \return a vector of all the primitives in the scene object
   */
  virtual std::vector<Primitive*> get_primitives() const = 0;

  /**
   * Get the surface BSDF of the object's surface.
   * \return the BSDF of the objects's surface
   */
  virtual BSDF* get_bsdf() const = 0;

};


/**
 * Interface for lights in the scene.
 */
class SceneLight {
 public:
  virtual Vector3D sample_L(const Vector3D p, Vector3D* wi,
                            double* distToLight, double* pdf) const = 0;
  // this is meant for bidirectional path tracing (to sample a initial ray 
  // from the light source)
  virtual Vector3D sample_Le(Ray *ray, double* point_pdf, double* dir_pdf,
                             Vector3D *normal) const = 0;
  // this is meant for bidirectional path tracing (to calculate the point
  // and directional pdf given the point)
  // * note: wi points towards the light source
  virtual Vector3D sample_Le_point(const Vector3D p, Vector3D* wi, Vector3D *point,
                            double* distToLight, double* point_pdf,
                            double* dir_pdf, Vector3D *normal) const = 0;
  // // this is meant for bidirectional path tracing (to calculate the point
  // // and directional pdf given the point and the light point)
  // virtual Vector3D sample_Le_dir(const Vector3D p, const Vector3D light_point,
  //                           double* dir_pdf) const = 0;
  virtual bool is_delta_light() const = 0;

};


/**
 * Represents a scene in a raytracer-friendly format. To speed up raytracing,
 * all data is already transformed to world space.
 */
struct Scene {
  Scene(const std::vector<SceneObject *>& objects,
        const std::vector<SceneLight *>& lights)
    : objects(objects), lights(lights) { }

  // kept to make sure they don't get deleted, in case the
  //  primitives depend on them (e.g. Mesh Triangles).
  std::vector<SceneObject*> objects;

  // for sake of consistency of the scene object Interface
  std::vector<SceneLight*> lights;

  // TODO (sky) :
  // Adding object with emission BSDFs as mesh lights and sphere lights so 
  // that light sampling configurations also applies to mesh lights. 
//  for (SceneObjects *obj : objects) {
//    if (obj->get_material().emit != Vector3D()) {
//      
//      // mesh light
//      if (dynamic_cast<Mesh*>(obj)) {
//        staticLights.push_back()
//      }
//      
//      // sphere light
//      if (dynamic_cast<Sphere*>(obj)) {
//
//      }
//  }

};

} // namespace SceneObjects
} // namespace CGL

#endif //CGL_STATICSCENE_SCENE_H
