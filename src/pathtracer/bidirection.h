#ifndef CGL_PATHTRACER_BIDIRECTION_H
#define CGL_PATHTRACER_BIDIRECTION_H

#include "CGL/timer.h"

#include "scene/bvh.h"
#include "pathtracer/sampler.h"
#include "pathtracer/intersection.h"

#include "application/renderer.h"

#include "scene/scene.h"
using CGL::SceneObjects::Scene;
using CGL::SceneObjects::Intersection;
using CGL::SceneObjects::SceneLight;

#include "scene/environment_light.h"
using CGL::SceneObjects::EnvironmentLight;

using CGL::SceneObjects::BVHNode;
using CGL::SceneObjects::BVHAccel;

#include "pathtracer/pathtracer.h"
using CGL::PathTracer;


namespace CGL {
    
    class PathVertex {
        public:
            PathVertex() : p(1.), q(1.), alpha(1.), is_light(false), light(nullptr),
                new_light_sample(false), light_dir_pdf(0.) {}

            Intersection isect;
            double p;
            double q; // probability that the path does not end here (in roulette)
                      // used to correct the probability sampling next direction
            Vector3D alpha, position;
            bool is_light;
            SceneLight *light;
            bool new_light_sample;
            double light_dir_pdf; // this value is only valid for the first point 
                                  // on the light source. it is for MIS with newly sampled
                                  // light point (or old light point)

    };

    std::ostream& operator<<(std::ostream& os, const PathVertex& v);


    class BidirectionalPathTracer: public PathTracer {
        public:

        void prepare_bidirectional_subpath(Ray r, const double point_pdf, 
            const double dir_pdf, vector<PathVertex>& path, 
            const Vector3D& init_radiance, const Vector3D& init_normal,
            bool is_light, SceneLight *light);

        // sample the first vertex and the second vertex of the light path
        Ray sample_light_ray(double &point_pdf, double &dir_pdf,
            Vector3D &init_radiance, Vector3D &light_init_normal,
            SceneLight *light);

        /* 1-indexed, 1 => on the light source / camera len
         * \param light_pdf point pdf of the init point of the light path, which
            should be obtained from sample_light_ray.
         */
        Vector3D estimate_bidirection_radiance(int i_eye, int i_light,
            const vector<PathVertex>& eye_path, const vector<PathVertex>& light_path);

        double multiple_importance_sampling_weight(int i_eye, int i_light,
            const vector<PathVertex>& eye_path, const vector<PathVertex>& light_path,
            const PathVertex& light_sample);

        size_t min_subpath_length = 2;
        Sampler1D discrete_sampler;

        // override

        Vector3D est_radiance_global_illumination(const Ray& r);
        void raytrace_pixel(size_t x, size_t y);
    };
}


#endif // CGL_PATHTRACER_BIDIRECTION_H