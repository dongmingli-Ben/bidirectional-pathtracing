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

#include "scene/environment_light.h"
using CGL::SceneObjects::EnvironmentLight;

using CGL::SceneObjects::BVHNode;
using CGL::SceneObjects::BVHAccel;

#include "pathtracer/pathtracer.h"
using CGL::PathTracer;


namespace CGL {
    
    class PathVertex {
        public:
            PathVertex() : p(1.), q(1.), alpha(1.) {}

            Intersection isect;
            double p;
            double q; // probability that the path does not end here (in roulette)
                      // used to correct the probability sampling next direction
            Vector3D alpha, position;
    };

    std::ostream& operator<<(std::ostream& os, const PathVertex& v);


    class BidirectionalPathTracer: public PathTracer {
        public:

        void prepare_bidirectional_subpath(Ray r, const double point_pdf, 
            const double dir_pdf, vector<PathVertex>& path, 
            const Vector3D& init_radiance, const Vector3D& init_normal);

        // sample the first vertex and the second vertex of the light path
        Ray sample_light_ray(double &point_pdf, double &dir_pdf,
            Vector3D &init_radiance, Vector3D &light_init_normal);

        /* 1-indexed, 1 => on the light source / camera len
         * \param light_pdf point pdf of the init point of the light path, which
            should be obtained from sample_light_ray.
         */
        Vector3D estimate_bidirection_radiance(int i_eye, int i_light,
            const vector<PathVertex>& eye_path, const vector<PathVertex>& light_path,
            double light_pdf);

        double multiple_importance_sampling_weight(int i_eye, int i_light,
            const vector<PathVertex>& eye_path, const vector<PathVertex>& light_path,
            double light_pdf);

        size_t min_subpath_length = 2;
        Sampler1D discrete_sampler;

        // override

        Vector3D est_radiance_global_illumination(const Ray& r);
        void raytrace_pixel(size_t x, size_t y);
    };
}


#endif // CGL_PATHTRACER_BIDIRECTION_H