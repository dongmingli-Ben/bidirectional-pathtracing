#include "camera.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include "CGL/misc.h"
#include "CGL/vector2D.h"
#include "CGL/vector3D.h"

using std::cout;
using std::endl;
using std::max;
using std::min;
using std::ifstream;
using std::ofstream;

namespace CGL {

using Collada::CameraInfo;

Ray Camera::generate_ray_for_thin_lens(double x, double y, double rndR, double rndTheta) const {

  // TODO Assignment 7: Part 4
  // compute position and direction of ray from the input sensor sample coordinate.
  // Note: use rndR and rndTheta to uniformly sample a unit disk.
  Vector3D pLens, pFocus, rayDir, world_ray_dir;
  pLens = Vector3D {lensRadius*sqrt(rndR)*cos(rndTheta), lensRadius*sqrt(rndR)*sin(rndTheta), 0};
  rayDir = Vector3D {(2*x - 1) * tan(hFov*PI/360), (2*y - 1) * tan(vFov*PI/360), -1};
  pFocus = rayDir * focalDistance;
  // cout << "pLens: " << pLens << endl;
  // cout << "rayDir: " << rayDir << endl;
  // cout << "pFocus: " << pFocus << endl;

  rayDir = pFocus - pLens;
  world_ray_dir = c2w * rayDir;
  world_ray_dir.normalize();
  Ray r(pos + c2w*pLens, world_ray_dir);
  r.min_t = nClip;
  r.max_t = fClip;
  
  return r;
}


} // namespace CGL
