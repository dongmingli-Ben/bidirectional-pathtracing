#include "bsdf.h"

#include <algorithm>
#include <iostream>
#include <utility>

#include "application/visual_debugger.h"

using std::max;
using std::min;
using std::swap;

namespace CGL {

// Mirror BSDF //

Vector3D MirrorBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D MirrorBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {

  // TODO Assignment 7: Part 1
  // Implement MirrorBSDF
  reflect(wo, wi);
  *pdf = 1;
  double costheta = fabs(wi->z) / wi->norm();
  return reflectance / costheta;
}


double MirrorBSDF::sample_pdf(const Vector3D& wo, const Vector3D& wi) const {
  // coefficient for proportion of dirac delta distribution
  return 1.;
}

void MirrorBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Mirror BSDF"))
  {
    DragDouble3("Reflectance", &reflectance[0], 0.005);
    ImGui::TreePop();
  }
}

// Microfacet BSDF //

double MicrofacetBSDF::G(const Vector3D wo, const Vector3D wi) {
  return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
}

double MicrofacetBSDF::D(const Vector3D h) {
  // TODO Assignment 7: Part 2
  // Compute Beckmann normal distribution function (NDF) here.
  // You will need the roughness alpha.
  double theta, out, nom, denom;
  theta = acos(h.z);  // todo: make sure h is normalized
  nom = exp(-pow(tan(theta)/alpha, 2));
  denom = PI * alpha*alpha * pow(cos(theta), 4);
  out = nom / denom;
  return out;
}

Vector3D MicrofacetBSDF::F(const Vector3D wi) {
  // TODO Assignment 7: Part 2
  // Compute Fresnel term for reflection on dielectric-conductor interface.
  // You will need both eta and etaK, both of which are Vector3D.
  Vector3D Rs, Rp;
  double costheta;
  costheta = fabs(wi.z) / wi.norm();
  Rs = ((eta*eta + k*k) - 2*eta*costheta + pow(costheta, 2))
     / ((eta*eta + k*k) + 2*eta*costheta + pow(costheta, 2));
  Rp = ((eta*eta + k*k)*pow(costheta, 2) - 2*eta*costheta + 1)
     / ((eta*eta + k*k)*pow(costheta, 2) + 2*eta*costheta + 1);

  return (Rs + Rp) / 2;
}

Vector3D MicrofacetBSDF::f(const Vector3D wo, const Vector3D wi) {
  // TODO Assignment 7: Part 2
  // Implement microfacet model here.
  // check wi and wo (because one bounce lighting will call this function)
  if (wo.z <= EPS_F || wi.z <= EPS_F) {
    return Vector3D();
  }
  Vector3D out, h;
  h = wo + wi;
  h.normalize();
  out = F(wi) * G(wo, wi) * D(h) / (4 * wo.z * wi.z);

  return out;
}

Vector3D MicrofacetBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
  // TODO Assignment 7: Part 2
  // *Importance* sample Beckmann normal distribution function (NDF) here.
  // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
  //       and return the sampled BRDF value.
  // *wi = cosineHemisphereSampler.get_sample(pdf);
  Vector2D r, theta_phi;
  r = sampler.get_sample();
  theta_phi = Vector2D {
    atan(sqrt(-alpha*alpha * log(1 - r.x))),
    2*PI*r.y
  };
  Vector3D h, d;  // d: from h to wo (perpendicular to h)
  double pdf_h, pdf_wi, costheta, p_theta, p_phi;
  h = Vector3D {
    sin(theta_phi.x)*cos(theta_phi.y),
    sin(theta_phi.x)*sin(theta_phi.y),
    cos(theta_phi.x)
  };  // h is already normalized
  costheta = dot(wo, h) / wo.norm();
  d = wo - h*costheta*wo.norm();
  *wi = h*costheta*wo.norm() - d;
  wi->normalize();
  // check wi and wo
  if (wo.z <= EPS_F || wi->z <= EPS_F) {
    // std::cout << "Invalid wo or wi, wo = " << wo << ", wi = " << *wi << std::endl;
    // std::cout << "theta_phi = " << theta_phi << std::endl;
    *pdf = 1;
    *wi = Vector3D(0, 0, 1);
    return Vector3D();
  }
  p_theta = 2*sin(theta_phi.x) * exp(-pow(tan(theta_phi.x)/alpha, 2))
     / (alpha*alpha * pow(cos(theta_phi.x), 3));
  p_phi = 1. / (2*PI);
  pdf_h = p_theta * p_phi / sin(theta_phi.x);
  pdf_wi = pdf_h / (4*dot(*wi, h));
  *pdf = pdf_wi;
  // if (*pdf <= 0 || *pdf >= INF_F) {
  //   std::cout << "invalid pdf " << *pdf << ", wo = " << wo << ", wi = " << *wi << std::endl;
  // }
  Vector3D out;
  out = MicrofacetBSDF::f(wo, *wi);
  // if (out.x < EPS_F || out.y < EPS_F || out.z < EPS_F || out.x >= INF_D || out.y >= INF_D || out.z >= INF_D) {
  //   std::cout << "Invalid output: " << out << std::endl;
  // }

  return out;
}


double MicrofacetBSDF::sample_pdf(const Vector3D& wo, const Vector3D& wi) const {
  std::cout << "sample_pdf not ready for MicrofacetBSDF" << std::endl;
  assert(0);
  return 0.;
}

void MicrofacetBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Micofacet BSDF"))
  {
    DragDouble3("eta", &eta[0], 0.005);
    DragDouble3("K", &k[0], 0.005);
    DragDouble("alpha", &alpha, 0.005);
    ImGui::TreePop();
  }
}

// Refraction BSDF //

Vector3D RefractionBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D RefractionBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
  // TODO Assignment 7: Part 1
  // Implement RefractionBSDF
  if (!refract(wo, wi, ior)) {
    return Vector3D();
  }
  *pdf = 1;
  double eta, costheta;
  eta = wo.z > 0 ? (double) 1 / ior : ior;
  costheta = fabs(wi->z) / wi->norm();
  return transmittance / costheta / (eta*eta);
}


double RefractionBSDF::sample_pdf(const Vector3D& wo, const Vector3D& wi) const {
  // proportion of dirac delta distribution
  return 1.;
}

void RefractionBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Refraction BSDF"))
  {
    DragDouble3("Transmittance", &transmittance[0], 0.005);
    DragDouble("ior", &ior, 0.005);
    ImGui::TreePop();
  }
}

// Glass BSDF //

Vector3D GlassBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D GlassBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {

  // TODO Assignment 7: Part 1
  // Compute Fresnel coefficient and either reflect or refract based on it.

  // compute Fresnel coefficient and use it as the probability of reflection
  // - Fundamentals of Computer Graphics page 305
  Vector3D wi_reflect, wi_refract;
  bool total_internal_reflection;
  reflect(wo, &wi_reflect);
  total_internal_reflection = !refract(wo, &wi_refract, ior);
  if (total_internal_reflection) {
    *pdf = 1;
    *wi = wi_reflect;
    double costheta = fabs(wi->z) / wo.norm();
    return reflectance / costheta;
  }
  // Schlick's approximation
  double wi_refract_costheta, eta, R, R0;
  wi_refract_costheta = fabs(wi_refract.z) / wi_refract.norm();
  eta = wo.z > 0 ? (double) 1 / ior : ior;
  R0 = pow((1-eta) / (1+eta), 2);
  R = R0 + (1-R0) * pow(1-wi_refract_costheta, 5);
  if (coin_flip(R)) {
    *wi = wi_reflect;
    *pdf = R;
    double costheta = fabs(wi->z) / wi->norm();
    return R * reflectance / costheta;
  } else {
    *wi = wi_refract;
    *pdf = 1 - R;
    double costheta = fabs(wi->z) / wi->norm();
    return (1-R) * transmittance / costheta / (eta*eta);
  }
}


double GlassBSDF::sample_pdf(const Vector3D& wo, const Vector3D& wi) const {
  std::cout << "sample_pdf not ready for GlassBSDF" << std::endl;
  assert(0);
  return 0.;
}

void GlassBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Refraction BSDF"))
  {
    DragDouble3("Reflectance", &reflectance[0], 0.005);
    DragDouble3("Transmittance", &transmittance[0], 0.005);
    DragDouble("ior", &ior, 0.005);
    ImGui::TreePop();
  }
}

void BSDF::reflect(const Vector3D wo, Vector3D* wi) {

  // TODO Assignment 7: Part 1
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
  *wi = Vector3D {-wo.x, -wo.y, wo.z};

}

bool BSDF::refract(const Vector3D wo, Vector3D* wi, double ior) {

  // TODO Assignment 7: Part 1
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.
  bool enter = false;
  if (wo.z > 0) {
    enter = true;
  }

  double eta = enter ? (double) 1 / ior : ior;
  double z_sq;
  z_sq = 1 - eta*eta * (1 - wo.z*wo.z);
  if (z_sq < 0) {
    return false;
  }
  double sgn = enter ? -1 : 1;
  *wi = Vector3D {-eta*wo.x, -eta*wo.y, sgn*sqrt(z_sq)};

  return true;

}

} // namespace CGL
