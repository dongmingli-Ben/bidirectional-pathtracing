#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.
  // printf("num primitives: %ld\n", end-start);
  if (end-start <= max_leaf_size) {

    BBox bbox = (*start)->get_bbox();

    for (auto p = start; p != end; p++) {
      BBox bb = (*p)->get_bbox();
      bbox.expand(bb);
    }

    BVHNode *node = new BVHNode(bbox);
    vector<Primitive *> *prims = new vector<Primitive*>(start, end);
    node->start = prims->begin();
    node->end = prims->end();
    // cout << "leaf bbox: " << bbox << endl;
    return node;
  }
  // select axis that has the largest range
  double x_max, x_min, y_max, y_min, z_max, z_min;
  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    x_max = p == start ? bb.centroid().x : max(x_max, bb.centroid().x);
    x_min = p == start ? bb.centroid().x : min(x_min, bb.centroid().x);
    y_max = p == start ? bb.centroid().y : max(y_max, bb.centroid().y);
    y_min = p == start ? bb.centroid().y : min(y_min, bb.centroid().y);
    z_max = p == start ? bb.centroid().z : max(z_max, bb.centroid().z);
    z_min = p == start ? bb.centroid().z : min(z_min, bb.centroid().z);
  }
  double x_range = x_max - x_min;
  double y_range = y_max - y_min;
  double z_range = z_max - z_min;
  double ranges[3] = { x_range, y_range, z_range };
  double mins[3] = { x_min, y_min, z_min };
  double max_range = max(x_range, max(y_range, z_range));
  assert(max_range > 0);
  int axis_index;
  for (axis_index = 0; axis_index < 3; axis_index++) {
    if (ranges[axis_index] == max_range) {
      break;
    }
  }
  // // for debug
  // for (int i = 0; i < 3; i++) {
  //   printf("%d, range: %f, min: %f\n", i, ranges[i], mins[i]);
  // }
  // printf("axis: %d, max_range: %f\n", axis_index, max_range);
  // // end for debug
  // split by spatial midpoint
  double midpoint = mins[axis_index] + ranges[axis_index]/2;
  vector<Primitive*> left, right;
  for (auto p = start; p != end; p++) {
    if ((*p)->get_bbox().centroid()[axis_index] <= midpoint) {
      left.push_back(*p);
    } else {
      right.push_back(*p);
    }
  }
  assert(left.size() > 0);
  assert(right.size() > 0);
  BVHNode *left_node, *right_node, *node;
  left_node = construct_bvh(left.begin(), left.end(), max_leaf_size);
  right_node = construct_bvh(right.begin(), right.end(), max_leaf_size);
  BBox bb(left_node->bb.min, left_node->bb.max);
  bb.expand(right_node->bb);
  node = new BVHNode(bb);
  node->l = left_node;
  node->r = right_node;
  // cout << end-start << ": node bbox: " << node->bb << endl;
  return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
  BBox bbox = node->bb;
  double t1 = ray.min_t, t2 = ray.max_t;
  if (!bbox.intersect(ray, t1, t2)) {
    return false;
  }
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      if ((*p)->has_intersection(ray)) {
        return true;
      }
    }
  }
  bool hit1, hit2;
  hit1 = has_intersection(ray, node->l);
  // returns as soon as it finds a hit
  if (hit1) {
    return true;
  }
  hit2 = has_intersection(ray, node->r);
  return hit1 || hit2;

}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // cout << node << endl;
  BBox bbox = node->bb;
  double t1 = ray.min_t, t2 = ray.max_t;
  if (!bbox.intersect(ray, t1, t2)) {
    return false;
  }
  if (node->isLeaf()) {
    bool hit = false, h;
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      h = (*p)->intersect(ray, i);
      // if (h) {
      //   cout << "intersection: " << i->t << endl;
      // }
      hit = h || hit;
    }
    return hit;
  }
  bool hit1, hit2;
  hit1 = intersect(ray, i, node->l);
  hit2 = intersect(ray, i, node->r);
  return hit1 || hit2;


}

} // namespace SceneObjects
} // namespace CGL
