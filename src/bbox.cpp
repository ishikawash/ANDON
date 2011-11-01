#include "bbox.hpp"

using namespace glm;
using namespace glm::gtx;


bool bbox_t::intersect(const ray_t &ray, const ivec3 &sign, const vec3& inv_direction) const {
	const bbox_t &box = *this;
  float tmin, tmax, tymin, tymax, tzmin, tzmax;

  tmin = (box[sign[0]].x - ray.origin.x) * inv_direction.x;
  tmax = (box[1-sign[0]].x - ray.origin.x) * inv_direction.x;
  tymin = (box[sign[1]].y - ray.origin.y) * inv_direction.y;
  tymax = (box[1-sign[1]].y - ray.origin.y) * inv_direction.y;
  if ( (tmin > tymax) || (tymin > tmax) ) 
    return false;
  if (tymin > tmin)
    tmin = tymin;
  if (tymax < tmax)
    tmax = tymax;

  tzmin = (box[sign[2]].z - ray.origin.z) * inv_direction.z;
  tzmax = (box[1-sign[2]].z - ray.origin.z) * inv_direction.z;
  if ( (tmin > tzmax) || (tzmin > tmax) ) 
    return false;
  if (tzmin > tmin)
    tmin = tzmin;
  if (tzmax < tmax)
    tmax = tzmax;

  return ( (tmin < ray.tmax) && (tmax > ray.tmin) );
}

std::string bbox_t::str() const {
	std::stringstream ss;
	ss << " max=" << string_cast::to_string(max_point);
	ss << " min=" << string_cast::to_string(min_point);
	return ss.str();
}

