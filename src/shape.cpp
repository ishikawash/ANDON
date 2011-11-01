#include "shape.hpp"

using namespace glm;


plane_t::plane_t(const glm::vec3 &p, const glm::vec3 &n) : __point(p), __normal(n) { 
	__bbox.max_point = glm::vec3(1.0f, __point.y, 1.0f);
	__bbox.min_point = glm::vec3(-1.0f, __point.y, -1.0f);
}

bool plane_t::intersect(const ray_t &ray, isect_t &isect) const {
	float d = -1.0f * dot(__point, __normal);  // distance from origin to plane
	
	float a = dot(ray.direction, __normal);		
  if (fabs(a) < 0.0f)
		return false;

	float t = -1.0 * (dot(ray.origin, __normal) + d) / a;
	if (t < isect.t) {
		isect.t = t;
		isect.shape = this;
		return true;
	} else {
		return false;
	}
}

sphere_t::sphere_t(const glm::vec3 &p, float r) : center(p), radius(r) {
	__bbox.max_point = center + vec3(r, r, r);
	__bbox.min_point = center + vec3(-r, -r, -r);
}

bool sphere_t::intersect(const ray_t &ray, isect_t &isect) const {
	vec3 rs = ray.origin - center;
	float b = dot(rs, ray.direction);
	float c = dot(rs, rs) - radius*radius;
	
	float det = b*b - c;
	if (det < 0.0f) {
		return false;
	}

	float t = -b - sqrtf(det);
	if (t < 0.0f) {
		return false;
	}
	
	if (t < isect.t) {
		isect.t = t;
		isect.shape = this;
		return true;		
	} else {
		return false;
	}
}