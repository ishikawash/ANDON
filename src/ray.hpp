#ifndef RAY_HPP
#define RAY_HPP

#include <glm/glm.hpp>

struct ray_t {
	glm::vec3  origin;
	glm::vec3  direction;
	float tmin;
	float tmax;

	ray_t(const glm::vec3 &o, const glm::vec3 &d) : origin(o), direction(d), tmin(0.0), tmax(INFINITY) { }

	glm::vec3 point_at(float t) const {
		return origin + direction * t;
	}
        
};

#endif
