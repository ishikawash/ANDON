#ifndef BBOX_HPP
#define BBOX_HPP

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

#include "ray.hpp"


struct bbox_t {
	
	glm::vec3 max_point;
	glm::vec3 min_point;
	
	bbox_t() : max_point(glm::vec3(-INFINITY)), min_point(glm::vec3(INFINITY)) { }
	
	bbox_t(const bbox_t &box) {
		max_point = box.max_point;
		min_point = box.min_point;
	}
	
	bbox_t& merge(const glm::vec3 &point) {
		max_point = glm::max(max_point, point);
		min_point = glm::min(min_point, point);
		return *this;
	}
	
	bbox_t& merge(const bbox_t &box) {
		merge(box.min_point);
		merge(box.max_point);
		return *this;
	}
	
	int maximum_extent() const {
		glm::vec3 v = max_point - min_point;
		return ( v.x > v.y ) ? ( (v.x > v.z) ? 0 : 2 ) : ( (v.y > v.z) ? 1 : 2 );
	}
	
	std::string str() const;

	const glm::vec3& operator[](int i) const {
		switch(i) {
			case 0: {
				return min_point;
			}
			case 1: {
				return max_point;
			}
			default: {
				assert(false);
			}
		}
	}

	bool intersect(const ray_t &ray, const glm::ivec3 &sign, const glm::vec3& inv_direction) const;

};

#endif

