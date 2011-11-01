#ifndef SHAPE_HPP
#define SHAPE_HPP

#include <boost/shared_ptr.hpp>
#include <glm/glm.hpp>

#include "ray.hpp"
#include "bbox.hpp"

struct shape_t;

typedef boost::shared_ptr<shape_t> shape_ref_t;

struct isect_t {
	float t;
	const shape_t *shape;
	
	isect_t() : t(INFINITY) { }
	
};

struct shape_t {
	
	virtual ~shape_t() { }
	
	virtual const bbox_t& bound() const = 0;
	virtual glm::vec3 normal(const glm::vec3 &p) const = 0;
	virtual bool intersect(const ray_t &ray, isect_t &isect) const = 0;
	
};


struct plane_t : public shape_t {
		
	plane_t(const glm::vec3 &p, const glm::vec3 &n);
	
	const bbox_t& bound() const {
		return __bbox;
	}
	
	glm::vec3 normal(const glm::vec3 &p) const {
		#pragma unused (p)
		return __normal;
	}
	
	bool intersect(const ray_t &ray, isect_t &isect) const;
	
private:
	glm::vec3 __point;
	glm::vec3 __normal;
	bbox_t __bbox;
	
};


struct sphere_t : public shape_t {
	
	sphere_t(const glm::vec3 &p, float r);
	
	const bbox_t& bound() const {
		return __bbox;
	}
	
	glm::vec3 normal(const glm::vec3 &p) const {
		return glm::normalize(p - center);
	}
	
	bool intersect(const ray_t &ray, isect_t &isect) const;
	
	glm::vec3 center;
	float radius;
	
private:
	bbox_t __bbox;
	
};


#endif
