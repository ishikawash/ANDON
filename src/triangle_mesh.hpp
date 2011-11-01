#ifndef TRIANGLE_MESH_HPP
#define TRIANGLE_MESH_HPP

#include <vector>
#include <algorithm>
#include <boost/shared_ptr.hpp>

// #define GLM_SWIZZLE
#include <glm/glm.hpp>

#include "ray.hpp"
#include "bbox.hpp"
#include "shape.hpp"


struct triangle_mesh_t {
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec3> normals;
	std::vector<unsigned int> indices;
	
	void refine(std::vector<shape_ref_t> &triangles) {
		refine_to_triangles(triangles);
		compute_vertex_normals(triangles);
	}
	
	void refine_to_triangles(std::vector<shape_ref_t> &triangles) const;
	
	void compute_vertex_normals(const std::vector<shape_ref_t> &triangles);
	
	static bool load(const char* ctm_filepath, triangle_mesh_t &mesh);
	
};

struct triangle_t : public shape_t {
		
	triangle_t(const triangle_mesh_t *m, size_t n);
	
	const bbox_t& bound() const {
		return __bbox;
	}
	
	glm::vec3 normal(const glm::vec3 &p) const;
	
	const glm::vec3& v(size_t i) const {
		return __mesh->vertices[indices[i]];
	}
	
	const glm::vec3& vn(size_t i) const {
		return __mesh->normals[indices[i]];
	}
	
	bool intersect(const ray_t &ray, isect_t &isect) const;
	
	const unsigned int *indices;
	
private:
	const triangle_mesh_t *__mesh;
	bbox_t __bbox;
	
};

#endif
