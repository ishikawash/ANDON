#include <openctmpp.h>
#include "shape.hpp"
#include "triangle_mesh.hpp"

using namespace std;
using namespace glm;
using namespace glm::gtx;


triangle_t::triangle_t(const triangle_mesh_t *m, size_t n) : __mesh(m) {
	indices = &__mesh->indices[3 * n];
	
	for (int i = 0; i < 3; i++)
		__bbox.merge(v(i));
}

bool triangle_t::intersect(const ray_t &ray, isect_t &isect) const {
	vec3 e0 = v(1) - v(0);
  vec3 e1 = v(2) - v(0);

  vec3 pv = cross(ray.direction, e1);
  float det = dot(e0, pv);
  if(fabs(det) < 0.0f)
    return false;

  float inv_det = 1.0f/det;

  vec3 tv = ray.origin - v(0);
  vec3 qv = cross(tv, e0);
  float u = dot(tv, pv) * inv_det;
	if (u < 0.0f)
		return false;
  float v = dot(ray.direction, qv) * inv_det;
	if (v < 0.0f)
		return false;
  if (u + v - 1.0f > 0.0f)
    return false;

	float t = dot(e1, qv) * inv_det;
	if (isect.t > t) {
		isect.t = t;
		isect.shape = this;
		return true;	
	} else {
		return false;
	}

}

vec3 triangle_t::normal(const vec3 &p) const {
	vec3 e0 = v(1) - v(0);
  vec3 e1 = v(2) - v(0);

	vec3 q = p - v(0);
	float a = dot(normalize(e0), q) / length(e0);
	float b = dot(normalize(e1), q) / length(e1);
	float c = 1.0f - ( a + b );
	return normalize(a * vn(1) + b * vn(2) + c * vn(0));
}

void triangle_mesh_t::refine_to_triangles(vector<shape_ref_t> &triangles) const {
	size_t triangle_count = indices.size() / 3;
	triangles.reserve(triangle_count);
	
	for (size_t i = 0; i < triangle_count; i++) {
		triangle_t *triangle = new triangle_t(this, i);
		shape_ref_t t = shape_ref_t(triangle);
		triangles.push_back(t);		
	}
	
}

void triangle_mesh_t::compute_vertex_normals(const vector<shape_ref_t> &triangles) {
	normals.resize(vertices.size());
	
	for (size_t i = 0; i < triangles.size(); i++) {
		shape_t *shape = triangles[i].get();
		triangle_t *t = dynamic_cast<triangle_t *>(shape);
		vec3 normal = cross(t->v(1) - t->v(0), t->v(2) - t->v(0));
		for (size_t j = 0; j < 3; j++) {
			normals[t->indices[j]] += normal;
		}
	}

	for (size_t i = 0; i < normals.size(); i++) {
		normals[i] = normalize(normals[i]);
	}

}

bool triangle_mesh_t::load(const char* ctm_filepath, triangle_mesh_t &mesh) {
	CTMimporter ctm;

	try {
		ctm.Load(ctm_filepath);

		CTMuint vertex_count = ctm.GetInteger(CTM_VERTEX_COUNT);
		const CTMfloat* vertices = ctm.GetFloatArray(CTM_VERTICES);
		
		mesh.vertices.reserve(vertex_count);				
		const CTMfloat* vp = vertices;
		for (unsigned int i = 0; i < vertex_count; i++) {
			vec3 v = vec3(*vp, *(vp + 1), *(vp + 2));
			mesh.vertices.push_back(v);
			vp += 3;
		}
		assert(mesh.vertices.size() == vertex_count);
		
		CTMuint triangle_count = ctm.GetInteger(CTM_TRIANGLE_COUNT);
		const CTMuint* indices = ctm.GetIntegerArray(CTM_INDICES);
		
		size_t index_count = triangle_count * 3;
		mesh.indices.resize(index_count);
		memcpy(&mesh.indices[0], indices, sizeof(unsigned int) * index_count);				
		assert(mesh.indices.size() == index_count);
		
	} catch (ctm_error &e) {
		cerr << "Loading CTM file failed: " << e.what() << endl;
		return false;
	}

	return true;
}

