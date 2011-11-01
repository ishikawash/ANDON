
#include <iostream>
#include <vector>
#include <cstdio>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_01.hpp>>
#include <boost/random/variate_generator.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "triangle_mesh.hpp"
#include "bvh.hpp"

using namespace std;
using namespace glm;
using namespace glm::gtx;

typedef boost::variate_generator< boost::random::mt19937, boost::random::uniform_01<float> > rng_t;

#define INSPECT(arg)  string_cast::to_string(arg)


bool write_image(const vector<unsigned char> &rgb, size_t width, size_t height) {
	FILE *fp = fopen("out.ppm", "wb"); 
	if (fp == NULL) {
		return false;
	}
	fprintf(fp, "P6\n%ld %ld\n%d\n", width, height, 255);
	fwrite((void *)&rgb[0], sizeof(unsigned char), rgb.size(), fp);
	fclose(fp);     
	return true;
}

vec3 uniform_sphere_sample(const sphere_t &sphere, const vec3 &point, rng_t &rng) {
	float cos_t = 2.0 * rng() - 1.0;
	float sin_t = sqrtf(1.0 - cos_t * cos_t);
	float phi = 2.0 * M_PI * rng();
	
	float x = sphere.radius * sin_t * cos(phi);
  float y = sphere.radius * sin_t * sin(phi);
  float z = sphere.radius * cos_t;
	
	vec3 P = vec3(x, y, z);
	vec3 L = point - sphere.center;
	if (dot(P, L) > 0.0f)
		P *= 1.0f;	
	return P + sphere.center;
}

void render(const char *ctm_filepath) {
	triangle_mesh_t mesh;
	if (!triangle_mesh_t::load(ctm_filepath, mesh)) {
		cerr << "Loading .ctm file failed: " << ctm_filepath << endl;
		return;
	}
	
	vector<shape_ref_t> shapes;
	mesh.refine(shapes);

	shape_ref_t plane(new plane_t(vec3(0.0, 0.05, 0.0), vec3(0.0, 1.0, 0.0)));
	shapes.push_back(plane);

	bvh_tree_t bvh_tree(shapes);
	bvh_tree.build();
	bvh_tree.flatten();
	
	int width = 800;
	int height = 600;
	float aspect_ratio = (float)height / (float)width;
	
	vector<unsigned char> rgb;
	rgb.resize(width * height * 3);
	
	sphere_t light(vec3(-1.0, 3.0, 1.0), 0.8);
	vec3 color = vec3(0.6, 0.6, 0.6);
	
	vec3 centroid = 0.5f * ( bvh_tree.nodes[0].bounds.max_point + bvh_tree.nodes[0].bounds.min_point );
	mat4 O = translate(mat4(1.0f), centroid); // origin of camera coordinate 
	
	vec3 eye = vec3(0.1, 0.05, 0.2);
	vec3 center = vec3(0.0, 0.0, 0.0);
	vec3 up = vec3(0.0, 1.0, 0.0);
	mat4 MC = lookAt(vec3(-eye.x, -eye.y, eye.z), center, up);
	
	vec3 origin = vec3(O * vec4(eye.x, eye.y, eye.z, 1.0));
	
	mat3 M = mat3(MC); // upper 3x3
	vec3 u = normalize(M * vec3(1.0, 0.0, 0.0));
	vec3 v = normalize(M * vec3(0.0, 1.0, 0.0));
	vec3 w = normalize(M * vec3(0.0, 0.0, -1.0));
	
	const int sample_size = 4;
	const float sample_size_inv = 1.0f / (float)sample_size;
	
	boost::random::mt19937 gen;
	boost::random::uniform_01<float> distro;
	rng_t rng(gen, distro);
	
	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			int k = 3 * (i + width * j);
			
			vec3 lr = vec3(0.0);
			for (int n = 0; n < sample_size; n++) {

				float r1 = 2.0f * rng();
				float r2 = 2.0f * rng();
				float dx = (r1 < 1.0f) ? sqrtf(r1) - 1.0f : 1.0f - sqrtf(2.0f - r1);
				float dy = (r2 < 1.0f) ? sqrtf(r2) - 1.0f : 1.0f - sqrtf(2.0f - r2);

				float a = ( (i - dx) - width/2.0 ) / (width/2.0);
				float b = ( height/2.0 - (j - dy) ) / (height/2.0) * aspect_ratio;

				vec3 direction = normalize(a*u + b*v + w);
				ray_t ray(origin, direction);

				isect_t isect;
				if (!bvh_tree.intersect(ray, isect))
					continue;
				
				const shape_t *shape = isect.shape;
				vec3 P = ray.point_at(isect.t);
				vec3 Q = uniform_sphere_sample(light, P, rng);			
				vec3 L = normalize(Q - P);

				vec3 N = shape->normal(P);
				float kd = clamp(dot(L, N), 0.0f, 1.0f);
				float ks = 0.0f;
				if (dot(L, N) > 0.0f) {
					vec3 H = normalize(L - P); // -P + L
					ks = glm::pow(glm::max(dot(H, N), 0.0f), 50.0f);
				}

				ray_t shadow_ray(P + 0.01f * L, L);
				float shadow = bvh_tree.intersect(shadow_ray) ? 0.6f : 1.0f;

				lr += glm::max(shadow * color * (kd + ks), 0.0);

			}
			vec3 radiance = clamp(lr * sample_size_inv, 0.0, 1.0);

			rgb[k] = glm::floor(255.0 * radiance.r);
			rgb[k + 1] = glm::floor(255.0 * radiance.g);
			rgb[k + 2] = glm::floor(255.0 * radiance.b);			
						
		}
	}
	
	write_image(rgb, width, height);
}

int main(int argc, char** argv) {
	if (argc != 2) {
		cerr << "CTM file required." << endl;
		return -1;
	}
	
	const char *ctm_filepath = argv[1];
	
	render(ctm_filepath);
	
	return 0;
}
