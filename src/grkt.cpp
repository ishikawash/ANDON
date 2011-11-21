#include "grkt.hpp"

using namespace std;
using namespace glm;
using namespace tbb;
using namespace grkt;


vec3 uniform_sphere_sample(const sphere_t &sphere, const vec3 &point, rng_t &rng) {
	float cos_t = 2.0 * rng() - 1.0;
	float sin_t = sqrtf(1.0 - cos_t * cos_t);
	float phi = 2.0 * M_PI * rng();
	
	float x = sphere.radius * sin_t * cos(phi);
  float y = sphere.radius * sin_t * sin(phi);
  float z = sphere.radius * cos_t;
	
	vec3 P = vec3(x, y, z);
	vec3 L = point - sphere.center;
	if (dot(P, L) < 0.0f)
		P *= -1.0f;	
	return P + sphere.center;
}

void renderer_t::operator() (const blocked_range<size_t>& range) const {
	size_t width = context->screen.width;
	size_t height = context->screen.height;
	
	const vec3 &origin = context->camera.origin;
	const vec3 &u = context->camera.bases[0];
	const vec3 &v = context->camera.bases[1];
	const vec3 &w = context->camera.bases[2];
	
	boost::random::mt19937 gen(static_cast<unsigned long>(time(0)));
	boost::random::uniform_01<float> distro;
	rng_t rng(gen, distro);
	
	for (size_t j = range.begin(); j < range.end(); j++) {
		for (size_t i = 0; i < width; i++) {
			int k = 3 * (i + width * j);
			
			vec3 lr = vec3(0.0);
			for (int n = 0; n < context->sample_size; n++) {

				float r1 = 2.0f * rng();
				float r2 = 2.0f * rng();
				float dx = (r1 < 1.0f) ? sqrtf(r1) - 1.0f : 1.0f - sqrtf(2.0f - r1);
				float dy = (r2 < 1.0f) ? sqrtf(r2) - 1.0f : 1.0f - sqrtf(2.0f - r2);

				float a = ( (i - dx) - width/2.0 ) / (width/2.0);
				float b = ( height/2.0 - (j - dy) ) / (height/2.0) * context->screen.aspect_ratio;

				vec3 direction = normalize(a*u + b*v + w);
				ray_t ray(origin, direction);

				isect_t isect;
				if (!context->bvh_tree->intersect(ray, isect))
					continue;

				const shape_t *shape = isect.shape;
				vec3 P = ray.point_at(isect.t);
				vec3 Q = uniform_sphere_sample(*context->scene_light, P, rng);			
				vec3 L = normalize(Q - P);

				vec3 N = shape->normal(P);
				float kd = clamp(dot(L, N), 0.0f, 1.0f);
				float ks = 0.0f;
				if (dot(L, N) > 0.0f) {
					vec3 H = normalize(L - P); // -P + L
					ks = glm::pow(glm::max(dot(H, N), 0.0f), 50.0f);
				}

				ray_t shadow_ray(P + 0.01f * L, L);
				float shadow = context->bvh_tree->intersect(shadow_ray) ? 0.6f : 1.0f;

				lr += glm::max(shadow * context->material_color * (kd + ks), 0.0);
			}
			vec3 radiance = clamp(lr * context->sample_size_inv, 0.0, 1.0);

			rgb[k] = glm::floor(255.0 * radiance.r);
			rgb[k + 1] = glm::floor(255.0 * radiance.g);
			rgb[k + 2] = glm::floor(255.0 * radiance.b);			
						
		}
	}
	
}

