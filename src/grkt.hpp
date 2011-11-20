#ifndef GRKT_RENDERER_HPP
#define GRKT_RENDERER_HPP

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_01.hpp>>
#include <boost/random/variate_generator.hpp>
#include <glm/glm.hpp>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#include "bvh.hpp"


namespace grkt {

	struct screen_t {
		size_t width;
		size_t height;
		float aspect_ratio;
	};
	
	struct camera_t {
		glm::vec3 origin;
		glm::vec3 bases[3];
	};

	struct context_t {
		
		screen_t screen;
		camera_t camera;	
		int sample_size;
		float sample_size_inv;
		
		const bvh_tree_t *bvh_tree;
		const sphere_t *scene_light;
		glm::vec3 material_color;
				
		context_t(const bvh_tree_t *tree, size_t width = 800, size_t height = 600) : bvh_tree(tree) {
			screen.width = width;
			screen.height = height;
			screen.aspect_ratio = (float)screen.height / (float)screen.width;	

			sample_size = 4;
			sample_size_inv = 1.0f / (float)sample_size;
		}
		
	};
	
	struct renderer_t {
	
		const context_t *context;
		unsigned char *rgb;
	
		renderer_t(const context_t *ctx, unsigned char *rgb_buf) : context(ctx), rgb(rgb_buf) { }	
		void operator() (const tbb::blocked_range<size_t>& range) const;
	
	};
	
}

typedef boost::variate_generator< boost::random::mt19937, boost::random::uniform_01<float> > rng_t;

#endif
