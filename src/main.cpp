
#include <iostream>
#include <vector>
#include <cstdio>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "triangle_mesh.hpp"
#include "bvh.hpp"
#include "grkt.hpp"


using namespace std;
using namespace glm;
using namespace glm::gtx;
using namespace tbb;

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
	
	grkt::context_t ctx(&bvh_tree);
	
	sphere_t sphere_light(vec3(-1.0, 3.0, 1.0), 0.8);
	ctx.scene_light = &sphere_light;
	ctx.material_color = vec3(0.6, 0.6, 0.6);
	
	vec3 centroid = 0.5f * ( ctx.bvh_tree->nodes[0].bounds.max_point + ctx.bvh_tree->nodes[0].bounds.min_point );
	mat4 O = translate(mat4(1.0f), centroid); // origin of camera coordinate 
	
	vec3 eye = vec3(0.1, 0.05, 0.2);
	vec3 center = vec3(0.0, 0.0, 0.0);
	vec3 up = vec3(0.0, 1.0, 0.0);
	mat4 MC = lookAt(vec3(-eye.x, -eye.y, eye.z), center, up);
	
	ctx.camera.origin = vec3(O * vec4(eye.x, eye.y, eye.z, 1.0));
	
	mat3 M = mat3(MC); // upper 3x3
	ctx.camera.bases[0] = normalize(M * vec3(1.0, 0.0, 0.0));
	ctx.camera.bases[1] = normalize(M * vec3(0.0, 1.0, 0.0));
	ctx.camera.bases[2] = normalize(M * vec3(0.0, 0.0, -1.0));
	
	vector<unsigned char> rgb;
	rgb.resize(ctx.screen.width * ctx.screen.height * 3);
	
	grkt::renderer_t renderer(&ctx, &rgb[0]);
	parallel_for(blocked_range<size_t>(0, ctx.screen.height), renderer);

	write_image(rgb, ctx.screen.width, ctx.screen.height);
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
