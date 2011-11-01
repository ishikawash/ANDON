#include "bvh.hpp"

using namespace std;
using namespace glm;

int bvh_node_t::node_id_sequence = 1;

bvh_node_t::bvh_node_t() {
	node_id = node_id_sequence++;
	children[0] = children[1] = NULL;
}

bvh_node_t& bvh_node_t::initialize_as_branch(int axis, bvh_node_t *child0, bvh_node_t *child1) {
	children[0] = child0;
	children[1] = child1;
	bounds = child0->bounds;
	bounds.merge(child1->bounds);
	split_axis = axis;
	return *this;
}

bvh_node_t& bvh_node_t::initialize_as_leaf(int first, int n, const bbox_t &b) {
	first_shape_offset = first;
	shape_num = n;
	bounds = b;
	return *this;		
}

bvh_tree_t::bvh_tree_t(const vector<shape_ref_t> &input_shapes) {
	shapes.resize(input_shapes.size());
	copy(input_shapes.begin(), input_shapes.end(), shapes.begin());
	
	root = NULL;
	nodes = NULL;
	total_node_count = 0;
}

void bvh_tree_t::build() {
	vector<bvh_node_info_t> node_info_list;
	node_info_list.reserve(shapes.size());
	for (size_t i = 0; i < shapes.size(); i++) {
		const bbox_t &bound = shapes[i]->bound();
		node_info_list.push_back(bvh_node_info_t(i, bound));
	}

	size_t total_nodes = 0;
	vector<shape_ref_t> ordered_shapes;
	ordered_shapes.reserve(shapes.size());
	root = recursive_build(node_info_list, 0, shapes.size(), &total_nodes, ordered_shapes);
	
	assert(shapes.size() == ordered_shapes.size());	
		
	shapes.swap(ordered_shapes);
	total_node_count = total_nodes;
}

bvh_node_t* bvh_tree_t::recursive_build(vector<bvh_node_info_t> &node_info_list, size_t start, size_t end, size_t *total_nodes, vector<shape_ref_t> &ordered_shapes) {
	(*total_nodes)++;

	bvh_node_t *node = new bvh_node_t();

	bbox_t bound;
	for (size_t i = start; i < end; i++) {
		bound.merge(node_info_list[i].bound);
	}

	size_t shape_num = end - start;
	if (shape_num < 1) {
		return NULL;
	} else if (shape_num <= 2) {
		leaf_node:
		
		size_t first_shape_offset = ordered_shapes.size();
		for (size_t i = start; i < end; i++) {
			size_t shape_index = node_info_list[i].shape_index;
			ordered_shapes.push_back(shapes[shape_index]);
		}
		
		node->initialize_as_leaf(first_shape_offset, shape_num, bound);			
		return node;
	} else {
		bbox_t centroid_bound;
		for (size_t i = start; i < end; i++) {
			centroid_bound.merge(node_info_list[i].centroid);
		}
		int dim = centroid_bound.maximum_extent();
		
		if (centroid_bound.max_point[dim] == centroid_bound.min_point[dim]) {
			goto leaf_node;
		}		
		float mid_point = 0.5f * (centroid_bound.max_point[dim] + centroid_bound.min_point[dim]);
		const bvh_node_info_t *node_info = partition(&node_info_list[start], &node_info_list[end - 1] + 1, bvh_mid_comparator_t(dim, mid_point));			
		size_t mid = node_info - &node_info_list[0];
		
		node->initialize_as_branch(
			dim,
			recursive_build(node_info_list, start, mid, total_nodes, ordered_shapes),
			recursive_build(node_info_list, mid, end, total_nodes, ordered_shapes)
		);		
		return node;
	}
}

void bvh_tree_t::flatten() {
	nodes = new bvh_linear_node_t[total_node_count];
	size_t offset = 0;
	recursive_flatten(root, &offset);
}

size_t bvh_tree_t::recursive_flatten(const bvh_node_t *node, size_t *offset) {
	bvh_linear_node_t &linear_node = nodes[*offset];
	linear_node.bounds = node->bounds;
	linear_node.node_id = node->node_id;
	size_t _offset = (*offset)++;
			
	if (node->shape_num > 0) {
		linear_node.shape_offset = node->first_shape_offset;
		linear_node.shape_num = node->shape_num;
	} else {
		linear_node.axis = node->split_axis;
		linear_node.shape_num = 0;
		recursive_flatten(node->first_child(), offset);
		linear_node.second_child_offset = recursive_flatten(node->second_child(), offset);
	}
	
	return _offset;
}

bool bvh_tree_t::intersect(const ray_t& ray, isect_t &isect) {
	node_ids_intersected.clear();
	return __intersect(ray, isect, node_ids_intersected);
}

bool bvh_tree_t::__intersect(const ray_t& ray, isect_t &isect, vector<int> &node_ids) const {
	bool hit = false;
	vec3 origin = ray.point_at(ray.tmin);
	vec3 inv_direction = 1.0f / ray.direction;
	ivec3 sign = ivec3(inv_direction.x < 0.0f, inv_direction.y < 0.0f, inv_direction.z < 0.0f);
	
	size_t node_num = 0;
	size_t todo_offset = 0;
	size_t todo[64];
	while (true) {
		bvh_linear_node_t *node = &nodes[node_num];
		if (node->bounds.intersect(ray, sign, inv_direction)) {
			node_ids.push_back(node->node_id);
			
			if (node->shape_num > 0) {
				for (size_t i = 0; i < node->shape_num; i++) {
					size_t k = node->shape_offset + i;
					assert(k < shapes.size());
					if (shapes[k]->intersect(ray, isect)) {
						hit = true;
					}
				}
				
				if (todo_offset == 0)
					break;
				node_num = todo[--todo_offset];	
				
			} else {
				if (sign[node->axis]) {
					todo[todo_offset++] = node_num + 1;
					node_num = node->second_child_offset;
				} else {
					todo[todo_offset++] = node->second_child_offset;
					node_num = node_num + 1;
				}
			}
		} else {
			if (todo_offset == 0)
				break;
			node_num = todo[--todo_offset];
		}
	}
	
	return hit;
}

