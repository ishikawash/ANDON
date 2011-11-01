#ifndef BVH_HPP
#define BVH_HPP

#include "triangle_mesh.hpp"


struct bvh_node_info_t {
	size_t shape_index;
	glm::vec3 centroid;
	bbox_t bound;
	
	bvh_node_info_t(size_t i, const bbox_t &bbox) : shape_index(i), bound(bbox) {
		centroid = 0.5f * ( bound.max_point + bound.min_point );
	}
	
};

struct bvh_node_t {
	bbox_t bounds;
	bvh_node_t *children[2];
	int split_axis;
	size_t first_shape_offset;
	size_t shape_num;
	
	int node_id;
	
	static int node_id_sequence;
	
	bvh_node_t();
	
	bool is_leaf() const {
		return ( children[0] == NULL ) && ( children[1] == NULL );
	}
	
	const bvh_node_t* first_child() const {
		return children[0];
	}

	const bvh_node_t* second_child() const {
		return children[1];
	}

	bvh_node_t& initialize_as_branch(int axis, bvh_node_t *child0, bvh_node_t *child1);

	bvh_node_t& initialize_as_leaf(int first, int n, const bbox_t &b);

};


struct bvh_mid_comparator_t {	
	int dim;
	float mid_point;
	
	bvh_mid_comparator_t(int d, float mp) : dim(d), mid_point(mp) { }
	
	bool operator()(const bvh_node_info_t &node_info) const {
		return node_info.centroid[dim] < mid_point;
	}
	
};

struct bvh_linear_node_t {
	bbox_t bounds;
	
	union {
		size_t shape_offset;
		size_t second_child_offset;
	};
	
	size_t shape_num;
	int axis;
	
	int node_id;
	
	bool is_leaf() const {
		return ( shape_num > 0 );
	}
	
};

struct bvh_tree_t {
	std::vector<shape_ref_t> shapes;
	size_t total_node_count;	
	bvh_node_t *root;
	bvh_linear_node_t *nodes;
	
	std::vector<int> node_ids_intersected;
	
	bvh_tree_t(const std::vector<shape_ref_t> &input_shapes);

	void build();
	bvh_node_t* recursive_build(std::vector<bvh_node_info_t> &node_info_list, size_t start, size_t end, size_t *total_nodes, std::vector<shape_ref_t> &ordered_shapes);
	
	void flatten();	
	size_t recursive_flatten(const bvh_node_t *node, size_t *offset);
	
	bool intersect(const ray_t& ray, isect_t &isect);	
	bool __intersect(const ray_t& ray, isect_t &isect, std::vector<int> &node_ids) const;
	
	bool intersect(const ray_t& ray) const {
		isect_t isect;
		std::vector<int> node_ids;
		return __intersect(ray, isect, node_ids);
	}
	
	void output_as_graph() {
		std::cout << "digraph G {" << std::endl;
		
		for (size_t i = 0; i < node_ids_intersected.size(); i++) {
			std::printf("%d [style=filled] \n", node_ids_intersected[i]);
		}
		
		recursive_output_as_graph();
		
		std::cout << "}" << std::endl;
	}
	
	void recursive_output_as_graph(size_t offset = 0) {
		const bvh_linear_node_t &node = nodes[offset];
		if (node.is_leaf()) {
			return;
		} else {
			size_t i = offset + 1;
			size_t j = node.second_child_offset;
			std::printf("%d -> %d ;\n", node.node_id, nodes[i].node_id);
			std::printf("%d -> %d ;\n", node.node_id, nodes[j].node_id);
			
			recursive_output_as_graph(i);
			recursive_output_as_graph(j);
		}
	}
	
};

#endif
