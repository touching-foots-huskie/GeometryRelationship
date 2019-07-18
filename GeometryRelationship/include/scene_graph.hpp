#include <map>
#include <vector>
#include <stack>
#include <iostream>
#include "geometry_feature.hpp"

namespace geometry_relation {

    struct ObjectNode {
        int object_id;
        int object_class;
        bool visited;
        std::vector<unsigned int> edges_ids;  // index_id for edges related to node
    };

    struct RelationEdge {
        int index_id1;
        int index_id2;
        int object_id1;
        int object_id2;
        int feature_id1;
        int feature_id2;
        ENUM_CONTACT edge_type;
        double edge_confidence;

        // feature property
        Coord feature_point1;
        Coord feature_point2;
        Coord feature_direction1;
        Coord feature_direction2;

        bool used;
    };

    class SceneGraph {
        public:
            SceneGraph () {
                // add root node 
                ObjectNode object_node0;
                object_node0.object_id = 1000;
                object_node0.object_class = 1000;
                object_node0.visited = false;
                object_node0.edges_ids.clear();  // for root, edge ids save the index id for surface
                this->node_vectors_.emplace_back(object_node0);
                this->index_2_id_.push_back(1000);
                this->id_2_index_[1000] = 0;
            };
            
            void GraphEstablish(std::vector<int>& objects_class1, std::vector<int>& objects_class2,
                                std::vector<int>& objects_id1, std::vector<int>& objects_id2,
								std::vector<ENUM_CONTACT>& contact_type_vector,
								std::vector<Coord>& feature_point_1, std::vector<Coord>& feature_point_2,
								std::vector<Coord>& feature_direction_1, std::vector<Coord>& feature_direction_2,
								std::vector<int>& feature_id_1, std::vector<int>& feature_id_2,
								std::vector<double>& noise_level_vector) {
                
                for(unsigned int i = 0; i < objects_id1.size(); i++) {
                    
                    // put every objects into graph
                    unsigned int edge_id = this->edge_vectors_.size();
                    // enumerate data
                    int index_id1;
                    int index_id2;
                    int object_id1 = objects_id1[i];
                    int object_id2 = objects_id2[i];
                    int feature_id1 = feature_id_1[i];
                    int feature_id2 = feature_id_2[i];
                    int object_class1 = objects_class1[i];
                    int object_class2 = objects_class2[i];

                    ENUM_CONTACT edge_type = contact_type_vector[i];
                    double edge_confidence = noise_level_vector[i];

                    // feature property
                    Coord feature_point1 = feature_point_1[i];
                    Coord feature_point2 = feature_point_2[i];
                    Coord feature_direction1 = feature_direction_1[i];
                    Coord feature_direction2 = feature_direction_2[i];

                    // insert Object Node : id1
                    std::map<int, unsigned int>::iterator it_id1 = this->id_2_index_.find(object_id1);
                    if(it_id1 == this->id_2_index_.end()) {
                        ObjectNode object_node1;
                        object_node1.object_id = object_id1;
                        object_node1.object_class = object_class1;
                        object_node1.visited = false;
                        object_node1.edges_ids.clear();
                        object_node1.edges_ids.emplace_back(edge_id);
                        this->node_vectors_.emplace_back(object_node1);

                        this->id_2_index_[object_id1] = this->index_2_id_.size();
                        index_id1 = this->index_2_id_.size();
                        this->index_2_id_.push_back(object_id1);

                        if(object_class1 == 0) {
                            // if it is a support & not in edges_id
                            bool is_in = false;
                            for(auto e_id : this->node_vectors_[0].edges_ids) {
                                if(e_id == object_id1) {
                                    is_in = true;
                                }
                            }

                            if(!is_in) {
                                this->node_vectors_[0].edges_ids.push_back(object_id1);
                            }
                            
                        }

                    }
                    else {
                        index_id1 = this->id_2_index_[object_id1];
                        this->node_vectors_[index_id1].edges_ids.push_back(edge_id);
                    }

                    // insert Object Node : id2
                    std::map<int, unsigned int>::iterator it_id2 = this->id_2_index_.find(object_id2);
                    if(it_id2 == this->id_2_index_.end()) {
                        ObjectNode object_node2;
                        object_node2.object_id = object_id2;
                        object_node2.object_class = object_class2;
                        object_node2.visited = false;
                        object_node2.edges_ids.clear();
                        object_node2.edges_ids.emplace_back(edge_id);
                        this->node_vectors_.emplace_back(object_node2);

                        this->id_2_index_[object_id2] = this->index_2_id_.size();
                        index_id2 = this->index_2_id_.size();

                        this->index_2_id_.push_back(object_id2);

                        if(object_class2 == 0) {
                            // if it is a support
                            bool is_in = false;
                            for(auto e_id : this->node_vectors_[0].edges_ids) {
                                if(e_id == object_id2) {
                                    is_in = true;
                                }
                            }

                            if(!is_in) {
                                this->node_vectors_[0].edges_ids.push_back(object_id2);
                            }
                        }

                    }
                    else {
                        index_id2 = this->id_2_index_[object_id2];
                        this->node_vectors_[index_id2].edges_ids.push_back(edge_id);
                    }

                    // process edge
                    RelationEdge relation_edge;
                    relation_edge.index_id1 = index_id1;
                    relation_edge.index_id2 = index_id2;
                    relation_edge.object_id1 = object_id1;
                    relation_edge.object_id2 = object_id2;
                    relation_edge.feature_id1 = feature_id1;
                    relation_edge.feature_id2 = feature_id2;
                    relation_edge.edge_type = edge_type;
                    relation_edge.edge_confidence = edge_confidence;
                    relation_edge.feature_point1 = feature_point1;
                    relation_edge.feature_point2 = feature_point2;
                    relation_edge.feature_direction1 = feature_direction1;
                    relation_edge.feature_direction2 = feature_direction2;
                    relation_edge.used = false;

                    this->edge_vectors_.emplace_back(relation_edge);
                    
                }
            };

            void TreeGeneration() {
                // starting iteration
                std::stack<unsigned int> current_stack;
                std::stack<unsigned int> next_stack;

                for(auto id : this->node_vectors_[0].edges_ids) {
                    unsigned int support_index = this->id_2_index_[id];
                    current_stack.push(support_index);  // Do we need copy function for struct?
                    index_2_level[support_index] = 0;  // all supported result belongs to level 0
                }

                int current_level = 0;  // current tree level
                while(!current_stack.empty()) {
                    while(!current_stack.empty()) {
                        unsigned int current_base_index = current_stack.top();
                        current_stack.pop();

                        index_2_children[current_base_index].clear();
                        for(auto current_edge : this->node_vectors_[current_base_index].edges_ids) {

                            if(this->edge_vectors_[current_edge].used) {
                                continue;
                            }
                            else {
                                this->edge_vectors_[current_edge].used = true;
                            }

                            int next_index1 = this->edge_vectors_[current_edge].index_id1;
                            int next_index2 = this->edge_vectors_[current_edge].index_id2;
                            assert((next_index1 == current_base_index) || (next_index2 == current_base_index));
                            if(next_index2 == current_base_index) {
                                next_index2 = next_index1;  // the opposite side of the edge
                            }

                            if(!this->node_vectors_[next_index2].visited) {
                                this->node_vectors_[next_index2].visited = true;
                                next_stack.push(next_index2);

                                // then next_index2 is the child of current_base_index
                                index_2_parent[next_index2] = current_base_index;
                                index_2_children[current_base_index].push_back(next_index2);

                                index_2_level[next_index2] = current_level + 1;
                                index_2_parent_edge_weight[next_index2] = this->edge_vectors_[current_edge].edge_confidence;
                                index_2_parent_edge[next_index2] = current_edge;
                            }

                            else {
                                int original_parent_index = index_2_parent[next_index2];
                                assert(original_parent_index != current_base_index);  // two node should only share one edge
                                if(index_2_level[original_parent_index] == current_level) {
                                    if(index_2_parent_edge_weight[next_index2] > this->edge_vectors_[current_edge].edge_confidence) {
                                        // if current parent is in same level & has less noise, change parents
                                        index_2_parent[next_index2] = current_base_index;
                                        index_2_children[current_base_index].push_back(next_index2);
                                        index_2_parent_edge_weight[next_index2] = this->edge_vectors_[current_edge].edge_confidence;
                                        index_2_parent_edge[next_index2] = current_edge;

                                        // remove children in previous parents
                                        std::vector<int> new_children;
                                        for(auto child_index : index_2_children[original_parent_index]) {
                                            if(child_index != next_index2) {
                                                new_children.push_back(child_index);
                                            }
                                        }
                                        index_2_children[original_parent_index].swap(new_children);

                                    }
                                }

                            }

                        } 
                    };
                    current_stack.swap(next_stack);
                    current_level ++;
                }
            };

            // Display related functions
            void IterativeDisplay(int object_index) {
                int object_id = this->index_2_id_[object_index];
                int object_level = this->index_2_level[object_index];
                int parent_index = this->index_2_parent[object_index];
                int parent_id = this->index_2_id_[parent_index];
                int edge_index = this->index_2_parent_edge[object_index];
                double contact_noise = this->edge_vectors_[edge_index].edge_confidence;

                ENUM_CONTACT contat_type = this->edge_vectors_[edge_index].edge_type;
                std::cout << "Object id : " << object_id << " | Level : "  << object_level 
                          << " | Supported by : " << parent_id 
                          << " | With : " << contat_type << " : " << contact_noise 
                          << std::endl;

                std::cout << " ---------------------------- " << std::endl;
                for(auto child_index : this->index_2_children[object_index]) {
                    IterativeDisplay(child_index);
                }
            };

            void DisplayResult() {
                // display related tree result
                for(auto id : this->node_vectors_[0].edges_ids) {
                    unsigned int support_index = this->id_2_index_[id];
                    std::cout << " ========= Supports =========== " << std::endl;
                    std::cout << "Object id : " << id << " | Level : 0 " << std::endl;
                    for(auto child_index : this->index_2_children[support_index]) {
                        this->IterativeDisplay(child_index);
                    }
                    std::cout << " ---------------------------- " << std::endl;
                }
            };

            // output simplified relationship
            void SimplifyRelationship(
                                    std::vector<int>& objects_id1, std::vector<int>& objects_id2,
                                    std::vector<ENUM_CONTACT>& contact_type_vector,
                                    std::vector<Coord>& feature_point_1, std::vector<Coord>& feature_point_2,
                                    std::vector<Coord>& feature_direction_1, std::vector<Coord>& feature_direction_2,
                                    std::vector<int>& feature_id_1, std::vector<int>& feature_id_2,
                                    std::vector<double>& noise_level_vector) {
                // clear previous result
                objects_id1.clear();
                objects_id2.clear();
                contact_type_vector.clear();
                feature_point_1.clear();
                feature_point_2.clear();
                feature_direction_1.clear();
                feature_direction_2.clear();
                feature_id_1.clear();
                feature_id_2.clear();
                noise_level_vector.clear();
                // support multiple-support structure
                for(auto id : this->node_vectors_[0].edges_ids) {
                    unsigned int support_index = this->id_2_index_[id];
                    for(auto child_index : this->index_2_children[support_index]) {
                        IterativeSimplify(child_index, objects_id1, objects_id2,
                                        contact_type_vector,
                                        feature_point_1, feature_point_2,
                                        feature_direction_1, feature_direction_2,
                                        feature_id_1, feature_id_2,
                                        noise_level_vector);
                    }
                }
            };

            void IterativeSimplify(int current_index,
                                    std::vector<int>& objects_id1, std::vector<int>& objects_id2,
                                    std::vector<ENUM_CONTACT>& contact_type_vector,
                                    std::vector<Coord>& feature_point_1, std::vector<Coord>& feature_point_2,
                                    std::vector<Coord>& feature_direction_1, std::vector<Coord>& feature_direction_2,
                                    std::vector<int>& feature_id_1, std::vector<int>& feature_id_2,
                                    std::vector<double>& noise_level_vector) {
                
                // add the parent edge to it
                int edge_index = this->index_2_parent_edge[current_index];
                RelationEdge relation_edge = this->edge_vectors_[edge_index];
                objects_id1.push_back(relation_edge.object_id1);
                objects_id2.push_back(relation_edge.object_id2); 
                contact_type_vector.push_back(relation_edge.edge_type);
                feature_point_1.push_back(relation_edge.feature_point1);
                feature_point_2.push_back(relation_edge.feature_point2);
                feature_direction_1.push_back(relation_edge.feature_direction1);
                feature_direction_2.push_back(relation_edge.feature_direction2);
                feature_id_1.push_back(relation_edge.feature_id1);
                feature_id_2.push_back(relation_edge.feature_id2);
                noise_level_vector.push_back(relation_edge.edge_confidence); 

                for(auto child_index : this->index_2_children[current_index]) {
                    IterativeSimplify(child_index, objects_id1, objects_id2,
                                        contact_type_vector,
                                        feature_point_1, feature_point_2,
                                        feature_direction_1, feature_direction_2,
                                        feature_id_1, feature_id_2,
                                        noise_level_vector);
                }                   
            };

        private:
            std::vector<ObjectNode> node_vectors_;
            std::vector<RelationEdge> edge_vectors_;
            std::vector<int> index_2_id_;  // inner index to object id
            std::map<int, unsigned int> id_2_index_;  // object id to inner index

            // tree related structure
            std::map<int, std::vector<int> > index_2_children;

            std::map<int, int> index_2_level;  // tree level corresponding to id
            std::map<int, int> index_2_parent;   
            std::map<int, int> index_2_parent_edge;  
            std::map<int, double> index_2_parent_edge_weight;

    };

}