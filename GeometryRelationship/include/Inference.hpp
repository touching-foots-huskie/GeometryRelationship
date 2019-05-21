#pragma once
#include <map>
#include <vector>
#include <string>
#include "geometry_feature.hpp"
#include "geometry_object.hpp"
#include "csvstream.hpp"
#include "geometry_parse.hpp"

namespace geometry_relation{

	class RelationInference 
	{
	public:
		RelationInference() {};

		void ConstructTempObjects(std::string geometry_config_file_path) {
			csvstream csvin(geometry_config_file_path);
			unsigned int object_label;
			ENUM_GEOMETRY_OBJECT object_type = unknown;
			double length = 0;
			double width = 0;
			double height = 0;
			Transform relative_transform;
			std::string object_name;

			// start parsing
			while (geometry_parse(csvin, object_label, object_name,
				length, width, height, object_type, relative_transform)) {
				switch (object_type) {
				case box: {
					// here inner_transform is inverse of relative transform
					shared_ptr<GeometryObject> _box_ptr = make_shared<Box>(length, width, height, relative_transform.inverse());
					this->template_objects_in_class[object_label] = _box_ptr;  // saved
					break;
				}
					
				case cylinder: {
					shared_ptr<GeometryObject> _cylinder_ptr = make_shared<Cylinder>(height, length / 2.0, relative_transform.inverse());
					this->template_objects_in_class[object_label] = _cylinder_ptr;
					break;
				}
					
				case support: {
					shared_ptr<GeometryObject> _support_ptr = make_shared<Support>(relative_transform.inverse());
					this->template_objects_in_class[object_label] = _support_ptr;
					break;
				}

				default: {
					break;
				}
				}

			}
		};

		void UpdateObjects(std::map<unsigned int, Transform> object_estimations) {
			// update all estimations after estimation
			for (auto it = object_estimations.begin(); it != object_estimations.end(); ++it)
			{
				objects[it->first]->SetPosition(it->second);
			}
		};

		void UpdateObject(unsigned int object_id, Transform object_estimation) {
			objects[object_id]->SetPosition(object_estimation);
		};

		void AddObject(unsigned int object_id, unsigned int object_class, Transform initial_pose) {
			shared_ptr<GeometryObject> object_ptr = this->template_objects_in_class[object_class];
			std::vector<double> dimension_scale;
			Transform transform_inside;
			object_ptr->make_ptr_from_template(dimension_scale, transform_inside);
			switch (object_ptr->geometry_type_) {
			case box: {
				shared_ptr<GeometryObject> new_obj_ptr = make_shared <Box>(dimension_scale[0],
					dimension_scale[1], dimension_scale[2], transform_inside);
				objects[object_id] = new_obj_ptr;
				break;
			}
			case cylinder: {
				shared_ptr<GeometryObject> new_obj_ptr = make_shared <Cylinder>(dimension_scale[0],
					dimension_scale[1], transform_inside);
				objects[object_id] = new_obj_ptr;
				break;
			}
			case support: {
				shared_ptr<GeometryObject> new_obj_ptr = make_shared <Support>(transform_inside);
				objects[object_id] = new_obj_ptr;
				break;
			}
			}
			
		};

		bool Isin(unsigned int object_id) {
			unsigned int id_times = this->objects.count(object_id);
			return !(id_times == 0);  // no means not in

		};

		void InferenceRelationship(std::vector<unsigned int>& objects_id1, std::vector<unsigned int>& objects_id2,
								std::vector<ENUM_CONTACT>& contact_type_vector,
								std::vector<Coord>& feature_point_1, std::vector<Coord>& feature_point_2,
								std::vector<Coord>& feature_direction_1, std::vector<Coord>& feature_direction_2,
								std::vector<int>& feature_id_1, std::vector<int>& feature_id_2,
								std::vector<double>& noise_level_vector) {
			// TODO: it++ or ++it
			bool contact_existence = false;
			int contact_this_pair = 0;
			for (auto it = objects.begin(); it != objects.end(); ++it) {
				for (auto it2 = next(it); it2 != objects.end(); ++it2) {
					// control a topology order
					contact_this_pair = 0;
					contact_existence = it->second->Object_Contact(*it2->second, contact_type_vector,
						feature_point_1, feature_point_2,
						feature_direction_1, feature_direction_2,
						feature_id_1, feature_id_2,
						noise_level_vector, contact_this_pair);
					for (int _i = 0; _i < contact_this_pair; _i++) {
						objects_id1.push_back(it->first);
						objects_id2.push_back(it2->first);
					}
#ifdef _TEST_OUTPUT_
					if (contact_existence) {
						std::cout << "Object ids are " << it->first << " : " << it2->first << std::endl;
#endif // _TEST_OUTPUT_ 
					}
				}
			}
		}
		std::map<unsigned int, shared_ptr<GeometryObject> > objects;  // id : objects
		std::map<unsigned int, shared_ptr<GeometryObject> > template_objects_in_class;
	};
}