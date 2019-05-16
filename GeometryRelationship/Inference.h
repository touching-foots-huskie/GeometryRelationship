#pragma once
#include <map>
#include <vector>
#include "geometry_feature.h"
#include "geometry_object.h"

namespace geometry_relation{

	class RelationInference 
	{
	public:
		RelationInference() {};

		void UpdateObjects(std::map<unsigned int, Transform> object_estimations) {
			// update all estimations after estimation
			for (auto it = object_estimations.begin(); it != object_estimations.end(); ++it)
			{
				objects[it->first].SetPosition(it->second);
			}
		};

		void InferenceRelationship(std::vector<unsigned int>& objects_id1, std::vector<unsigned int>& objects_id2,
								std::vector<ENUM_CONTACT>& contact_type_vector,
								std::vector<Coord>& feature_point_1, std::vector<Coord>& feature_point_2,
								std::vector<Coord>& feature_direction_1, std::vector<Coord>& feature_direction_2,
								std::vector<double>& noise_level_vector) {
			// TODO: it++ or ++it
			for (auto it = objects.begin(); it != objects.end(); ++it) {
				for (auto it2 = it; it2 != objects.end(); ++it2) {
					// control a topology order
					it->second.Object_Contact(it2->second, contact_type_vector,
						feature_point_1, feature_point_2,
						feature_direction_1, feature_direction_2,
						noise_level_vector);
				}
			}
		}


		std::map<unsigned int, GeometryObject> objects;  // id : objects

	};
}