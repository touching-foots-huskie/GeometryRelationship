// GeometryRelationship.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#define _DISABLE_EXTENDED_ALIGNED_STORAGE
#define EIGEN_DONT_ALIGN_STATICALLY
#define _TEST_OUTPUT_ 


#include <iostream>
#include <string>
#include <Eigen/Eigen>
#include "csvstream.hpp"
#include "Inference.hpp"
#include "geometry_feature.hpp"
#include "measure_parse.hpp"
#include "pointcloud_log.hpp"


int main()
{
	// test geometry template constrcution
	geometry_relation::RelationInference relation_inferencer;  // default construction
	std::string geometry_config_file = "/home/harvey/CppProjects/GeometryRelationship/GeometryRelationship/data/geometry_config.csv";
	relation_inferencer.ConstructTempObjects(geometry_config_file);

	// test geometry inference structure
	std::string measure_file = "/home/harvey/CppProjects/GeometryRelationship/GeometryRelationship/data/0515_plane2plane.csv";
	csvstream csvin(measure_file);

	// store data
	int frame, label, id = 0;
	Eigen::Vector4d bbox = Eigen::Vector4d::Zero();
	Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
	string image_path;
	double detection_score = 0.0;
	double raw_pose_score = 0.0;
	double render_score = 0.0;

	Eigen::Matrix4d robot_pose = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d landmark_pose = Eigen::Matrix4d::Identity();

	while (measure_parse(csvin, frame, label, id, bbox, transform, image_path,
		detection_score, raw_pose_score, render_score)) {
		// Add measurements:
        if (frame > 10) {
            break;
        }
		if (label == -1) {
			robot_pose = transform;
		}
		else {
			landmark_pose = robot_pose * transform;
			// add or update
			if (relation_inferencer.Isin(id)) {
				relation_inferencer.UpdateObject(id, landmark_pose);
			}
			else {
				relation_inferencer.AddObject(id, label, landmark_pose);
			}
		}

	}

	std::vector<unsigned int> objects_id1;
	std::vector<unsigned int> objects_id2;

	std::vector<geometry_relation::ENUM_CONTACT> contact_type_vector;
	std::vector<geometry_relation::Coord> feature_point_1;
	std::vector<geometry_relation::Coord> feature_point_2;
	std::vector<geometry_relation::Coord> feature_direction_1;
	std::vector<geometry_relation::Coord> feature_direction_2;
	std::vector<double> noise_level_vector;

	// Add feature id
	std::vector<int> feature_id_1;
	std::vector<int> feature_id_2;

	// Make Inference:
	relation_inferencer.InferenceRelationship(objects_id1, objects_id2,
		contact_type_vector, feature_point_1, feature_point_2,
		feature_direction_1, feature_direction_2, 
		feature_id_1, feature_id_2,
		noise_level_vector);

	// log contacts into points
	std::vector<geometry_relation::Coord> logged_points;
	for (int i = 0; i < feature_id_1.size(); i++) {
		logged_points.clear();
		relation_inferencer.objects[objects_id1[i]]->GenerateFeaturePoints(logged_points, feature_id_1[i]);
		relation_inferencer.objects[objects_id2[i]]->GenerateFeaturePoints(logged_points, feature_id_2[i]);
		std::cout << "pause" << std::endl;
		// I can log them & show them here.
        vis_points_in_pcl(logged_points, "/home/harvey/CppProjects/GeometryRelationship/GeometryRelationship/data/Contact_" + std::to_string(i));
	}
	// Generate a set of points.
	return 0;
}
