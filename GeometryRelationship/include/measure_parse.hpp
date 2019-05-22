#include "csvstream.hpp"
#include <iostream>
#include <string>
#include <map>
#include <Eigen/Eigen>

using namespace std;

#ifndef DATAPARSE
#define DATAPARSE

bool measure_parse(csvstream& csvin, int& frame, int& label, int& id,
	Eigen::Vector4d& bbox, Eigen::Matrix4d& transform, string& image_path,
	double detection_score, double raw_pose_score, double render_score)
{
	float value;
	map<string, string> row;
	if (csvin >> row)
	{
		frame = stoi(row["frame"]);
		label = stoi(row["label"]);
		id = stoi(row["id"]);
		image_path = row["image_path"];

		//

		stringstream ssin(row["transform"]);
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				ssin >> value;
				transform(i, j) = value;
			}
		}

		// score & bbox:
		if ((label != -1) && (label != 0)) {
			stringstream sd1in(row["detection_score"]);
			stringstream sd2in(row["raw_pose_score"]);
			stringstream sd3in(row["render_score"]);

			sd1in >> detection_score;
			sd2in >> raw_pose_score;
			sd3in >> render_score;

			// log bbox
			stringstream sbbin(row["render_score"]);
			for (int i = 0; i < 4; i++) {
				sbbin >> bbox[i];
			}
		}
		else {
			detection_score = 0;
			raw_pose_score = 0;
			render_score = 0;
			bbox = Eigen::Vector4d::Zero();
		}
	}
	else
	{
		return false;
	}
	return true;
};

#endif