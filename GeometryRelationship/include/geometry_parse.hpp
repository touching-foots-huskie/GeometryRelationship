#include "csvstream.hpp"
#include <iostream>
#include <string>
#include <map>
#include <unistd.h>
#include <Eigen/Core>
#include "geometry_object.hpp"

using namespace std;

#ifndef GEOMETRYPARSE
#define GEOMETRYPARSE

bool geometry_parse(csvstream& csvin, unsigned int& object_label, string& object_name, 
			double& length, double& width, double& height,
			geometry_relation::ENUM_GEOMETRY_OBJECT& object_type,
			Eigen::Matrix4d& transform)
{
	// parse geometry file from configuration data 
	double value;
	map<string, string> row;
	if (csvin >> row)
	{
		object_label = stoi(row["object_label"]);
		object_name = row["object_name"];

		stringstream dsin_a(row["length"]); 
		stringstream dsin_b(row["width"]);
		stringstream dsin_c(row["height"]);

		dsin_a >> length;
		dsin_b >> width;
		dsin_c >> height;

		switch (stoi(row["type"]))
		{
		case 0:
			object_type = geometry_relation::support;
			break;
		case 1:
			object_type = geometry_relation::box; 
			break;
		case 2:
			object_type = geometry_relation::cylinder;
			break;
		default:
			object_type = geometry_relation::unknown;
			break;
		}

		stringstream ssin(row["relative_transformation"]);
		if (row["relative_transformation"].size() != 0) {
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					ssin >> value;
					transform(i, j) = value;
				}
			}
		}
		else {
			transform = Eigen::Matrix4d::Identity();
		}
	}
	else
	{
		return false;
	}
	return true;
};

#endif
