#pragma once
#include <math.h>
#include <Eigen/Core>
#include <iostream>

namespace geometry_relation {

	using Transform = Eigen::Matrix<double, 4, 4>;
	using Coord = Eigen::Matrix<double, 4, 1>;

	enum ENUM_GEOMETRY_FEATURE {
		unknown_feature, line, plane, surface
	};

	enum ENUM_CONTACT {
		unknown_contact, line_plane, plane_line, plane_plane, surface_plane, plane_surface
	};

	// small tools in Rages:
	Transform Rogas(Coord direction, double theta) {
		Transform rotation_matrix = Transform::Identity();
		rotation_matrix *= cos(theta);

		rotation_matrix += (1 - cos(theta)) * (direction * direction.transpose());

		Transform d_upper = Transform::Zero();
		d_upper(1, 0) = direction(2);
		d_upper(0, 1) = -direction(2);
		d_upper(2, 0) = -direction(1);
		d_upper(0, 2) = direction(1);
		d_upper(2, 1) = direction(0);
		d_upper(1, 2) = -direction(0);

		rotation_matrix += sin(theta) * d_upper;
		return rotation_matrix;
	};

	class GeometryFeature
	{
	public:
		GeometryFeature() : point_(Coord::Zero()), direction_(Coord::Zero()) {};
		GeometryFeature(Coord point, Coord direction, ENUM_GEOMETRY_FEATURE feature_type, double noise_bound) :
			point_(point), direction_(direction), feature_type_(feature_type), noise_bound_(noise_bound) {};

		GeometryFeature(Coord point, Coord direction, ENUM_GEOMETRY_FEATURE feature_type, double noise_bound, double other_value1) :
			point_(point), direction_(direction), feature_type_(feature_type), noise_bound_(noise_bound), other_value1_(other_value1) {};

		GeometryFeature(const GeometryFeature& obj) {
			this->feature_type_ = obj.feature_type_;
			this->point_ = obj.point_;
			this->direction_ = obj.direction_;
			this->noise_bound_ = obj.noise_bound_;
			this->other_value1_ = obj.other_value1_;

		}

		virtual double Contact(GeometryFeature& geometry_feature, ENUM_CONTACT& contact_type,
								Transform pose_estimation1, Transform pose_estimation2) = 0;  // return Contact error

		virtual void GenerateRenderPoints(std::vector<Coord>& render_points, 
			Transform pose_estimation, int render_points_num, std::vector<double> dimension_scale) = 0;
		ENUM_GEOMETRY_FEATURE feature_type_;
		Coord point_;  // the unique representation for geometry feature
		Coord direction_;
		double noise_bound_;
		double other_value1_; // save value 1: r for surface.

	};

	class LineFeature : public GeometryFeature
	{
	public:
		LineFeature() : GeometryFeature() {};
		LineFeature(Coord point, Coord direction, double noise_bound) : GeometryFeature(point, direction, line, noise_bound) {};
		LineFeature(const LineFeature& obj) : GeometryFeature(obj) {};
		
		virtual double Contact(GeometryFeature& geometry_feature, ENUM_CONTACT& contact_type,
			Transform pose_estimation1, Transform pose_estimation2) override {
			Coord world_point1 = pose_estimation1 * this->point_;
			Coord world_point2 = pose_estimation2 * geometry_feature.point_;
			Coord world_direction1 = pose_estimation1 * this->direction_;
			Coord world_direction2 = pose_estimation2 * geometry_feature.direction_;

			double direction_error, point_error, total_error;
			Coord o1o2;

			switch (geometry_feature.feature_type_)
			{
			case unknown_feature: {
				contact_type = unknown_contact;
				return -1.0;
			}

			case line: {
				// no Contact between line & line
				contact_type = unknown_contact;
				return -1.0;
			}
				
			case plane: {
				direction_error = abs(world_direction1.dot(world_direction2));
				o1o2 = world_point1 - world_point2;
				o1o2 /= o1o2.norm();  // normalization
				point_error = abs(o1o2.dot(world_direction2));
				total_error = direction_error + point_error;
				if (total_error < this->noise_bound_)
					contact_type = line_plane;
				else
					contact_type = unknown_contact;
				return total_error;
			}
				

			case surface: {
				contact_type = unknown_contact;
				return -1.0;
			}
				
			default: {
				std::cout << "No such geometry feature" << std::endl;
				return 1.0;
			}
				
			}
		};

		virtual void GenerateRenderPoints(std::vector<Coord>& render_points, 
			Transform pose_estimation, int render_points_num, std::vector<double> dimension_scale) override {
			Coord world_point = pose_estimation * this->point_;
			Coord world_direction = pose_estimation * this->direction_;

			double line_width = dimension_scale[0];
			double gap = (line_width * 2.0) / (double)render_points_num;
			
			for (int i = 1; i < (render_points_num / 2.0); i++) {
				render_points.push_back(world_point + gap * i * world_direction);
				render_points.push_back(world_point - gap * i * world_direction);
			}

		};
	};


	class PlaneFeature : public GeometryFeature
	{
	public:
		PlaneFeature() : GeometryFeature() {};
		PlaneFeature(Coord point, Coord direction, double noise_bound) : GeometryFeature(point, direction, plane, noise_bound) {};
		PlaneFeature(Coord point, Coord direction, double noise_bound, Coord x_axis, Coord z_axis) : 
			GeometryFeature(point, direction, plane, noise_bound), x_axis_(x_axis), z_axis_(z_axis) {};
		PlaneFeature(const PlaneFeature& obj) : GeometryFeature(obj) {};
		
		virtual double Contact(GeometryFeature& geometry_feature, ENUM_CONTACT& contact_type,
			Transform pose_estimation1, Transform pose_estimation2) override {
			Coord world_point1 = pose_estimation1 * this->point_;
			Coord world_point2 = pose_estimation2 * geometry_feature.point_;
			Coord world_direction1 = pose_estimation1 * this->direction_;
			Coord world_direction2 = pose_estimation2 * geometry_feature.direction_;

			double direction_error, point_error, total_error;
			Coord o1o2;
			Coord moved_point;

			switch (geometry_feature.feature_type_)
			{
			case unknown_feature: {
				contact_type = unknown_contact;
				return -1.0;
			}

			case line: {

				// we want two normals to be orthogonal to each other
				direction_error = abs(world_direction1.dot(world_direction2));
				o1o2 = world_point2 - world_point1;
				o1o2 /= o1o2.norm();  // normalization
				point_error = abs(o1o2.dot(world_direction1));
				total_error = direction_error + point_error;
				if (total_error < this->noise_bound_)
					contact_type = plane_line;
				else
					contact_type = unknown_contact;
				return total_error;
			}

			case plane: {
				// we want two normals to be exact the opposite to each other
				// plane plane support requires the plane to be face to face
				direction_error = 1.0 + world_direction1.dot(world_direction2);

				o1o2 = world_point2 - world_point1;
				o1o2 /= o1o2.norm();  // normalization
				point_error = abs(o1o2.dot(world_direction1));
				total_error = direction_error + point_error;
				if (total_error < this->noise_bound_)
					contact_type = plane_plane;
				else
					contact_type = unknown_contact;
				return total_error;
			}

			case surface: {
				direction_error = abs(world_direction1.dot(world_direction2));

				moved_point = geometry_feature.other_value1_ * world_direction1 + world_point1;
				o1o2 = world_point2 - moved_point;
				o1o2 /= o1o2.norm();  // normalization
				point_error = abs(o1o2.dot(world_direction1));
				total_error = direction_error + point_error;

				if (total_error < this->noise_bound_)
					contact_type = plane_surface;
				else
					contact_type = unknown_contact;
				return total_error;
			}

			default: {
				std::cout << "No such geometry feature" << std::endl;
				return 1.0;
			}

			}
		};

		virtual void GenerateRenderPoints(std::vector<Coord>& render_points,
			Transform pose_estimation, int render_points_num, std::vector<double> dimension_scale) override {
			// this number is the number in one dimension
			double length = dimension_scale[0];
			double width = dimension_scale[1];

			double _r = sqrt(length * width);
			Coord world_point = pose_estimation * this->point_;
			Coord world_direction = pose_estimation * this->direction_;
			Coord world_x_axis = pose_estimation * this->x_axis_;
			Coord world_z_axis = pose_estimation * this->z_axis_;
			double length_gap = length / render_points_num;
			double width_gap = width / render_points_num;
			for (int i = 0; i < render_points_num; i++) {
				for (int j = 0; j < render_points_num; j++) {
					render_points.push_back(world_point + length_gap * i * world_x_axis + width_gap * j * world_z_axis);
					render_points.push_back(world_point - length_gap * i * world_x_axis + width_gap * j * world_z_axis);
					render_points.push_back(world_point + length_gap * i * world_x_axis - width_gap * j * world_z_axis);
					render_points.push_back(world_point - length_gap * i * world_x_axis - width_gap * j * world_z_axis);
				}
			}

		};

		// inner data axis:
		Coord x_axis_;
		Coord z_axis_;
	};


	class SurfaceFeature : public GeometryFeature
	{
	public:
		SurfaceFeature() : GeometryFeature() {};
		SurfaceFeature(Coord point, Coord direction, double noise_bound, double r) : GeometryFeature(point, direction, plane, noise_bound, r) {};
		SurfaceFeature(const SurfaceFeature& obj) : GeometryFeature(obj) {};
		
		virtual double Contact(GeometryFeature& geometry_feature, ENUM_CONTACT& contact_type,
			Transform pose_estimation1, Transform pose_estimation2) override {
			Coord world_point1 = pose_estimation1 * this->point_;
			Coord world_point2 = pose_estimation2 * geometry_feature.point_;
			Coord world_direction1 = pose_estimation1 * this->direction_;
			Coord world_direction2 = pose_estimation2 * geometry_feature.direction_;

			double direction_error, point_error, total_error;
			Coord o1o2;
			Coord moved_point;

			switch (geometry_feature.feature_type_)
			{
			case unknown_feature: {
				contact_type = unknown_contact;
				return -1.0;
			}

			case line: {
				contact_type = unknown_contact;
				return -1.0;
			}

			case plane: {
				direction_error = abs(world_direction1.dot(world_direction2));

				moved_point = this->other_value1_ * world_direction2 + world_point2;
				o1o2 = world_point1 - moved_point;
				o1o2 /= o1o2.norm();  // normalization
				point_error = abs(o1o2.dot(world_direction2));
				total_error = direction_error + point_error;

				if (total_error < this->noise_bound_)
					contact_type = surface_plane;
				else
					contact_type = unknown_contact;
				return total_error;
			}

			case surface: {
				contact_type = unknown_contact;
				return -1.0;
			}

			default: {
				std::cout << "No such geometry feature" << std::endl;
				break;
			}

			}
		};

		virtual void GenerateRenderPoints(std::vector<Coord>& render_points,
			Transform pose_estimation, int render_points_num, std::vector<double> dimension_scale) override {
			Coord world_point = pose_estimation * this->point_;
			Coord world_direction = pose_estimation * this->direction_;
			double height = dimension_scale[0];
			double radius = dimension_scale[1];
			double height_gap = height / render_points_num;
			double theta_gap = M_PI / render_points_num;
			std::vector<Coord> points_in_circle;
			// find an orthogonal vector first
			Coord arbitrary_vector1;
			arbitrary_vector1 << 1.0, 0.0, 0.0, 0.0;
			Coord arbitrary_vector2;
			arbitrary_vector2 << 0.0, 1.0, 0.0, 0.0;

			Coord orthogonal_vector;
			if (arbitrary_vector1 != this->direction_) {
				orthogonal_vector = arbitrary_vector1.cross3(world_direction);
			}
			else {
				orthogonal_vector = arbitrary_vector2.cross3(world_direction);
			}

			Transform rotation_matrix;
			for (int i = 0; i < render_points_num; i++) {
				rotation_matrix = Rogas(world_direction, theta_gap * i);
				points_in_circle.push_back(rotation_matrix * orthogonal_vector + world_point);
			}

			for (int i = 0; i < render_points_num; i++) {
				for (int j = 0; j < render_points_num; j++) {
					render_points.push_back(points_in_circle[j] + height_gap * i * world_direction);
					render_points.push_back(points_in_circle[j] - height_gap * i * world_direction);
				}
			}

		};
	};

};