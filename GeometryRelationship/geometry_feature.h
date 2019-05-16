#pragma once
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


	class GeometryFeature
	{
	public:
		GeometryFeature() {};
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

		virtual double Contact(GeometryFeature& geometry_feature, ENUM_CONTACT& contact_type) = 0;  // return Contact error

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
		
		virtual double Contact(GeometryFeature& geometry_feature, ENUM_CONTACT& contact_type) override {
			switch (geometry_feature.feature_type_)
			{
			case unknown_feature:
				contact_type = unknown_contact;
				return -1.0;

			case line:
				// no Contact between line & line
				contact_type = unknown_contact;
				return -1.0;

			case plane:
				double direction_error = abs(this->direction_.dot(geometry_feature.direction_));
				Coord o1o2 = this->point_ - geometry_feature.point_;
				o1o2 /= o1o2.norm();  // normalization
				double point_error = abs(o1o2.dot(geometry_feature.direction_));
				double total_error = direction_error + point_error;
				if (total_error < this->noise_bound_)
					contact_type = line_plane;
				else
					contact_type = unknown_contact;
				return total_error;

			case surface:
				contact_type = unknown_contact;
				return -1.0;

			default:
				std::cout << "No such geometry feature" << std::endl;
				return 1.0;
			}
		};
	};


	class PlaneFeature : public GeometryFeature
	{
	public:
		PlaneFeature() : GeometryFeature() {};
		PlaneFeature(Coord point, Coord direction, double noise_bound) : GeometryFeature(point, direction, plane, noise_bound) {};
		PlaneFeature(const PlaneFeature& obj) : GeometryFeature(obj) {};
		
		virtual double Contact(GeometryFeature& geometry_feature, ENUM_CONTACT& contact_type) override{
			switch (geometry_feature.feature_type_)
			{
			case unknown_feature:
				contact_type = unknown_contact;
				return -1.0;

			case line:
				// we want two normals to be orthogonal to each other
				double direction_error = abs(this->direction_.dot(geometry_feature.direction_));
				Coord o1o2 = geometry_feature.point_ - this->point_;
				o1o2 /= o1o2.norm();  // normalization
				double point_error = abs(o1o2.dot(this->direction_));
				double total_error = direction_error + point_error;
				if (total_error < this->noise_bound_)
					contact_type = plane_line;
				else
					contact_type = unknown_contact;
				return total_error;

			case plane:
				// we want two normals to be exact the opposite to each other
				// plane plane support requires the plane to be face to face
				double direction_error = 1.0 + this->direction_.dot(geometry_feature.direction_);

				Coord o1o2 = geometry_feature.point_ - this->point_;
				o1o2 /= o1o2.norm();  // normalization
				double point_error = abs(o1o2.dot(this->direction_));
				double total_error = direction_error + point_error;
				if (total_error < this->noise_bound_)
					contact_type = plane_plane;
				else
					contact_type = unknown_contact;
				return total_error;

			case surface:
				double direction_error = abs(this->direction_.dot(geometry_feature.direction_));

				Coord moved_point = geometry_feature.other_value1_ * this->direction_ + this->point_;
				Coord o1o2 = geometry_feature.point_ - moved_point;
				o1o2 /= o1o2.norm();  // normalization
				double point_error = abs(o1o2.dot(this->direction_));
				double total_error = direction_error + point_error;

				if (total_error < this->noise_bound_)
					contact_type = plane_surface;
				else
					contact_type = unknown_contact;
				return total_error;

			default:
				std::cout << "No such geometry feature" << std::endl;
				return 1.0;
			}
		}
	};


	class SurfaceFeature : public GeometryFeature
	{
	public:
		SurfaceFeature() : GeometryFeature() {};
		SurfaceFeature(Coord point, Coord direction, double noise_bound, double r) : GeometryFeature(point, direction, plane, noise_bound, r) {};
		SurfaceFeature(const SurfaceFeature& obj) : GeometryFeature(obj) {};
		
		virtual double Contact(GeometryFeature& geometry_feature, ENUM_CONTACT& contact_type) override {
			switch (geometry_feature.feature_type_)
			{
			case unknown_feature:
				contact_type = unknown_contact;
				return -1.0;

			case line:
				contact_type = unknown_contact;
				return -1.0;

			case plane:
				double direction_error = abs(this->direction_.dot(geometry_feature.direction_));

				Coord moved_point = this->other_value1_ * geometry_feature.direction_ + geometry_feature.point_;
				Coord o1o2 = this->point_ - moved_point;
				o1o2 /= o1o2.norm();  // normalization
				double point_error = abs(o1o2.dot(geometry_feature.direction_));
				double total_error = direction_error + point_error;

				if (total_error < this->noise_bound_)
					contact_type = surface_plane;
				else
					contact_type = unknown_contact;
				return total_error;

			case surface:
				contact_type = unknown_contact;
				return -1.0;

			default:
				std::cout << "No such geometry feature" << std::endl;
				break;
			}
		}
	};

};