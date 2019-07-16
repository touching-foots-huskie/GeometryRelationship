#include <math.h>
#include <vector>
#include <unistd.h>
#include <Eigen/Core>
#include <iostream>

#ifndef GEOMETRYFEATURE
#define GEOMETRYFEATURE

namespace geometry_relation {

	using Transform = Eigen::Matrix<double, 4, 4>;
	using Coord = Eigen::Matrix<double, 4, 1>;
    using Coord3 = Eigen::Matrix<double, 3, 1>;
	using Coord_vector4 = std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >;
	using Coord_vector = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;

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
		GeometryFeature(Coord point, Coord direction, ENUM_GEOMETRY_FEATURE feature_type, double rotation_noise_bound,
            double transition_noise_bound) :
			    point_(point), direction_(direction), feature_type_(feature_type), rotation_noise_bound_(rotation_noise_bound),
                    transition_noise_bound_(transition_noise_bound){};

        GeometryFeature(Coord point, Coord direction, Coord direction2, ENUM_GEOMETRY_FEATURE feature_type, double rotation_noise_bound,
            double transition_noise_bound) :
			    point_(point), direction_(direction), direction2_(direction2), feature_type_(feature_type), rotation_noise_bound_(rotation_noise_bound),
                    transition_noise_bound_(transition_noise_bound){};

		GeometryFeature(Coord point, Coord direction, ENUM_GEOMETRY_FEATURE feature_type, double rotation_noise_bound,
            double transition_noise_bound, double other_value1) :
			    point_(point), direction_(direction), feature_type_(feature_type), rotation_noise_bound_(rotation_noise_bound),
                    transition_noise_bound_(transition_noise_bound), other_value1_(other_value1) {};
		// add new Geometry feature ctor for plane feature
		GeometryFeature(Coord point, Coord direction, ENUM_GEOMETRY_FEATURE feature_type, double rotation_noise_bound,
            double transition_noise_bound, Coord_vector4 corner_points, Coord axis1, Coord axis2) :
			    point_(point), direction_(direction), feature_type_(feature_type), rotation_noise_bound_(rotation_noise_bound),
                    transition_noise_bound_(transition_noise_bound), g_corner_points_(corner_points), g_x_axis_(axis1), g_z_axis_(axis2) {};
		GeometryFeature(Coord point, Coord direction, Coord direction2, ENUM_GEOMETRY_FEATURE feature_type, double rotation_noise_bound,
            double transition_noise_bound, Coord_vector4 end_points) : 
				point_(point), direction_(direction), feature_type_(feature_type), rotation_noise_bound_(rotation_noise_bound),
                    transition_noise_bound_(transition_noise_bound), g_end_points_(end_points) {};

		GeometryFeature(const GeometryFeature& obj) {
			this->feature_type_ = obj.feature_type_;
			this->point_ = obj.point_;
			this->direction_ = obj.direction_;
            this->direction2_ = obj.direction2_;
			this->transition_noise_bound_ = obj.transition_noise_bound_;
            this->rotation_noise_bound_ = obj.rotation_noise_bound_;
			this->other_value1_ = obj.other_value1_;

		}

		virtual double Contact(GeometryFeature& geometry_feature, ENUM_CONTACT& contact_type,
								Transform pose_estimation1, Transform pose_estimation2, std::vector<double>& individual_noise) = 0;  // return Contact error

		virtual void GenerateRenderPoints(std::vector<Coord>& render_points, 
			Transform pose_estimation, int render_points_num, std::vector<double> dimension_scale) = 0;
		
		
		bool overlap_in_axis(Eigen::Vector3d& projection_axis, Coord_vector& points_box_1, Coord_vector& points_box_2,
        double& overlap_ratio_1, double& overlap_ratio_2) {

			double max_1, max_2, min_1, min_2;
			for(unsigned int i = 0; i < points_box_1.size(); i++) {
				double project_value = projection_axis.dot(points_box_1[i]);
				if(i == 0) {
					min_1 = project_value;
					max_1 = project_value;
				}
				else {
					if(project_value > max_1) {
						max_1 = project_value;
					}
					else if (project_value < min_1) {
						min_1 = project_value;
					}
				}
			}

			for(unsigned int i = 0; i < points_box_2.size(); i++) {
				double project_value = projection_axis.dot(points_box_2[i]);
				if(i == 0) {
					min_2 = project_value;
					max_2 = project_value;
				}
				else {
					if(project_value > max_2) {
						max_2 = project_value;
					}
					else if (project_value < min_2) {
						min_2 = project_value;
					}
				}
			}

			double len_1 = max_1 - min_1;
			double len_2 = max_2 - min_2;

			if( (min_1 > max_2) || (min_2 > max_1) ) {
				overlap_ratio_1 = 0;  
				overlap_ratio_2 = 0;
				return false;
			}           
			else
			{
				double overlap_value = std::min(max_1, max_2) - std::max(min_1, min_2);
				overlap_ratio_1 *= (overlap_value / len_1);
				overlap_ratio_2 *= (overlap_value / len_2);
				return true;
			}
    	};

		double seperate_axis_overlap(Coord_vector& points_box_1, Coord_vector& points_box_2,
			Coord_vector& axis_box_1, Coord_vector& axis_box_2) {
			
			double overlap_ratio_1 = 1.0;
			double overlap_ratio_2 = 1.0;

			// check for axis in 1
			for(auto axis_1 : axis_box_1) {
				if( !overlap_in_axis(axis_1, points_box_1, points_box_2, overlap_ratio_1, overlap_ratio_2) ) 
					return 0.0;
			}

			// check for axis in 2
			for(auto axis_2 : axis_box_2) {
				if( !overlap_in_axis(axis_2, points_box_1, points_box_2, overlap_ratio_1, overlap_ratio_2) ) 
					return 0.0;
			}

			return std::max(overlap_ratio_1, overlap_ratio_2);
		};

		Coord point_;  // the unique representation for geometry feature
		Coord direction_;
        Coord direction2_;
		

        ENUM_GEOMETRY_FEATURE feature_type_;
		double rotation_noise_bound_;
        double transition_noise_bound_;
		double other_value1_; // save value 1: r for surface.
		// property of plane feature		
		Coord_vector4 g_corner_points_;
		Coord g_x_axis_;
		Coord g_z_axis_;
		// property of line feature
		Coord_vector4 g_end_points_;

	};

	class LineFeature : public GeometryFeature
	{
	public:
		LineFeature() : GeometryFeature() {};
		LineFeature(Coord point, Coord direction, double rotation_noise_bound,
            double transition_noise_bound) : GeometryFeature(point, direction, line, rotation_noise_bound, transition_noise_bound) {};
        LineFeature(Coord point, Coord direction, Coord direction2, double rotation_noise_bound,
            double transition_noise_bound) : GeometryFeature(point, direction, direction2, line, rotation_noise_bound, transition_noise_bound) {};
		LineFeature(Coord point, Coord direction, Coord direction2, double rotation_noise_bound,
			double transition_noise_bound, Coord_vector4 end_points) : GeometryFeature(point, direction, direction2, line, rotation_noise_bound, transition_noise_bound, end_points), end_points_(end_points){};
		LineFeature(const LineFeature& obj) : GeometryFeature(obj){};

		bool check_line2plane_overlap(Transform& pose1, Transform& pose2, GeometryFeature& geometry_feature){
			Coord_vector points1, points2;
			Coord_vector axis1, axis2;
			for (unsigned int i=0; i < end_points_.size(); i++){
				Coord p1 = pose1 * end_points_[i];
				points1.push_back(p1.block(0, 0, 3, 1));
			}
			for (unsigned int i=0; i < geometry_feature.g_corner_points_.size(); i++){
				Coord p2 = pose2 * geometry_feature.g_corner_points_[i];
				points2.push_back(p2.block(0, 0, 3, 1));
			}
			axis1.clear();
			Coord ax1 = pose2 * geometry_feature.g_x_axis_;
			Coord ax2 = pose2 * geometry_feature.g_z_axis_;
			axis2.push_back(ax1.block(0, 0, 3, 1));
			axis2.push_back(ax2.block(0, 0, 3, 1));

			double overlap_ratio = seperate_axis_overlap(points1, points2, axis1, axis2);
			double line2plane_overlap_thresh = 0.2;
			if (overlap_ratio > line2plane_overlap_thresh)
				return true;
			else
				return false;
		};

		

		virtual double Contact(GeometryFeature& geometry_feature, ENUM_CONTACT& contact_type,
			Transform pose_estimation1, Transform pose_estimation2, std::vector<double>& individual_noise) override {
			Coord world_point1 = pose_estimation1 * this->point_;
			Coord world_point2 = pose_estimation2 * geometry_feature.point_;
			Coord world_direction1 = pose_estimation1 * this->direction_;
			Coord world_direction2 = pose_estimation2 * geometry_feature.direction_;
            Coord world_direction_out = pose_estimation1 * this->direction2_;

			double direction_error, point_error, total_error, supporting_relationship;
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

				bool is_line2plane_overlap = check_line2plane_overlap(pose_estimation1, pose_estimation2, geometry_feature);
				direction_error = abs(world_direction1.dot(world_direction2));
				o1o2 = world_point1 - world_point2;

				point_error = abs(o1o2.dot(world_direction2));
				total_error = direction_error + point_error;

                // if supporting each other, supporting relationship should be less than 0
                supporting_relationship = world_direction_out.dot(world_direction2);
                individual_noise.push_back(direction_error);
                individual_noise.push_back(point_error);
				if ((direction_error < this->rotation_noise_bound_) && (point_error < this->transition_noise_bound_)
                        &&(supporting_relationship < 0) && (is_line2plane_overlap))
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
		Coord_vector4 end_points_;

	};


	class PlaneFeature : public GeometryFeature
	{
	public:
		PlaneFeature() : GeometryFeature() {};
		PlaneFeature(Coord point, Coord direction, double rotation_noise_bound,
            double transition_noise_bound) : GeometryFeature(point, direction, plane, rotation_noise_bound, transition_noise_bound) {};
		PlaneFeature(Coord point, Coord direction, double rotation_noise_bound,
            double transition_noise_bound, Coord x_axis, Coord z_axis) : 
			GeometryFeature(point, direction, plane, rotation_noise_bound, transition_noise_bound), x_axis_(x_axis), z_axis_(z_axis){};
		PlaneFeature(Coord point, Coord direction, double rotation_noise_bound, // add a new ctor
            double transition_noise_bound, Coord x_axis, Coord z_axis, Coord_vector4 corner_points) : 
			GeometryFeature(point, direction, plane, rotation_noise_bound, transition_noise_bound, corner_points, x_axis, z_axis), x_axis_(x_axis), z_axis_(z_axis), corner_points_(corner_points) {};
		PlaneFeature(const PlaneFeature& obj) : GeometryFeature(obj), x_axis_(obj.x_axis_), z_axis_(obj.z_axis_) {};

		bool check_plane_overlap(Transform& pose1, Transform& pose2, GeometryFeature& geometry_feature){
			// transfer all the points to world 
			Coord_vector points1, points2;
			Coord_vector axis1, axis2;

			for (unsigned int i=0; i < corner_points_.size(); i ++){
				Coord p1 = pose1 * corner_points_[i];
				Coord p2 = pose2 * geometry_feature.g_corner_points_[i];
				points1.push_back(p1.block(0, 0, 3, 1));
				points2.push_back(p2.block(0, 0, 3, 1));
			}

			Coord ax1 = pose1 * x_axis_;
			Coord ax2 = pose1 * z_axis_;
			Coord ax3 = pose2 * geometry_feature.g_x_axis_;
			Coord ax4 = pose2 * geometry_feature.g_z_axis_;
			axis1.push_back(ax1.block(0, 0, 3, 1));
			axis1.push_back(ax2.block(0, 0, 3, 1));
			axis2.push_back(ax3.block(0, 0, 3, 1));
			axis2.push_back(ax4.block(0, 0, 3, 1));

			double overlap_ratio = seperate_axis_overlap(points1, points2, axis1, axis2);
			double plane_overlap_thresh = 0.2;
			if (overlap_ratio > plane_overlap_thresh)
				return true; //there is enough overlapped area of these two planes they probabily contact each other 
			else
				return false;
		}

		bool check_plane2line_overlap(Transform& pose1, Transform& pose2, GeometryFeature& geometry_feature){
			Coord_vector points1, points2;
			Coord_vector axis1, axis2;
			for (unsigned int i=0; i < corner_points_.size(); i++){
				Coord p1 = pose1 * corner_points_[i];
				points1.push_back(p1.block(0, 0, 3, 1));
			}
			for (unsigned int i=0; i < geometry_feature.g_end_points_.size(); i++){
				Coord p2 = pose2 * geometry_feature.g_end_points_[i];
				points2.push_back(p2.block(0, 0, 3, 1));
			}
			axis2.clear();
			Coord ax1 = pose1 * x_axis_;
			Coord ax2 = pose1 * z_axis_;
			axis1.push_back(ax1.block(0, 0, 3, 1));
			axis1.push_back(ax2.block(0, 0, 3, 1));

			double overlap_ratio = seperate_axis_overlap(points1, points2, axis1, axis2);
			double line_plane_overlap_thresh = 0.2;
			if (overlap_ratio > line_plane_overlap_thresh)
				return true;
			else
				return false;
		};

		virtual double Contact(GeometryFeature& geometry_feature, ENUM_CONTACT& contact_type,
			Transform pose_estimation1, Transform pose_estimation2, std::vector<double>& individual_noise) override {
			Coord world_point1 = pose_estimation1 * this->point_;
			Coord world_point2 = pose_estimation2 * geometry_feature.point_;
			Coord world_direction1 = pose_estimation1 * this->direction_;
			Coord world_direction2 = pose_estimation2 * geometry_feature.direction_;
            Coord world_direction_out;  // in case for line feature
			double direction_error, point_error, total_error, supporting_relationship;
			Coord o1o2;
			Coord moved_point;

			
			switch (geometry_feature.feature_type_)
			{
			case unknown_feature: {
				contact_type = unknown_contact;
				return -1.0;
			}

			case line: {
                bool is_plane2line_overlap = check_plane2line_overlap(pose_estimation1, pose_estimation2, geometry_feature);
                world_direction_out = pose_estimation2 * geometry_feature.direction2_;
                supporting_relationship = world_direction_out.dot(world_direction1);
				// we want two normals to be orthogonal to each other
				direction_error = abs(world_direction1.dot(world_direction2));
				o1o2 = world_point2 - world_point1;

				point_error = abs(o1o2.dot(world_direction1));
				total_error = direction_error + point_error;

                individual_noise.push_back(direction_error);
                individual_noise.push_back(point_error);
				if ((direction_error < this->rotation_noise_bound_) && (point_error < this->transition_noise_bound_)
                        &&(supporting_relationship < 0) && (is_plane2line_overlap))
					contact_type = plane_line;
				else
					contact_type = unknown_contact;
				return total_error;
			}

			case plane: {
				// we want two normals to be exact the opposite to each other
				// plane plane support requires the plane to be face to face
				//TODO: Compute the 2D bounding box overlapping here 
				bool is_plane_overlap = check_plane_overlap(pose_estimation1, pose_estimation2, geometry_feature);
				direction_error = 1.0 + world_direction1.dot(world_direction2);

				o1o2 = world_point2 - world_point1;

				point_error = abs(o1o2.dot(world_direction1));
				total_error = direction_error + point_error;

                individual_noise.push_back(direction_error);
                individual_noise.push_back(point_error);
				if ((direction_error < this->rotation_noise_bound_) && (point_error < this->transition_noise_bound_) && (is_plane_overlap))
					contact_type = plane_plane;
				else
					contact_type = unknown_contact;
				return total_error;
			}

			case surface: {
				direction_error = abs(world_direction1.dot(world_direction2));

				moved_point = geometry_feature.other_value1_ * world_direction1 + world_point1;
				o1o2 = world_point2 - moved_point;

				point_error = abs(o1o2.dot(world_direction1));
				total_error = direction_error + point_error;
                individual_noise.push_back(direction_error);
                individual_noise.push_back(point_error);

				if ((direction_error < this->rotation_noise_bound_) && (point_error < this->transition_noise_bound_))
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

			Coord world_point = pose_estimation * this->point_;
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
		Coord_vector4 corner_points_; 
		
	};


	class SurfaceFeature : public GeometryFeature
	{
	public:
		SurfaceFeature() : GeometryFeature() {};
		SurfaceFeature(Coord point, Coord direction, double rotation_noise_bound,
            double transition_noise_bound, double r) : 
                GeometryFeature(point, direction, surface, rotation_noise_bound, transition_noise_bound, r) {};
		SurfaceFeature(const SurfaceFeature& obj) : GeometryFeature(obj) {};
		
		virtual double Contact(GeometryFeature& geometry_feature, ENUM_CONTACT& contact_type,
			Transform pose_estimation1, Transform pose_estimation2, std::vector<double>& individual_noise) override {
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

				point_error = abs(o1o2.dot(world_direction2));
				total_error = direction_error + point_error;

                individual_noise.push_back(direction_error);
                individual_noise.push_back(point_error);

				if ((direction_error < this->rotation_noise_bound_) && (point_error < this->transition_noise_bound_))
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
            return - 1.0;
        };

		virtual void GenerateRenderPoints(std::vector<Coord>& render_points,
			Transform pose_estimation, int render_points_num, std::vector<double> dimension_scale) override {
			Coord world_point = pose_estimation * this->point_;
			Coord world_direction = pose_estimation * this->direction_;
			double height = dimension_scale[0];
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

#endif
