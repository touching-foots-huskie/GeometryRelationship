#include <map>
#include <vector>
#include <memory>
#include <unistd.h>
#include <Eigen/Core>
#include <iostream>
#include "geometry_feature.hpp"

// pcl related:
#include <pcl/visualization/pcl_visualizer.h>

#ifndef GEOMETRYOBJECT
#define GEOMETRYOBJECT

namespace geometry_relation {
	enum ENUM_GEOMETRY_OBJECT {
		box, cylinder, support, unknown
	};

	class GeometryObject
	{
	public:
		GeometryObject(ENUM_GEOMETRY_OBJECT geometry_type, Transform object_position) :
			geometry_type_(geometry_type), object_position_(object_position) {
			this->Inner_transfrom_ = Transform::Identity();
		};

		GeometryObject(ENUM_GEOMETRY_OBJECT geometry_type, Transform object_position, Transform Inner_transform) :
			geometry_type_(geometry_type), object_position_(object_position), Inner_transfrom_(Inner_transform) {};

		GeometryObject(const GeometryObject& obj) {
			// can't copy ptr
			this->geometry_type_ = obj.geometry_type_;
			this->object_position_ = obj.object_position_;
			this->Inner_transfrom_ = obj.Inner_transfrom_;
			this->feature_num = obj.feature_num;

		};

		void SetPosition(Transform objection_position) {
			this->object_position_ = objection_position;
		}
		void get_data_from_plane(Coord p1, Coord p2, Coord p3, Coord p4, Coord_vector4& points){
			points.clear();
			/* 
			points.push_back(p1.block(0, 0, 3, 1));
			points.push_back(p2.block(0, 0, 3, 1));
			points.push_back(p3.block(0, 0, 3, 1));
			points.push_back(p4.block(0, 0, 3, 1));
			*/
			points.push_back(p1);
			points.push_back(p2);
			points.push_back(p3);
			points.push_back(p4);
		};

		void get_data_from_line(Coord p1, Coord p2, Coord_vector4& points){
			points.clear();
			points.push_back(p1);
			points.push_back(p2);
		};

		// Abstract contact setting : There are still a lot to do on how to avoid redaudent constraints
		bool Object_Contact(GeometryObject& geometry_object, std::vector<ENUM_CONTACT>& contact_type_vector,
							std::vector<Coord>& feature_point_1, std::vector<Coord>& feature_point_2,
							std::vector<Coord>& feature_direction_1, std::vector<Coord>& feature_direction_2,
							std::vector<int>& feature_id_1, std::vector<int>& feature_id_2,
							std::vector<double>& noise_level_vector, int& contact_this_pair){

			bool contact_exist = false;
			std::map<int, bool> other_constraints_made;

			ENUM_CONTACT _temp_contact;
			double _temp_noise_level;
            
            std::vector<double> _temp_individual_noise;

			// if (geometry_type_ == cylinder || geometry_object.geometry_type_ == cylinder){
			// 	return false;
			// }
			for (int i = 0; i < this->feature_num; i++) {
				for (int j = 0; j < geometry_object.feature_num; j++) {
					
					_temp_noise_level = this->feature_array_[i]->Contact(
						*geometry_object.feature_array_[j], _temp_contact,
						this->object_position_, geometry_object.object_position_,
                        _temp_individual_noise);
				
					if (other_constraints_made[j])
						continue;  // no same feature for different constraints.
					
					// if feature contact, it will return a contact value
					if (_temp_contact != unknown_contact) {
						// log the local feature vector
						contact_this_pair++;
						contact_type_vector.push_back(_temp_contact);
						// log feature vectors
						feature_point_1.push_back(this->feature_array_[i]->point_);
						feature_point_2.push_back(geometry_object.feature_array_[j]->point_);

						feature_direction_1.push_back(this->feature_array_[i]->direction_);
						feature_direction_2.push_back(geometry_object.feature_array_[j]->direction_);

						// log feature id
						feature_id_1.push_back(i);
						feature_id_2.push_back(j);

						noise_level_vector.push_back(_temp_noise_level);

						other_constraints_made[j] = true;
						contact_exist = true;

#ifdef _TEST_OUTPUT_ 
						std::vector<std::string> contact_type_string = {
							"unknown", "line_plane", "plane_line", "plane_plane", "surface_plane", "plane_surface"
						};

						std::vector<std::string> geometry_type_string = {
							"box", "cylinder", "support", "unknown"
						};
						std::cout << "========= Contact Found ========" << std::endl;
						std::cout << "Contact type: " << contact_type_string[_temp_contact] << std::endl;
						std::cout << "Between " << geometry_type_string[this->geometry_type_] << " : " << geometry_type_string[geometry_object.geometry_type_] << std::endl;
						std::cout << "Noise level " << _temp_individual_noise[0] << " , " << _temp_individual_noise[1] << std::endl;

#endif // _TEST_OUTPUT_ 
						
						//  break;  // if plane-plane, then there no plane-edge.
                        return contact_exist;  //  there is at most only one relation between two objects
						
					}
                    _temp_individual_noise.clear();
				}
			}
			return contact_exist;
			// filter redundant relations:

		};

		virtual void make_ptr_from_template(std::vector<double>& dimension_scale, Transform& transform_inside) = 0;
		virtual void GenerateFeaturePoints(std::vector<Coord>& render_points, int feature_id) = 0;

        virtual void GenerateCoefficients(pcl::ModelCoefficients& coefficients, Transform& camera_pose) = 0;
		~GeometryObject() {
			delete[] feature_array_;
		};
		//

		ENUM_GEOMETRY_OBJECT geometry_type_;
		Transform object_position_;
		Transform Inner_transfrom_;  // relative transfrom to standard coord

		GeometryFeature** feature_array_;
		int feature_num;
	};

	class Box : public GeometryObject
	{

    // box is special : including 3d bounding box
	public:
		Box(double a, double b, double c, Transform inner_transform) : 
			GeometryObject(box, Transform::Identity(), inner_transform), a_(a), b_(b), c_(c)
		{
			this->feature_num = 18;
			this->feature_array_ = new GeometryFeature * [18];
			this->edges = new LineFeature[12];
			this->planes = new PlaneFeature[6];
			
			// 6 planes:
			Coord p1;
			p1 << a / 2.0, 0.0, 0.0, 1.0;
			
			Coord pc1, pc2, pc3, pc4; //corner point of this plane
			pc1 <<a / 2.0, b / 2.0, -c / 2.0, 1.0; 
			pc2 <<a / 2.0, b / 2.0, c / 2.0, 1.0; 
			pc3 <<a / 2.0, -b / 2.0, c / 2.0, 1.0; 
			pc4 <<a / 2.0, -b / 2.0, -c / 2.0, 1.0; 
			pc1 = this->Inner_transfrom_ * pc1;
			pc2 = this->Inner_transfrom_ * pc2;
			pc3 = this->Inner_transfrom_ * pc3;
			pc4 = this->Inner_transfrom_ * pc4;

			Coord_vector4 points1;
			get_data_from_plane(pc1, pc2, pc3, pc4, points1);

			Coord d1;//plane normal direction 
			d1 << 1.0, 0.0, 0.0, 0.0;
			Coord dx1;
			dx1 << 0.0, 1.0, 0.0, 0.0;
			Coord dz1;
			dz1 = d1.cross3(dx1);
			// Transform into estimation coords
			p1 = this->Inner_transfrom_ * p1;
			d1 = this->Inner_transfrom_ * d1;
			dx1 =this->Inner_transfrom_ * dx1;
			dz1 =this->Inner_transfrom_ * dz1;
			this->planes[0] = PlaneFeature(p1, d1, plane_r_noise, plane_t_noise, dx1, dz1, points1);// use dx1 and dz1 for overlapping check

			Coord p2;
			p2 << -a / 2.0, 0.0, 0.0, 1.0;

			pc1 <<-a / 2.0, b / 2.0, -c / 2.0, 1.0; 
			pc2 <<-a / 2.0, b / 2.0, c / 2.0, 1.0; 
			pc3 <<-a / 2.0, -b / 2.0, c / 2.0, 1.0; 
			pc4 <<-a / 2.0, -b / 2.0, -c / 2.0, 1.0; 

			pc1 = this->Inner_transfrom_ * pc1;
			pc2 = this->Inner_transfrom_ * pc2;
			pc3 = this->Inner_transfrom_ * pc3;
			pc4 = this->Inner_transfrom_ * pc4;

			Coord_vector4 points2;
			get_data_from_plane(pc1, pc2, pc3, pc4, points2);

			Coord d2;
			d2 << -1.0, 0.0, 0.0, 0.0;

			Coord dx2;
			dx2 << 0.0, 1.0, 0.0, 0.0;
			Coord dz2;
			dz2 = d2.cross3(dx2);
			
			p2 = this->Inner_transfrom_ * p2;
			d2 = this->Inner_transfrom_ * d2;
			dx2 = this->Inner_transfrom_ * dx2;
			dz2 = this->Inner_transfrom_ * dz2;
			this->planes[1] = PlaneFeature(p2, d2, plane_r_noise, plane_t_noise, dx2, dz2, points2);

			Coord p3;
			p3 << 0.0, b / 2.0, 0.0, 1.0;

			pc1 <<a / 2.0, b / 2.0, c / 2.0, 1.0; 
			pc2 <<a / 2.0, b / 2.0, -c / 2.0, 1.0; 
			pc3 <<-a / 2.0, b / 2.0, -c / 2.0, 1.0; 
			pc4 <<-a / 2.0, b / 2.0, c / 2.0, 1.0; 
			pc1 = this->Inner_transfrom_ * pc1;
			pc2 = this->Inner_transfrom_ * pc2;
			pc3 = this->Inner_transfrom_ * pc3;
			pc4 = this->Inner_transfrom_ * pc4;

			Coord_vector4 points3;
			get_data_from_plane(pc1, pc2, pc3, pc4, points3);

			Coord d3;
			d3 << 0.0, 1.0, 0.0, 0.0;

			Coord dx3;
			dx3 << 1.0, 0.0, 0.0, 0.0;
			Coord dz3;
			dz3 = d3.cross3(dx3);

			p3 = this->Inner_transfrom_ * p3;
			d3 = this->Inner_transfrom_ * d3;
			dx3 = this->Inner_transfrom_ * dx3;
			dz3 = this->Inner_transfrom_ * dz3;
			this->planes[2] = PlaneFeature(p3, d3, plane_r_noise, plane_t_noise, dx3, dz3, points3);

			Coord p4;
			p4 << 0.0, -b / 2.0, 0.0, 1.0;


			pc1 <<a / 2.0, -b / 2.0, c / 2.0, 1.0; 
			pc2 <<a / 2.0, -b / 2.0, -c / 2.0, 1.0; 
			pc3 <<-a / 2.0, -b / 2.0, -c / 2.0, 1.0; 
			pc4 <<-a / 2.0, -b / 2.0, c / 2.0, 1.0; 
			pc1 = this->Inner_transfrom_ * pc1;
			pc2 = this->Inner_transfrom_ * pc2;
			pc3 = this->Inner_transfrom_ * pc3;
			pc4 = this->Inner_transfrom_ * pc4;
			Coord_vector4 points4;
			get_data_from_plane(pc1, pc2, pc3, pc4, points4);

			Coord d4;
			d4 << 0.0, -1.0, 0.0, 0.0;

			Coord dx4;
			dx4 << 1.0, 0.0, 0.0, 0.0;
			Coord dz4;
			dz4 = d4.cross3(dx4);

			p4 = this->Inner_transfrom_ * p4;
			d4 = this->Inner_transfrom_ * d4;
			dx4 = this->Inner_transfrom_ * dx4;
			dz4 = this->Inner_transfrom_ * dz4;
			this->planes[3] = PlaneFeature(p4, d4, plane_r_noise, plane_t_noise, dx4, dz4, points4);

			Coord p5;
			p5 << 0.0, 0.0, c / 2.0, 1.0;

			pc1 <<a / 2.0, b / 2.0, c / 2.0, 1.0; 
			pc2 <<a / 2.0, -b / 2.0, c / 2.0, 1.0; 
			pc3 <<-a / 2.0, -b / 2.0, c / 2.0, 1.0; 
			pc4 <<-a / 2.0, b / 2.0, c / 2.0, 1.0; 
			pc1 = this->Inner_transfrom_ * pc1;
			pc2 = this->Inner_transfrom_ * pc2;
			pc3 = this->Inner_transfrom_ * pc3;
			pc4 = this->Inner_transfrom_ * pc4;
			Coord_vector4 points5;
			get_data_from_plane(pc1, pc2, pc3, pc4, points5);
	
			Coord d5;
			d5 << 0.0, 0.0, 1.0, 0.0;

			Coord dx5;
			dx5 << 1.0, 0.0, 0.0, 0.0;
			Coord dz5;
			dz5 = d5.cross3(dx5);

			p5 = this->Inner_transfrom_ * p5;
			d5 = this->Inner_transfrom_ * d5;
			dx5 = this->Inner_transfrom_ * dx5;
			dz5 = this->Inner_transfrom_ * dz5;

			this->planes[4] = PlaneFeature(p5, d5, plane_r_noise, plane_t_noise, dx5, dz5, points5);

			Coord p6;
			p6 << 0.0, 0.0, -c / 2.0, 1.0;

			pc1 <<a / 2.0, b / 2.0, -c / 2.0, 1.0; 
			pc2 <<a / 2.0, -b / 2.0, -c / 2.0, 1.0; 
			pc3 <<-a / 2.0, -b / 2.0, -c / 2.0, 1.0; 
			pc4 <<-a / 2.0, b / 2.0, -c / 2.0, 1.0; 
			pc1 = this->Inner_transfrom_ * pc1;
			pc2 = this->Inner_transfrom_ * pc2;
			pc3 = this->Inner_transfrom_ * pc3;
			pc4 = this->Inner_transfrom_ * pc4;

			Coord_vector4 points6;
			get_data_from_plane(pc1, pc2, pc3, pc4, points6);

			Coord d6;
			d6 << 0.0, 0.0, -1.0, 0.0;

			Coord dx6;
			dx6 << 1.0, 0.0, 0.0, 0.0;
			Coord dz6;
			dz6 = d6.cross3(dx6);

			p6 = this->Inner_transfrom_ * p6;
			d6 = this->Inner_transfrom_ * d6;
			dx6 = this->Inner_transfrom_ * dx6;
			dz6 = this->Inner_transfrom_ * dz6;
			this->planes[5] = PlaneFeature(p6, d6, plane_r_noise, plane_t_noise, dx6, dz6, points6);

			// 12 lines:
			Coord ep1;
			ep1 << 0.0, b / 2.0, c / 2.0, 1.0;
			// two end points 
			Coord ep1_1, ep1_2; 
			ep1_1 << a / 2.0, b / 2.0, c / 2.0, 1.0;
			ep1_2 << -a / 2.0, b / 2.0, c / 2.0, 1.0;

			Coord ed1;
			ed1 << 1.0, 0.0, 0.0, 0.0;

            Coord ed2_1;
			ed2_1 << 0.0, b / 2.0, c / 2.0, 0.0;

			ep1 = this->Inner_transfrom_ * ep1;
			ep1_1 = this->Inner_transfrom_ * ep1_1;
			ep1_2 = this->Inner_transfrom_ * ep1_2;

			ed1 = this->Inner_transfrom_ * ed1;
            ed2_1 = this->Inner_transfrom_ * ed2_1;
			Coord_vector4 end_points1; 
			get_data_from_line(ep1_1, ep1_2, end_points1);
			this->edges[0] = LineFeature(ep1, ed1, ed2_1, edge_r_noise, edge_t_noise, end_points1);

			Coord ep2;
			ep2 << 0.0, b / 2.0, -c / 2.0, 1.0;

			Coord ep2_1, ep2_2; 
			ep2_1 << a / 2.0, b / 2.0, -c / 2.0, 1.0;
			ep2_2 << -a / 2.0, b / 2.0, -c / 2.0, 1.0;
			Coord_vector4 end_points2;
			get_data_from_line(ep2_1,ep2_2, end_points2);

			Coord ed2;
			ed2 << 1.0, 0.0, 0.0, 0.0;

            Coord ed2_2;
			ed2_2 << 0.0, b / 2.0, -c / 2.0, 0.0;

			ep2 = this->Inner_transfrom_ * ep2;
			ed2 = this->Inner_transfrom_ * ed2;
            ed2_2 = this->Inner_transfrom_ * ed2_2;
			this->edges[1] = LineFeature(ep2, ed2, ed2_2, edge_r_noise, edge_t_noise, end_points2);

			Coord ep3;
			ep3 << 0.0, -b / 2.0, c / 2.0, 1.0;

			Coord ep3_1, ep3_2; 
			ep3_1 << a / 2.0, -b / 2.0, c / 2.0, 1.0;
			ep3_2 << -a / 2.0, -b / 2.0, c / 2.0, 1.0;
			Coord_vector4 end_points3;
			get_data_from_line(ep3_1,ep3_2, end_points3);

			Coord ed3;
			ed3 << 1.0, 0.0, 0.0, 0.0;

            Coord ed2_3;
			ed2_3 << 0.0, -b / 2.0, c / 2.0, 0.0;

			ep3 = this->Inner_transfrom_ * ep3;
			ed3 = this->Inner_transfrom_ * ed3;
            ed2_3 = this->Inner_transfrom_ * ed2_3;
			this->edges[2] = LineFeature(ep3, ed3, ed2_3, edge_r_noise, edge_t_noise, end_points3);

			Coord ep4;
			ep4 << 0.0, -b / 2.0, -c / 2.0, 1.0;

			Coord ep4_1, ep4_2; 
			ep4_1 << a / 2.0, -b / 2.0, -c / 2.0, 1.0;
			ep4_2 << -a / 2.0, -b / 2.0, -c / 2.0, 1.0;
			Coord_vector4 end_points4;
			get_data_from_line(ep4_1,ep4_2, end_points4);
			
			Coord ed4;
			ed4 << 1.0, 0.0, 0.0, 0.0;

            Coord ed2_4;
			ed2_4 << 0.0, -b / 2.0, -c / 2.0, 0.0;

			ep4 = this->Inner_transfrom_ * ep4;
			ed4 = this->Inner_transfrom_ * ed4;
            ed2_4 = this->Inner_transfrom_ * ed2_4;
			this->edges[3] = LineFeature(ep4, ed4, ed2_4, edge_r_noise, edge_t_noise, end_points4);

			Coord ep5;
			ep5 << a / 2.0, 0.0, c / 2.0, 1.0;

			Coord ep5_1, ep5_2; 
			ep5_1 << a / 2.0, -b / 2.0, c / 2.0, 1.0;
			ep5_2 << a / 2.0, b / 2.0, c / 2.0, 1.0;
			Coord_vector4 end_points5;
			get_data_from_line(ep5_1,ep5_2, end_points5);
			
			Coord ed5;
			ed5 << 0.0, 1.0, 0.0, 0.0;

            Coord ed2_5;
			ed2_5 << a / 2.0, 0.0, c / 2.0, 0.0;
			
            ep5 = this->Inner_transfrom_ * ep5;
			ed5 = this->Inner_transfrom_ * ed5;
            ed2_5 = this->Inner_transfrom_ * ed2_5;

			this->edges[4] = LineFeature(ep5, ed5, ed2_5, edge_r_noise, edge_t_noise, end_points5);

			Coord ep6;
			ep6 << a / 2.0, 0.0, -c / 2.0, 1.0;

			Coord ep6_1, ep6_2; 
			ep6_1 << a / 2.0, -b / 2.0, -c / 2.0, 1.0;
			ep6_2 << a / 2.0, b / 2.0, -c / 2.0, 1.0;
			Coord_vector4 end_points6;
			get_data_from_line(ep5_1,ep5_2, end_points6);

			Coord ed6;
			ed6 << 0.0, 1.0, 0.0, 0.0;

            Coord ed2_6;
			ed2_6 << a / 2.0, 0.0, -c / 2.0, 0.0;

			ep6 = this->Inner_transfrom_ * ep6;
			ed6 = this->Inner_transfrom_ * ed6;
            ed2_6 = this->Inner_transfrom_ * ed2_6;

			this->edges[5] = LineFeature(ep6, ed6, ed2_6, edge_r_noise, edge_t_noise, end_points6);

			Coord ep7;
			ep7 << -a / 2.0, 0.0, c / 2.0, 1.0;

			Coord ep7_1, ep7_2; 
			ep7_1 << -a / 2.0, -b / 2.0, c / 2.0, 1.0;
			ep7_2 << -a / 2.0, b / 2.0, c / 2.0, 1.0;
			Coord_vector4 end_points7;
			get_data_from_line(ep7_1,ep7_2, end_points7);

			Coord ed7;
			ed7 << 0.0, 1.0, 0.0, 0.0;

            Coord ed2_7;
			ed2_7 << -a / 2.0, 0.0, c / 2.0, 0.0;

			ep7 = this->Inner_transfrom_ * ep7;
			ed7 = this->Inner_transfrom_ * ed7;
            ed2_7 = this->Inner_transfrom_ * ed2_7;

			this->edges[6] = LineFeature(ep7, ed7, ed2_7, edge_r_noise, edge_t_noise, end_points7);

			Coord ep8;
			ep8 << -a / 2.0, 0.0, -c / 2.0, 1.0;

			Coord ep8_1, ep8_2; 
			ep8_1 << -a / 2.0, -b / 2.0, -c / 2.0, 1.0;
			ep8_2 << -a / 2.0, b / 2.0, -c / 2.0, 1.0;
			Coord_vector4 end_points8;
			get_data_from_line(ep8_1,ep8_2, end_points8);

			Coord ed8;
			ed8 << 0.0, 1.0, 0.0, 0.0;

            Coord ed2_8;
			ed2_8 << -a / 2.0, 0.0, -c / 2.0, 0.0;

			ep8 = this->Inner_transfrom_ * ep8;
			ed8 = this->Inner_transfrom_ * ed8;
            ed2_8 = this->Inner_transfrom_ * ed2_8;
			this->edges[7] = LineFeature(ep8, ed8, ed2_8, edge_r_noise, edge_t_noise, end_points8);

			Coord ep9;
			ep9 << a / 2.0, b / 2.0, 0.0, 1.0;

			Coord ep9_1, ep9_2; 
			ep9_1 << a / 2.0, b / 2.0, c / 2.0, 1.0;
			ep9_2 << a / 2.0, b / 2.0, -c / 2.0, 1.0;
			Coord_vector4 end_points9;
			get_data_from_line(ep9_1,ep9_2, end_points9);

			Coord ed9;
			ed9 << 0.0, 0.0, 1.0, 0.0;

            Coord ed2_9;
			ed2_9 << a / 2.0, b / 2.0, 0.0, 0.0;

			ep9 = this->Inner_transfrom_ * ep9;
			ed9 = this->Inner_transfrom_ * ed9;
            ed2_9 = this->Inner_transfrom_ * ed2_9;
			this->edges[8] = LineFeature(ep9, ed9, ed2_9, edge_r_noise, edge_t_noise, end_points9);

			Coord ep10;
			ep10 << a / 2.0, -b / 2.0, 0.0, 1.0;

			Coord ep10_1, ep10_2; 
			ep10_1 << a / 2.0, -b / 2.0, c / 2.0, 1.0;
			ep10_2 << a / 2.0, -b / 2.0, -c / 2.0, 1.0;
			Coord_vector4 end_points10;
			get_data_from_line(ep10_1,ep10_2, end_points10);

			Coord ed10;
			ed10 << 0.0, 0.0, 1.0, 0.0;

            Coord ed2_10;
			ed2_10 << a / 2.0, -b / 2.0, 0.0, 0.0;

			ep10 = this->Inner_transfrom_ * ep10;
			ed10 = this->Inner_transfrom_ * ed10;
            ed2_10 = this->Inner_transfrom_ * ed2_10;
			this->edges[9] = LineFeature(ep10, ed10, ed2_10, edge_r_noise, edge_t_noise, end_points10);

			Coord ep11;
			ep11 << -a / 2.0, b / 2.0, 0.0, 1.0;

			Coord ep11_1, ep11_2; 
			ep11_1 << -a / 2.0, b / 2.0, c / 2.0, 1.0;
			ep11_2 << -a / 2.0, b / 2.0, -c / 2.0, 1.0;
			Coord_vector4 end_points11;
			get_data_from_line(ep11_1,ep11_2, end_points11);

			Coord ed11;
			ed11 << 0.0, 0.0, 1.0, 0.0;

            Coord ed2_11;
			ed2_11 << -a / 2.0, b / 2.0, 0.0, 0.0;

			ep11 = this->Inner_transfrom_ * ep11;
			ed11 = this->Inner_transfrom_ * ed11;
            ed2_11 = this->Inner_transfrom_ * ed2_11;
			this->edges[10] = LineFeature(ep11, ed11, ed2_11, edge_r_noise, edge_t_noise, end_points11);

			Coord ep12;
			ep12 << -a / 2.0, -b / 2.0, 0.0, 1.0;

			Coord ep12_1, ep12_2; 
			ep10_1 << -a / 2.0, -b / 2.0, c / 2.0, 1.0;
			ep10_2 << -a / 2.0, -b / 2.0, -c / 2.0, 1.0;
			Coord_vector4 end_points12;
			get_data_from_line(ep12_1,ep12_2, end_points12);

			Coord ed12;
			ed12 << 0.0, 0.0, 1.0, 0.0;

            Coord ed2_12;
			ed2_12 << -a / 2.0, -b / 2.0, 0.0, 0.0;

			ep12 = this->Inner_transfrom_ * ep12;
			ed12 = this->Inner_transfrom_ * ed12;
            ed2_12 = this->Inner_transfrom_ * ed2_12;
			this->edges[11] = LineFeature(ep12, ed12, ed2_12, edge_r_noise, edge_t_noise, end_points12);

			// add into feature array:
			for (int i = 0; i < 6; i++) {
				this->feature_array_[i] = &this->planes[i];
			}

			for (int i = 0; i < 12; i++) {
				this->feature_array_[i + 6] = &this->edges[i];
			}
		}

		virtual void make_ptr_from_template(std::vector<double>& dimension_scale, Transform& transform_inside) override {
			dimension_scale.clear();
			dimension_scale.push_back(a_);
			dimension_scale.push_back(b_);
			dimension_scale.push_back(c_);

			transform_inside = this->Inner_transfrom_;
		};

		virtual void GenerateFeaturePoints(std::vector<Coord>& render_points, int feature_id) override {

			if (feature_id < 2) {
				// yz-plane render
				std::vector<double> dimension_scale = { this->b_, this->c_ };
				this->feature_array_[feature_id]->GenerateRenderPoints(render_points,
					this->object_position_, 50, dimension_scale);
			}
			else if((feature_id >= 2) && (feature_id < 4)){
				std::vector<double> dimension_scale = { this->a_, this->c_ };
				this->feature_array_[feature_id]->GenerateRenderPoints(render_points,
					this->object_position_, 50, dimension_scale);
			}
			else if ((feature_id >= 4) && (feature_id < 6)) {
				std::vector<double> dimension_scale = { this->a_, this->b_ };
				this->feature_array_[feature_id]->GenerateRenderPoints(render_points,
					this->object_position_, 50, dimension_scale);
			}
			else if ((feature_id >= 6) && (feature_id < 10)) {
				std::vector<double> dimension_scale = { this->a_};
				this->feature_array_[feature_id]->GenerateRenderPoints(render_points,
					this->object_position_, 100, dimension_scale);
			}
			else if ((feature_id >= 10) && (feature_id < 14)) {
				std::vector<double> dimension_scale = { this->b_ };
				this->feature_array_[feature_id]->GenerateRenderPoints(render_points,
					this->object_position_, 100, dimension_scale);
			}
			else {
				std::vector<double> dimension_scale = { this->c_ };
				this->feature_array_[feature_id]->GenerateRenderPoints(render_points,
					this->object_position_, 100, dimension_scale);
			}
		};

        virtual void GenerateCoefficients(pcl::ModelCoefficients& coefficients, Transform& camera_pose) {
            Transform pose_in_camera = camera_pose.inverse() * this->object_position_ * this->Inner_transfrom_;
            
            Eigen::Matrix3d object_rotation_matrix(pose_in_camera.block(0, 0, 3, 3));
            Eigen::Quaterniond object_rotation(object_rotation_matrix);
            Eigen::Vector3d object_transition(pose_in_camera.block(0, 3, 3, 1));
            coefficients.values.resize(10);
            coefficients.values[0] = object_transition(0);
            coefficients.values[1] = object_transition(1);
            coefficients.values[2] = object_transition(2);
            
            coefficients.values[3] = object_rotation.x();
            coefficients.values[4] = object_rotation.y();
            coefficients.values[5] = object_rotation.z();
            coefficients.values[6] = object_rotation.w();

            coefficients.values[7] = this->a_;
            coefficients.values[8] = this->b_;
            coefficients.values[9] = this->c_;
 
        };

		~Box() {
			delete[] this->edges;
			delete[] this->planes;
		};
		double a_, b_, c_;
		double edge_t_noise = 0.03;
        double edge_r_noise = 0.03;
		double plane_t_noise = 0.03;
        double plane_r_noise = 0.03;

		LineFeature* edges;
		PlaneFeature* planes;
		
	};

	class Cylinder : public GeometryObject {
	public:
		Cylinder(double length, double radius, Transform inner_transform) :
			GeometryObject(cylinder, Transform::Identity(), inner_transform), length_(length), radius_(radius) {
			this->feature_num = 3;
			this->feature_array_ = new GeometryFeature * [3];
			this->planes = new PlaneFeature[2];
			this->surfaces = new SurfaceFeature[1];

			Coord p1;
			p1 << 0.0, 0.0, length / 2.0, 1.0;

			Coord pc1, pc2, pc3, pc4;// four corner points of this plane 
			pc1 << radius, 0, length/2.0, 1.0;
			pc2 << -radius, 0, length/2.0, 1.0;
			pc3 << 0, radius, length/2.0, 1.0;
			pc4 << 0, -radius, length/2.0, 1.0;
			pc1 = this->Inner_transfrom_ * pc1;
			pc2 = this->Inner_transfrom_ * pc2;
			pc3 = this->Inner_transfrom_ * pc3;
			pc4 = this->Inner_transfrom_ * pc4;

			Coord_vector4 points1;
			get_data_from_plane(pc1, pc2, pc3, pc4, points1);

			Coord d1;
			d1 << 0.0, 0.0, 1.0, 0.0;

			Coord dx1;
			dx1 << 1.0, 0.0, 0.0, 0.0;
			Coord dz1;
			dz1 = d1.cross3(dx1);

			p1 = this->Inner_transfrom_ * p1;
			d1 = this->Inner_transfrom_ * d1;
			dx1 = this->Inner_transfrom_ * dx1;
			dz1 = this->Inner_transfrom_ * dz1;
			this->planes[0] = PlaneFeature(p1, d1, plane_r_noise,  plane_t_noise, dx1, dz1, points1);

			Coord p2;
			p2 << 0.0, 0.0, -length / 2.0, 1.0;

			pc1 << radius, 0, -length/2.0, 1.0;
			pc2 << -radius, 0, -length/2.0, 1.0;
			pc3 << 0, radius, -length/2.0, 1.0;
			pc4 << 0, -radius, -length/2.0, 1.0;
			pc1 = this->Inner_transfrom_ * pc1;
			pc2 = this->Inner_transfrom_ * pc2;
			pc3 = this->Inner_transfrom_ * pc3;
			pc4 = this->Inner_transfrom_ * pc4;

			Coord_vector4 points2;
			get_data_from_plane(pc1, pc2, pc3, pc4, points2);
			
			Coord d2;
			d2 << 0.0, 0.0, -1.0, 0.0;

			Coord dx2;
			dx2 << 1.0, 0.0, 0.0, 0.0;
			Coord dz2;
			dz2 = d2.cross3(dx2);

			p2 = this->Inner_transfrom_ * p2;
			d2 = this->Inner_transfrom_ * d2;
			dx2 = this->Inner_transfrom_ * dx2;
			dz2 = this->Inner_transfrom_ * dz2;
			this->planes[1] = PlaneFeature(p2, d2, plane_r_noise,  plane_t_noise, dx2, dz2, points2);

			Coord p3;
			p3 << 0.0, 0.0, 0.0, 1.0;
			Coord d3;
			d3 << 0.0, 0.0, 1.0, 0.0;
			p3 = this->Inner_transfrom_ * p3;
			d3 = this->Inner_transfrom_ * d3;
			this->surfaces[0] = SurfaceFeature(p3, d3, surface_r_noise,  surface_t_noise, radius);

			this->feature_array_[0] = &this->planes[0];
			this->feature_array_[1] = &this->planes[1];
			this->feature_array_[2] = &this->surfaces[0];

		};

		virtual void make_ptr_from_template(std::vector<double>& dimension_scale, Transform& transform_inside) override {
			dimension_scale.clear();
			dimension_scale.push_back(length_);
			dimension_scale.push_back(radius_);

			transform_inside = this->Inner_transfrom_;
		};

		virtual void GenerateFeaturePoints(std::vector<Coord>& render_points, int feature_id) override {
			if (feature_id < 2) {
				std::vector<double> dimension_scale = { this->radius_, this->radius_ };
				this->feature_array_[feature_id]->GenerateRenderPoints(render_points,
					this->object_position_, 50, dimension_scale);
			}
			else {
				std::vector<double> dimension_scale = { this->length_, this->radius_ };
				this->feature_array_[feature_id]->GenerateRenderPoints(render_points,
					this->object_position_, 50, dimension_scale);
			}
		};

        virtual void GenerateCoefficients(pcl::ModelCoefficients& coefficients, Transform& camera_pose) {
            Transform pose_in_camera = camera_pose.inverse() * this->object_position_ * this->Inner_transfrom_;
            
            Eigen::Matrix3d object_rotation_matrix(pose_in_camera.block(0, 0, 3, 3));
            Eigen::Quaterniond object_rotation(object_rotation_matrix);
            Eigen::Vector3d object_transition(pose_in_camera.block(0, 3, 3, 1));
            coefficients.values.resize(10);
            coefficients.values[0] = object_transition(0);
            coefficients.values[1] = object_transition(1);
            coefficients.values[2] = object_transition(2);
            
            coefficients.values[3] = object_rotation.x();
            coefficients.values[4] = object_rotation.y();
            coefficients.values[5] = object_rotation.z();
            coefficients.values[6] = object_rotation.w();

            coefficients.values[7] = this->radius_;
            coefficients.values[8] = this->radius_;
            coefficients.values[9] = this->length_;
        };

		~Cylinder() {
			delete[] planes;
			delete[] surfaces;
		}

		double length_, radius_;
        double plane_t_noise = 0.01;
        double plane_r_noise = 0.01;
        double surface_t_noise = 0.01;
        double surface_r_noise = 0.01;

		PlaneFeature* planes;
		SurfaceFeature* surfaces;
	};

	class Support : public GeometryObject {
	public:
		Support(Transform inner_transform) : GeometryObject(support, Transform::Identity(), inner_transform) {
			// Support only has one plane
			this->feature_num = 1;
			this->feature_array_ = new GeometryFeature * [1];
			this->planes = new PlaneFeature[1];
			double radius = 0.9144; //table radius

			Coord p1;
			p1 << 0.0, 0.0, 0.0, 1.0;

			Coord pc1, pc2, pc3, pc4;// four corner points of this plane 
			pc1 << radius, 0, 0, 1.0;
			pc2 << -radius, 0, 0, 1.0;
			pc3 << 0, radius, 0, 1.0;
			pc4 << 0, -radius, 0, 1.0;
			pc1 = this->Inner_transfrom_ * pc1;
			pc2 = this->Inner_transfrom_ * pc2;
			pc3 = this->Inner_transfrom_ * pc3;
			pc4 = this->Inner_transfrom_ * pc4;

			Coord_vector4 points1;
			get_data_from_plane(pc1, pc2, pc3, pc4, points1);

			Coord d1;
			d1 << 0.0, 0.0, 1.0, 0.0;

			Coord dx1;
			dx1 << 1.0, 0.0, 0.0, 0.0;
			Coord dz1;
			dz1 = d1.cross3(dx1);
			p1 = this->Inner_transfrom_ * p1;
			d1 = this->Inner_transfrom_ * d1;
			dx1 = this->Inner_transfrom_ * dx1;
			dz1 = this->Inner_transfrom_ * dz1;
			this->planes[0] = PlaneFeature(p1, d1, plane_r_noise, plane_t_noise, dx1, dz1, points1);
			this->feature_array_[0] = &this->planes[0];

		};

		virtual void make_ptr_from_template(std::vector<double>& dimension_scale, Transform& transform_inside) override {
			dimension_scale.clear();
			transform_inside = this->Inner_transfrom_;
		};

		virtual void GenerateFeaturePoints(std::vector<Coord>& render_points, int feature_id) override {
			std::vector<double> dimension_scale = { 1.0, 1.0 };
			this->feature_array_[0]->GenerateRenderPoints(render_points, this->object_position_, 50, dimension_scale);
		};

        virtual void GenerateCoefficients(pcl::ModelCoefficients& coefficients, Transform& camera_pose) {
            Transform pose_in_camera = camera_pose.inverse() * this->object_position_ ;
            Coord point_in_camera = pose_in_camera * this->planes[0].point_;
            Coord direction_in_camera = pose_in_camera * this->planes[0].direction_;
            
            coefficients.values.resize(4);
            coefficients.values[0] = direction_in_camera(0);
            coefficients.values[1] = direction_in_camera(1);
            coefficients.values[2] = direction_in_camera(2);
            coefficients.values[3] = - point_in_camera.dot(direction_in_camera);
        };
		~Support() {
			delete[] planes;
		};

		PlaneFeature* planes;
		double plane_t_noise = 0.03;
        double plane_r_noise = 0.03;
	};
}

#endif
