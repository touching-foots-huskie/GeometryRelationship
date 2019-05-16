#pragma once
#include <map>
#include <vector>
#include <Eigen/Core>
#include "geometry_feature.h"

// TODO: change adapter for logging into csv files.
// TODO: Do we really need different implementation for Contact in different obejects?

namespace geometry_relation {
	enum ENUM_GEOMETRY_OBJECT {
		box, cylinder, support
	};

	class GeometryObject
	{
	public:
		GeometryObject(ENUM_GEOMETRY_OBJECT geometry_type, Transform object_position) :
			geometry_type_(geometry_type), object_position_(object_position) {};

		void SetPosition(Transform objection_position) {
			this->object_position_ = objection_position;
		}

		// Abstract contact setting : There are still a lot to do on how to avoid redaudent constraints
		bool Object_Contact(GeometryObject& geometry_object, std::vector<ENUM_CONTACT>& contact_type_vector,
							std::vector<Coord>& feature_point_1, std::vector<Coord>& feature_point_2,
							std::vector<Coord>& feature_direction_1, std::vector<Coord>& feature_direction_2,
							std::vector<double>& noise_level_vector){

			std::map<int, bool> other_constraints_made;

			ENUM_CONTACT _temp_contact;
			double _temp_noise_level;
			for (int i = 0; i < this->feature_num; i++) {
				for (int j = 0; j < geometry_object.feature_num; j++) {
					_temp_noise_level = this->feature_array_[i]->Contact(
						*geometry_object.feature_array_[j], _temp_contact);
					if (other_constraints_made[j])
						continue;  // no same feature for different constraints.

					if (_temp_contact != unknown_contact) {
						contact_type_vector.push_back(_temp_contact);
						// log feature vectors
						feature_point_1.push_back(this->feature_array_[i]->point_);
						feature_point_2.push_back(geometry_object.feature_array_[j]->point_);

						feature_direction_1.push_back(this->feature_array_[i]->direction_);
						feature_direction_2.push_back(geometry_object.feature_array_[j]->direction_);

						noise_level_vector.push_back(_temp_noise_level);

						other_constraints_made[j] = true;
						break;  // if plane-plane, then there no plane-edge.
					}
				}
			}

			// filter redundant relations:

		};
		//

		ENUM_GEOMETRY_OBJECT geometry_type_;
		Transform object_position_;

		GeometryFeature** feature_array_;
		int feature_num;
	};

	class Box : public GeometryObject
	{
	public:
		Box(double a, double b, double c) : 
			GeometryObject(box, Transform::Identity()), a_(a), b_(b), c_(c)
		{
			this->feature_num = 18;
			this->feature_array_ = new GeometryFeature * [18];
			this->edges = new LineFeature[12];
			this->planes = new PlaneFeature[6];

			// 6 planes:
			Coord p1;
			p1 << a / 2.0, 0.0, 0.0, 1.0;
			Coord d1;
			d1 << 1.0, 0.0, 0.0, 0.0;
			this->planes[0] = PlaneFeature(p1, d1, plane_noise);

			Coord p2;
			p2 << -a / 2.0, 0.0, 0.0, 1.0;
			Coord d2;
			d2 << -1.0, 0.0, 0.0, 0.0;
			this->planes[1] = PlaneFeature(p2, d2, plane_noise);

			Coord p3;
			p3 << 0.0, b / 2.0, 0.0, 1.0;
			Coord d3;
			d3 << 0.0, 1.0, 0.0, 0.0;
			this->planes[2] = PlaneFeature(p3, d3, plane_noise);

			Coord p4;
			p4 << 0.0, -b / 2.0, 0.0, 1.0;
			Coord d4;
			d4 << 0.0, -1.0, 0.0, 0.0;
			this->planes[3] = PlaneFeature(p4, d4, plane_noise);

			Coord p5;
			p5 << 0.0, 0.0, c / 2.0, 1.0;
			Coord d5;
			d5 << 0.0, 0.0, 1.0, 0.0;
			this->planes[4] = PlaneFeature(p5, d5, plane_noise);

			Coord p6;
			p6 << 0.0, 0.0, -c / 2.0, 1.0;
			Coord d6;
			d6 << 0.0, 0.0, -1.0, 0.0;
			this->planes[5] = PlaneFeature(p6, d6, plane_noise);

			// 12 lines:
			Coord ep1;
			ep1 << 0.0, b / 2.0, c / 2.0, 1.0;
			Coord ed1;
			ed1 << 1.0, 0.0, 0.0, 0.0;
			this->edges[0] = LineFeature(ep1, ed1, edge_noise);

			Coord ep2;
			ep2 << 0.0, b / 2.0, -c / 2.0, 1.0;
			Coord ed2;
			ed2 << 1.0, 0.0, 0.0, 0.0;
			this->edges[1] = LineFeature(ep2, ed2, edge_noise);

			Coord ep3;
			ep3 << 0.0, -b / 2.0, c / 2.0, 1.0;
			Coord ed3;
			ed3 << 1.0, 0.0, 0.0, 0.0;
			this->edges[2] = LineFeature(ep3, ed3, edge_noise);

			Coord ep4;
			ep4 << 0.0, -b / 2.0, -c / 2.0, 1.0;
			Coord ed4;
			ed4 << 1.0, 0.0, 0.0, 0.0;
			this->edges[3] = LineFeature(ep4, ed4, edge_noise);

			Coord ep5;
			ep5 << a / 2.0, 0.0, c / 2.0, 1.0;
			Coord ed5;
			ed5 << 0.0, 1.0, 0.0, 0.0;
			this->edges[4] = LineFeature(ep5, ed5, edge_noise);

			Coord ep6;
			ep6 << a / 2.0, 0.0, -c / 2.0, 1.0;
			Coord ed6;
			ed6 << 0.0, 1.0, 0.0, 0.0;
			this->edges[5] = LineFeature(ep6, ed6, edge_noise);

			Coord ep7;
			ep7 << -a / 2.0, 0.0, c / 2.0, 1.0;
			Coord ed7;
			ed7 << 0.0, 1.0, 0.0, 0.0;
			this->edges[6] = LineFeature(ep7, ed7, edge_noise);

			Coord ep8;
			ep8 << -a / 2.0, 0.0, -c / 2.0, 1.0;
			Coord ed8;
			ed8 << 0.0, 1.0, 0.0, 0.0;
			this->edges[7] = LineFeature(ep8, ed8, edge_noise);

			Coord ep9;
			ep9 << a / 2.0, b / 2.0, 0.0, 1.0;
			Coord ed9;
			ed9 << 0.0, 0.0, 1.0, 0.0;
			this->edges[8] = LineFeature(ep9, ed9, edge_noise);

			Coord ep10;
			ep10 << a / 2.0, -b / 2.0, 0.0, 1.0;
			Coord ed10;
			ed10 << 0.0, 0.0, 1.0, 0.0;
			this->edges[9] = LineFeature(ep10, ed10, edge_noise);

			Coord ep11;
			ep11 << -a / 2.0, b / 2.0, 0.0, 1.0;
			Coord ed11;
			ed11 << 0.0, 0.0, 1.0, 0.0;
			this->edges[10] = LineFeature(ep11, ed11, edge_noise);

			Coord ep12;
			ep12 << -a / 2.0, -b / 2.0, 0.0, 1.0;
			Coord ed12;
			ed12 << 0.0, 0.0, 1.0, 0.0;
			this->edges[11] = LineFeature(ep12, ed12, edge_noise);

			// add into feature array:
			for (int i = 0; i < 6; i++) {
				this->feature_array_[i] = &this->planes[i];
			}

			for (int i = 0; i < 12; i++) {
				this->feature_array_[i + 6] = &this->edges[i];
			}
		}

		double a_, b_, c_;
		double edge_noise = 0.1;
		double plane_noise = 0.1;

		LineFeature* edges;
		PlaneFeature* planes;
		
	};

	class Cylinder : public GeometryObject {
	public:
		Cylinder(double length, double radius) :
			GeometryObject(cylinder, Transform::Identity()), length_(length), radius_(radius) {
			this->feature_num = 3;
			this->planes = new PlaneFeature[2];
			this->surfaces = new SurfaceFeature[1];

			Coord p1;
			p1 << 0.0, length / 2.0, 0.0, 1.0;
			Coord d1;
			d1 << 0.0, 1.0, 0.0, 0.0;
			this->planes[0] = PlaneFeature(p1, d1, plane_noise);

			Coord p2;
			p2 << 0.0, -length / 2.0, 0.0, 1.0;
			Coord d2;
			d2 << 0.0, -1.0, 0.0, 0.0;
			this->planes[1] = PlaneFeature(p2, d2, plane_noise);

			Coord p3;
			p3 << 0.0, 0.0, 0.0, 1.0;
			Coord d3;
			d3 << 0.0, 1.0, 0.0, 0.0;
			this->surfaces[0] = SurfaceFeature(p3, d3, surface_noise, radius);

			this->feature_array_[0] = &this->planes[0];
			this->feature_array_[1] = &this->planes[1];
			this->feature_array_[2] = &this->surfaces[0];

		}

		double length_, radius_;
		double plane_noise = 0.1;
		double surface_noise = 0.1;

		PlaneFeature* planes;
		SurfaceFeature* surfaces;
	};

	class Support : public GeometryObject {
	public:
		Support() : GeometryObject(support, Transform::Identity()) {
			// Support only has one plane
			this->feature_num = 1;
			this->planes = new PlaneFeature[1];

			Coord p1;
			p1 << 0.0, 0.0, 0.0, 1.0;
			Coord d1;
			d1 << 0.0, 1.0, 0.0, 0.0;
			this->planes[0] = PlaneFeature(p1, d1, plane_noise);
			this->feature_array_[0] = &this->planes[0];

		};

		PlaneFeature* planes;
		double plane_noise = 0.1;
	};
}
