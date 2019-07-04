// using seperate axis theorem to calculate the overlapping ratio
#include <math.h>
#include <vector>
#include <Eigen/StdVector>
#include "geometry_feature.hpp"
#include "geometry_object.hpp"

#ifndef SATOVERLAP
#define SATOVERLAP

namespace geometry_relation {

    using Coord_vector = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;

    void get_data_from_box(Eigen::Matrix4d box_pose, double a, double b, double c, Coord_vector& points, Coord_vector& axis) {
        // a , b, c are length, width, height
        Coord p1, p2, p3, p4, p5, p6, p7, p8;
        p1 << a / 2.0, b / 2.0, c / 2.0, 1.0;
        p2 << -a / 2.0, b / 2.0, c / 2.0, 1.0;
        p3 << a / 2.0, -b / 2.0, c / 2.0, 1.0;
        p4 << a / 2.0, b / 2.0, -c / 2.0, 1.0;
        p5 << -a / 2.0, -b / 2.0, c / 2.0, 1.0;
        p6 << -a / 2.0, b / 2.0, -c / 2.0, 1.0;
        p7 << a / 2.0, -b / 2.0, -c / 2.0, 1.0;
        p8 << -a / 2.0, -b / 2.0, -c / 2.0, 1.0;

        p1 = box_pose * p1;
        p2 = box_pose * p2;
        p3 = box_pose * p3;
        p4 = box_pose * p4;
        p5 = box_pose * p5;
        p6 = box_pose * p6;
        p7 = box_pose * p7;
        p8 = box_pose * p8;

        points.clear();
        points.push_back(p1.block(0, 0, 3, 1));
        points.push_back(p2.block(0, 0, 3, 1));
        points.push_back(p3.block(0, 0, 3, 1));
        points.push_back(p4.block(0, 0, 3, 1));
        points.push_back(p5.block(0, 0, 3, 1));
        points.push_back(p6.block(0, 0, 3, 1));
        points.push_back(p7.block(0, 0, 3, 1));
        points.push_back(p8.block(0, 0, 3, 1));

        Coord ax1, ax2, ax3;
        ax1 << 1.0, 0.0, 0.0, 0.0;
        ax2 << 0.0, 1.0, 0.0, 0.0;
        ax3 << 0.0, 0.0, 1.0, 0.0;

        ax1 = box_pose * ax1;
        ax2 = box_pose * ax2;
        ax3 = box_pose * ax3;
        axis.clear();
        axis.push_back(ax1.block(0, 0, 3, 1));
        axis.push_back(ax2.block(0, 0, 3, 1));
        axis.push_back(ax3.block(0, 0, 3, 1));
    };

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
};

#endif
