#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <unistd.h>
#include <Eigen/Dense>

#include <vector>

void vis_points_in_pcl(const std::vector<Eigen::Vector4d> points, std::string file_name) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	plane_cloud->width = points.size();
	plane_cloud->height = 1;
	//plane_cloud->points.resize(plane_cloud->width);

	for (const auto& p : points) {
		pcl::PointXYZ pp;
		pp.x = p(0);
		pp.y = p(1);
		pp.z = p(2);
		plane_cloud->points.push_back(pp);
	}

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>(file_name + ".pcd", *plane_cloud, false);
};
