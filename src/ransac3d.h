#ifndef RANSAC3D_H_
#define RANSAC3D_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>

template<typename PointT>
class Ransac3d {

public: 
    Ransac3d();

    ~Ransac3d();

    std::unordered_set<int> MyRansac3d(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

};
#endif

//constructor:
template<typename PointT>
Ransac3d<PointT>::Ransac3d() {}


//de-constructor:
template<typename PointT>
Ransac3d<PointT>::~Ransac3d() {}

template<typename PointT>
std::unordered_set<int> Ransac3d<PointT>::MyRansac3d(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;

    auto points = cloud->points;

	srand(time(NULL));
	
	// TODO: Fill in this function
	while (maxIterations--)
	{
		// Randomly pick two pints

		std::unordered_set<int> inliers;
		while (inliers.size() < 3)
		{
			inliers.insert(rand()%(cloud->points.size()));
		}

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto itr = inliers.begin();
		x1 = points[*itr].x;
		y1 = points[*itr].y;
		z1 = points[*itr].z;
		itr++;
		x2 = points[*itr].x;
		y2 = points[*itr].y;
		z2 = points[*itr].z;
		itr++;
		x3 = points[*itr].x;
		y3 = points[*itr].y;
		z3 = points[*itr].z;

		float a = ((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1));
		float b = ((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1));
		float c = ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1));
		float d = -((a * x1) + (b * y1) + (c * z1));

		for (int index = 0; index < cloud->points.size(); index++)
		{
			if (inliers.count(index) > 0)
				continue;

			pcl::PointXYZI point = cloud->points[index];
			float x3 = point.x;
			float y3 = point.y;
			float z3 = point.z;

			float distance = fabs(a*x3 + b*y3 + c*z3 + d)/sqrt(a*a + b*b + c*c);

			if (distance <= distanceTol)
				inliers.insert(index);
		}

		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
 	}
	
	return inliersResult;
}