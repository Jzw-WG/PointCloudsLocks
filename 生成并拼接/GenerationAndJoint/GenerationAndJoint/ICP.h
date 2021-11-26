#pragma once
#include <iostream>
#include <string>
#include <pcl/io/ply_io.h>    //PLY相关头文件
#include <pcl/point_types.h>  //
#include <sstream>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h> 

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;   //定义点云的格式

Eigen::Matrix4f ICPTrans(PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_tar, PointCloudT::Ptr cloud_in_origin, PointCloudT::Ptr cloud_tar_origin, PointCloudT::Ptr result, PointCloudT::Ptr result_filtered, int iterations = 1, bool reverse = false);