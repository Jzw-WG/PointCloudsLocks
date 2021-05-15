#pragma once
#include <iostream>
#include <string>
#include <pcl/io/ply_io.h>    //PLY���ͷ�ļ�
#include <pcl/point_types.h>  //
#include <sstream>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h> 

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;   //������Ƶĸ�ʽ

int showICPviewer(PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_tar, int iterations = 1);