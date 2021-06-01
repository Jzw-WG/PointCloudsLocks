#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <ctime>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/fpfh_omp.h> //包含fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
#include <pcl/keypoints/sift_keypoint.h>   // shift关键点相关
#include <pcl/features/vfh.h>                     //VFH特征估计类头文件

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudN;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

Eigen::Matrix4f startSAC_IA(PointCloud::Ptr source, PointCloud::Ptr target, PointCloud::Ptr source_origin, PointCloud::Ptr target_origin,
	PointCloud::Ptr &result, pcl::PointCloud<pcl::Normal>::Ptr source_normals, pcl::PointCloud<pcl::Normal>::Ptr target_normals);
void eraseInfPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
void est_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals, PointCloud::Ptr origin_cloud = NULL);