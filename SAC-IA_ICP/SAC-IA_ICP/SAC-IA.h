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
#include <pcl/features/fpfh_omp.h> //����fpfh���ټ����omp(��˲��м���)
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //�����Ĵ����Ӧ��ϵȥ��
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //�������һ����ȥ��
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/keypoints/sift_keypoint.h>   // shift�ؼ������
#include <pcl/features/vfh.h>                     //VFH����������ͷ�ļ�

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

PointCloud::Ptr startSAC_IA(PointCloud::Ptr source, PointCloud::Ptr target);
void eraseInfPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);