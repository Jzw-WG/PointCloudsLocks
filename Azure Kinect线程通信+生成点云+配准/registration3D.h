#pragma once
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

struct init_param_t
{
	Eigen::Matrix3d R;
	Eigen::Vector3d T;
	double s = 0;
	Eigen::Matrix<double, 1, 2> I;
};

struct next_param_t
{
	Eigen::Matrix3d R;
	Eigen::Vector3d T;
	double s = 0;
	double en = 0;
	double en1 = 0;
	double fitness = 0;
};

struct show_param_t
{
	Eigen::Matrix3d R;
	Eigen::Vector3d T;
	double s = 0;
	Eigen::Matrix<double, 1, 2> I;
	double en = 0;
	double fitness = 0;
	int iteration = 0;
};

init_param_t init(pcl::PointCloud<pcl::PointXYZ>::Ptr X, pcl::PointCloud<pcl::PointXYZ>::Ptr Y, int pointX, int pointY);

void searchNearestPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr X, pcl::PointCloud<pcl::PointXYZ>::Ptr Y, pcl::PointCloud<pcl::PointXYZ>::Ptr zk);

//计算E(k+1)和fitness
double computeEAndFitness(double s, Eigen::Matrix3d R, Eigen::Vector3d T, Eigen::MatrixXd Xi, Eigen::MatrixXd Zi, double& fitness);

//计算E(k+1)
double computeE(double s, Eigen::Matrix3d R, Eigen::Vector3d T, Eigen::MatrixXd Xi, Eigen::MatrixXd Zi);

//计算R(n+1)
Eigen::Matrix3f computeR(Eigen::MatrixXf Xi, Eigen::MatrixXf Zi);

//计算S
double computeS(Eigen::Matrix<double, 1, 2> I, Eigen::Matrix3d Rn1, Eigen::MatrixXd Xi, Eigen::MatrixXd Zi);

//计算T
Eigen::Vector3d computeT(Eigen::Vector3d centroidZi, double sn, Eigen::Matrix3d Rn, Eigen::Vector3d centroidXi);

next_param_t Solvecircle(double s, Eigen::Matrix3d R, Eigen::Vector3d T, Eigen::Matrix<double, 1, 2> I,
	pcl::PointCloud<pcl::PointXYZ>::Ptr X, pcl::PointCloud<pcl::PointXYZ>::Ptr Y);

show_param_t reg3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1);

void showResult(show_param_t showParam, pcl::PointCloud<pcl::PointXYZ>::Ptr X, pcl::PointCloud<pcl::PointXYZ>::Ptr Y);