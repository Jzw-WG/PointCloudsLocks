#pragma once
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>  //�˲����
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <boost/thread/thread.hpp>
#include <vector>
#include <algorithm>
#include <ctime>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/convolution.h>
#include <pcl/surface/mls.h>
#include <pcl/point_types.h>
#include <stdlib.h>//������ת�����ַ���

using namespace std;  // ���Լ��� std �������ռ�
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int voxelFilter(PointCloud::Ptr inputcloud, PointCloud::Ptr outputcloud, float lx, float ly, float lz);
int movingLeastSquaresFilter(PointCloud::Ptr inputcloud, pcl::PointCloud<pcl::PointNormal> mls_points, double radius);
int gaussionFilter(PointCloud::Ptr inputcloud, PointCloud::Ptr outputcloud, double radius);
int mediumnFilter(PointCloud::Ptr inputcloud, PointCloud::Ptr outputcloud, float max_allow, int win_size);
int statisticalOutlierRemovalFilter(PointCloud::Ptr inputcloud, PointCloud::Ptr outputcloud, int nr_k, double stddev_mult, bool nagetive = false);
int radiusFilter(PointCloud::Ptr inputcloud, PointCloud::Ptr outputcloud, double radius, int min_pts);
int pathThroughFilter(PointCloud::Ptr inputcloud, PointCloud::Ptr outputcloud, string field, float limit_min, float limit_max);