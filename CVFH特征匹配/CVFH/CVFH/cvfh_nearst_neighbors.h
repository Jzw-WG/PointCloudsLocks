#pragma once
#include <stdio.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/features/cvfh.h>    //cvFH
#include <pcl/visualization/pcl_plotter.h>//��ʾ������
#include <pcl/filters/voxel_grid.h>
using namespace std;

typedef std::pair<std::string, std::vector<float> > cvfh_model;//ǰ�����ڴ洢���ƣ��������ڴ洢vfh����

void nearestKSearch(flann::Index<flann::ChiSquareDistance<float> >& index, const cvfh_model& model, int k, flann::Matrix<int>& indices, flann::Matrix<float>& distances);
bool loadFileList(std::vector<cvfh_model>& models, const std::string& filename);
void calcuate_cvfh(const string name, pcl::PointCloud<pcl::VFHSignature308>& vfhs, float normal_r = 0.6);