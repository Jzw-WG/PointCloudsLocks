#pragma once
#include <vtkAutoInit.h>//可视化时需要使用vtk
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <fstream>
#include <cvfh_const.h>

typedef std::pair<std::string, std::vector<float> > feature_model;//一个用于存储名称，一个用于存储特征

int build(std::string path, std::vector<std::string> files, std::vector<std::string> model_files);