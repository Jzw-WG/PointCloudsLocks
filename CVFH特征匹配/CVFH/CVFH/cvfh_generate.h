#pragma once
#include <vtkVersion.h>
#include <vtkPlaneSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL2);
//
//VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>
#include <pcl/keypoints/harris_3D.h>//harris
#include <pcl/keypoints/sift_keypoint.h>//sift
#include <pcl/features/cvfh.h>    //CVFH
#include <pcl/visualization/pcl_plotter.h>

using namespace pcl;
using namespace std;

int save_cvfh(string path, vector<string> files);