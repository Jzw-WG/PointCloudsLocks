#pragma warning(disable:4996)
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
using namespace std;

namespace pcl
{
	template<>
	struct SIFTKeypointFieldSelector<PointXYZ>
	{
		inline float
			operator () (const PointXYZ& p) const
		{
			return p.z;
		}
	};
}

int
main(int argc, char* argv[])
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPLYFile("D:\\研究生毕业DOC\\实验点云\\cloud_cluster_2.ply", *cloud_xyz);

	const float min_scale = 1;             //设置尺度空间中最小尺度的标准偏差          
	const int n_octaves = 6;               //设置高斯金字塔组（octave）的数目            
	const int n_scales_per_octave = 6;     //设置每组（octave）计算的尺度  
	const float min_contrast = 0.01;          //设置限制关键点检测的阈值       

	pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;//创建sift关键点检测对象
	pcl::PointCloud<pcl::PointWithScale> result;
	sift.setInputCloud(cloud_xyz);//设置输入点云
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	sift.setSearchMethod(tree);//创建一个空的kd树对象tree，并把它传递给sift检测对象
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);//指定搜索关键点的尺度范围
	sift.setMinimumContrast(min_contrast);//设置限制关键点检测的阈值
	sift.compute(result);//执行sift关键点检测，保存结果在result

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	copyPointCloud(result, *cloud_temp);//将点类型pcl::PointWithScale的数据转换为点类型pcl::PointXYZ的数据

	//可视化输入点云和关键点
	pcl::visualization::PCLVisualizer viewer("Sift keypoint");
	viewer.setBackgroundColor(255, 255, 255);
	viewer.addPointCloud(cloud_xyz, "cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "cloud");
	viewer.addPointCloud(cloud_temp, "keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "keypoints");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return 0;

}