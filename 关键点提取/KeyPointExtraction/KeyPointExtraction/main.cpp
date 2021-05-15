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
	pcl::io::loadPLYFile("D:\\�о�����ҵDOC\\ʵ�����\\cloud_cluster_2.ply", *cloud_xyz);

	const float min_scale = 1;             //���ó߶ȿռ�����С�߶ȵı�׼ƫ��          
	const int n_octaves = 6;               //���ø�˹�������飨octave������Ŀ            
	const int n_scales_per_octave = 6;     //����ÿ�飨octave������ĳ߶�  
	const float min_contrast = 0.01;          //�������ƹؼ��������ֵ       

	pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;//����sift�ؼ��������
	pcl::PointCloud<pcl::PointWithScale> result;
	sift.setInputCloud(cloud_xyz);//�����������
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	sift.setSearchMethod(tree);//����һ���յ�kd������tree�����������ݸ�sift������
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);//ָ�������ؼ���ĳ߶ȷ�Χ
	sift.setMinimumContrast(min_contrast);//�������ƹؼ��������ֵ
	sift.compute(result);//ִ��sift�ؼ����⣬��������result

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	copyPointCloud(result, *cloud_temp);//��������pcl::PointWithScale������ת��Ϊ������pcl::PointXYZ������

	//���ӻ�������ƺ͹ؼ���
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