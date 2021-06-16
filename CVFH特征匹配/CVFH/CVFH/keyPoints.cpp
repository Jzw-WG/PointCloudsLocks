//#include <vtkVersion.h>
//#include <vtkPlaneSource.h>
//#include <vtkPolyData.h>
//#include <vtkSmartPointer.h>
//#include <vtkPolyDataMapper.h>
//#include <vtkActor.h>
//#include <vtkRenderWindow.h>
//#include <vtkRenderer.h>
//#include <vtkRenderWindowInteractor.h>
//
//#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL2);
//
//VTK_MODULE_INIT(vtkInteractionStyle);
//VTK_MODULE_INIT(vtkRenderingFreeType);
//
//
//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/point_types.h>
//#include <pcl/common/io.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/console/time.h>
//#include <pcl/keypoints/harris_3D.h>//harris
//#include <pcl/keypoints/sift_keypoint.h>//sift
//#include <pcl/features/cvfh.h>    //CVFH
//#include <pcl/visualization/pcl_plotter.h>//显示描述子
//
//
//using namespace pcl;
//
////cvfh全局特性
//int main(int argc, char** argv)
//{
//	for (int i = 0; i < 6; i++)
//	{
//		std::stringstream ss;
//		ss << "modelData\\model_" << i << ".ply";
//		//读取点云
//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
//		pcl::io::loadPLYFile<pcl::PointXYZ>(ss.str(), *cloud_in);
//
//		//估计法线
//		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
//		est_normal.setKSearch(16);         //设置k邻域搜索阈值为20个点
//		est_normal.setInputCloud(cloud_in);   //设置输入模型点云
//		est_normal.setSearchMethod(tree);
//		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//		est_normal.compute(*normals);//计算点云法线
//
//		//VFH
//		pcl::CVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> est_vfh;
//		est_vfh.setInputCloud(cloud_in);
//		est_vfh.setInputNormals(normals);
//		//创建一个空的kd树表示法
//		pcl::search::KdTree<PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
//		est_vfh.setSearchMethod(tree1);
//		//输出的数据集
//		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>());
//		est_vfh.compute(*vfhs);
//		std::stringstream ss1;
//		ss1 << "modelData\\model_cvfh_" << i << ".pcd";
//		pcl::io::savePCDFile(ss1.str(), *vfhs);
//
//		//显示vfh特征
//		pcl::visualization::PCLPlotter plotter;
//		plotter.addFeatureHistogram<pcl::VFHSignature308>(*vfhs, "vfh", 0);
//		plotter.plot();
//	}
//	cout << "ok" << endl;
//
//	system("pause");
//	return 0;
//}
//
//
