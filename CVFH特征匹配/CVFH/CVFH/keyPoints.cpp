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
//#include <pcl/visualization/pcl_plotter.h>//��ʾ������
//
//
//using namespace pcl;
//
////cvfhȫ������
//int main(int argc, char** argv)
//{
//	for (int i = 0; i < 6; i++)
//	{
//		std::stringstream ss;
//		ss << "modelData\\model_" << i << ".ply";
//		//��ȡ����
//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
//		pcl::io::loadPLYFile<pcl::PointXYZ>(ss.str(), *cloud_in);
//
//		//���Ʒ���
//		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
//		est_normal.setKSearch(16);         //����k����������ֵΪ20����
//		est_normal.setInputCloud(cloud_in);   //��������ģ�͵���
//		est_normal.setSearchMethod(tree);
//		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//		est_normal.compute(*normals);//������Ʒ���
//
//		//VFH
//		pcl::CVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> est_vfh;
//		est_vfh.setInputCloud(cloud_in);
//		est_vfh.setInputNormals(normals);
//		//����һ���յ�kd����ʾ��
//		pcl::search::KdTree<PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
//		est_vfh.setSearchMethod(tree1);
//		//��������ݼ�
//		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>());
//		est_vfh.compute(*vfhs);
//		std::stringstream ss1;
//		ss1 << "modelData\\model_cvfh_" << i << ".pcd";
//		pcl::io::savePCDFile(ss1.str(), *vfhs);
//
//		//��ʾvfh����
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
