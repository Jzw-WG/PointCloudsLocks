#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>						
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

//简化声明，方便代码阅读
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//标注用户是否请求进行下次迭代
bool next_iteration = false;

//打印4×4的旋转矩阵
void
print4x4Matrix(const Eigen::Matrix4d& matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

//检测空格键是否按下
void
keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
	void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}

int
main(int argc,
	char* argv[])
{
	//创建点云指针
	// The point clouds we will be using
	PointCloudT::Ptr cloud_in(new PointCloudT);  // Original point cloud//原始点云
	PointCloudT::Ptr cloud_tr(new PointCloudT);  // Transformed point cloud//转换后的点云
	PointCloudT::Ptr cloud_icp(new PointCloudT);  // ICP output point cloud//ICP输出点云

	//读取pcd文件
	pcl::console::TicToc time;
	time.tic();
	if (pcl::io::loadPLYFile<pcl::PointXYZ>("E:\\locks\\data\\bunny\\data\\bun000.ply", *cloud_in) == -1)
	{
		PCL_ERROR("Couldn't read file1 \n");
		return (-1);
	}
	std::cout << "Loaded " << cloud_in->size() << " data points from file1" << std::endl;
	if (pcl::io::loadPLYFile<pcl::PointXYZ>("E:\\locks\\data\\bunny\\reconstruction\\bun2.ply", *cloud_icp) == -1)
	{
		PCL_ERROR("Couldn't read file2 \n");
		return (-1);
	}

	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	Eigen::Matrix3d R;
	R << 0.89, -0.417878, 0.172348,
		0.42, 0.906859, 0.015,
		-0.164688, 0.05, 0.985;
	double s = 0.98;

	Eigen::Vector3d T;
	T << 11.2174,
		-14.9277,
		-386.032;
	transformation_matrix.block(0, 0, 3, 3) << s * R;
	transformation_matrix.block(0, 3, 3, 1) << T;


	////执行转换
	pcl::transformPointCloud(*cloud_in, *cloud_tr, transformation_matrix);


	//可视窗口初始化
	pcl::visualization::PCLVisualizer viewer;
	//viewer.setCameraFieldOfView(0.785398);      // fov 大概45度
	viewer.setBackgroundColor(0, 0, 0);   // 背景设为灰色

	//可视窗口加入点云
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> inColorHandler(cloud_tr, 255, 255, 255);// 白色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outColorHandler(cloud_icp, 230, 20, 20); // 红色
	viewer.addPointCloud(cloud_tr, inColorHandler, "transformed_X");
	viewer.addPointCloud(cloud_icp, outColorHandler, "Y");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed_X");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Y");
	//viewer.addCoordinateSystem(0.5);
	viewer.spin();
	
	system("pause");
	return (0);
}