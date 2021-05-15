#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>  //滤波相关
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <vector>
#include <algorithm>
#include <ctime>
#include <pcl/visualization/pcl_plotter.h>
#include <fstream>//写入txt
#include<string>
#include <stdlib.h>//将整型转换成字符型
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/convolution.h>
#include <pcl/surface/mls.h>

using namespace std;  // 可以加入 std 的命名空间
//体素滤波
int main(int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("D:\\研究生毕业DOC\\实验点云\\工件2_模型.ply", *cloud) == -1) {
		PCL_ERROR("Couldnot read file.\n");
		system("pause");
		return(-1);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);//输入
	sor.setLeafSize(4,4,4);//分别率,越小越密,参数分别是xyz
	sor.filter(*outputcloud);//输出

	pcl::io::savePLYFile("D:\\研究生毕业DOC\\实验点云\\体素滤波输出_工件2_模型_4mm.ply", *outputcloud);
}

/*
* 移动最小二乘法
int main(int argc, char** argv)
{// 将一个适当类型的输入文件加载到对象PointCloud中
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	// 加载bun0.pcd文件，加载的文件在 PCL的测试数据中是存在的 
	pcl::io::loadPLYFile("D:\\研究生毕业DOC\\实验点云\\半径滤波输出30-2-程序输出.ply", *cloud);
	// 创建一个KD树
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	// 输出文件中有PointNormal类型，用来存储移动最小二乘法算出的法线
	pcl::PointCloud<pcl::PointNormal> mls_points;
	// 定义对象 (第二种定义类型是为了存储法线, 即使用不到也需要定义出来)
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;
	mls.setComputeNormals(true);
	//设置参数
	mls.setInputCloud(cloud);
	mls.setPolynomialOrder(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(4);
	// 曲面重建
	mls.process(mls_points);
	// 保存结果
	pcl::io::savePCDFile("D:\\研究生毕业DOC\\实验点云\\test.pcd", mls_points);

	pcl::io::loadPCDFile("D:\\研究生毕业DOC\\实验点云\\test.pcd", *cloud);
	pcl::io::savePLYFile("D:\\研究生毕业DOC\\实验点云\\test.ply", *cloud);
}
*/
/*
* 双边
int main() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("D:\\研究生毕业DOC\\实验点云\\半径滤波输出30-2.ply", *cloud) == -1) {
		PCL_ERROR("Couldnot read file.\n");
		system("pause");
		return(-1);
	}
	pcl::PointCloud<pcl::PointXYZRGB> outcloud;
	pcl::BilateralFilter<pcl::PointXYZRGB> bf;
	pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
	bf.setInputCloud(cloud);
	bf.setHalfSize(5);
	bf.setStdDev(0.3);
	//bf.setSigmaS(5);//设置双边滤波器用于空间邻域/窗口的高斯的标准偏差
	//bf.setSigmaR(0.003);//设置高斯的标准偏差用于控制相邻像素由于强度差异而下降多少（在我们的情况下为深度）
	bf.filter(outcloud);

	// 保存滤波输出点云文件  
	pcl::io::savePLYFile("D:\\研究生毕业DOC\\实验点云\\双边滤波输出5-0.3.ply", outcloud);
	return (0);
}
*/
/*
* 高斯
int main(int argc, char** argv) {
	//Create the input and filtered cloud objects
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	//Read in the input file
	if (pcl::io::loadPLYFile("D:\\研究生毕业DOC\\实验点云\\半径滤波输出30-2.ply", *inputcloud) == -1)
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd\n");
		return(-1);
	}

	//Set up the Gaussian Kernel
	pcl::filters::GaussianKernel<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr kernel(new pcl::filters::GaussianKernel<pcl::PointXYZRGB, pcl::PointXYZRGB>);
	(*kernel).setSigma(6);
	(*kernel).setThresholdRelativeToSigma(4);
	std::cout << "Kernel made" << std::endl;

	//Set up the KDTree
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	(*kdtree).setInputCloud(inputcloud);
	std::cout << "KdTree made" << std::endl;

	//Set up the Convolution Filter
	pcl::filters::Convolution3D<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::filters::GaussianKernel<pcl::PointXYZRGB, pcl::PointXYZRGB>> convolution;
	convolution.setKernel(*kernel);
	convolution.setInputCloud(inputcloud);
	convolution.setSearchMethod(kdtree);
	convolution.setRadiusSearch(20);
	std::cout << "Convolution Start" << std::endl;

	convolution.convolve(*outputcloud);
	std::cout << "Convoluted" << std::endl;

	pcl::io::savePLYFile("D:\\研究生毕业DOC\\实验点云\\高斯滤波输出20-2.ply", *outputcloud);
}
*/
/*
* 中值
int main(int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("D:\\研究生毕业DOC\\实验点云\\半径滤波输出30-2.ply", *cloud) == -1) {
		PCL_ERROR("Couldnot read file.\n");
		system("pause");
		return(-1);
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::MedianFilter<pcl::PointXYZRGB> fbf;
	fbf.setInputCloud(cloud);
	fbf.setMaxAllowedMovement(0.5);
	fbf.setWindowSize(20);
	fbf.filter(*cloud_out);

	pcl::io::savePLYFile("D:\\研究生毕业DOC\\实验点云\\中值滤波输出20-2.ply", *cloud_out);
}
*/


/*
int main(int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter2(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("D:\\研究生毕业DOC\\实验点云\\直通滤波后的点云1.ply", *cloud) == -1) {
		PCL_ERROR("Couldnot read file.\n");
		system("pause");
		return(-1);
	}
	

	//统计滤波
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(20); //K近邻搜索点个数
	sor.setStddevMulThresh(1.0); //标准差倍数
	sor.setNegative(false); //保留未滤波点（内点）
	sor.filter(*cloud_filter2);  //保存滤波结果到cloud_filter

	pcl::io::savePLYFile("D:\\研究生毕业DOC\\实验点云\\统计滤波输出20-1.ply", *cloud_filter2);
	
	
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("D:\\研究生毕业DOC\\实验点云\\统计滤波输出20-2.ply", *cloud_filter2) == -1) {
		PCL_ERROR("Couldnot read file.\n");
		system("pause");
		return(-1);
	}
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;//创建半径滤波器对象
	outrem.setInputCloud(cloud_filter2);			//设置输入点云
	outrem.setRadiusSearch(2);					//设置半径为4cm
	outrem.setMinNeighborsInRadius(2);				//设置最小邻接点个数阈值,半径范围内其他点个数少于5的点将被滤除
	outrem.filter(*cloud_filter1);				//执行滤波

	pcl::io::savePLYFile("D:\\研究生毕业DOC\\实验点云\\半径滤波输出20-2.ply", *cloud_filter1);
	
}
*/
