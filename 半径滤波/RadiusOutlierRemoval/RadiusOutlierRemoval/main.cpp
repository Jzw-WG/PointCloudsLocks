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
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
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
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//体素滤波
int voxelFilter(PointCloud::Ptr inputcloud, PointCloud::Ptr outputcloud, float lx, float ly, float lz) {
	if (inputcloud->points.size() == 0) {
		return -1;
	}
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setInputCloud(inputcloud);//输入
	voxel_grid.setLeafSize(lx,ly,lz);//分别率,越小越密,参数分别是xyz
	voxel_grid.filter(*outputcloud);//输出
	return 0;
}

//移动最小二乘法
int movingLeastSquaresFilter(PointCloud::Ptr inputcloud, pcl::PointCloud<pcl::PointNormal> mls_points, double radius) {
	if (inputcloud->points.size() == 0) {
		return -1;
	}
	// 创建一个KD树
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	// 输出文件中有PointNormal类型，用来存储移动最小二乘法算出的法线
	// 定义对象 (第二种定义类型是为了存储法线, 即使用不到也需要定义出来)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals(true);
	//设置参数
	mls.setInputCloud(inputcloud);
	mls.setPolynomialOrder(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(radius);//4
	// 曲面重建
	mls.process(mls_points);
	return 0;
}

////双边
//int bilateralFilter(string inputFileName, string outputFileName) {
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(inputFileName, *cloud) == -1) {
//		PCL_ERROR("Couldnot read file.\n");
//		system("pause");
//		return(-1);
//	}
//	pcl::PointCloud<pcl::PointXYZRGB> outcloud;
//	pcl::BilateralFilter<pcl::PointXYZRGB> bf;
//	pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
//	bf.setInputCloud(cloud);
//	bf.setHalfSize(5);
//	bf.setStdDev(0.3);
//	//bf.setSigmaS(5);//设置双边滤波器用于空间邻域/窗口的高斯的标准偏差
//	//bf.setSigmaR(0.003);//设置高斯的标准偏差用于控制相邻像素由于强度差异而下降多少（在我们的情况下为深度）
//	bf.filter(outcloud);
//
//	// 保存滤波输出点云文件  
//	pcl::io::savePLYFile(outputFileName, outcloud);
//	return (0);
//}

//高斯
int gaussionFilter(PointCloud::Ptr inputcloud, PointCloud::Ptr outputcloud, double radius) {
	if (inputcloud->points.size() == 0) {
		return -1;
	}
	//Set up the Gaussian Kernel
	pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>::Ptr kernel(new pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>);
	(*kernel).setSigma(1);
	(*kernel).setThresholdRelativeToSigma(3);
	std::cout << "Kernel made" << std::endl;

	//Set up the KDTree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	(*kdtree).setInputCloud(inputcloud);
	std::cout << "KdTree made" << std::endl;

	//Set up the Convolution Filter
	pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> convolution;
	convolution.setKernel(*kernel);
	convolution.setInputCloud(inputcloud);
	convolution.setSearchMethod(kdtree);
	convolution.setRadiusSearch(radius);//20
	std::cout << "Convolution Start" << std::endl;

	convolution.convolve(*outputcloud);
	std::cout << "Convoluted" << std::endl;
	return 0;
}



//中值
int mediumnFilter(PointCloud::Ptr inputcloud, PointCloud::Ptr outputcloud, float max_allow, int win_size) {
	if (inputcloud->points.size() == 0) {
		return -1;
	}
	pcl::MedianFilter<pcl::PointXYZ> fbf;
	fbf.setInputCloud(inputcloud);
	fbf.setMaxAllowedMovement(max_allow);//0.5
	fbf.setWindowSize(win_size);//20
	fbf.filter(*outputcloud);
	return 0;
}

//统计
int statisticalOutlierRemovalFilter(PointCloud::Ptr inputcloud, PointCloud::Ptr outputcloud, int nr_k, double stddev_mult, bool nagetive = false) {
	if (inputcloud->points.size() == 0) {
		return -1;
	}
	//统计滤波
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(inputcloud);
	sor.setMeanK(nr_k); //K近邻搜索点个数 20
	sor.setStddevMulThresh(stddev_mult); //标准差倍数 0.1
	sor.setNegative(nagetive); //保留未滤波点（内点）
	sor.filter(*outputcloud);  //保存滤波结果到cloud_filter
	return 0;
}

int radiusFilter(PointCloud::Ptr inputcloud, PointCloud::Ptr outputcloud, double radius, int min_pts)
{
	if (inputcloud->points.size() == 0) {
		return -1;
	}
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;//创建半径滤波器对象
	outrem.setInputCloud(inputcloud);			//设置输入点云
	outrem.setRadiusSearch(radius);					//设置半径为4cm
	//outrem.setMinNeighborsInRadius(min_pts);				//设置最小邻接点个数阈值,半径范围内其他点个数少于5的点将被滤除
	outrem.filter(*outputcloud);				//执行滤波
	return 0;
}

int pathThroughFilter(PointCloud::Ptr inputcloud, PointCloud::Ptr outputcloud, string field, float limit_min, float limit_max)
{
	if (inputcloud->points.size() == 0) {
		return -1;
	}
	cout << "there are " << inputcloud->points.size() << " points before filtering." << endl;

	////2.取得点云坐标极值,写在函数外
	//pcl::PointXYZ minPt, maxPt;
	//pcl::getMinMax3D(*inputcloud, minPt, maxPt);

	//3.直通滤波
	pcl::PassThrough<pcl::PointXYZ> pass;     //创建滤波器对象
	pass.setInputCloud(inputcloud);                //设置待滤波的点云
	pass.setFilterFieldName(field);             //设置在Z轴方向上进行滤波 "x" "y" "z"
	pass.setFilterLimits(limit_min, limit_max);    //设置滤波范围(从最高点向下12米去除)
	//pass.setFilterFieldName("y");             //设置在Z轴方向上进行滤波
	//pass.setFilterLimits(0, 0.3);    //设置滤波范围(从最高点向下12米去除)
	pass.setFilterLimitsNegative(false);      //保留
	pass.filter(*outputcloud);               //滤波并存储
	cout << "there are " << outputcloud->points.size() << " points after filtering." << endl;
	return 0;
}

int main(int argc, char** argv) {
	string inputFileName = "..\\..\\..\\..\\data\\gen\\lock_1_000.ply";
	string outputFileName = "..\\..\\..\\..\\data\\gen\\handled\\lock_1_000_pass.ply";
	PointCloud::Ptr inputcloud(new PointCloud);
	PointCloud::Ptr outputcloud(new PointCloud);
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(inputFileName, *inputcloud) == -1) {
		PCL_ERROR("Couldnot read file.\n");
		system("pause");
		return(-1);
	}

	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*inputcloud, minPt, maxPt);

	//statisticalOutlierRemovalFilter(inputFileName, outputFileName);
	pathThroughFilter(inputcloud, outputcloud, "x", -0.16, 0.16);
	pathThroughFilter(outputcloud, outputcloud, "y", maxPt.y - 0.06 - 0.3, maxPt.y - 0.06);
	pathThroughFilter(outputcloud, outputcloud, "z", 0.6, 0.9 - 0.15);
	
	statisticalOutlierRemovalFilter(outputcloud, outputcloud, 20, 0.2);

	pcl::io::savePLYFile(outputFileName, *outputcloud);
	return 0;
}
