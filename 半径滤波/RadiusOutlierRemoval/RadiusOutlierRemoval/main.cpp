#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>  //�˲����
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
#include <fstream>//д��txt
#include<string>
#include <stdlib.h>//������ת�����ַ���
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/convolution.h>
#include <pcl/surface/mls.h>

using namespace std;  // ���Լ��� std �������ռ�
//�����˲�
int main(int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("D:\\�о�����ҵDOC\\ʵ�����\\����2_ģ��.ply", *cloud) == -1) {
		PCL_ERROR("Couldnot read file.\n");
		system("pause");
		return(-1);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);//����
	sor.setLeafSize(4,4,4);//�ֱ���,ԽСԽ��,�����ֱ���xyz
	sor.filter(*outputcloud);//���

	pcl::io::savePLYFile("D:\\�о�����ҵDOC\\ʵ�����\\�����˲����_����2_ģ��_4mm.ply", *outputcloud);
}

/*
* �ƶ���С���˷�
int main(int argc, char** argv)
{// ��һ���ʵ����͵������ļ����ص�����PointCloud��
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	// ����bun0.pcd�ļ������ص��ļ��� PCL�Ĳ����������Ǵ��ڵ� 
	pcl::io::loadPLYFile("D:\\�о�����ҵDOC\\ʵ�����\\�뾶�˲����30-2-�������.ply", *cloud);
	// ����һ��KD��
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	// ����ļ�����PointNormal���ͣ������洢�ƶ���С���˷�����ķ���
	pcl::PointCloud<pcl::PointNormal> mls_points;
	// ������� (�ڶ��ֶ���������Ϊ�˴洢����, ��ʹ�ò���Ҳ��Ҫ�������)
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;
	mls.setComputeNormals(true);
	//���ò���
	mls.setInputCloud(cloud);
	mls.setPolynomialOrder(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(4);
	// �����ؽ�
	mls.process(mls_points);
	// ������
	pcl::io::savePCDFile("D:\\�о�����ҵDOC\\ʵ�����\\test.pcd", mls_points);

	pcl::io::loadPCDFile("D:\\�о�����ҵDOC\\ʵ�����\\test.pcd", *cloud);
	pcl::io::savePLYFile("D:\\�о�����ҵDOC\\ʵ�����\\test.ply", *cloud);
}
*/
/*
* ˫��
int main() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("D:\\�о�����ҵDOC\\ʵ�����\\�뾶�˲����30-2.ply", *cloud) == -1) {
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
	//bf.setSigmaS(5);//����˫���˲������ڿռ�����/���ڵĸ�˹�ı�׼ƫ��
	//bf.setSigmaR(0.003);//���ø�˹�ı�׼ƫ�����ڿ���������������ǿ�Ȳ�����½����٣������ǵ������Ϊ��ȣ�
	bf.filter(outcloud);

	// �����˲���������ļ�  
	pcl::io::savePLYFile("D:\\�о�����ҵDOC\\ʵ�����\\˫���˲����5-0.3.ply", outcloud);
	return (0);
}
*/
/*
* ��˹
int main(int argc, char** argv) {
	//Create the input and filtered cloud objects
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	//Read in the input file
	if (pcl::io::loadPLYFile("D:\\�о�����ҵDOC\\ʵ�����\\�뾶�˲����30-2.ply", *inputcloud) == -1)
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

	pcl::io::savePLYFile("D:\\�о�����ҵDOC\\ʵ�����\\��˹�˲����20-2.ply", *outputcloud);
}
*/
/*
* ��ֵ
int main(int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("D:\\�о�����ҵDOC\\ʵ�����\\�뾶�˲����30-2.ply", *cloud) == -1) {
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

	pcl::io::savePLYFile("D:\\�о�����ҵDOC\\ʵ�����\\��ֵ�˲����20-2.ply", *cloud_out);
}
*/


/*
int main(int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter2(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("D:\\�о�����ҵDOC\\ʵ�����\\ֱͨ�˲���ĵ���1.ply", *cloud) == -1) {
		PCL_ERROR("Couldnot read file.\n");
		system("pause");
		return(-1);
	}
	

	//ͳ���˲�
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(20); //K�������������
	sor.setStddevMulThresh(1.0); //��׼���
	sor.setNegative(false); //����δ�˲��㣨�ڵ㣩
	sor.filter(*cloud_filter2);  //�����˲������cloud_filter

	pcl::io::savePLYFile("D:\\�о�����ҵDOC\\ʵ�����\\ͳ���˲����20-1.ply", *cloud_filter2);
	
	
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("D:\\�о�����ҵDOC\\ʵ�����\\ͳ���˲����20-2.ply", *cloud_filter2) == -1) {
		PCL_ERROR("Couldnot read file.\n");
		system("pause");
		return(-1);
	}
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;//�����뾶�˲�������
	outrem.setInputCloud(cloud_filter2);			//�����������
	outrem.setRadiusSearch(2);					//���ð뾶Ϊ4cm
	outrem.setMinNeighborsInRadius(2);				//������С�ڽӵ������ֵ,�뾶��Χ���������������5�ĵ㽫���˳�
	outrem.filter(*cloud_filter1);				//ִ���˲�

	pcl::io::savePLYFile("D:\\�о�����ҵDOC\\ʵ�����\\�뾶�˲����20-2.ply", *cloud_filter1);
	
}
*/
