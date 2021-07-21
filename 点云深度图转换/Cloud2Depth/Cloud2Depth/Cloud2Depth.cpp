#include <tchar.h> 
#include <boost/thread/thread.hpp> 
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/png_io.h>

#include <pcl/io/obj_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/common.h>
#include <pcl/io/io.h>
#include <pcl/console/parse.h>
#include <pcl/range_image/range_image_planar.h>

typedef pcl::PointXYZ PointType;
//void displayCloudImage(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &points);
//void setViewerPose(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose);

//定义参数
float angular_resolution_x = 0.3f;
float angular_resolution_y = angular_resolution_x;
float angularResolution = (float)(1.0f * (M_PI / 180.0f));//角分辨率是1度，也就是说由相邻像素表示的光束相差一弧度
float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));
float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));//模拟的范围传感器有一个完整的360度全景图
Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);//传感器的位置定义了虚拟传感器的6 DOF位置，它的原点是滚动=俯仰=偏航=0。
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;//x朝向右，y向下，z轴是向前的,另一种选择是激光框架，x面向前方，y向左，z向上。
float noiseLevel = 0.00;//对于噪声，噪声是0，范围图像是使用普通的z缓冲区创建的
float minRange = 0.0f;// minRange >0 ,所有更近的点都将被忽略
int borderSize = 0;//边界大小>0,边界将会在裁剪时留下一个未被观察到的点的边界。

int main()
{
	pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>& pointcloud = *point_cloud_ptr;
	if (pcl::io::loadPLYFile("..\\..\\..\\..\\data\\gen\\handled\\lock_1_000_statistic.ply", pointcloud) == -1)
	{
		PCL_ERROR("Couldn't read PCD file \n");
		return (-1);
	}


	pcl::RangeImage rangeImage;
	// 	rangeImage.createFromPointCloud(pointcloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose,
	// 		coordinate_frame, noiseLevel, minRange, borderSize);
	rangeImage.createFromPointCloud(pointcloud, angularResolution,
		pcl::deg2rad(180.0f), pcl::deg2rad(180.0f),
		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	float* ranges = rangeImage.getRangesArray();
	unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ranges, rangeImage.width, rangeImage.height);
	//pcl::io::saveRgbPNGFile("saveRangeImageRGB.png", rgb_image, rangeImage.width, rangeImage.height);
	pcl::io::saveRgbPNGFile("..\\..\\..\\..\\data\\gen\\handled\\depth\\lock_1_000_statistic.png", rgb_image, rangeImage.width, rangeImage.height);

	system("pause");
	return 0;
}