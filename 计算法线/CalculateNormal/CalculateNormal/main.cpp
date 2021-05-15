#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h> 
#include<ctime>
#include<cstdlib>
#include <windows.h>
#include <pcl/common/centroid.h>//计算点云的重心
#include <pcl/features/normal_3d.h>//计算点云的法线方向
#include <pcl/common/common_headers.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>
#include<pcl/features/principal_curvatures.h>


using namespace pcl;
using namespace std;
typedef PointXYZ PoinT;

int main()
{
	//我算算法线方向，是否一致
	PointCloud<PoinT>::Ptr cloud(new PointCloud<PoinT>);
	if (io::loadPLYFile("D:\\TEST\\plys\\radis-out-40-3.ply", *cloud) == -1)
	{
		PCL_ERROR("read false");
		return 0;
	}
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());    //以八叉树的搜索方式查找点   //前面已定义
	search::KdTree<PoinT>::Ptr tree(new search::KdTree<PoinT>);
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
	ne.setKSearch(20);		//影响法向量准确性的主要因素！
							//ne.setRadisuSearch(0.3);
	ne.compute(*cloud_normals);		//法向量的计算

	//曲率计算
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	pc.setInputCloud(cloud);
	pc.setInputNormals(cloud_normals);
	pc.setSearchMethod(tree);
	pc.setKSearch(20);
	pc.compute(*cloud_curvatures);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("法线 Viewer"));  //设置一个boost共享对象，并分配内存空间
	viewer->setBackgroundColor(0, 0, 0);		//黑色背景色
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud); 
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb(cloud);
	pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> v(cloud);  //该句的意思是：对输入的点云着色，Random表示的是随机上色，上面是其他两种渲染色彩的方式
	viewer->addPointCloud<pcl::PointXYZ>(cloud, v, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");//渲染属性，可视化工具，3维数据
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 15, 15, "normal"); //其中，参数2表示整个点云中每2个点显示一个法向量（若全部显示，可设置为1，可根据自己要求设置）；  0.9表示法向量的长度
	viewer->addPointCloudPrincipalCurvatures<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, cloud_curvatures, 15, 20, "cloud_curvatures");
	
	viewer->addCoordinateSystem(1.0);   //看复杂的点云，经常让人感到没有方向感，为了保持正确的坐标判断，需要显示坐标系统方向，
	//viewer->initCameraParameters();		//照相机初始化，不打开这个会造成无法启动程序
	//viewer->resetCamera();	//相机位置在点云居中重置,不开这个，坐标太大的点有可能找不到

	viewer->spin();
	system("pause");
	return 0;
}