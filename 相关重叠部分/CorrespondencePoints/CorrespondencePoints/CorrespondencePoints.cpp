#include<pcl/registration/correspondence_estimation.h>
#include<pcl/io/ply_io.h>
#include<pcl/kdtree/io.h>
#include<vector>
#include<fstream>

using namespace std;

//可视化
void visualizeMatch(PointCloud::Ptr source, PointCloud::Ptr target, PointCloud::Ptr align, boost::shared_ptr<pcl::Correspondences> cru_correspondences) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("fpfh match"));
	int v1;
	int v2;

	view->createViewPort(0, 0.0, 0.5, 1.0, v1);
	view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	view->setBackgroundColor(0, 0, 0, v1);
	view->setBackgroundColor(0, 0, 0, v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(source, 0, 255, 0);
	view->addPointCloud(source, sources_cloud_color, "sources_cloud_v1", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(target, 255, 0, 0);
	view->addPointCloud(target, target_cloud_color, "target_cloud_v1", v1);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sources_cloud_v1");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>aligend_cloud_color(align, 0, 255, 0);
	view->addPointCloud(align, aligend_cloud_color, "aligend_cloud_v2", v2);
	view->addPointCloud(target, target_cloud_color, "target_cloud_v2", v2);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligend_cloud_v2");
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v2");

	//view->addCorrespondences<pcl::PointXYZ>(source, target, *cru_correspondences, "correspondence", v1);//添加显示对应点对

	while (!view->wasStopped())
	{
		view->spinOnce();
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

int main()
{
	//ofstream oa, ob;
	//oa.open("overlap_a.txt");
	//ob.open("overlap_b.txt");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);

	//读取PCD点云文件
	pcl::io::loadPLYFile<pcl::PointXYZ>("..\\..\\..\\..\\data\\bunny\\data\\bun045.ply", *cloudA);
	pcl::io::loadPLYFile<pcl::PointXYZ>("..\\..\\..\\..\\data\\bunny\\data\\bun090.ply", *cloudB);

	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core;
	core.setInputSource(cloudA);
	core.setInputTarget(cloudB);

	boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);   //共享所有权的智能指针，以kdtree做索引

	core.determineReciprocalCorrespondences(*cor, 0.25);   //点之间的最大距离,cor对应索引

	//构造重叠点云的PCD格式文件
	pcl::PointCloud<pcl::PointXYZ>overlapA;
	pcl::PointCloud<pcl::PointXYZ>overlapB;

	overlapA.width = cor->size();
	overlapA.height = 1;
	overlapA.is_dense = false;
	overlapA.points.resize(overlapA.width * overlapA.height);

	overlapB.width = cor->size();
	overlapB.height = 1;
	overlapB.is_dense = false;
	overlapB.points.resize(overlapB.width * overlapB.height);

	//for (size_t i = 0; i < cor->size(); i++) {
	//	//overlapA写入txt文件
	//	oa << cloudA->points[cor->at(i).index_query].x << " " << cloudA->points[cor->at(i).index_query].y << " "      //cor->at(i).index_query]对应点的索引
	//		<< cloudA->points[cor->at(i).index_query].z << endl;

	//	//overlapA写入pcd文件
	//	overlapA.points[i].x = cloudA->points[cor->at(i).index_query].x;
	//	overlapA.points[i].y = cloudA->points[cor->at(i).index_query].y;
	//	overlapA.points[i].z = cloudA->points[cor->at(i).index_query].z;

	//	//overlapB写入txt文件
	//	ob << cloudB->points[cor->at(i).index_match].x << " " << cloudB->points[cor->at(i).index_match].y << " "
	//		<< cloudB->points[cor->at(i).index_match].z << endl;

	//	//overlapB写入pcd文件
	//	overlapB.points[i].x = cloudB->points[cor->at(i).index_match].x;
	//	overlapB.points[i].y = cloudB->points[cor->at(i).index_match].y;
	//	overlapB.points[i].z = cloudB->points[cor->at(i).index_match].z;
	//}
	////存储pcd文件
	//pcl::io::savePCDFileASCII("overlapA.pcd", overlapA);
	//pcl::io::savePCDFileASCII("overlapB.pcd", overlapB);

	//oa, ob.close();
	return 0;
}