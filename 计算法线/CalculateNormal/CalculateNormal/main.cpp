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
#include <pcl/common/centroid.h>//������Ƶ�����
#include <pcl/features/normal_3d.h>//������Ƶķ��߷���
#include <pcl/common/common_headers.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>
#include<pcl/features/principal_curvatures.h>


using namespace pcl;
using namespace std;
typedef PointXYZ PoinT;

int main()
{
	//�����㷨�߷����Ƿ�һ��
	PointCloud<PoinT>::Ptr cloud(new PointCloud<PoinT>);
	if (io::loadPLYFile("D:\\TEST\\plys\\radis-out-40-3.ply", *cloud) == -1)
	{
		PCL_ERROR("read false");
		return 0;
	}
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());    //�԰˲�����������ʽ���ҵ�   //ǰ���Ѷ���
	search::KdTree<PoinT>::Ptr tree(new search::KdTree<PoinT>);
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
	ne.setKSearch(20);		//Ӱ�취����׼ȷ�Ե���Ҫ���أ�
							//ne.setRadisuSearch(0.3);
	ne.compute(*cloud_normals);		//�������ļ���

	//���ʼ���
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	pc.setInputCloud(cloud);
	pc.setInputNormals(cloud_normals);
	pc.setSearchMethod(tree);
	pc.setKSearch(20);
	pc.compute(*cloud_curvatures);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("���� Viewer"));  //����һ��boost������󣬲������ڴ�ռ�
	viewer->setBackgroundColor(0, 0, 0);		//��ɫ����ɫ
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud); 
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb(cloud);
	pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> v(cloud);  //�þ����˼�ǣ�������ĵ�����ɫ��Random��ʾ���������ɫ������������������Ⱦɫ�ʵķ�ʽ
	viewer->addPointCloud<pcl::PointXYZ>(cloud, v, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");//��Ⱦ���ԣ����ӻ����ߣ�3ά����
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 15, 15, "normal"); //���У�����2��ʾ����������ÿ2������ʾһ������������ȫ����ʾ��������Ϊ1���ɸ����Լ�Ҫ�����ã���  0.9��ʾ�������ĳ���
	viewer->addPointCloudPrincipalCurvatures<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, cloud_curvatures, 15, 20, "cloud_curvatures");
	
	viewer->addCoordinateSystem(1.0);   //�����ӵĵ��ƣ��������˸е�û�з���У�Ϊ�˱�����ȷ�������жϣ���Ҫ��ʾ����ϵͳ����
	//viewer->initCameraParameters();		//�������ʼ�����������������޷���������
	//viewer->resetCamera();	//���λ���ڵ��ƾ�������,�������������̫��ĵ��п����Ҳ���

	viewer->spin();
	system("pause");
	return 0;
}