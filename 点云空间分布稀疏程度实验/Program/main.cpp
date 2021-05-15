#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <ctime>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/fpfh_omp.h> //包含fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/keypoints/sift_keypoint.h>   // shift关键点相关


using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;


double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    double resolution = 0.0;
    int numberOfPoints = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> squaredDistances(2);
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (!pcl_isfinite((*cloud)[i].x))
            continue;

        // Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
        if (nres == 2)
        {
            resolution += sqrt(squaredDistances[1]);
            ++numberOfPoints;
        }
    }
    if (numberOfPoints != 0)
        resolution /= numberOfPoints;

    return resolution;
}

int main(int argc, char** argv) {
    
    pointcloud::Ptr cloud(new pointcloud);
    double resolution = 0;
    /*
    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\1080P\\0.80cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "0.80:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\1080P\\0.85cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "0.85:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\1080P\\0.90cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "0.90:" << resolution << endl;
    
    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\1080P\\0.95cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "0.95:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\1080P\\1.00cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.00:" << resolution << endl;


    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\1080P\\1.05cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.05:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\1080P\\1.10cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.10:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\1080P\\1.15cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.15:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\1080P\\1.20cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.20:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\1080P\\1.25cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.25:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\1080P\\1.30cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.30:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\1080P\\1.35cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.35:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\1080P\\1.40cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.40:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\1080P\\1.45cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.45:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\1080P\\1.50cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.50:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\1080P\\1.55cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.55:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\1080P\\1.60cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.60:" << resolution << endl;
    */

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\720P\\0.90cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "0.90:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\720P\\0.95cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "0.95:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\720P\\1.00cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.00:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\720P\\1.05cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.05:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\720P\\1.10cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.10:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\720P\\1.15cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.15:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\720P\\1.20cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.20:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\720P\\1.25cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.25:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\720P\\1.30cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.30:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\720P\\1.35cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.35:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\720P\\1.40cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.40:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\720P\\1.45cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.45:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\720P\\1.50cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.50:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\720P\\1.55cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.55:" << resolution << endl;

    pcl::io::loadPLYFile("C:\\Users\\ZhuRuiHong\\Desktop\\分辨率测试\\720P\\1.60cropped.ply", *cloud);
    resolution = computeCloudResolution(cloud);
    cout << "1.60:" << resolution << endl;
}