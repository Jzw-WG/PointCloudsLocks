#include <ICP.h>
#include <iostream>
#include <SAC-IA.h>


using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main()
{
    PointCloud::Ptr result(new PointCloud);
    PointCloud::Ptr source(new PointCloud);
    PointCloud::Ptr target(new PointCloud);
    PointCloud::Ptr source_filterd(new PointCloud);
    PointCloud::Ptr target_filterd(new PointCloud);

    //加载点云
    pcl::io::loadPLYFile("..\\..\\..\\data\\bunny\\data\\bun045.ply", *source);
    pcl::io::loadPLYFile("..\\..\\..\\data\\bunny\\data\\bun090.ply", *target);
    //pcl::io::loadPLYFile("..\\..\\..\\data\\bunny\\reconstruction\\bun_zipper.ply", *target);
    //cout << "/" << endl;
    cout << "原始model点云数量：" << target->size() << endl;
    cout << "原始scene点云数量：" << source->size() << endl;

    eraseInfPoint(target);
    eraseInfPoint(source);

    //滤波
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(0.005, 0.005, 0.005);
    voxel_grid.setInputCloud(source);
    voxel_grid.filter(*source_filterd);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid2;
    voxel_grid2.setLeafSize(0.005, 0.005, 0.005);
    voxel_grid2.setInputCloud(target);
    voxel_grid2.filter(*target_filterd);

    Eigen::Matrix4f sac_trans;
    sac_trans = startSAC_IA(source_filterd, target_filterd, result);
    pcl::transformPointCloud(*source, *source, sac_trans);

    showICPviewer(result, target_filterd, source, target);
    return 0;
}