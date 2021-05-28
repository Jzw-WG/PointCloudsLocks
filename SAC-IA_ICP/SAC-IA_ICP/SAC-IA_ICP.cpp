#include <ICP.h>
#include <iostream>
#include <SAC-IA.h>


using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void voxelFilter(PointCloud::Ptr cloud, PointCloud::Ptr cloud_filtered, float lx, float ly, float lz)
{
    if (lx == 0 || ly == 0 || lz == 0) {
        *cloud_filtered = *cloud;
        return;
    }
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(lx, ly, lz);
    voxel_grid.setInputCloud(cloud);
    voxel_grid.filter(*cloud_filtered);
}

int main()
{
    PointCloud::Ptr result(new PointCloud);
    PointCloud::Ptr source(new PointCloud);
    PointCloud::Ptr target(new PointCloud);
    PointCloud::Ptr source_filtered(new PointCloud);
    PointCloud::Ptr target_filtered(new PointCloud);

    //加载点云
    //pcl::io::loadPLYFile("..\\..\\..\\data\\bunny\\data\\bun090.ply", *source);//TODO:交换顺序结果不一致？？？？
    //pcl::io::loadPLYFile("..\\..\\..\\data\\bunny\\data\\bun000.ply", *target);
    pcl::io::loadPLYFile("..\\..\\..\\data\\gen\\handled\\lock_1_045-2.ply", *source);
    pcl::io::loadPLYFile("..\\..\\..\\data\\gen\\handled\\lock_1_000-2.ply", *target);
    //pcl::io::loadPLYFile("..\\..\\..\\data\\bunny\\reconstruction\\bun_zipper.ply", *source);
    //cout << "/" << endl;

    eraseInfPoint(target);
    eraseInfPoint(source);

    cout << "原始model点云数量：" << target->size() << endl;
    cout << "原始scene点云数量：" << source->size() << endl;

    //滤波
    //voxelFilter(source, source_filtered, 0, 0, 0);
    //voxelFilter(target, target_filtered, 0, 0, 0);
    voxelFilter(source, source_filtered, 0.01, 0.01, 0.01);
    voxelFilter(target, target_filtered, 0.01, 0.01, 0.01);

    cout << "滤波后model点云数量：" << target_filtered->size() << endl;
    cout << "滤波后scene点云数量：" << source_filtered->size() << endl;

    pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>());

    Eigen::Matrix4f sac_trans;
    sac_trans = startSAC_IA(source_filtered, target_filtered, source, target, result, source_normals, target_normals);//会提取已计算的法线
    
    pcl::transformPointCloud(*source, *source, sac_trans);

    showICPviewer(result, target_filtered, source, target);
    return 0;
}