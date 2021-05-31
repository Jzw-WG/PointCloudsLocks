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

Eigen::Matrix4f rot_mat(const Eigen::Vector3f& point, const Eigen::Vector3f& vector, const float t)
{
    float u = vector(0);
    float v = vector(1);
    float w = vector(2);
    float a = point(0);
    float b = point(1);
    float c = point(2);

    Eigen::Matrix4f matrix;
    matrix << u * u + (v * v + w * w) * cos(t), u* v* (1 - cos(t)) - w * sin(t), u* w* (1 - cos(t)) + v * sin(t), (a * (v * v + w * w) - u * (b * v + c * w))* (1 - cos(t)) + (b * w - c * v) * sin(t),
        u* v* (1 - cos(t)) + w * sin(t), v* v + (u * u + w * w) * cos(t), v* w* (1 - cos(t)) - u * sin(t), (b * (u * u + w * w) - v * (a * u + c * w))* (1 - cos(t)) + (c * u - a * w) * sin(t),
        u* w* (1 - cos(t)) - v * sin(t), v* w* (1 - cos(t)) + u * sin(t), w* w + (u * u + v * v) * cos(t), (c * (u * u + v * v) - w * (a * u + b * v))* (1 - cos(t)) + (a * v - b * u) * sin(t),
        0, 0, 0, 1;
    return matrix;
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

    cout << "原始model点云数量:" << target->size() << endl;
    cout << "原始scene点云数量:" << source->size() << endl;

    //滤波
    //voxelFilter(source, source_filtered, 0, 0, 0);
    //voxelFilter(target, target_filtered, 0, 0, 0);
    voxelFilter(source, source_filtered, 0.005, 0.005, 0.005);
    voxelFilter(target, target_filtered, 0.005, 0.005, 0.005);

    cout << "滤波后model点云数量:" << target_filtered->size() << endl;
    cout << "滤波后scene点云数量:" << source_filtered->size() << endl;

    pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>());

    Eigen::Matrix4f rot_trans;
    Eigen::Vector3f point(0, 0.1, 0.7);
    Eigen::Vector3f direction(0, -1, 0);
    rot_trans = rot_mat(point, direction, M_PI / 4);
    pcl::transformPointCloud(*source_filtered, *result, rot_trans);
    pcl::transformPointCloud(*source, *source, rot_trans);

    //Eigen::Matrix4f sac_trans;

    //sac_trans = startSAC_IA(source_filtered, target_filtered, source, target, result, source_normals, target_normals);//会提取已计算的法线
    
    //pcl::transformPointCloud(*source, *source, sac_trans);

    showICPviewer(result, target_filtered, source, target);
    return 0;
}