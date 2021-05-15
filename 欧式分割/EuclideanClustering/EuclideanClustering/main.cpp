#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

int color_bar[][3] =
{
    { 255,0,0},
    { 0,255,0 },
    { 0,0,255 },
    { 0,255,255 },
    { 255,255,0 },
    { 255,255,255 },
    { 255,0,255 }
};

void eraseInfPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
    //去除NaN点 防止compute报错
    pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_in->points.begin();
    while (it != cloud_in->points.end())
    {
        float x, y, z;
        x = it->x;
        y = it->y;
        z = it->z;
        if (!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z))
        {
            it = cloud_in->points.erase(it);
        }
        else
            ++it;
    }
}

int main(int argc, char** argv)
{
    // Read in the cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPLYFile<pcl::PointXYZ>("..\\..\\..\\..\\data\\bunny\\data\\bun000.ply", *cloud) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }

    eraseInfPoint(cloud);

    std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl; 
    pcl::PCDWriter writer;
    
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.002, 0.002, 0.002);
    vg.filter(*cloud);
    std::cout << "PointCloud after filtering has: " << cloud->points.size() << " data points." << std::endl; //*
    
    /*
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(2);
    
    int i = 0, nr_points = (int)cloud_filtered->points.size();
    while (cloud_filtered->points.size() > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Write the planar inliers to disk
        extract.filter(*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_filtered = cloud_f;
    }
    */
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.002); //设置近邻搜索的搜索半径为2cm
    ec.setMinClusterSize(300);    //设置一个聚类需要的最少点数目为100
    ec.setMaxClusterSize(10000);  //设置一个聚类需要的最大点数目为25000
    ec.setSearchMethod(tree);     //设置点云的搜索机制
    ec.setInputCloud(cloud); //设置原始点云 
    ec.extract(cluster_indices);  //从点云中提取聚类

                                   // 可视化部分
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("segmention"));  //设置一个boost共享对象，并分配内存空间
    // 我们将要使用的颜色
    float bckgr_gray_level = 0.0;  // 黑色
    float txt_gray_lvl = 1.0 - bckgr_gray_level;
    int num = cluster_indices.size();

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
            cloud_cluster->points.push_back(cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
        //std::stringstream ss;
        //ss << "cloud_cluster_" << j << ".pcd";
        //writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
        
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(cloud,
            color_bar[j][0],
            color_bar[j][1],
            color_bar[j][2]);//赋予显示点云的颜色
        viewer->addPointCloud(cloud_cluster, cloud_in_color_h, std::to_string(j));
        
        j++;
    }

    viewer->spin();

    return (0);
}