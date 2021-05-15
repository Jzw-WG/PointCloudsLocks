#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>//点云查看窗口头文件
#include <boost/random.hpp>
#include <pcl/console/time.h>

void addGaussnoise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr icloud, size_t miu, size_t sigma)
{
    icloud->points.resize(cloud->points.size());//将点云的cloud的size赋值给噪声 
    icloud->header = cloud->header;
    icloud->width = cloud->width;
    icloud->height = cloud->height;

    boost::mt19937 zhongzi;
    zhongzi.seed(static_cast<unsigned int>(time(0)));
    boost::normal_distribution<> nd(miu, sigma);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<>> ok(zhongzi, nd);
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        icloud->points[i].x = cloud->points[i].x + static_cast<float>(ok());
        icloud->points[i].y = cloud->points[i].y + static_cast<float>(ok());
        icloud->points[i].z = cloud->points[i].z + static_cast<float>(ok());
    }
    return;
}
int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）
    pcl::io::loadPLYFile<pcl::PointXYZ>("D:\\研究生毕业DOC\\实验点云\\体素滤波输出_工件2_模型.ply", *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfinal(new pcl::PointCloud<pcl::PointXYZ>);
    size_t a = 0;
    size_t b = 1;
    addGaussnoise(cloud, cloudfinal, 0, 1);
    pcl::io::savePLYFile<pcl::PointXYZ>("D:\\研究生毕业DOC\\实验点云\\体素滤波输出_工件2_模型_加噪声.ply", *cloudfinal);
}

