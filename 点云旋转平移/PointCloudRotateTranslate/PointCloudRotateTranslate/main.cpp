/*
 * 功能： 点云刚体变换
 * 头文件： #include <pcl/common/transforms.h>
 * 功能函数： pcl::transformPointCloud(*pPointCloudIn, *pPointCloudOut, transform_1);
 */


#include <iostream>
 // pcl
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

int main()
{
    // 1. 读入点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloudIn(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::io::loadPLYFile("D:\\研究生毕业DOC\\实验点云\\体素滤波输出_工件2_模型.ply", *pPointCloudIn);

    // 2. 可视窗口初始化
    pcl::visualization::PCLVisualizer viewer;
    viewer.setCameraFieldOfView(0.785398);      // fov 大概45度
    viewer.setBackgroundColor(0.5, 0.5, 0.5);   // 背景设为灰色
    viewer.setCameraPosition(
        0, 0, 5,                                // camera位置
        0, 0, -1,                               // view向量--相机朝向
        0, 1, 0                                 // up向量
    );

    // 3. 点云旋转
    /* 提示: 变换矩阵工作原理 :

    |-------> 这一列表示平移向量
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> 左侧的 3x3 matrix表示旋转，这里用无旋转举例
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> 这行没有实际意义只是为了计算补充的 (始终为 0,0,0,1)

    方法 #1: 使用一个Matrix4f类型矩阵（f代表float）
    这是手工方式，易于理解但是容易出错！
    */
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();      // 单位阵

    // 定义一个旋转矩阵表示绕z轴转PI/4 (从这里可以查看旋转矩阵 https://en.wikipedia.org/wiki/Rotation_matrix)
    //float theta = M_PI / 6; // 旋转的角度，以弧度为单位
    float theta = 0;
    double s = 1.5;
    transform_1(0, 0) = s * cos(theta);
    transform_1(0, 1) = s* (-sin(theta));
    transform_1(1, 0) = s * sin(theta);
    transform_1(1, 1) = s * cos(theta);
    // 圆括号内的意义(元素在矩阵中的行, 元素在矩阵中的列)

    // 定义沿着x轴平移2.5m
    //transform_1(0, 3) = 2.5;

    // 打印变换矩阵
    printf("Method #1: using a Matrix4f\n");
    std::cout << transform_1 << std::endl;

    // 执行变换
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloudOut(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*pPointCloudIn, *pPointCloudOut, transform_1);

    //保存变换后的点云
    pcl::io::savePLYFile("D:\\研究生毕业DOC\\实验点云\\体素滤波输出_工件2_模型_加尺度_1.5.ply",*pPointCloudOut);

    // 4. 点云可视化
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> inColorHandler(pPointCloudIn, 255, 255, 255);// 白色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> outColorHandler(pPointCloudOut, 230, 20, 20); // 红色
    //viewer.addPointCloud(pPointCloudIn, inColorHandler, "In");
    viewer.addPointCloud(pPointCloudOut, outColorHandler, "Out");
    viewer.addCoordinateSystem(1.0, "cloud", 0);
    while (!viewer.wasStopped()) { // 显示，直到‘q’键被按下
        viewer.spinOnce();
    }

    std::system("pause");
    return 0;
}