#include <ICP.h>

bool next_iteration = false;
bool icp_finished = false;

void print4x4Matrix(const Eigen::Matrix4f& matrix)    //打印旋转矩阵和平移矩阵
{
    printf("Rotation matrix :\n");
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

Eigen::Matrix4f ICPTrans(PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_tar, PointCloudT::Ptr cloud_in_origin, PointCloudT::Ptr cloud_tar_origin, PointCloudT::Ptr result, PointCloudT::Ptr result_filtered, int iterations) // 默认的ICP迭代次数
{
    // 申明点云将要使用的
    PointCloudT::Ptr cloud_tr(new PointCloudT);  // 转换后的点云
    PointCloudT::Ptr cloud_icp(new PointCloudT);  // ICP 输出点云
    PointCloudT::Ptr cloud_icp_origin(new PointCloudT);  // ICP 输出点云

    pcl::console::TicToc time;     //申明时间记录
    time.tic();       //time.tic开始  time.toc结束时间
    
    //定义旋转矩阵和平移向量Matrix4d是为4*4的矩阵
    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();  //初始化

    //// 旋转矩阵的定义可以参考 ( https://en.wikipedia.org/wiki/Rotation_matrix)
    //double theta = M_PI / 8;  // 旋转的角度用弧度的表示方法
    //transformation_matrix(0, 0) = cos(theta);
    //transformation_matrix(0, 1) = -sin(theta);
    //transformation_matrix(1, 0) = sin(theta);
    //transformation_matrix(1, 1) = cos(theta);

    //// Z轴的平移向量 (0.4 meters)
    //transformation_matrix(2, 3) = 0.4;

    ////打印转换矩阵
    //std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
    //print4x4Matrix(transformation_matrix);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // //执行点云转换
    //pcl::transformPointCloud(*cloud_in, *cloud_icp, transformation_matrix);
    //*cloud_tr = *cloud_icp;  // 备份cloud_icp赋值给cloud_tr为后期使用
    *cloud_icp = *cloud_in;
    *cloud_tr = *cloud_in_origin;
    *cloud_icp_origin = *cloud_in_origin;
    // 迭代最近点算法
    time.tic();        //时间
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(iterations);    //设置最大迭代次数iterations=true
    icp.setInputSource(cloud_icp);   //设置输入的点云
    icp.setInputTarget(cloud_tar);    //目标点云
    icp.setMaxCorrespondenceDistance(0.005);
    icp.align(*cloud_icp);          //匹配后源点云
    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

    if (icp.hasConverged())//icp.hasConverged ()=1（true）输出变换矩阵的适合性评估
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
        std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix = icp.getFinalTransformation();
        pcl::transformPointCloud(*cloud_in_origin, *cloud_icp_origin, transformation_matrix);
        print4x4Matrix(transformation_matrix);
    }
    else
    {
        PCL_ERROR("\nICP has not converged.\n");
        return transformation_matrix;
    }
    *result_filtered = *cloud_icp;
    *result = *cloud_icp_origin;

    return transformation_matrix;
}