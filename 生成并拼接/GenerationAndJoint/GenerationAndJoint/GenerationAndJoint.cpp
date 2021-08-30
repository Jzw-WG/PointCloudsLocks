#include <ICP.h>
#include <iostream>
#include <SAC-IA.h>
#include <PointCloudFilters.h>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void visualViewer(PointCloud::Ptr leftCloud, PointCloud::Ptr rightCloud) {
    // 可视化ICP的过程与结果
    pcl::visualization::PCLVisualizer viewer("ICPTrans demo");
    // 创建两个观察视点
    int v1(0);
    int v2(1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    // 定义显示的颜色信息
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // 原始的点云设置为白色的
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(NULL, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
        (int)255 * txt_gray_lvl);
    viewer.addPointCloud(leftCloud, cloud_in_color_h, "origin_joint", v1);//设置原始的点云都是显示为白色

    // ICP配准后的点云为红色
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(NULL, 180, 20, 20);
    viewer.addPointCloud(rightCloud, cloud_icp_color_h, "result_joint", v2);

    // 加入文本的描述在各自的视口界面
   //在指定视口viewport=v1添加字符串“white 。。。”其中"icp_info_1"是添加字符串的ID标志，（10，15）为坐标16为字符大小 后面分别是RGB值
    viewer.addText("Original point clouds", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText("ICP aligned joined point clouds", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

    // 设置背景颜色
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // 设置相机的坐标和方向
    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize(1280, 1024);  // 可视化窗口的大小

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        //boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
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
    vector<PointCloud::Ptr> sourceList;
    vector<PointCloud::Ptr> source_filteredList;
    PointCloud::Ptr result_joint(new PointCloud);
    PointCloud::Ptr result_joint_filtered(new PointCloud);
    PointCloud::Ptr origin_joint(new PointCloud);

    //加载点云
    PointCloud::Ptr source1(new PointCloud);
    pcl::io::loadPLYFile("..\\..\\..\\..\\data\\gen\\handled\\lock_1_000-3.ply", *source1);
    sourceList.push_back(source1);
    *origin_joint += *source1;
    PointCloud::Ptr source2(new PointCloud);
    pcl::io::loadPLYFile("..\\..\\..\\..\\data\\gen\\handled\\lock_1_045-3.ply", *source2);
    sourceList.push_back(source2);
    *origin_joint += *source2;
    PointCloud::Ptr source3(new PointCloud);
    pcl::io::loadPLYFile("..\\..\\..\\..\\data\\gen\\handled\\lock_1_090-3.ply", *source3);
    sourceList.push_back(source3);
    *origin_joint += *source3;
    PointCloud::Ptr source4(new PointCloud);
    pcl::io::loadPLYFile("..\\..\\..\\..\\data\\gen\\handled\\lock_1_135-3.ply", *source4);
    sourceList.push_back(source4);
    *origin_joint += *source4;
    PointCloud::Ptr source5(new PointCloud);
    pcl::io::loadPLYFile("..\\..\\..\\..\\data\\gen\\handled\\lock_1_180-3.ply", *source5);
    sourceList.push_back(source5);
    *origin_joint += *source5;
    PointCloud::Ptr source6(new PointCloud);
    pcl::io::loadPLYFile("..\\..\\..\\..\\data\\gen\\handled\\lock_1_225-3.ply", *source6);
    sourceList.push_back(source6);
    *origin_joint += *source6;
    PointCloud::Ptr source7(new PointCloud);
    pcl::io::loadPLYFile("..\\..\\..\\..\\data\\gen\\handled\\lock_1_270-3.ply", *source7);
    sourceList.push_back(source7);
    *origin_joint += *source7;
    PointCloud::Ptr source8(new PointCloud);
    pcl::io::loadPLYFile("..\\..\\..\\..\\data\\gen\\handled\\lock_1_315-3.ply", *source8);
    sourceList.push_back(source8);
    *origin_joint += *source8;
    //pcl::io::loadPLYFile("..\\..\\..\\data\\bunny\\reconstruction\\bun_zipper.ply", *target);
    //cout << "/" << endl;
    clock_t start, end, time;
    start = clock();
    for (int i = 0; i < sourceList.size(); i++)
    {
        PointCloud::Ptr source_filtered(new PointCloud);
        eraseInfPoint(sourceList[i]);
        cout << "原始点云数量：" << sourceList[i]->size() << endl;
        //滤波
        voxelFilter(sourceList[i], source_filtered, 0.005, 0.005, 0.005);
        cout << "滤波后点云数量：" << source_filtered->size() << endl;
        source_filteredList.push_back(source_filtered);
    }
    end = clock();
    cout << "read and filter time is: " << float(end - start) / CLOCKS_PER_SEC << endl;

    Eigen::Matrix4f icp_trans = Eigen::Matrix4f::Identity();
    *result_joint = *sourceList[0];
    *result_joint_filtered = *source_filteredList[0];
    for (int i = 1; i < sourceList.size(); i++)
    {
        PointCloud::Ptr result_sac(new PointCloud);
        PointCloud::Ptr result_rot(new PointCloud);
        PointCloud::Ptr result_icp(new PointCloud);
        PointCloud::Ptr result_icp_filtered(new PointCloud);
        PointCloud::Ptr source = sourceList[i];
        PointCloud::Ptr target = sourceList[i - 1];
        PointCloud::Ptr source_filtered = source_filteredList[i];
        PointCloud::Ptr target_filtered = source_filteredList[i - 1];
        pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>());
        pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>());


        Eigen::Matrix4f rot_trans;
        Eigen::Vector3f point(0, 0.1, 0.7);//根据具体模型位置设置，当前为手动测量值大致值，后续可能通过点云计算
        Eigen::Vector3f direction(0, 1, 0);//同上，交换源和目标顺序需要改变旋转方向
        rot_trans = rot_mat(point, direction, M_PI * i / 4);
        pcl::transformPointCloud(*source_filtered, *result_rot, rot_trans);
        pcl::transformPointCloud(*result_rot, *result_rot, icp_trans);
        pcl::transformPointCloud(*source, *source, rot_trans);
        pcl::transformPointCloud(*source, *source, icp_trans);
        pcl::transformPointCloud(*source_filtered, *source_filtered, rot_trans);
        pcl::transformPointCloud(*source_filtered, *source_filtered, icp_trans);

        //Eigen::Matrix4f sac_trans;
        //sac_trans = startSAC_IA(source_filtered, target_filtered, source, target, result_sac, source_normals, target_normals);//会提取已计算的法线
        //pcl::transformPointCloud(*source_filtered, *result_rot, sac_trans);
        //pcl::transformPointCloud(*source, *source, sac_trans);
        //pcl::transformPointCloud(*source_filtered, *source_filtered, sac_trans);

        Eigen::Matrix4f pre_icp_trans = Eigen::Matrix4f::Identity();
        pre_icp_trans = icp_trans;
        icp_trans = ICPTrans(result_rot, target_filtered, source, target, result_icp, result_icp_filtered, 100);
        icp_trans = pre_icp_trans * icp_trans;
        *result_joint += *result_icp;
        *result_joint_filtered += *result_icp_filtered;
        *sourceList[i] = *result_icp;
        *source_filteredList[i] = *result_icp_filtered;

        voxelFilter(result_joint, result_joint, 0.005, 0.005, 0.005);
        //visualViewer(origin_joint, result_joint);
    }
    radiusFilter(result_joint, result_joint, 0.005, 4);
    pcl::io::savePLYFile("..\\..\\..\\..\\data\\gen\\model\\lock_1_model.ply", *result_joint);
    visualViewer(origin_joint, result_joint);
    return 0;
}