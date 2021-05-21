#include <ICP.h>
#include <iostream>
#include <SAC-IA.h>


using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void voxelFilter(PointCloud::Ptr cloud, PointCloud::Ptr cloud_filtered, float lx, float ly, float lz)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(lx, ly, lz);
    voxel_grid.setInputCloud(cloud);
    voxel_grid.filter(*cloud_filtered);
}

int main()
{
    vector<PointCloud::Ptr> sourceList;
    vector<PointCloud::Ptr> source_filteredList;
    PointCloud::Ptr result_joint(new PointCloud);
    PointCloud::Ptr origin_joint(new PointCloud);

    //加载点云
    PointCloud::Ptr source1(new PointCloud);
    pcl::io::loadPLYFile("..\\..\\..\\..\\data\\bunny\\data\\bun045.ply", *source1);
    sourceList.push_back(source1);
    *origin_joint += *source1;
    PointCloud::Ptr source2(new PointCloud);
    pcl::io::loadPLYFile("..\\..\\..\\..\\data\\bunny\\data\\bun090.ply", *source2);
    sourceList.push_back(source2);
    *origin_joint += *source2;
    PointCloud::Ptr source3(new PointCloud);
    pcl::io::loadPLYFile("..\\..\\..\\..\\data\\bunny\\data\\bun045.ply", *source3);
    sourceList.push_back(source3);
    *origin_joint += *source3;
    PointCloud::Ptr source4(new PointCloud);
    pcl::io::loadPLYFile("..\\..\\..\\..\\data\\bunny\\data\\bun090.ply", *source4);
    sourceList.push_back(source4);
    *origin_joint += *source4;
    PointCloud::Ptr source5(new PointCloud);
    pcl::io::loadPLYFile("..\\..\\..\\..\\data\\bunny\\data\\bun045.ply", *source5);
    sourceList.push_back(source5);
    *origin_joint += *source5;
    PointCloud::Ptr source6(new PointCloud);
    pcl::io::loadPLYFile("..\\..\\..\\..\\data\\bunny\\data\\bun090.ply", *source6);
    sourceList.push_back(source6);
    *origin_joint += *source6;
    PointCloud::Ptr source7(new PointCloud);
    pcl::io::loadPLYFile("..\\..\\..\\..\\data\\bunny\\data\\bun045.ply", *source7);
    sourceList.push_back(source7);
    *origin_joint += *source7;
    PointCloud::Ptr source8(new PointCloud);
    pcl::io::loadPLYFile("..\\..\\..\\..\\data\\bunny\\data\\bun090.ply", *source8);
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

    PointCloud::Ptr result_sac(new PointCloud);
    PointCloud::Ptr result_icp(new PointCloud);
    *result_joint = *sourceList[0];
    for (int i = 1; i < sourceList.size(); i++)
    {
        PointCloud::Ptr source = sourceList[i];
        PointCloud::Ptr target = sourceList[i - 1];
        PointCloud::Ptr source_filtered = source_filteredList[i];
        PointCloud::Ptr target_filtered = source_filteredList[i - 1];
        pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>());
        pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>());

        Eigen::Matrix4f sac_trans;
        sac_trans = startSAC_IA(source_filtered, target_filtered, source, target, result_sac, source_normals, target_normals);//会提取已计算的法线

        pcl::transformPointCloud(*source, *source, sac_trans);

        ICPTrans(result_sac, target_filtered, source, target, result_icp, 40);
        *result_joint += *result_icp;
    }
    

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
    viewer.addPointCloud(origin_joint, cloud_in_color_h, "origin_joint", v1);//设置原始的点云都是显示为白色

    // ICP配准后的点云为红色
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(NULL, 180, 20, 20);
    viewer.addPointCloud(result_joint, cloud_icp_color_h, "result_joint", v2);

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
    return 0;
}