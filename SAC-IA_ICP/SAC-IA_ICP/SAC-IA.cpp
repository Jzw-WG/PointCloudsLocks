#include <SAC-IA.h>



// sift相关
namespace pcl
{
    template<>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
        inline float
            operator () (const PointXYZ& p) const
        {
            return p.z;
        }
    };
}

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

//估计法线
void est_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals, PointCloud::Ptr origin_cloud)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
    est_normal.setKSearch(16);         //设置k邻域搜索阈值为20个点
    //est_normal.setRadiusSearch(0.02);
    est_normal.setInputCloud(cloud_in);   //设置输入模型点云
    est_normal.setSearchMethod(tree);
    if (origin_cloud != NULL) {
        est_normal.setSearchSurface(origin_cloud);
    }
    
    est_normal.compute(*normals);//计算点云法线
}

//计算SIFT
void compute_sift(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints)
{
    clock_t start = clock();

    const float min_scale = 1;             //设置尺度空间中最小尺度的标准偏差
    const int n_octaves = 6;               //设置高斯金字塔组（octave）的数目
    const int n_scales_per_octave = 6;     //设置每组（octave）计算的尺度
    const float min_contrast = 0.01;          //设置限制关键点检测的阈值

    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;//创建sift关键点检测对象
    pcl::PointCloud<pcl::PointWithScale> result;
    sift.setInputCloud(cloud);//设置输入点云
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    sift.setSearchMethod(tree);//创建一个空的kd树对象tree，并把它传递给sift检测对象
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);//指定搜索关键点的尺度范围
    sift.setMinimumContrast(min_contrast);//设置限制关键点检测的阈值
    sift.compute(result);//执行sift关键点检测，保存结果在result

    copyPointCloud(result, *keypoints);//将点类型pcl::PointWithScale的数据转换为点类型pcl::PointXYZ的数据
    clock_t end = clock();

    cout << "sift关键点提取时间：" << (double)(end - start) / CLOCKS_PER_SEC << endl;
    cout << "sift关键点数量" << keypoints->size() << endl;
}


//计算FPFH
fpfhFeature::Ptr compute_fpfh_feature(PointCloud::Ptr input_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, PointCloud::Ptr origin_cloud = NULL)
{
    //fpfh 估计
    fpfhFeature::Ptr fpfh(new fpfhFeature);
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    est_fpfh.setNumberOfThreads(4); //指定4核计算
    est_fpfh.setInputCloud(input_cloud);
    est_fpfh.setInputNormals(normals);
    est_fpfh.setSearchMethod(tree);
    est_fpfh.setKSearch(16);
    if (origin_cloud != NULL) {
        est_fpfh.setSearchSurface(origin_cloud);
    }
    est_fpfh.compute(*fpfh);

    return fpfh;
}

//计算VFH
pcl::PointCloud<pcl::VFHSignature308>::Ptr computure_vfh_feature(PointCloud::Ptr input_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) {

    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> est_vfh;
    est_vfh.setInputCloud(input_cloud);
    est_vfh.setInputNormals(normals);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//创建一个空的kd树对象，并把它传递给FPFH估计对象。
    est_vfh.setSearchMethod(tree);
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh(new pcl::PointCloud<pcl::VFHSignature308>());//输出数据集
    est_vfh.compute(*vfh);//计算特征值

    return vfh;
}

//FPFH配准
Eigen::Matrix4f FPFHmatch(PointCloud::Ptr source, PointCloud::Ptr target, fpfhFeature::Ptr source_fpfh, fpfhFeature::Ptr target_fpfh) {
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(source);
    sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(target);
    sac_ia.setTargetFeatures(target_fpfh);
    PointCloud::Ptr align(new PointCloud);
    //sac_ia.setMaxCorrespondenceDistance(0.01);
    //sac_ia.setRANSACOutlierRejectionThreshold(0.2);
    //sac_ia.setRANSACIterations(1000);
    sac_ia.setNumberOfSamples(20);
    sac_ia.setCorrespondenceRandomness(20); //设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
    sac_ia.align(*align);

    Eigen::Matrix4f sac_trans;
    sac_trans = sac_ia.getFinalTransformation();
    //使用创建的变换对未过滤的输入点云进行变换
    //pcl::io::savePLYFile("E:\\locks\\data\\result\\fpfh\\FPFH粗配准后的.ply", *align);
    return sac_trans;
}

//计算FPFH对应关系
boost::shared_ptr<pcl::Correspondences> CalculateCorrespondences(fpfhFeature::Ptr source_fpfh, fpfhFeature::Ptr target_fpfh) {
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;

    boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
    crude_cor_est.setInputSource(source_fpfh);
    crude_cor_est.setInputTarget(target_fpfh);

    //  crude_cor_est.determineCorrespondences(cru_correspondences);
    crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences);
    cout << "crude size is:" << cru_correspondences->size() << endl;
    return cru_correspondences;
}

//可视化
void visualizeMatch(PointCloud::Ptr source, PointCloud::Ptr target, PointCloud::Ptr align, boost::shared_ptr<pcl::Correspondences> cru_correspondences) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("fpfh match"));
    int v1;
    int v2;

    view->createViewPort(0, 0.0, 0.5, 1.0, v1);
    view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    view->setBackgroundColor(0, 0, 0, v1);
    view->setBackgroundColor(0, 0, 0, v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(source, 0, 255, 0);
    view->addPointCloud(source, sources_cloud_color, "sources_cloud_v1", v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(target, 255, 0, 0);
    view->addPointCloud(target, target_cloud_color, "target_cloud_v1", v1);
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sources_cloud_v1");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>aligend_cloud_color(align, 0, 255, 0);
    view->addPointCloud(align, aligend_cloud_color, "aligend_cloud_v2", v2);
    view->addPointCloud(target, target_cloud_color, "target_cloud_v2", v2);
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligend_cloud_v2");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v2");

    view->addCorrespondences<pcl::PointXYZ>(source, target, *cru_correspondences, "correspondence", v1);//添加显示对应点对

    while (!view->wasStopped())
    {
        view->spinOnce();
        //boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}



Eigen::Matrix4f startSAC_IA(PointCloud::Ptr source, PointCloud::Ptr target, PointCloud::Ptr source_origin, PointCloud::Ptr target_origin,
    PointCloud::Ptr &result, pcl::PointCloud<pcl::Normal>::Ptr source_normals, pcl::PointCloud<pcl::Normal>::Ptr target_normals)
{
    clock_t start, end, time;
    start = clock();

    fpfhFeature::Ptr source_fpfh(new fpfhFeature());  // fpfh特征
    fpfhFeature::Ptr target_fpfh(new fpfhFeature());

    pcl::PointCloud<pcl::VFHSignature308>::Ptr source_vfh(new pcl::PointCloud<pcl::VFHSignature308>());  // vfh特征
    pcl::PointCloud<pcl::VFHSignature308>::Ptr target_vfh(new pcl::PointCloud<pcl::VFHSignature308>());

    //计算法线
    est_normals(source, source_normals, source_origin);//使用源数据计算法线效果提升明显
    est_normals(target, target_normals, target_origin);

    ////计算SIFT
    //pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints_sift(new pcl::PointCloud<pcl::PointXYZ>);  // sift关键点
    //pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints_sift(new pcl::PointCloud<pcl::PointXYZ>);  // sift关键点
    //compute_sift(source, source_keypoints_sift);
    //compute_sift(target, target_keypoints_sift);

    /*
    //计算VFH
    target_vfh = computure_vfh_feature(target, target_normals);
    pcl::io::savePLYFile("", *target_vfh);
    */
    //计算FPFH
    source_fpfh = compute_fpfh_feature(source, source_normals/*, source_origin*/);
    target_fpfh = compute_fpfh_feature(target, target_normals/*, target_origin*/);

    //FPFH配准
    Eigen::Matrix4f sac_trans;
    sac_trans = FPFHmatch(source, target, source_fpfh, target_fpfh);

    end = clock();
    cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC << endl;

    //计算对应关系
    boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
    cru_correspondences = CalculateCorrespondences(source_fpfh, target_fpfh);

    PointCloud::Ptr sac_ia_result(new PointCloud);
    //使用创建的变换对未过滤的输入点云进行变换
    pcl::transformPointCloud(*source, *sac_ia_result, sac_trans);
    result = sac_ia_result;
    //可视化
    visualizeMatch(source, target, sac_ia_result, cru_correspondences);
    return sac_trans;
}


