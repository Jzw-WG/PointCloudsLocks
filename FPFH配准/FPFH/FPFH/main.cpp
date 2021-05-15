#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <ctime>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/fpfh_omp.h> //包含fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/keypoints/sift_keypoint.h>   // shift关键点相关
#include <pcl/features/vfh.h>                     //VFH特征估计类头文件

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

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
void est_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
    est_normal.setKSearch(16);         //设置k邻域搜索阈值为20个点
    est_normal.setInputCloud(cloud_in);   //设置输入模型点云
    est_normal.setSearchMethod(tree);
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
fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
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
    est_fpfh.compute(*fpfh);

    return fpfh;
}

//计算VFH
pcl::PointCloud<pcl::VFHSignature308>::Ptr computure_vfh_feature(pointcloud::Ptr input_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) {

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
pointcloud::Ptr FPFHmatch(pointcloud::Ptr source, pointcloud::Ptr target, fpfhFeature::Ptr source_fpfh, fpfhFeature::Ptr target_fpfh) {
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(source);
    sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(target);
    sac_ia.setTargetFeatures(target_fpfh);
    pointcloud::Ptr align(new pointcloud);
    sac_ia.setCorrespondenceRandomness(6); //设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
    sac_ia.align(*align);

    //Eigen::Matrix4f sac_trans;
    //sac_trans = sac_ia.getFinalTransformation();
    pcl::io::savePLYFile("E:\\locks\\data\\result\\fpfh\\FPFH粗配准后的.ply", *align);
    return align;
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
void visualizeMatch(pointcloud::Ptr source, pointcloud::Ptr target, pointcloud::Ptr align, boost::shared_ptr<pcl::Correspondences> cru_correspondences) {
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

    //view->addCorrespondences<pcl::PointXYZ>(source, target, *cru_correspondences, "correspondence", v1);//添加显示对应点对

    while (!view->wasStopped())
    {
        view->spinOnce();
        //boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}



int main(int argc, char** argv)
{
    clock_t start, end, time;
    start = clock();

    pointcloud::Ptr source(new pointcloud);
    pointcloud::Ptr target(new pointcloud);

    //加载点云
    pcl::io::loadPLYFile("E:\\locks\\data\\bunny\\data\\bun045.ply", *source);
    pcl::io::loadPLYFile("E:\\locks\\data\\bunny\\data\\bun090.ply", *target);
    cout << "/" << endl;
    cout << "原始model点云数量：" << target->size() << endl;
    cout << "原始scene点云数量：" << source->size() << endl;
    eraseInfPoint(target);
    eraseInfPoint(source);

    fpfhFeature::Ptr source_fpfh(new fpfhFeature());  // fpfh特征
    fpfhFeature::Ptr target_fpfh(new fpfhFeature());

    pcl::PointCloud<pcl::VFHSignature308>::Ptr source_vfh(new pcl::PointCloud<pcl::VFHSignature308>());  // vfh特征
    pcl::PointCloud<pcl::VFHSignature308>::Ptr target_vfh(new pcl::PointCloud<pcl::VFHSignature308>());

    pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>());

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(0.005, 0.005, 0.005);
    voxel_grid.setInputCloud(source);
    voxel_grid.filter(*source);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid2;
    voxel_grid2.setLeafSize(0.005, 0.005, 0.005);
    voxel_grid2.setInputCloud(target);
    voxel_grid2.filter(*target);


    //计算法线
    est_normals(source, source_normals);
    est_normals(target, target_normals);

    //计算SIFT
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints_sift(new pcl::PointCloud<pcl::PointXYZ>);  // sift关键点
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints_sift(new pcl::PointCloud<pcl::PointXYZ>);  // sift关键点
    compute_sift(source, source_keypoints_sift);
    compute_sift(target, target_keypoints_sift);
    /*
    //计算VFH
    target_vfh = computure_vfh_feature(target, target_normals);
    pcl::io::savePLYFile("", *target_vfh);
    */
    //计算FPFH
    source_fpfh = compute_fpfh_feature(source, source_normals);
    target_fpfh = compute_fpfh_feature(target, target_normals);

    //FPFH配准
    pointcloud::Ptr align(new pointcloud);
    align = FPFHmatch(source, target, source_fpfh, target_fpfh);

    end = clock();
    cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC << endl;


    //计算对应关系
    boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
    cru_correspondences = CalculateCorrespondences(source_fpfh, target_fpfh);

    //可视化
    visualizeMatch(source, target, align, cru_correspondences);
    return 0;
}


