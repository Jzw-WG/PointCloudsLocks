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
#include <pcl/features/fpfh_omp.h> //����fpfh���ټ����omp(��˲��м���)
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //�����Ĵ����Ӧ��ϵȥ��
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //�������һ����ȥ��
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/keypoints/sift_keypoint.h>   // shift�ؼ������
#include <pcl/features/vfh.h>                     //VFH����������ͷ�ļ�

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

// sift���
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
    //ȥ��NaN�� ��ֹcompute����
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

//���Ʒ���
void est_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
    est_normal.setKSearch(16);         //����k����������ֵΪ20����
    est_normal.setInputCloud(cloud_in);   //��������ģ�͵���
    est_normal.setSearchMethod(tree);
    est_normal.compute(*normals);//������Ʒ���
}

//����SIFT
void compute_sift(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints)
{
    clock_t start = clock();

    const float min_scale = 1;             //���ó߶ȿռ�����С�߶ȵı�׼ƫ��
    const int n_octaves = 6;               //���ø�˹�������飨octave������Ŀ
    const int n_scales_per_octave = 6;     //����ÿ�飨octave������ĳ߶�
    const float min_contrast = 0.01;          //�������ƹؼ��������ֵ

    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;//����sift�ؼ��������
    pcl::PointCloud<pcl::PointWithScale> result;
    sift.setInputCloud(cloud);//�����������
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    sift.setSearchMethod(tree);//����һ���յ�kd������tree�����������ݸ�sift������
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);//ָ�������ؼ���ĳ߶ȷ�Χ
    sift.setMinimumContrast(min_contrast);//�������ƹؼ��������ֵ
    sift.compute(result);//ִ��sift�ؼ����⣬��������result

    copyPointCloud(result, *keypoints);//��������pcl::PointWithScale������ת��Ϊ������pcl::PointXYZ������
    clock_t end = clock();

    cout << "sift�ؼ�����ȡʱ�䣺" << (double)(end - start) / CLOCKS_PER_SEC << endl;
    cout << "sift�ؼ�������" << keypoints->size() << endl;
}


//����FPFH
fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    //fpfh ����
    fpfhFeature::Ptr fpfh(new fpfhFeature);
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    est_fpfh.setNumberOfThreads(4); //ָ��4�˼���
    est_fpfh.setInputCloud(input_cloud);
    est_fpfh.setInputNormals(normals);
    est_fpfh.setSearchMethod(tree);
    est_fpfh.setKSearch(16);
    est_fpfh.compute(*fpfh);

    return fpfh;
}

//����VFH
pcl::PointCloud<pcl::VFHSignature308>::Ptr computure_vfh_feature(pointcloud::Ptr input_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) {

    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> est_vfh;
    est_vfh.setInputCloud(input_cloud);
    est_vfh.setInputNormals(normals);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//����һ���յ�kd�����󣬲��������ݸ�FPFH���ƶ���
    est_vfh.setSearchMethod(tree);
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh(new pcl::PointCloud<pcl::VFHSignature308>());//������ݼ�
    est_vfh.compute(*vfh);//��������ֵ

    return vfh;
}

//FPFH��׼
pointcloud::Ptr FPFHmatch(pointcloud::Ptr source, pointcloud::Ptr target, fpfhFeature::Ptr source_fpfh, fpfhFeature::Ptr target_fpfh) {
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(source);
    sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(target);
    sac_ia.setTargetFeatures(target_fpfh);
    pointcloud::Ptr align(new pointcloud);
    sac_ia.setCorrespondenceRandomness(6); //���ü���Э����ʱѡ����ٽ��ڵ㣬��ֵԽ��Э����Խ��ȷ�����Ǽ���Ч��Խ��.(��ʡ)
    sac_ia.align(*align);

    //Eigen::Matrix4f sac_trans;
    //sac_trans = sac_ia.getFinalTransformation();
    pcl::io::savePLYFile("E:\\locks\\data\\result\\fpfh\\FPFH����׼���.ply", *align);
    return align;
}

//����FPFH��Ӧ��ϵ
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

//���ӻ�
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

    //view->addCorrespondences<pcl::PointXYZ>(source, target, *cru_correspondences, "correspondence", v1);//�����ʾ��Ӧ���

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

    //���ص���
    pcl::io::loadPLYFile("E:\\locks\\data\\bunny\\data\\bun045.ply", *source);
    pcl::io::loadPLYFile("E:\\locks\\data\\bunny\\data\\bun090.ply", *target);
    cout << "/" << endl;
    cout << "ԭʼmodel����������" << target->size() << endl;
    cout << "ԭʼscene����������" << source->size() << endl;
    eraseInfPoint(target);
    eraseInfPoint(source);

    fpfhFeature::Ptr source_fpfh(new fpfhFeature());  // fpfh����
    fpfhFeature::Ptr target_fpfh(new fpfhFeature());

    pcl::PointCloud<pcl::VFHSignature308>::Ptr source_vfh(new pcl::PointCloud<pcl::VFHSignature308>());  // vfh����
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


    //���㷨��
    est_normals(source, source_normals);
    est_normals(target, target_normals);

    //����SIFT
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints_sift(new pcl::PointCloud<pcl::PointXYZ>);  // sift�ؼ���
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints_sift(new pcl::PointCloud<pcl::PointXYZ>);  // sift�ؼ���
    compute_sift(source, source_keypoints_sift);
    compute_sift(target, target_keypoints_sift);
    /*
    //����VFH
    target_vfh = computure_vfh_feature(target, target_normals);
    pcl::io::savePLYFile("", *target_vfh);
    */
    //����FPFH
    source_fpfh = compute_fpfh_feature(source, source_normals);
    target_fpfh = compute_fpfh_feature(target, target_normals);

    //FPFH��׼
    pointcloud::Ptr align(new pointcloud);
    align = FPFHmatch(source, target, source_fpfh, target_fpfh);

    end = clock();
    cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC << endl;


    //�����Ӧ��ϵ
    boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
    cru_correspondences = CalculateCorrespondences(source_fpfh, target_fpfh);

    //���ӻ�
    visualizeMatch(source, target, align, cru_correspondences);
    return 0;
}


