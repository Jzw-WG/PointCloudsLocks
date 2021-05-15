
#include <iostream>

int main()
{
    //生成CPC分割器
    pcl::CPCSegmentation<PointT>::CPCSegmentation seg;
    //输入超体聚类结果
    seg.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
    //设置分割参数
    setCutting(max_cuts = 20,
        cutting_min_segments = 0,
        cutting_min_score = 0.16,
        locally_constrained = true,
        directed_cutting = true,
        clean_cutting = false)；
        seg.setRANSACIterations(ransac_iterations);
    seg.segment();
    seg.relabelCloud(pcl::PointCloud<pcl::PointXYZL> &labeled_cloud_arg);

    std::cout << "Hello World!\n";
}
