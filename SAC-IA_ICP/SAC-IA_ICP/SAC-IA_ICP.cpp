#include <ICP.h>
#include <iostream>
#include <SAC-IA.h>


using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main()
{
    PointCloud::Ptr result(new PointCloud);
    PointCloud::Ptr source(new PointCloud);
    PointCloud::Ptr target(new PointCloud);

    //加载点云
    pcl::io::loadPLYFile("E:\\locks\\data\\bunny\\data\\bun045.ply", *source);
    pcl::io::loadPLYFile("E:\\locks\\data\\bunny\\data\\bun090.ply", *target);
    //cout << "/" << endl;
    cout << "原始model点云数量：" << target->size() << endl;
    cout << "原始scene点云数量：" << source->size() << endl;

    result = startSAC_IA(source, target);
    showICPviewer(result, target);
    return 0;
}