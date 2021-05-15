/*
 * ���ܣ� ���Ƹ���任
 * ͷ�ļ��� #include <pcl/common/transforms.h>
 * ���ܺ����� pcl::transformPointCloud(*pPointCloudIn, *pPointCloudOut, transform_1);
 */


#include <iostream>
 // pcl
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

int main()
{
    // 1. �������
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloudIn(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::io::loadPLYFile("D:\\�о�����ҵDOC\\ʵ�����\\�����˲����_����2_ģ��.ply", *pPointCloudIn);

    // 2. ���Ӵ��ڳ�ʼ��
    pcl::visualization::PCLVisualizer viewer;
    viewer.setCameraFieldOfView(0.785398);      // fov ���45��
    viewer.setBackgroundColor(0.5, 0.5, 0.5);   // ������Ϊ��ɫ
    viewer.setCameraPosition(
        0, 0, 5,                                // cameraλ��
        0, 0, -1,                               // view����--�������
        0, 1, 0                                 // up����
    );

    // 3. ������ת
    /* ��ʾ: �任������ԭ�� :

    |-------> ��һ�б�ʾƽ������
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> ���� 3x3 matrix��ʾ��ת������������ת����
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> ����û��ʵ������ֻ��Ϊ�˼��㲹��� (ʼ��Ϊ 0,0,0,1)

    ���� #1: ʹ��һ��Matrix4f���;���f����float��
    �����ֹ���ʽ��������⵫�����׳���
    */
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();      // ��λ��

    // ����һ����ת�����ʾ��z��תPI/4 (��������Բ鿴��ת���� https://en.wikipedia.org/wiki/Rotation_matrix)
    //float theta = M_PI / 6; // ��ת�ĽǶȣ��Ի���Ϊ��λ
    float theta = 0;
    double s = 1.5;
    transform_1(0, 0) = s * cos(theta);
    transform_1(0, 1) = s* (-sin(theta));
    transform_1(1, 0) = s * sin(theta);
    transform_1(1, 1) = s * cos(theta);
    // Բ�����ڵ�����(Ԫ���ھ����е���, Ԫ���ھ����е���)

    // ��������x��ƽ��2.5m
    //transform_1(0, 3) = 2.5;

    // ��ӡ�任����
    printf("Method #1: using a Matrix4f\n");
    std::cout << transform_1 << std::endl;

    // ִ�б任
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloudOut(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*pPointCloudIn, *pPointCloudOut, transform_1);

    //����任��ĵ���
    pcl::io::savePLYFile("D:\\�о�����ҵDOC\\ʵ�����\\�����˲����_����2_ģ��_�ӳ߶�_1.5.ply",*pPointCloudOut);

    // 4. ���ƿ��ӻ�
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> inColorHandler(pPointCloudIn, 255, 255, 255);// ��ɫ
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> outColorHandler(pPointCloudOut, 230, 20, 20); // ��ɫ
    //viewer.addPointCloud(pPointCloudIn, inColorHandler, "In");
    viewer.addPointCloud(pPointCloudOut, outColorHandler, "Out");
    viewer.addCoordinateSystem(1.0, "cloud", 0);
    while (!viewer.wasStopped()) { // ��ʾ��ֱ����q����������
        viewer.spinOnce();
    }

    std::system("pause");
    return 0;
}