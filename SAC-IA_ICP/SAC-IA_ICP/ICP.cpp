#include <ICP.h>

bool next_iteration = false;

void print4x4Matrix(const Eigen::Matrix4d& matrix)    //��ӡ��ת�����ƽ�ƾ���
{
    printf("Rotation matrix :\n");
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
    void* nothing)
{  //ʹ�ÿո�������ӵ�����������������ʾ
    if (event.getKeySym() == "space" && event.keyDown())
        next_iteration = true;
}

int showICPviewer(PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_tar, int iterations) // Ĭ�ϵ�ICP��������
{
    // �������ƽ�Ҫʹ�õ�
    PointCloudT::Ptr cloud_tr(new PointCloudT);  // ת����ĵ���
    PointCloudT::Ptr cloud_icp(new PointCloudT);  // ICP �������

    pcl::console::TicToc time;     //����ʱ���¼
    time.tic();       //time.tic��ʼ  time.toc����ʱ��
    
    //������ת�����ƽ������Matrix4d��Ϊ4*4�ľ���
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();  //��ʼ��

    //// ��ת����Ķ�����Բο� ( https://en.wikipedia.org/wiki/Rotation_matrix)
    //double theta = M_PI / 8;  // ��ת�ĽǶ��û��ȵı�ʾ����
    //transformation_matrix(0, 0) = cos(theta);
    //transformation_matrix(0, 1) = -sin(theta);
    //transformation_matrix(1, 0) = sin(theta);
    //transformation_matrix(1, 1) = cos(theta);

    //// Z���ƽ������ (0.4 meters)
    //transformation_matrix(2, 3) = 0.4;

    ////��ӡת������
    //std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
    //print4x4Matrix(transformation_matrix);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudT::iterator it = cloud_in -> points.begin();
    while (it != cloud_in -> points.end())
    {
        float x, y, z, rgb;
        x = it->x;
        y = it->y;
        z = it->z;
        //cout << "x: " << x << "  y: " << y << "  z: " << z << "  rgb: " << rgb << endl;
        if (!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z) || !pcl_isfinite(rgb))
        {
            it = cloud_in ->points.erase(it);
        }
        else
            ++it;
    }

    // //ִ�е���ת��
    //pcl::transformPointCloud(*cloud_in, *cloud_icp, transformation_matrix);
    //*cloud_tr = *cloud_icp;  // ����cloud_icp��ֵ��cloud_trΪ����ʹ��
    *cloud_icp = *cloud_in;
    *cloud_tr = *cloud_tar;
    // ����������㷨
    time.tic();        //ʱ��
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(iterations);    //��������������iterations=true
    icp.setInputSource(cloud_icp);   //��������ĵ���
    icp.setInputTarget(cloud_tar);    //Ŀ�����
    icp.align(*cloud_icp);          //ƥ���Դ����
    icp.setMaximumIterations(1);  // ����Ϊ1�Ա��´ε���
    icp.setMaxCorrespondenceDistance(0.05);
    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

    if (icp.hasConverged())//icp.hasConverged ()=1��true������任������ʺ�������
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
        std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix = icp.getFinalTransformation().cast<double>();
        print4x4Matrix(transformation_matrix);
    }
    else
    {
        PCL_ERROR("\nICP has not converged.\n");
        return (-1);
    }

    // ���ӻ�ICP�Ĺ�������
    pcl::visualization::PCLVisualizer viewer("ICP demo");
    // ���������۲��ӵ�
    int v1(0);
    int v2(1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    // ������ʾ����ɫ��Ϣ
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // ԭʼ�ĵ�������Ϊ��ɫ��
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_tar, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
        (int)255 * txt_gray_lvl);
    viewer.addPointCloud(cloud_tar, cloud_in_color_h, "cloud_in_v1", v1);//����ԭʼ�ĵ��ƶ�����ʾΪ��ɫ
    viewer.addPointCloud(cloud_tar, cloud_in_color_h, "cloud_in_v2", v2);

    // ת����ĵ�����ʾΪ��ɫ
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 20, 180, 20);
    viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

    // ICP��׼��ĵ���Ϊ��ɫ
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_icp, 180, 20, 20);
    viewer.addPointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

    // �����ı��������ڸ��Ե��ӿڽ���
   //��ָ���ӿ�viewport=v1����ַ�����white ������������"icp_info_1"������ַ�����ID��־����10��15��Ϊ����16Ϊ�ַ���С ����ֱ���RGBֵ
    viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

    std::stringstream ss;
    ss << iterations;            //����ĵ����Ĵ���
    std::string iterations_cnt = "ICP iterations = " + ss.str();
    viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

    // ���ñ�����ɫ
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // �������������ͷ���
    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize(1280, 1024);  // ���ӻ����ڵĴ�С

    // ע�ᰴ���ص�����
    viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);

    // ��ʾ
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();

        //���¿ո���ĺ���
        if (next_iteration)
        {
            // ���������㷨
            time.tic();
            icp.align(*cloud_icp);
            std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;

            if (icp.hasConverged())
            {
                printf("\033[11A");  // Go up 11 lines in terminal output.
                printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
                std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
                transformation_matrix *= icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate!
                print4x4Matrix(transformation_matrix);  // ��ӡ����任

                ss.str("");
                ss << iterations;
                std::string iterations_cnt = "ICP iterations = " + ss.str();
                viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
                viewer.updatePointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
            }
            else
            {
                PCL_ERROR("\nICP has not converged.\n");
                return (-1);
            }
        }
        next_iteration = false;
    }
    return (0);
}