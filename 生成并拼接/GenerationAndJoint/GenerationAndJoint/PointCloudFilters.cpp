#include <PointCloudFilters.h>

//�����˲�
int voxelFilter(PointCloud::Ptr inputcloud, PointCloud::Ptr outputcloud, float lx, float ly, float lz) {
	if (inputcloud->points.size() == 0) {
		return -1;
	}
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setInputCloud(inputcloud);//����
	voxel_grid.setLeafSize(lx, ly, lz);//�ֱ���,ԽСԽ��,�����ֱ���xyz
	voxel_grid.filter(*outputcloud);//���
	return 0;
}

//�ƶ���С���˷�
int movingLeastSquaresFilter(PointCloud::Ptr inputcloud, pcl::PointCloud<pcl::PointNormal> mls_points, double radius) {
	if (inputcloud->points.size() == 0) {
		return -1;
	}
	// ����һ��KD��
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	// ����ļ�����PointNormal���ͣ������洢�ƶ���С���˷�����ķ���
	// ������� (�ڶ��ֶ���������Ϊ�˴洢����, ��ʹ�ò���Ҳ��Ҫ�������)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals(true);
	//���ò���
	mls.setInputCloud(inputcloud);
	mls.setPolynomialOrder(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(radius);//4
	// �����ؽ�
	mls.process(mls_points);
	return 0;
}

////˫��
//int bilateralFilter(string inputFileName, string outputFileName) {
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(inputFileName, *cloud) == -1) {
//		PCL_ERROR("Couldnot read file.\n");
//		system("pause");
//		return(-1);
//	}
//	pcl::PointCloud<pcl::PointXYZRGB> outcloud;
//	pcl::BilateralFilter<pcl::PointXYZRGB> bf;
//	pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
//	bf.setInputCloud(cloud);
//	bf.setHalfSize(5);
//	bf.setStdDev(0.3);
//	//bf.setSigmaS(5);//����˫���˲������ڿռ�����/���ڵĸ�˹�ı�׼ƫ��
//	//bf.setSigmaR(0.003);//���ø�˹�ı�׼ƫ�����ڿ���������������ǿ�Ȳ�����½����٣������ǵ������Ϊ��ȣ�
//	bf.filter(outcloud);
//
//	// �����˲���������ļ�  
//	pcl::io::savePLYFile(outputFileName, outcloud);
//	return (0);
//}

//��˹
int gaussionFilter(PointCloud::Ptr inputcloud, PointCloud::Ptr outputcloud, double radius) {
	if (inputcloud->points.size() == 0) {
		return -1;
	}
	//Set up the Gaussian Kernel
	pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>::Ptr kernel(new pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>);
	(*kernel).setSigma(1);
	(*kernel).setThresholdRelativeToSigma(3);
	std::cout << "Kernel made" << std::endl;

	//Set up the KDTree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	(*kdtree).setInputCloud(inputcloud);
	std::cout << "KdTree made" << std::endl;

	//Set up the Convolution Filter
	pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> convolution;
	convolution.setKernel(*kernel);
	convolution.setInputCloud(inputcloud);
	convolution.setSearchMethod(kdtree);
	convolution.setRadiusSearch(radius);//20
	std::cout << "Convolution Start" << std::endl;

	convolution.convolve(*outputcloud);
	std::cout << "Convoluted" << std::endl;
	return 0;
}



//��ֵ
int mediumnFilter(PointCloud::Ptr inputcloud, PointCloud::Ptr outputcloud, float max_allow, int win_size) {
	if (inputcloud->points.size() == 0) {
		return -1;
	}
	pcl::MedianFilter<pcl::PointXYZ> fbf;
	fbf.setInputCloud(inputcloud);
	fbf.setMaxAllowedMovement(max_allow);//0.5
	fbf.setWindowSize(win_size);//20
	fbf.filter(*outputcloud);
	return 0;
}

//ͳ��
int statisticalOutlierRemovalFilter(PointCloud::Ptr inputcloud, PointCloud::Ptr outputcloud, int nr_k, double stddev_mult, bool nagetive) {
	if (inputcloud->points.size() == 0) {
		return -1;
	}
	//ͳ���˲�
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(inputcloud);
	sor.setMeanK(nr_k); //K������������� 20
	sor.setStddevMulThresh(stddev_mult); //��׼��� 0.1
	sor.setNegative(nagetive); //����δ�˲��㣨�ڵ㣩
	sor.filter(*outputcloud);  //�����˲������cloud_filter
	return 0;
}

int radiusFilter(PointCloud::Ptr inputcloud, PointCloud::Ptr outputcloud, double radius, int min_pts)
{
	if (inputcloud->points.size() == 0) {
		return -1;
	}
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;//�����뾶�˲�������
	outrem.setInputCloud(inputcloud);			//�����������
	outrem.setRadiusSearch(radius);					//���ð뾶Ϊ4cm
	//outrem.setMinNeighborsInRadius(min_pts);				//������С�ڽӵ������ֵ,�뾶��Χ���������������5�ĵ㽫���˳�
	outrem.filter(*outputcloud);				//ִ���˲�
	return 0;
}

int pathThroughFilter(PointCloud::Ptr inputcloud, PointCloud::Ptr outputcloud, string field, float limit_min, float limit_max)
{
	if (inputcloud->points.size() == 0) {
		return -1;
	}
	cout << "there are " << inputcloud->points.size() << " points before filtering." << endl;

	////2.ȡ�õ������꼫ֵ,д�ں�����
	//pcl::PointXYZ minPt, maxPt;
	//pcl::getMinMax3D(*inputcloud, minPt, maxPt);

	//3.ֱͨ�˲�
	pcl::PassThrough<pcl::PointXYZ> pass;     //�����˲�������
	pass.setInputCloud(inputcloud);                //���ô��˲��ĵ���
	pass.setFilterFieldName(field);             //������Z�᷽���Ͻ����˲� "x" "y" "z"
	pass.setFilterLimits(limit_min, limit_max);    //�����˲���Χ(����ߵ�����12��ȥ��)
	//pass.setFilterFieldName("y");             //������Z�᷽���Ͻ����˲�
	//pass.setFilterLimits(0, 0.3);    //�����˲���Χ(����ߵ�����12��ȥ��)
	pass.setFilterLimitsNegative(false);      //����
	pass.filter(*outputcloud);               //�˲����洢
	cout << "there are " << outputcloud->points.size() << " points after filtering." << endl;
	return 0;
}