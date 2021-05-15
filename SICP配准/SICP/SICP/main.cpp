#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

struct init_param_t
{
	Eigen::Matrix3d R;
	Eigen::Vector3d T;
	double s = 0;
	Eigen::Matrix<double, 1, 2> I;
};

struct next_param_t
{
	Eigen::Matrix3d R;
	Eigen::Vector3d T;
	double s = 0;
	double en = 0;
	double en1 = 0;
	double fitness = 0;
};

struct show_param_t
{
	Eigen::Matrix3d R;
	Eigen::Vector3d T;
	double s = 0;
	Eigen::Matrix<double, 1, 2> I;
	double en = 0;
	double fitness = 0;
	int iteration = 0;
};

//��ʱ����Ϊȫ�ֱ���
int max_point = 0;

static init_param_t init(pcl::PointCloud<pcl::PointXYZ>::Ptr X, pcl::PointCloud<pcl::PointXYZ>::Ptr Y, int pointX, int pointY) {
	Eigen::Matrix<double, 4, 1> centroidX;
	Eigen::Matrix<double, 4, 1> centroidY;
	pcl::compute3DCentroid(*X, centroidX);
	pcl::compute3DCentroid(*Y, centroidY);

	//����������������Э�������
	Eigen::Matrix<double, 3, 3> convariance_matrix_X;  // Э�������
	pcl::computeCovarianceMatrix(*X, centroidX, convariance_matrix_X);

	Eigen::Matrix<double, 3, 3> convariance_matrix_Y;  // Э�������
	pcl::computeCovarianceMatrix(*Y, centroidY, convariance_matrix_Y);

	//��������ֵ����������
	Eigen::Matrix3d eigenVectors_X;
	Eigen::Vector3d eigenValues_X;
	pcl::eigen33(convariance_matrix_X, eigenVectors_X, eigenValues_X);

	Eigen::Matrix3d eigenVectors_Y;
	Eigen::Vector3d eigenValues_Y;
	pcl::eigen33(convariance_matrix_Y, eigenVectors_Y, eigenValues_Y);

	//s�ĳ�ʼֵ
	Eigen::Matrix<double, 1, 3> sq = eigenValues_Y.cwiseQuotient(eigenValues_X);//��Ԫ�س���
	sq << sq.cwiseSqrt();//��Ԫ�ؼ���ƽ����
	double s = sq.sum() / 3;

	//I�������� [minI, maxI]
	Eigen::Matrix<double, 1, 2> I;
	I(0, 0) = sq.minCoeff();
	I(0, 1) = sq.maxCoeff();

	//R�ĳ�ʼֵ
	Eigen::Vector3d p1, p2, p3;
	Eigen::Vector3d q1, q2, q3;
	p1 << eigenVectors_X.col(0);
	p2 << eigenVectors_X.col(1);
	q1 << eigenVectors_Y.col(0);
	q2 << eigenVectors_Y.col(1);
	q3 << eigenVectors_Y.col(2);
	double f = 0.8; //��ֵf

	//���������ԽС��˵����н�Խ���ҳ��нǴ���ĳֵ��������
	if (p1.dot(q1) < f)
		p1 << -p1;
	if (p2.dot(q2) < f)
		p2 << -p2;
	p3 << p1.cross(p2);

	Eigen::Matrix3d temp1, temp2, R;
	temp1.col(0) << p1 / p1.sum();
	temp1.col(1) << p2 / p2.sum();
	temp1.col(2) << p3 / p3.sum();

	temp2.col(0) << q1 / q1.sum();
	temp2.col(1) << q2 / q2.sum();
	temp2.col(2) << q3 / q3.sum();
	R << temp2 * (temp1.inverse());

	//������T�ĳ�ʼֵ
	Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	transform.block(0, 0, 3, 3) << s * R;
	pcl::transformPointCloud(*X, *transformed_cloud, transform);

	Eigen::Matrix<double, 4, 1> centroidX2;
	pcl::compute3DCentroid(*transformed_cloud, centroidX2);
	Eigen::Vector3d T;
	Eigen::Matrix<double, 4, 1> T_temp = centroidY - centroidX2;
	T << T_temp.block(0, 0, 3, 1);

	//���س�ʼ����ֵ
	init_param_t initParam;
	initParam.R << R;
	initParam.T << T;
	initParam.s = s;
	initParam.I << I;
	return initParam;
}

/*
* XO���ò��������任��ĵ���
*/
static void searchNearestPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr X,
	pcl::PointCloud<pcl::PointXYZ>::Ptr Y, pcl::PointCloud<pcl::PointXYZ>::Ptr zk) {

	//����KdTreeFLANN����
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	//��Y����Ϊ�����ռ�
	kdtree.setInputCloud(Y);

	int K = 1;
	std::vector<int> pointIdxNKNSearch(K);//�洢��ѯ���������
	std::vector<float> pointNKNSquaredDistance(K); //�洢���ڵ��Ӧ����ƽ��

	for (int i = 0; i < X->points.size(); i++) {
		pcl::PointXYZ searchPoint = X->points[i];
		if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)  //ִ��K��������
		{
			zk->points[i].x = Y->points[pointIdxNKNSearch[0]].x;
			zk->points[i].y = Y->points[pointIdxNKNSearch[0]].y;
			zk->points[i].z = Y->points[pointIdxNKNSearch[0]].z;
		}
	}
}

//����E(k+1)��fitness
static double computeEAndFitness(double s, Eigen::Matrix3d R, Eigen::Vector3d T, Eigen::MatrixXd Xi, Eigen::MatrixXd Zi, int& fitness_count) {
	Eigen::MatrixXd c = s * (R * Xi) + T.replicate(1, Xi.cols()) - Zi;//������1�Σ�������Xi.cols
	Eigen::MatrixXd c_dot = c.cwiseProduct(c);
	Eigen::VectorXd distance = c_dot.colwise().sum();//�������
	double en = distance.sum();//�����������

	int count = 0;
	for (int i = 0; i < distance.size(); i++) {
		if (distance[i] < 12) count += 1;
	}
	fitness_count = count;
	//fitness = count / distance.size();
	return en;
}

//����E(k+1)
static double computeE(double s, Eigen::Matrix3d R, Eigen::Vector3d T, Eigen::MatrixXd Xi, Eigen::MatrixXd Zi) {
	Eigen::MatrixXd c = s * (R * Xi) + T.replicate(1, Xi.cols()) - Zi;
	Eigen::MatrixXd c_dot = c.cwiseProduct(c);
	Eigen::VectorXd distance = c_dot.colwise().sum();//�������	
	double en1 = distance.sum();//�����������
	return en1;
}


//����R(n+1)
static Eigen::Matrix3f computeR(Eigen::MatrixXf Xi, Eigen::MatrixXf Zi) {
	//����H����
	Eigen::Matrix3f H = Xi * Zi.transpose();
	//SVD��������USV
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Matrix3f V = svd.matrixV(), U = svd.matrixU();
	Eigen::MatrixXf C(3, 1);
	C << svd.singularValues();
	Eigen::Matrix3f  S = U.inverse() * H * V.transpose().inverse();
	//�ж�����
	Eigen::Matrix3f Rn1 = V * U.transpose();

	if (round(Rn1.determinant()) == 1) {
		return Rn1;
	}
	else if (round(Rn1.determinant()) == -1) {
		Eigen::Matrix3f x;
		x << 1, 0, 0,
			0, 1, 0,
			1, 0, -1;
		Rn1 << V * x * U.transpose();
	}
	return Rn1;
}


//����S
static double computeS(Eigen::Matrix<double, 1, 2> I, Eigen::Matrix3d Rn1, Eigen::MatrixXd Xi, Eigen::MatrixXd Zi) {
	Eigen::MatrixXd Xo = Rn1 * Xi;
	Xo << Xo.cwiseProduct(Zi);//��ӦԪ�ص��
	Eigen::VectorXd colSumXo = Xo.colwise().sum();//�������
	double sum1 = colSumXo.sum();

	Eigen::MatrixXd xi_dot = Xi.cwiseProduct(Xi);
	Eigen::VectorXd colSumXi = xi_dot.colwise().sum();//�������
	double sum2 = colSumXi.sum();

	double sn1 = sum1 / sum2;
	if (sn1 <= I(0, 0))
		sn1 = I(0, 0);
	else if (sn1 >= I(0, 1))
		sn1 = I(0, 1);
	return sn1;
}


//����T
static Eigen::Vector3d computeT(Eigen::Vector3d centroidZi, double sn, Eigen::Matrix3d Rn, Eigen::Vector3d centroidXi) {
	Eigen::Vector3d Tn;
	Tn << centroidZi - sn * Rn * centroidXi;
	return Tn;
}


static next_param_t Solvecircle(double s, Eigen::Matrix3d R, Eigen::Vector3d T, Eigen::Matrix<double, 1, 2> I,
	pcl::PointCloud<pcl::PointXYZ>::Ptr X, pcl::PointCloud<pcl::PointXYZ>::Ptr Y) {

	//����ʼ����Ӧ�õ�����X
	Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();//��λ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	transform.block(0, 0, 3, 3) << s * R;
	transform.block(0, 3, 3, 1) << T;
	pcl::transformPointCloud(*X, *transformed_cloud, transform);

	/*˵���Ե����*/
	std::cout << "    ��ʼ����zk" << std::endl;

	//��Y������transformed_cloud�ĵ���ӽ��ĵ�ļ���zk
	pcl::PointCloud<pcl::PointXYZ>::Ptr zk(new pcl::PointCloud<pcl::PointXYZ>);
	zk->width = X->width;
	zk->height = X->height;
	zk->points.resize(zk->width * zk->height);

	//ȥ��NaN�ĵ�
	pcl::PointCloud<pcl::PointXYZ>::iterator it = transformed_cloud->points.begin();
	while (it != transformed_cloud->points.end())
	{
		float x, y, z;
		x = it->x;
		y = it->y;
		z = it->z;
		if (!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z))
		{
			it = transformed_cloud->points.erase(it);
		}
		else
			++it;
	}

	it = Y->points.begin();
	while (it != Y->points.end())
	{
		float x, y, z;
		x = it->x;
		y = it->y;
		z = it->z;
		if (!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z))
		{
			it = Y->points.erase(it);
		}
		else
			++it;
	}

	//��kdtree���������
	searchNearestPoints(transformed_cloud, Y, zk);

	/*˵���Ե����*/
	std::cout << "    ��������" << std::endl;

	/*˵���Ե����*/
	std::cout << "    ��ʼ����ת��Ϊz_ x_����" << std::endl;

	//��X��zk���Ƶĵ�תΪ����X_��Z_
	Eigen::MatrixXf Xii;
	Eigen::MatrixXf Zii;

	pcl::PCLPointCloud2::Ptr cloudX(new pcl::PCLPointCloud2);
	pcl::PCLPointCloud2::Ptr cloudZ(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*X, *cloudX);
	pcl::toPCLPointCloud2(*zk, *cloudZ);

	pcl::getPointCloudAsEigen(*cloudX, Xii);//Xii��4�еģ�ǰ����Ϊx,y,z�����һ��Ϊ1
	pcl::getPointCloudAsEigen(*cloudZ, Zii);//ͬ��

	Eigen::MatrixXd X_;
	Eigen::MatrixXd Z_;
	X_ = Xii.cast<double>().block(0, 0, 3, Xii.cols());
	Z_ = Zii.cast<double>().block(0, 0, 3, Zii.cols());

	/*˵���Ե����*/
	std::cout << "    ��������" << std::endl;

	/*˵���Ե����*/
	std::cout << "    ��ʼ����en" << std::endl;

	//���㵱ǰenֵ������n�ε��������ֵ�����任��ĵ��ƺ�Y֮��ĵ�Ĳ�ֵƽ��
	double fitness = 0;
	int fitness_count = 0;
	double en = computeEAndFitness(s, R, T, X_, Z_, fitness_count);
	fitness = (double)fitness_count / max_point;

	/*˵���Ե����*/
	std::cout << "    ��������" << std::endl;

	//������������
	Eigen::Vector3d centroidX_ = X_.rowwise().mean();
	Eigen::Vector3d centroidZ_ = Z_.rowwise().mean();
	//�����ȥ��������ľ���Xi,Zi
	Eigen::MatrixXd Xi = X_ - centroidX_.replicate(1, X_.cols());
	Eigen::MatrixXd Zi = Z_ - centroidZ_.replicate(1, Z_.cols());

	/*˵���Ե����*/
	std::cout << "    ��ʼ����Rn1" << std::endl;

	//������һ�εĲ���R n+1
	Eigen::Matrix3f Rn1_f = computeR(Xi.cast<float>(), Zi.cast<float>());
	Eigen::Matrix3d Rn1 = Rn1_f.cast<double>();

	/*˵���Ե����*/
	std::cout << "    ��������" << std::endl;

	/*˵���Ե����*/
	std::cout << "    ��ʼ����sn1" << std::endl;

	//������һ�εĲ���s n+1
	double sn1 = computeS(I, Rn1, Xi, Zi);

	/*˵���Ե����*/
	std::cout << "    ��������" << std::endl;

	/*˵���Ե����*/
	std::cout << "    ��ʼ����Tn1" << std::endl;

	//����T n+1
	Eigen::Vector3d Tn1 = computeT(centroidZ_, sn1, Rn1, centroidX_);

	/*˵���Ե����*/
	std::cout << "    ��������" << std::endl;

	/*˵���Ե����*/
	std::cout << "    ��ʼ����en1" << std::endl;

	//����n+1�ε��������ݵ����ֵen1���м���
	double en1 = computeE(sn1, Rn1, Tn1, X_, Z_);

	/*˵���Ե����*/
	std::cout << "    ��������" << std::endl;

	next_param_t nextParam;
	nextParam.R << Rn1;
	nextParam.s = sn1;
	nextParam.T << Tn1;
	nextParam.en = en;
	nextParam.en1 = en1;
	nextParam.fitness = fitness;
	return nextParam;

}


static show_param_t reg3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1) {

	//��ȡcloud��cloud1�ĵ���
	int pointX = cloud->points.size();
	int pointY = cloud1->points.size();

	max_point = pointX > pointY ? pointX : pointY;

	/*˵���Ե����*/
	std::cout << "  ��ʼ�����ʼ������" << std::endl;

	//��ʼ��R��T��S
	init_param_t initParam;
	initParam = init(cloud, cloud1, pointX, pointY);

	/*˵���Ե����*/
	std::cout << "  ��������" << std::endl;

	Eigen::Matrix<double, 1, 2> I = initParam.I;
	//��һ������
	next_param_t nextParam0 = Solvecircle(initParam.s, initParam.R, initParam.T, I, cloud, cloud1);
	Eigen::Matrix3d R = nextParam0.R;
	Eigen::Vector3d T = nextParam0.T;
	double s = nextParam0.s;
	double etemp = nextParam0.en1;

	double o = 0.01;//������ֹ����
	int iteration = 2;//��������
	double q = 1;
	double en1 = 0, fitness = 0;
	/*˵���Ե����*/
	std::cout << "  ��ʼ����" << std::endl;

	while (q > o) {
		//ִ�б任����������һ�εĲ���ֵ
		next_param_t nextParam = Solvecircle(s, R, T, I, cloud, cloud1);
		R << nextParam.R;
		T << nextParam.T;
		s = nextParam.s;
		en1 = nextParam.en1;
		fitness = nextParam.fitness;

		q = 1 - en1 / etemp;//�ȸ���q
		etemp = en1;//����en1
		iteration = iteration + 1;
	}

	/*˵���Ե����*/
	std::cout << "  ��������" << std::endl;


	show_param_t showParam;
	showParam.s = s;
	showParam.R << R;
	showParam.T << T;
	showParam.I << I;
	showParam.en = en1;
	showParam.fitness = fitness;
	showParam.iteration = iteration - 1;
	return showParam;
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr X(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Y(new pcl::PointCloud<pcl::PointXYZ>);
	/*
	std::string filenameX = std::string(argv[1]);
	std::string filenameY = std::string(argv[2]);
	*/
	
	std::string filenameX = "E:\\locks\\data\\bunny\\reconstruction\\bun1.ply";//����
	std::string filenameY = "E:\\locks\\data\\bunny\\reconstruction\\bun_zipper.ply";//ģ��
	

	if (pcl::io::loadPLYFile<pcl::PointXYZ>(filenameX, *X) == -1 || pcl::io::loadPLYFile<pcl::PointXYZ>(filenameY, *Y) == -1)
	{
		PCL_ERROR("Couldn't read file \n");
		system("PAUSE");
		return (-1);
	}

	show_param_t showParam;

	/*˵���Ե����*/
	std::cout << "��ʼ����showParam" << std::endl;

	showParam = reg3D(X, Y);

	std::cout << "s = " << showParam.s << std::endl;
	std::cout << "R = " << showParam.R << std::endl;
	std::cout << "T = " << showParam.T << std::endl;
	std::cout << "e = " << showParam.en << std::endl;
	std::cout << "fitness = " << showParam.fitness << std::endl;
	std::cout << "iteration = " << showParam.iteration << std::endl;
	if (showParam.s < 0.5 || showParam.s > 1.5 || showParam.fitness < 0.5) {
		std::cout << "����ֲ����ţ����߲�ƥ�� " << std::endl;
	}

	/*˵���Ե����*/
	std::cout << "��������" << std::endl;

	/*˵���Ե����*/
	std::cout << "���ӻ�" << std::endl;
	
	//X��ִ�б任
	Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();//��λ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	transform.block(0, 0, 3, 3) << showParam.s * showParam.R;
	transform.block(0, 3, 3, 1) << showParam.T;
	pcl::transformPointCloud(*X, *transformed_cloud, transform);

	//���Ӵ��ڳ�ʼ��
	pcl::visualization::PCLVisualizer viewer;
	//viewer.setCameraFieldOfView(0.785398);      // fov ���45��
	viewer.setBackgroundColor(0,0,0);   // ������Ϊ��ɫ

	//���Ӵ��ڼ������
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> inColorHandler(transformed_cloud, 255, 255, 255);// ��ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outColorHandler(Y, 230, 20, 20); // ��ɫ
	viewer.addPointCloud(transformed_cloud, inColorHandler, "transformed_X");
	viewer.addPointCloud(Y, outColorHandler, "Y");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed_X");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Y");
	//viewer.addCoordinateSystem(0.5);
	viewer.spin();
}