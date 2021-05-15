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

//暂时定义为全局变量
int max_point = 0;

static init_param_t init(pcl::PointCloud<pcl::PointXYZ>::Ptr X, pcl::PointCloud<pcl::PointXYZ>::Ptr Y, int pointX, int pointY) {
	Eigen::Matrix<double, 4, 1> centroidX;
	Eigen::Matrix<double, 4, 1> centroidY;
	pcl::compute3DCentroid(*X, centroidX);
	pcl::compute3DCentroid(*Y, centroidY);

	//计算点云重心坐标和协方差矩阵
	Eigen::Matrix<double, 3, 3> convariance_matrix_X;  // 协方差矩阵
	pcl::computeCovarianceMatrix(*X, centroidX, convariance_matrix_X);

	Eigen::Matrix<double, 3, 3> convariance_matrix_Y;  // 协方差矩阵
	pcl::computeCovarianceMatrix(*Y, centroidY, convariance_matrix_Y);

	//计算特征值、特征向量
	Eigen::Matrix3d eigenVectors_X;
	Eigen::Vector3d eigenValues_X;
	pcl::eigen33(convariance_matrix_X, eigenVectors_X, eigenValues_X);

	Eigen::Matrix3d eigenVectors_Y;
	Eigen::Vector3d eigenValues_Y;
	pcl::eigen33(convariance_matrix_Y, eigenVectors_Y, eigenValues_Y);

	//s的初始值
	Eigen::Matrix<double, 1, 3> sq = eigenValues_Y.cwiseQuotient(eigenValues_X);//逐元素除法
	sq << sq.cwiseSqrt();//逐元素计算平方根
	double s = sq.sum() / 3;

	//I区间设置 [minI, maxI]
	Eigen::Matrix<double, 1, 2> I;
	I(0, 0) = sq.minCoeff();
	I(0, 1) = sq.maxCoeff();

	//R的初始值
	Eigen::Vector3d p1, p2, p3;
	Eigen::Vector3d q1, q2, q3;
	p1 << eigenVectors_X.col(0);
	p2 << eigenVectors_X.col(1);
	q1 << eigenVectors_Y.col(0);
	q2 << eigenVectors_Y.col(1);
	q3 << eigenVectors_Y.col(2);
	double f = 0.8; //阈值f

	//两向量点积越小，说明其夹角越大，找出夹角大于某值的两向量
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

	//重新求T的初始值
	Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	transform.block(0, 0, 3, 3) << s * R;
	pcl::transformPointCloud(*X, *transformed_cloud, transform);

	Eigen::Matrix<double, 4, 1> centroidX2;
	pcl::compute3DCentroid(*transformed_cloud, centroidX2);
	Eigen::Vector3d T;
	Eigen::Matrix<double, 4, 1> T_temp = centroidY - centroidX2;
	T << T_temp.block(0, 0, 3, 1);

	//返回初始参数值
	init_param_t initParam;
	initParam.R << R;
	initParam.T << T;
	initParam.s = s;
	initParam.I << I;
	return initParam;
}

/*
* XO：用参数经过变换后的点云
*/
static void searchNearestPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr X,
	pcl::PointCloud<pcl::PointXYZ>::Ptr Y, pcl::PointCloud<pcl::PointXYZ>::Ptr zk) {

	//创建KdTreeFLANN对象
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	//把Y设置为搜索空间
	kdtree.setInputCloud(Y);

	int K = 1;
	std::vector<int> pointIdxNKNSearch(K);//存储查询点近邻索引
	std::vector<float> pointNKNSquaredDistance(K); //存储近邻点对应距离平方

	for (int i = 0; i < X->points.size(); i++) {
		pcl::PointXYZ searchPoint = X->points[i];
		if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)  //执行K近邻搜索
		{
			zk->points[i].x = Y->points[pointIdxNKNSearch[0]].x;
			zk->points[i].y = Y->points[pointIdxNKNSearch[0]].y;
			zk->points[i].z = Y->points[pointIdxNKNSearch[0]].z;
		}
	}
}

//计算E(k+1)和fitness
static double computeEAndFitness(double s, Eigen::Matrix3d R, Eigen::Vector3d T, Eigen::MatrixXd Xi, Eigen::MatrixXd Zi, int& fitness_count) {
	Eigen::MatrixXd c = s * (R * Xi) + T.replicate(1, Xi.cols()) - Zi;//横向复制1次，纵向复制Xi.cols
	Eigen::MatrixXd c_dot = c.cwiseProduct(c);
	Eigen::VectorXd distance = c_dot.colwise().sum();//按列求和
	double en = distance.sum();//对行向量求和

	int count = 0;
	for (int i = 0; i < distance.size(); i++) {
		if (distance[i] < 12) count += 1;
	}
	fitness_count = count;
	//fitness = count / distance.size();
	return en;
}

//计算E(k+1)
static double computeE(double s, Eigen::Matrix3d R, Eigen::Vector3d T, Eigen::MatrixXd Xi, Eigen::MatrixXd Zi) {
	Eigen::MatrixXd c = s * (R * Xi) + T.replicate(1, Xi.cols()) - Zi;
	Eigen::MatrixXd c_dot = c.cwiseProduct(c);
	Eigen::VectorXd distance = c_dot.colwise().sum();//按列求和	
	double en1 = distance.sum();//对行向量求和
	return en1;
}


//计算R(n+1)
static Eigen::Matrix3f computeR(Eigen::MatrixXf Xi, Eigen::MatrixXf Zi) {
	//计算H矩阵
	Eigen::Matrix3f H = Xi * Zi.transpose();
	//SVD方法计算USV
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Matrix3f V = svd.matrixV(), U = svd.matrixU();
	Eigen::MatrixXf C(3, 1);
	C << svd.singularValues();
	Eigen::Matrix3f  S = U.inverse() * H * V.transpose().inverse();
	//判断条件
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


//计算S
static double computeS(Eigen::Matrix<double, 1, 2> I, Eigen::Matrix3d Rn1, Eigen::MatrixXd Xi, Eigen::MatrixXd Zi) {
	Eigen::MatrixXd Xo = Rn1 * Xi;
	Xo << Xo.cwiseProduct(Zi);//对应元素点乘
	Eigen::VectorXd colSumXo = Xo.colwise().sum();//按列求和
	double sum1 = colSumXo.sum();

	Eigen::MatrixXd xi_dot = Xi.cwiseProduct(Xi);
	Eigen::VectorXd colSumXi = xi_dot.colwise().sum();//按列求和
	double sum2 = colSumXi.sum();

	double sn1 = sum1 / sum2;
	if (sn1 <= I(0, 0))
		sn1 = I(0, 0);
	else if (sn1 >= I(0, 1))
		sn1 = I(0, 1);
	return sn1;
}


//计算T
static Eigen::Vector3d computeT(Eigen::Vector3d centroidZi, double sn, Eigen::Matrix3d Rn, Eigen::Vector3d centroidXi) {
	Eigen::Vector3d Tn;
	Tn << centroidZi - sn * Rn * centroidXi;
	return Tn;
}


static next_param_t Solvecircle(double s, Eigen::Matrix3d R, Eigen::Vector3d T, Eigen::Matrix<double, 1, 2> I,
	pcl::PointCloud<pcl::PointXYZ>::Ptr X, pcl::PointCloud<pcl::PointXYZ>::Ptr Y) {

	//将初始条件应用到点云X
	Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();//单位阵
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	transform.block(0, 0, 3, 3) << s * R;
	transform.block(0, 3, 3, 1) << T;
	pcl::transformPointCloud(*X, *transformed_cloud, transform);

	/*说明性的语句*/
	std::cout << "    开始计算zk" << std::endl;

	//在Y中找与transformed_cloud的点最接近的点的集合zk
	pcl::PointCloud<pcl::PointXYZ>::Ptr zk(new pcl::PointCloud<pcl::PointXYZ>);
	zk->width = X->width;
	zk->height = X->height;
	zk->points.resize(zk->width * zk->height);

	//去除NaN的点
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

	//用kdtree计算最近邻
	searchNearestPoints(transformed_cloud, Y, zk);

	/*说明性的语句*/
	std::cout << "    结束计算" << std::endl;

	/*说明性的语句*/
	std::cout << "    开始计算转换为z_ x_矩阵" << std::endl;

	//将X和zk点云的点转为矩阵X_，Z_
	Eigen::MatrixXf Xii;
	Eigen::MatrixXf Zii;

	pcl::PCLPointCloud2::Ptr cloudX(new pcl::PCLPointCloud2);
	pcl::PCLPointCloud2::Ptr cloudZ(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*X, *cloudX);
	pcl::toPCLPointCloud2(*zk, *cloudZ);

	pcl::getPointCloudAsEigen(*cloudX, Xii);//Xii是4行的，前三行为x,y,z，最后一行为1
	pcl::getPointCloudAsEigen(*cloudZ, Zii);//同上

	Eigen::MatrixXd X_;
	Eigen::MatrixXd Z_;
	X_ = Xii.cast<double>().block(0, 0, 3, Xii.cols());
	Z_ = Zii.cast<double>().block(0, 0, 3, Zii.cols());

	/*说明性的语句*/
	std::cout << "    结束计算" << std::endl;

	/*说明性的语句*/
	std::cout << "    开始计算en" << std::endl;

	//计算当前en值（即第n次迭代的误差值），变换后的点云和Y之间的点的差值平方
	double fitness = 0;
	int fitness_count = 0;
	double en = computeEAndFitness(s, R, T, X_, Z_, fitness_count);
	fitness = (double)fitness_count / max_point;

	/*说明性的语句*/
	std::cout << "    结束计算" << std::endl;

	//计算质心坐标
	Eigen::Vector3d centroidX_ = X_.rowwise().mean();
	Eigen::Vector3d centroidZ_ = Z_.rowwise().mean();
	//定义减去质心坐标的矩阵Xi,Zi
	Eigen::MatrixXd Xi = X_ - centroidX_.replicate(1, X_.cols());
	Eigen::MatrixXd Zi = Z_ - centroidZ_.replicate(1, Z_.cols());

	/*说明性的语句*/
	std::cout << "    开始计算Rn1" << std::endl;

	//计算下一次的参数R n+1
	Eigen::Matrix3f Rn1_f = computeR(Xi.cast<float>(), Zi.cast<float>());
	Eigen::Matrix3d Rn1 = Rn1_f.cast<double>();

	/*说明性的语句*/
	std::cout << "    结束计算" << std::endl;

	/*说明性的语句*/
	std::cout << "    开始计算sn1" << std::endl;

	//计算下一次的参数s n+1
	double sn1 = computeS(I, Rn1, Xi, Zi);

	/*说明性的语句*/
	std::cout << "    结束计算" << std::endl;

	/*说明性的语句*/
	std::cout << "    开始计算Tn1" << std::endl;

	//计算T n+1
	Eigen::Vector3d Tn1 = computeT(centroidZ_, sn1, Rn1, centroidX_);

	/*说明性的语句*/
	std::cout << "    结束计算" << std::endl;

	/*说明性的语句*/
	std::cout << "    开始计算en1" << std::endl;

	//对于n+1次迭代的数据的误差值en1进行计算
	double en1 = computeE(sn1, Rn1, Tn1, X_, Z_);

	/*说明性的语句*/
	std::cout << "    结束计算" << std::endl;

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

	//获取cloud和cloud1的点数
	int pointX = cloud->points.size();
	int pointY = cloud1->points.size();

	max_point = pointX > pointY ? pointX : pointY;

	/*说明性的语句*/
	std::cout << "  开始计算初始化参数" << std::endl;

	//初始化R、T、S
	init_param_t initParam;
	initParam = init(cloud, cloud1, pointX, pointY);

	/*说明性的语句*/
	std::cout << "  结束计算" << std::endl;

	Eigen::Matrix<double, 1, 2> I = initParam.I;
	//第一组数据
	next_param_t nextParam0 = Solvecircle(initParam.s, initParam.R, initParam.T, I, cloud, cloud1);
	Eigen::Matrix3d R = nextParam0.R;
	Eigen::Vector3d T = nextParam0.T;
	double s = nextParam0.s;
	double etemp = nextParam0.en1;

	double o = 0.01;//迭代终止条件
	int iteration = 2;//迭代次数
	double q = 1;
	double en1 = 0, fitness = 0;
	/*说明性的语句*/
	std::cout << "  开始迭代" << std::endl;

	while (q > o) {
		//执行变换，并计算下一次的参数值
		next_param_t nextParam = Solvecircle(s, R, T, I, cloud, cloud1);
		R << nextParam.R;
		T << nextParam.T;
		s = nextParam.s;
		en1 = nextParam.en1;
		fitness = nextParam.fitness;

		q = 1 - en1 / etemp;//先更新q
		etemp = en1;//更新en1
		iteration = iteration + 1;
	}

	/*说明性的语句*/
	std::cout << "  结束迭代" << std::endl;


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
	
	std::string filenameX = "E:\\locks\\data\\bunny\\reconstruction\\bun1.ply";//工件
	std::string filenameY = "E:\\locks\\data\\bunny\\reconstruction\\bun_zipper.ply";//模型
	

	if (pcl::io::loadPLYFile<pcl::PointXYZ>(filenameX, *X) == -1 || pcl::io::loadPLYFile<pcl::PointXYZ>(filenameY, *Y) == -1)
	{
		PCL_ERROR("Couldn't read file \n");
		system("PAUSE");
		return (-1);
	}

	show_param_t showParam;

	/*说明性的语句*/
	std::cout << "开始计算showParam" << std::endl;

	showParam = reg3D(X, Y);

	std::cout << "s = " << showParam.s << std::endl;
	std::cout << "R = " << showParam.R << std::endl;
	std::cout << "T = " << showParam.T << std::endl;
	std::cout << "e = " << showParam.en << std::endl;
	std::cout << "fitness = " << showParam.fitness << std::endl;
	std::cout << "iteration = " << showParam.iteration << std::endl;
	if (showParam.s < 0.5 || showParam.s > 1.5 || showParam.fitness < 0.5) {
		std::cout << "陷入局部最优，二者不匹配 " << std::endl;
	}

	/*说明性的语句*/
	std::cout << "结束计算" << std::endl;

	/*说明性的语句*/
	std::cout << "可视化" << std::endl;
	
	//X先执行变换
	Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();//单位阵
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	transform.block(0, 0, 3, 3) << showParam.s * showParam.R;
	transform.block(0, 3, 3, 1) << showParam.T;
	pcl::transformPointCloud(*X, *transformed_cloud, transform);

	//可视窗口初始化
	pcl::visualization::PCLVisualizer viewer;
	//viewer.setCameraFieldOfView(0.785398);      // fov 大概45度
	viewer.setBackgroundColor(0,0,0);   // 背景设为灰色

	//可视窗口加入点云
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> inColorHandler(transformed_cloud, 255, 255, 255);// 白色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outColorHandler(Y, 230, 20, 20); // 红色
	viewer.addPointCloud(transformed_cloud, inColorHandler, "transformed_X");
	viewer.addPointCloud(Y, outColorHandler, "Y");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed_X");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Y");
	//viewer.addCoordinateSystem(0.5);
	viewer.spin();
}