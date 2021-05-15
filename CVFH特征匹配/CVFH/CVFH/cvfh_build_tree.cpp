//#include <vtkAutoInit.h>//可视化时需要使用vtk
//VTK_MODULE_INIT(vtkRenderingOpenGL2);
//VTK_MODULE_INIT(vtkInteractionStyle);
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/console/parse.h>
//#include <pcl/console/print.h>
//#include <pcl/io/pcd_io.h>
//#include <boost/filesystem.hpp>
//#include <flann/flann.h>
//#include <flann/io/hdf5.h>
//#include <fstream>
//
//typedef std::pair<std::string, std::vector<float> > cvfh_model;//一个用于存储名称，一个用于存储vrf特征
//using namespace std;
//
////根据vfh特征数据，生成kd_tree，即建立模板库
////其中vfhData文件夹下的.pcd文件储存了vfh特征和xyz格式的模板点云数据
//int main(int argc, char** argv)
//{
//	//加载vfh特征
//	std::vector<cvfh_model> models;
//	for (int i = 0; i < 6; i++)
//	{
//		//读取vfh特征
//		std::stringstream ss;
//		ss << "modelData\\model_cvfh_" << i << ".pcd";
//		cvfh_model cvfh;//存储名称和vfh特征
//		int cvfh_idx = 1;
//		pcl::PointCloud<pcl::VFHSignature308> cvfhs;
//		pcl::io::loadPCDFile<pcl::VFHSignature308>(ss.str(), cvfhs);
//		cvfh.second.resize(308);
//
//		//将vfh信息存入vector容器
//		std::stringstream ss1;
//		ss1 << "model_" << i;
//		cvfh.first = ss1.str();
//		for (size_t j = 0; j < 308; j++)
//		{
//			cvfh.second[j] = cvfhs.points[0].histogram[j];
//		}
//		models.push_back(cvfh);
//	}
//
//	//训练数据
//	std::string kdtree_idx_file_name = "kdtree.idx";
//	std::string training_data_h5_file_name = "training_data.h5";
//	std::string training_data_list_file_name = "training_data.list";
//	pcl::console::print_highlight("Loaded %d VFH models. Creating training data %s/%s.\n", (int)models.size(), training_data_h5_file_name.c_str(), training_data_list_file_name.c_str());
//
//	//转化数据为FLANN格式
//	flann::Matrix<float> data(new float[models.size() * models[0].second.size()], models.size(), models[0].second.size());
//	for (size_t i = 0; i < data.rows; ++i)
//		for (size_t j = 0; j < data.cols; ++j)
//			data[i][j] = models[i].second[j];
//	//cout << data.rows << endl;
//	//cout << data.cols << endl;
//	cout << models.size() << endl;
//	//保存数据到磁盘
//	flann::save_to_file(data, training_data_h5_file_name, "training_data");
//	std::ofstream fs;
//	fs.open(training_data_list_file_name.c_str());
//	for (size_t i = 0; i < models.size(); ++i)
//		fs << models[i].first << "\n";
//	fs.close();
//
//	//建立树的索引并将其存储在磁盘上
//	pcl::console::print_error("Building the kdtree index (%s) for %d elements...\n", kdtree_idx_file_name.c_str(), (int)data.rows);
//	flann::Index<flann::ChiSquareDistance<float> > index(data, flann::LinearIndexParams());
//	//flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
//	index.buildIndex();
//	index.save(kdtree_idx_file_name);
//	delete[] data.ptr();
//
//	system("pause");
//	return 0;
//}
