﻿#include <cvfh_build_tree.h>

using namespace std;

//根据vfh特征数据，生成kd_tree，即建立模板库
//其中vfhData文件夹下的.pcd文件储存了vfh特征和xyz格式的模板点云数据
int build(string path, vector<string> files, vector<string> model_files)
{
	//加载vfh特征
	std::vector<cvfh_model> models;
	int idx = 0;
	for (string s : files)
	{
		//读取vfh特征
		cvfh_model cvfh;//存储名称和vfh特征
		int cvfh_idx = 1;
		pcl::PointCloud<pcl::VFHSignature308> cvfhs;
		pcl::io::loadPCDFile<pcl::VFHSignature308>(s, cvfhs);
		cvfh.second.resize(308);

		//将vfh信息存入vector容器
		cvfh.first = model_files[idx];
		for (size_t j = 0; j < 308; j++)
		{
			cvfh.second[j] = cvfhs.points[0].histogram[j];
		}
		models.push_back(cvfh);
		idx++;
	}

	//训练数据
	std::string kdtree_idx_file_name = "kdtree.idx";
	std::string training_data_h5_file_name = "training_data.h5";
	std::string training_data_list_file_name = "training_data.list";
	pcl::console::print_highlight("Loaded %d VFH models. Creating training data %s/%s.\n", (int)models.size(), training_data_h5_file_name.c_str(), training_data_list_file_name.c_str());

	//转化数据为FLANN格式
	flann::Matrix<float> data(new float[models.size() * models[0].second.size()], models.size(), models[0].second.size());
	for (size_t i = 0; i < data.rows; ++i)
		for (size_t j = 0; j < data.cols; ++j)
			data[i][j] = models[i].second[j];
	//cout << data.rows << endl;
	//cout << data.cols << endl;
	cout << models.size() << endl;
	//保存数据到磁盘
	string p;
	flann::save_to_file(data, p.assign(path).append(training_data_h5_file_name), "training_data");
	std::ofstream fs;
	fs.open(p.assign(path).append(training_data_list_file_name));
	for (size_t i = 0; i < models.size(); ++i)
		fs << models[i].first << "\n";
	fs.close();

	//建立树的索引并将其存储在磁盘上
	pcl::console::print_error("Building the kdtree index (%s) for %d elements...\n", kdtree_idx_file_name.c_str(), (int)data.rows);
	flann::Index<flann::ChiSquareDistance<float> > index(data, flann::LinearIndexParams());
	//flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
	index.buildIndex();
	index.save(p.assign(path).append(kdtree_idx_file_name));
	delete[] data.ptr();

	system("pause");
	return 0;
}
