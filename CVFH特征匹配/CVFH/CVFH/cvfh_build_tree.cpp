#include <cvfh_build_tree.h>

using namespace std;

void getBox(string file, string& h, string& w, string& d) {
	string::size_type iPos = file.find_last_of('\\') + 1;
	string filename = file.substr(iPos, file.length() - iPos);
	string name = filename.substr(0, filename.rfind("."));
	string suffix_str = filename.substr(filename.find_last_of('.') + 1);
	int boxpos = filename.find(GConst::g_box);
	string bounding_box = "";
	h = "";
	w = "";
	if (boxpos > 0) {
		bounding_box = name.substr(boxpos + 3 + 1);
		h = bounding_box.substr(0, bounding_box.rfind("-"));
		w = bounding_box.substr(bounding_box.rfind("-") + 1);
	}
}

//根据cvfh特征数据，生成kd_tree，即建立模板库
//其中CVFH文件夹下的.pcd文件储存了cvfh特征和xyz格式的模板点云数据
int cvfh_model_build(string path, vector<string> files, vector<string> model_files)
{
	//加载cvfh特征
	std::vector<feature_model> models;
	int idx = 0;
	for (string s : files)
	{
		//找到cvfh文件中的包围盒信息
		string maxh_str = "";
		string maxw_str = "";
		string maxd_str = "";
		getBox(s,maxh_str,maxw_str,maxd_str);
		
		//读取cvfh特征
		feature_model cvfh;//存储名称和特征
		int cvfh_idx = 1;
		pcl::PointCloud<pcl::VFHSignature308> cvfhs;
		pcl::io::loadPCDFile<pcl::VFHSignature308>(s, cvfhs);
		cvfh.second.resize(308);

		//将cvfh信息存入vector容器
		cvfh.first = model_files[idx];
		for (size_t j = 0; j < 308; j++)
		{
			cvfh.second[j] = cvfhs.points[0].histogram[j];
		}
		models.push_back(cvfh);
		idx++;
	}
	//按包围盒分类，按大小过滤，加速识别速度 TODO


	//训练数据
	pcl::console::print_highlight("Loaded %d CVFH models. Creating training data %s/%s.\n", (int)models.size(), GConst::training_data_h5_file_name.c_str(), GConst::training_data_list_file_name.c_str());

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
	flann::save_to_file(data, p.assign(path).append(GConst::training_data_h5_file_name), "training_data");
	std::ofstream fs;
	fs.open(p.assign(path).append(GConst::training_data_list_file_name));
	for (size_t i = 0; i < models.size(); ++i)
		fs << models[i].first << "\n";
	fs.close();

	//建立树的索引并将其存储在磁盘上
	pcl::console::print_error("Building the kdtree index (%s) for %d elements...\n", GConst::kdtree_idx_file_name.c_str(), (int)data.rows);
	flann::Index<flann::ChiSquareDistance<float> > index(data, flann::LinearIndexParams());
	//flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
	index.buildIndex();
	index.save(p.assign(path).append(GConst::kdtree_idx_file_name));
	delete[] data.ptr();

	system("pause");
	return 0;
}

//根据vfh特征数据，生成kd_tree，即建立模板库
//其中VFH文件夹下的.pcd文件储存了vfh特征和xyz格式的模板点云数据
int vfh_model_build(string path, vector<string> files, vector<string> model_files)
{
	//加载vfh特征
	std::vector<feature_model> models;
	int idx = 0;
	for (string s : files)
	{
		//找到vfh文件中的包围盒信息
		string maxh_str = "";
		string maxw_str = "";
		string maxd_str = "";
		getBox(s, maxh_str, maxw_str, maxd_str);

		//读取vfh特征
		feature_model vfh;//存储名称和特征
		int vfh_idx = 1;
		pcl::PointCloud<pcl::VFHSignature308> vfhs;
		pcl::io::loadPCDFile<pcl::VFHSignature308>(s, vfhs);
		vfh.second.resize(308);

		//将cvfh信息存入vector容器
		vfh.first = model_files[idx];
		for (size_t j = 0; j < 308; j++)
		{
			vfh.second[j] = vfhs.points[0].histogram[j];
		}
		models.push_back(vfh);
		idx++;
	}
	//按包围盒分类，按大小过滤，加速识别速度 TODO


	//训练数据
	pcl::console::print_highlight("Loaded %d VFH models. Creating training data %s/%s.\n", (int)models.size(), GConst::training_data_h5_file_name.c_str(), GConst::training_data_list_file_name.c_str());

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
	flann::save_to_file(data, p.assign(path).append(GConst::training_data_h5_file_name), "training_data");
	std::ofstream fs;
	fs.open(p.assign(path).append(GConst::training_data_list_file_name));
	for (size_t i = 0; i < models.size(); ++i)
		fs << models[i].first << "\n";
	fs.close();

	//建立树的索引并将其存储在磁盘上
	pcl::console::print_error("Building the kdtree index (%s) for %d elements...\n", GConst::kdtree_idx_file_name.c_str(), (int)data.rows);
	flann::Index<flann::ChiSquareDistance<float> > index(data, flann::LinearIndexParams());
	//flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
	index.buildIndex();
	index.save(p.assign(path).append(GConst::kdtree_idx_file_name));
	delete[] data.ptr();

	system("pause");
	return 0;
}

//根据shot特征数据，生成kd_tree，即建立模板库
//其中SHOT文件夹下的.pcd文件储存了shot特征和xyz格式的模板点云数据
int shot_model_build(string path, vector<string> files, vector<string> model_files)
{
	//加载shot特征
	std::vector<feature_model> models;
	int idx = 0;
	for (string s : files)
	{
		//找到shot文件中的包围盒信息
		string maxh_str = "";
		string maxw_str = "";
		string maxd_str = "";
		getBox(s, maxh_str, maxw_str, maxd_str);

		//读取shot特征
		feature_model shot;//存储名称和shot特征
		int shot_idx = 1;
		pcl::PointCloud<pcl::SHOT352> shots;
		pcl::io::loadPCDFile<pcl::SHOT352>(s, shots);
		cout << "loadpcd: " << idx << " " << "succeeded" << endl;
		shot.second.resize(352);

		//将shot信息存入vector容器
		shot.first = model_files[idx];
		for (size_t j = 0; j < 352; j++)
		{
			shot.second[j] = shots.points[0].descriptor[j];
		}
		models.push_back(shot);
		idx++;
	}
	//按包围盒分类，按大小过滤，加速识别速度 TODO


	//训练数据
	pcl::console::print_highlight("Loaded %d SHOT models. Creating training data %s/%s.\n", (int)models.size(), GConst::training_data_h5_file_name.c_str(), GConst::training_data_list_file_name.c_str());

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
	flann::save_to_file(data, p.assign(path).append(GConst::training_data_h5_file_name), "training_data");
	std::ofstream fs;
	fs.open(p.assign(path).append(GConst::training_data_list_file_name));
	for (size_t i = 0; i < models.size(); ++i)
		fs << models[i].first << "\n";
	fs.close();

	//建立树的索引并将其存储在磁盘上
	pcl::console::print_error("Building the kdtree index (%s) for %d elements...\n", GConst::kdtree_idx_file_name.c_str(), (int)data.rows);
	flann::Index<flann::ChiSquareDistance<float> > index(data, flann::LinearIndexParams());
	//flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
	index.buildIndex();
	index.save(p.assign(path).append(GConst::kdtree_idx_file_name));
	delete[] data.ptr();

	system("pause");
	return 0;
}

int feature_model_build(string path, vector<string> files, vector<string> model_files, string feature_name) {
	if (feature_name == GConst::g_shot) {
		return shot_model_build(path, files, model_files);
	}
	else if (feature_name == GConst::g_cvfh) {
		return cvfh_model_build(path, files, model_files);
	}
	else if (feature_name == GConst::g_vfh) {
		return vfh_model_build(path, files, model_files);
	}
	return -1;
}
