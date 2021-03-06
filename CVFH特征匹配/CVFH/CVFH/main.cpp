#include <cvfh_const.h>
#include <cvfh_build_tree.h>
#include <cvfh_nearst_neighbors.h>
#include <cvfh_generate.h>
#include <io.h>

void getAllFiles(string path, vector<string>& files, string nameCondition, string extention)
{
	//文件句柄 
	intptr_t  hFile = 0;
	//文件信息结构体
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*" + nameCondition + extention).c_str(), &fileinfo)) != -1)
	{
		do
		{
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				;
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

int main(int argc, char** argv)
{
	/*string path = "..\\..\\..\\..\\data\\gen\\model";
	string cvfh_path = path + "\\CVFH";
	string build_path = path + "\\build\\";*/

	string path = "..\\..\\..\\..\\data\\gen\\raw8\\model";
	string cvfh_path = path + "\\CVFH";
	string vfh_path = path + "\\VFH";
	string shot_path = path + "\\SHOT";
	string cvfh_build_path = path + "\\cvfh_build\\";
	string vfh_build_path = path + "\\vfh_build\\";
	string shot_build_path = path + "\\shot_build\\";
	
	//parameter
	//string feature_name = GConst::g_shot;
	string feature_name = GConst::g_cvfh;
	string mode = GConst::GENMODE;

	string build_path = "";
	string feature_path = "";
	if (feature_name == GConst::g_shot) {
		build_path = shot_build_path;
		feature_path = shot_path;
	}
	else if (feature_name == GConst::g_cvfh) {
		build_path = cvfh_build_path;
		feature_path = cvfh_path;
	}
	else if (feature_name == GConst::g_vfh) {
		build_path = vfh_build_path;
		feature_path = vfh_path;
	}
	else {
		build_path = cvfh_build_path;
		feature_path = cvfh_path;
	}
	vector<string> model_files;
	getAllFiles(path, model_files, "", ".ply");

	vector<string> feature_files;
	getAllFiles(feature_path, feature_files, "", ".pcd");

	if (mode == GConst::BUILDMODE) {
		feature_model_build(build_path, feature_files, model_files, feature_name);
	}
	else if (mode == GConst::RECOGMODE) {
		int k = 6;//要显示的数量
		double thresh = DBL_MAX;// 设置一个相似度阈值，不满足阈值的会被区别显示。默认无阈值
		//thresh = 200.0;
		pcl::visualization::PCLPlotter plotter;
		feature_model feature_descriptor;
		if (feature_name == GConst::g_shot) {
			//加载或者计算目标shot特性
			pcl::PointCloud<pcl::SHOT352> shots;
			//pcl::io::loadPCDFile<pcl::VFHSignature308>("test_vfh1.pcd", cvfhs);

			calcuate_shot("..\\..\\..\\..\\data\\gen\\scene\\handled\\lock_3_000_statistic.ply", shots);
			//calcuate_shot("..\\..\\..\\..\\data\\gen\\model\\lock_1_model.ply", shots);
			//calcuate_shot("..\\..\\..\\..\\data\\dragon\\dragon_up\\dragonUpRight_0.ply", shots);
			//calcuate_shot("..\\..\\..\\..\\data\\bunny\\data\\bun090.ply", shots);
			//calcuate_shot("..\\..\\..\\..\\data\\bunny\\reconstruction\\bun_zipper.ply", shots);
			plotter.addFeatureHistogram<pcl::SHOT352>(shots, "shot", 0);
			feature_model  descriptor;//存储名称和shot特征
			int cvfh_idx = 1;
			descriptor.second.resize(352);
			descriptor.first = "target_shot";
			for (size_t j = 0; j < 352; j++)
			{
				descriptor.second[j] = shots.points[0].descriptor[j];
			}
			feature_descriptor = descriptor;
		}
		else if (feature_name == GConst::g_cvfh) {
			//加载或者计算目标cvfh特性
			pcl::PointCloud<pcl::VFHSignature308> cvfhs;
			//pcl::io::loadPCDFile<pcl::VFHSignature308>("test_cvfh1.pcd", cvfhs);

			calcuate_cvfh("..\\..\\..\\..\\data\\gen\\scene\\handled\\lock_3_000_statistic.ply", cvfhs);
			//calcuate_cvfh("..\\..\\..\\..\\data\\gen\\model\\lock_1_model.ply", cvfhs);
			//calcuate_cvfh("..\\..\\..\\..\\data\\dragon\\dragon_up\\dragonUpRight_0.ply", cvfhs);
			//calcuate_cvfh("..\\..\\..\\..\\data\\bunny\\data\\bun090.ply", cvfhs);
			//calcuate_cvfh("..\\..\\..\\..\\data\\bunny\\reconstruction\\bun_zipper.ply", cvfhs);
			plotter.addFeatureHistogram<pcl::VFHSignature308>(cvfhs, "vfh", 0);
			feature_model  histogram;//存储名称和cvfh特征
			int cvfh_idx = 1;
			histogram.second.resize(308);
			histogram.first = "target_cvfh";
			for (size_t j = 0; j < 308; j++)
			{
				histogram.second[j] = cvfhs.points[0].histogram[j];
			}
			feature_descriptor = histogram;
		}
		else if (feature_name == GConst::g_vfh) {
			//加载或者计算目标vfh特性
			pcl::PointCloud<pcl::VFHSignature308> vfhs;
			//pcl::io::loadPCDFile<pcl::VFHSignature308>("test_vfh1.pcd", vfhs);

			calcuate_vfh("..\\..\\..\\..\\data\\gen\\scene\\handled\\lock_2_090-3_nd.ply", vfhs);
			//calcuate_vfh("..\\..\\..\\..\\data\\gen\\model\\lock_1_model.ply", vfhs);
			//calcuate_vfh("..\\..\\..\\..\\data\\dragon\\dragon_up\\dragonUpRight_0.ply", vfhs);
			//calcuate_vfh("..\\..\\..\\..\\data\\bunny\\data\\bun090.ply", vfhs);
			//calcuate_vfh("..\\..\\..\\..\\data\\bunny\\reconstruction\\bun_zipper.ply", vfhs);
			plotter.addFeatureHistogram<pcl::VFHSignature308>(vfhs, "vfh", 0);
			feature_model  histogram;//存储名称和vfh特征
			int vfh_idx = 1;
			histogram.second.resize(308);
			histogram.first = "target_vfh";
			for (size_t j = 0; j < 308; j++)
			{
				histogram.second[j] = vfhs.points[0].histogram[j];
			}
			feature_descriptor = histogram;
		}

		std::vector<feature_model> models;
		flann::Matrix<int> k_indices;
		flann::Matrix<float> k_distances;
		flann::Matrix<float> data;

		//从.list文件中加载各模型名称t
		if (!boost::filesystem::exists(build_path + GConst::training_data_h5_file_name) || !boost::filesystem::exists(build_path + GConst::training_data_list_file_name))
		{
			pcl::console::print_error("Could not find training data models files %s and %s!\n",
				GConst::training_data_h5_file_name.c_str(), GConst::training_data_list_file_name.c_str());
			return (-1);
		}
		else
		{
			loadFileList(models, build_path + GConst::training_data_list_file_name);
			flann::load_from_file(data, build_path + GConst::training_data_h5_file_name, "training_data");
			pcl::console::print_highlight("Training data found. Loaded %d CVFH models from %s/%s.\n",
				(int)data.rows, GConst::training_data_h5_file_name.c_str(), GConst::training_data_list_file_name.c_str());
		}
		//进行k近邻搜索
		if (!boost::filesystem::exists(build_path + GConst::kdtree_idx_file_name))
		{
			pcl::console::print_error("Could not find kd-tree index in file %s!", GConst::kdtree_idx_file_name.c_str());
			return (-1);
		}
		else
		{
			flann::Index<flann::ChiSquareDistance<float> > index(data, flann::SavedIndexParams(build_path + GConst::kdtree_idx_file_name));
			index.buildIndex();
			nearestKSearch(index, feature_descriptor, k, k_indices, k_distances);
		}

		// 屏幕上近邻结果
		for (int i = 0; i < k; ++i)
			pcl::console::print_info("    %d - %s (%d) with a distance of: %f\n",
				i, models.at(k_indices[0][i]).first.c_str(), k_indices[0][i], k_distances[0][i]);


		//多个视图显示结果
		pcl::visualization::PCLVisualizer viewer("Feature Cluster Classifier");
		int viewport = 0, l = 0, m = 0;
		//显示布局
		int y_s = (int)floor(sqrt((double)k));
		int x_s = y_s + (int)ceil((k / (double)y_s) - y_s);
		double x_step = (double)(1 / (double)x_s);
		double y_step = (double)(1 / (double)y_s);
		pcl::console::print_highlight("Preparing to load ");
		pcl::console::print_value("%d", k);
		pcl::console::print_info(" files (");
		pcl::console::print_value("%d", x_s);
		pcl::console::print_info("x");
		pcl::console::print_value("%d", y_s);
		pcl::console::print_info(" / ");
		pcl::console::print_value("%f", x_step);
		pcl::console::print_info("x");
		pcl::console::print_value("%f", y_step);
		pcl::console::print_info(")\n");
		for (int i = 0; i < k; ++i)
		{
			std::string cloud_name = models.at(k_indices[0][i]).first;//模板名称
			viewer.createViewPort(l * x_step, m * y_step, (l + 1) * x_step, (m + 1) * y_step, viewport);
			l++;
			if (l >= x_s)
			{
				l = 0;
				m++;
			}
			//添加点云
			pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::console::print_highlight(stderr, "Loading "); pcl::console::print_value(stderr, "%s ", cloud_name.c_str());
			std::stringstream ply_name;
			ply_name << cloud_name;
			if (pcl::io::loadPLYFile(ply_name.str(), *temp_cloud) == -1)
			{
				cout << "load failed!" << endl;
				break;
			}
			//将点云平移到原点，便于显示
			Eigen::Vector4f centroid;//计算重心
			pcl::compute3DCentroid(*temp_cloud, centroid);
			for (size_t i = 0; i < temp_cloud->points.size(); ++i)
			{
				temp_cloud->points[i].x = temp_cloud->points[i].x - centroid[0];
				temp_cloud->points[i].y = temp_cloud->points[i].y - centroid[1];
				temp_cloud->points[i].z = temp_cloud->points[i].z - centroid[2];
			}
			std::stringstream viewer_name;
			viewer_name << "viewer_cloud" << i;
			viewer.addPointCloud(temp_cloud, viewer_name.str(), viewport);
			pcl::console::print_info("load point size:");
			pcl::console::print_value("%d\n", (int)(*temp_cloud).points.size());

			//如果不满足所设置的阈值，则名称用红色显示且用直线删掉。默认没有阈值
			std::stringstream ss;
			ss << k_distances[0][i];
			if (k_distances[0][i] > thresh)
			{
				viewer.addText(ss.str(), 20, 30, 1, 0, 0, ss.str(), viewport);  // display the text with red
																				// Create a red line（创建一条红线）
				pcl::PointXYZ min_p, max_p;
				pcl::getMinMax3D(*temp_cloud, min_p, max_p);
				std::stringstream line_name;
				line_name << "line_" << i;
				viewer.addLine(min_p, max_p, 1, 0, 0, line_name.str(), viewport);
				viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, line_name.str(), viewport);
			}
			else
			{
				viewer.addText(ss.str(), 20, 30, 0, 1, 0, ss.str(), viewport);
			}
			//设置字体大小
			viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 18, ss.str(), viewport);
			//显示模板名称
			viewer.addText(cloud_name, 20, 10, cloud_name, viewport);
		}
		viewer.setCameraPosition(0, -30, 0, 0, 0, 0, 0, 0, 1, 0);//视角
		plotter.plot();//显示特征
		viewer.spin();
	}
	else
	{
		save_feature(feature_path, model_files, feature_name);
	}
	
	return 0;
}