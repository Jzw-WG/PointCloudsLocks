#include <cvfh_build_tree.h>
#include <cvfh_nearst_neighbors.h>
int main(int argc, char** argv)
{
	string BUILDMODE = "build";
	string RECOGMODE = "recognition";

	string mode = BUILDMODE;

	if (mode == BUILDMODE) {
		build();
	}
	else if (mode == RECOGMODE) {
		int k = 6;//Ҫ��ʾ������
		double thresh = DBL_MAX;// ����һ�����ƶ���ֵ����������ֵ�Ļᱻ������ʾ��Ĭ������ֵ
		//thresh = 200.0;

		//���ػ��߼���Ŀ��vfh����
		pcl::PointCloud<pcl::VFHSignature308> cvfhs;
		//pcl::io::loadPCDFile<pcl::VFHSignature308>("test_vfh1.pcd", cvfhs);

		calcuate_cvfh("..\\..\\..\\..\\data\\gen\\handled\\lock_1_000_statistic.ply", cvfhs);
		pcl::visualization::PCLPlotter plotter;
		plotter.addFeatureHistogram<pcl::VFHSignature308>(cvfhs, "vfh", 0);
		cvfh_model  histogram;//�洢���ƺ�vfh����
		int cvfh_idx = 1;
		histogram.second.resize(308);
		histogram.first = "target_vfh";
		for (size_t j = 0; j < 308; j++)
		{
			histogram.second[j] = cvfhs.points[0].histogram[j];
		}


		std::string kdtree_idx_file_name = "kdtree.idx";
		std::string training_data_h5_file_name = "training_data.h5";
		std::string training_data_list_file_name = "training_data.list";
		std::vector<cvfh_model> models;
		flann::Matrix<int> k_indices;
		flann::Matrix<float> k_distances;
		flann::Matrix<float> data;

		//��.list�ļ��м��ظ�ģ������t
		if (!boost::filesystem::exists("training_data.h5") || !boost::filesystem::exists("training_data.list"))
		{
			pcl::console::print_error("Could not find training data models files %s and %s!\n",
				training_data_h5_file_name.c_str(), training_data_list_file_name.c_str());
			return (-1);
		}
		else
		{
			loadFileList(models, training_data_list_file_name);
			flann::load_from_file(data, training_data_h5_file_name, "training_data");
			pcl::console::print_highlight("Training data found. Loaded %d VFH models from %s/%s.\n",
				(int)data.rows, training_data_h5_file_name.c_str(), training_data_list_file_name.c_str());
		}
		//����k��������
		if (!boost::filesystem::exists(kdtree_idx_file_name))
		{
			pcl::console::print_error("Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str());
			return (-1);
		}
		else
		{
			flann::Index<flann::ChiSquareDistance<float> > index(data, flann::SavedIndexParams("kdtree.idx"));
			index.buildIndex();
			nearestKSearch(index, histogram, k, k_indices, k_distances);
		}

		// ��Ļ�Ͻ��ڽ��
		for (int i = 0; i < k; ++i)
			pcl::console::print_info("    %d - %s (%d) with a distance of: %f\n",
				i, models.at(k_indices[0][i]).first.c_str(), k_indices[0][i], k_distances[0][i]);


		//�����ͼ��ʾ���
		pcl::visualization::PCLVisualizer viewer("VFH Cluster Classifier");
		int viewport = 0, l = 0, m = 0;
		//��ʾ����
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
			std::string cloud_name = models.at(k_indices[0][i]).first;//ģ������
			viewer.createViewPort(l * x_step, m * y_step, (l + 1) * x_step, (m + 1) * y_step, viewport);
			l++;
			if (l >= x_s)
			{
				l = 0;
				m++;
			}
			//��ӵ���
			pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::console::print_highlight(stderr, "Loading "); pcl::console::print_value(stderr, "%s ", cloud_name.c_str());
			std::stringstream ply_name;
			ply_name << "modelData\\" << cloud_name << ".ply";
			if (pcl::io::loadPLYFile(ply_name.str(), *temp_cloud) == -1)
			{
				cout << "load failed!" << endl;
				break;
			}
			//������ƽ�Ƶ�ԭ�㣬������ʾ
			Eigen::Vector4f centroid;//��������
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

			//��������������õ���ֵ���������ú�ɫ��ʾ����ֱ��ɾ����Ĭ��û����ֵ
			std::stringstream ss;
			ss << k_distances[0][i];
			if (k_distances[0][i] > thresh)
			{
				viewer.addText(ss.str(), 20, 30, 1, 0, 0, ss.str(), viewport);  // display the text with red
																				// Create a red line������һ�����ߣ�
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
			//���������С
			viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 18, ss.str(), viewport);
			//��ʾģ������
			viewer.addText(cloud_name, 20, 10, cloud_name, viewport);
		}
		viewer.setCameraPosition(0, -30, 0, 0, 0, 0, 0, 0, 1, 0);//�ӽ�
		plotter.plot();//��ʾcvfh����
		viewer.spin();
	}
	
	return 0;
}