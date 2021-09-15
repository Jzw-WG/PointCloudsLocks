// ObjectRecongnition.cpp: 定义控制台应用程序的入口点。
#include <stdlib.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>    //Kdtree库
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>//点云查看窗口头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/boost.h>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);



typedef pcl::PointXYZ PointType; //文中的PintType即为PointXYZRGBA
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;//SHOT特征的数据结构

std::string model_filename_;
std::string scene_filename_;

//Algorithm params
bool show_keypoints_(true);
bool show_correspondences_(true);
bool use_cloud_resolution_(false);
bool use_hough_(true);
float model_ss_(7.5f); //0.01f
float scene_ss_(20.0f);//0.03f-20
float rf_rad_(10.0f);  //0.015f-10
float descr_rad_(15.0f);//0.02f -19 - 
float cg_size_(10.0f);  //0.01f-10
float cg_thresh_(5.0f);

void
showHelp(char* filename)
{
	std::cout << std::endl;
	std::cout << "***************************************************************************" << std::endl;
	std::cout << "*                                                                         *" << std::endl;
	std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
	std::cout << "*                                                                         *" << std::endl;
	std::cout << "***************************************************************************" << std::endl << std::endl;
	std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
	std::cout << "Options:" << std::endl;
	std::cout << "     -h:                     Show this help." << std::endl;
	std::cout << "     -k:                     Show used keypoints." << std::endl;
	std::cout << "     -c:                     Show used correspondences." << std::endl;
	std::cout << "     -r:                     Compute the model cloud resolution and multiply" << std::endl;
	std::cout << "                             each radius given by that value." << std::endl;
	std::cout << "     --algorithm (Hough|GC): Clustering algorithm used (default Hough)." << std::endl;
	std::cout << "     --model_ss val:         Model uniform sampling radius (default 0.01)" << std::endl;
	std::cout << "     --scene_ss val:         Scene uniform sampling radius (default 0.03)" << std::endl;
	std::cout << "     --rf_rad val:           Reference frame radius (default 0.015)" << std::endl;
	std::cout << "     --descr_rad val:        Descriptor radius (default 0.02)" << std::endl;
	std::cout << "     --cg_size val:          Cluster size (default 0.01)" << std::endl;
	std::cout << "     --cg_thresh val:        Clustering threshold (default 5)" << std::endl << std::endl;
}

void
parseCommandLine(int argc, char* argv[])
{
	//Show help
	if (pcl::console::find_switch(argc, argv, "-h"))
	{
		showHelp(argv[0]);
		exit(0);
	}

	//Model & scene filenames
	std::vector<int> filenames;
	filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
	if (filenames.size() != 2)
	{
		std::cout << "Filenames missing.\n";
		showHelp(argv[0]);
		exit(-1);
	}

	model_filename_ = argv[filenames[0]];
	scene_filename_ = argv[filenames[1]];

	//Program behavior
	if (pcl::console::find_switch(argc, argv, "-k"))
	{
		show_keypoints_ = true;
	}
	if (pcl::console::find_switch(argc, argv, "-c"))
	{
		show_correspondences_ = true;
	}
	if (pcl::console::find_switch(argc, argv, "-r"))
	{
		use_cloud_resolution_ = true;
	}

	std::string used_algorithm;
	if (pcl::console::parse_argument(argc, argv, "--algorithm", used_algorithm) != -1)
	{
		if (used_algorithm.compare("Hough") == 0)
		{
			use_hough_ = true;
		}
		else if (used_algorithm.compare("GC") == 0)
		{
			use_hough_ = false;
		}
		else
		{
			std::cout << "Wrong algorithm name.\n";
			showHelp(argv[0]);
			exit(-1);
		}
	}

	//General parameters
	pcl::console::parse_argument(argc, argv, "--model_ss", model_ss_);
	pcl::console::parse_argument(argc, argv, "--scene_ss", scene_ss_);
	pcl::console::parse_argument(argc, argv, "--rf_rad", rf_rad_);
	pcl::console::parse_argument(argc, argv, "--descr_rad", descr_rad_);
	pcl::console::parse_argument(argc, argv, "--cg_size", cg_size_);
	pcl::console::parse_argument(argc, argv, "--cg_thresh", cg_thresh_);
}

/**下一个函数对给定点云执行空间分辨率计算，平均每个浊点与其最近邻居之间的距离。**/

double computeCloudResolution(const pcl::PointCloud<PointType>::ConstPtr& cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<PointType> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!std::isfinite((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}

int main(int argc, char* argv[])
{
	//parseCommandLine(argc, argv); //解析控制行命令

	pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>()); //定义模型点云类型，和特征点
	pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());

	pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>());
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>());

	//
	//  Load clouds
	//
	/*if (pcl::io::loadPCDFile(model_filename_, *model) < 0)
	{
	std::cout << "Error loading model cloud." << std::endl;
	showHelp(argv[0]);
	return (-1);
	}
	if (pcl::io::loadPCDFile(scene_filename_, *scene) < 0)
	{
	std::cout << "Error loading scene cloud." << std::endl;
	showHelp(argv[0]);
	return (-1);
	}
	*/

	pcl::io::loadPLYFile("..\\..\\..\\..\\data\\gen\\raw8\\handled\\lock_1_045_statistic.ply", *model);
	pcl::io::loadPLYFile("..\\..\\..\\..\\data\\gen\\raw8\\lock_1_000.ply", *scene);
	//pcl::io::loadPLYFile("..\\..\\..\\..\\data\\gen\\raw8\\handled\\lock_2_000-3.ply", *model);
	//pcl::io::loadPLYFile("..\\..\\..\\..\\data\\gen\\raw8\\lock_2_000.ply", *scene);

	/*
	pcl::visualization::CloudViewer viewer("Correspondence Grouping");
	viewer.showCloud(scene);
	while (!viewer.wasStopped())
	{
	}*/

	//
	//  Set up resolution invariance 
	//设置解析不变率,作为第二步，
	//仅当在命令行中启用了分辨率不变标志时，程序才会调整将在下一部分中使用的半径，将它们乘以估计的模型云分辨率。
	//
	use_cloud_resolution_ = true;
	if (use_cloud_resolution_) //初始值为false
	{
		float resolution = static_cast<float> (computeCloudResolution(model));
		if (resolution != 0.0f)
		{
			model_ss_ *= resolution;
			scene_ss_ *= resolution;
			rf_rad_ *= resolution;
			descr_rad_ *= resolution;
			cg_size_ *= resolution;
		}

		std::cout << "Model resolution:       " << resolution << std::endl;
		std::cout << "Model sampling size:    " << model_ss_ << std::endl;
		std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
		std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
		std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
		std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
	}

	//
	//  Compute Normals
	//接下来，它使用：pcl::NormalEstimationOMP <pcl :: NormalEstimationOMP>`估算器
	//计算模型和场景云的每个点的法线，使用每个点的10个最近邻居（此参数似乎相当不错 许多数据集，而不仅仅是提供的数据集）。
	//
	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	norm_est.setKSearch(10);
	norm_est.setNumberOfThreads(4); //为了解决报错问题：User Error 1001: argument to num_threads clause must be positive
	//由于设置的线程数必须为正，而程序中可能没有设置，有时候甚至环境变量中设置了，但是依然报错，我们不妨手动设置一下
	norm_est.setInputCloud(model);
	norm_est.compute(*model_normals);

	norm_est.setInputCloud(scene);
	norm_est.compute(*scene_normals);

	//
	//  Downsample Clouds to Extract keypoints
	//然后，它对每个云进行下采样，以便找到少量关键点，然后将这些关键点与3D描述符相关联，以便执行关键点匹配并确定点对点对应关系。 
	//用于：pcl：`UniformSampling <pcl :: UniformSampling>的半径是使用命令行开关设置的半径或默认值。
	//

	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud(model);
	uniform_sampling.setRadiusSearch(model_ss_);
	uniform_sampling.filter(*model_keypoints);
	std::cout << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;

	uniform_sampling.setInputCloud(scene);
	uniform_sampling.setRadiusSearch(scene_ss_);
	uniform_sampling.filter(*scene_keypoints);
	std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;


	//
	//  Compute Descriptor for keypoints
	//下一阶段包括将3D描述符与每个模型和场景关键点相关联。 
	//在我们的教程中，我们使用以下命令计算SHOT描述符：pcl：`SHOTEstimationOMP <pcl :: SHOTEstimationOMP>`。
	//
	pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
	descr_est.setRadiusSearch(descr_rad_);

	descr_est.setInputCloud(model_keypoints);
	descr_est.setInputNormals(model_normals);
	descr_est.setNumberOfThreads(4);
	descr_est.setSearchSurface(model);
	descr_est.compute(*model_descriptors);

	descr_est.setInputCloud(scene_keypoints);
	descr_est.setInputNormals(scene_normals);
	descr_est.setNumberOfThreads(4);
	descr_est.setSearchSurface(scene);
	descr_est.compute(*scene_descriptors);


	//  Find Model-Scene Correspondences with KdTree
	//现在我们需要确定模型描述符和场景描述符之间的点对点对应关系。 
	//为此，程序使用：pcl：`KdTreeFLANN <pcl :: KdTreeFLANN>`，其输入云已设置为包含模型描述符的云。 
	//对于与场景关键点相关联的每个描述符，它基于欧几里德距离有效地找到最相似的模型描述符，
	//并将该对添加到：pcl：`Correspondences <pcl :: Correspondences>`vector
	//（仅当两个描述符是 足够相似，即它们的平方距离小于阈值，设置为0.25）。
	//
	pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud(model_descriptors);

	//For each scene keypoint descriptor,
		//find nearest neighbor into the model keypoints descriptor cloudand add it to the correspondences vector.
		for (size_t i = 0; i < scene_descriptors->size(); ++i)
		{
			std::vector<int> neigh_indices(1);
			std::vector<float> neigh_sqr_dists(1);
			if (!std::isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
			{
				continue;
			}
			int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
			if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
			{
				pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
				model_scene_corrs->push_back(corr);
			}
		}
	std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

	//
	//  Actual Clustering
	//管道的最后一个阶段是先前找到的对应关系的实际聚类。
	//默认算法是：pcl：`Hough3DGrouping <pcl::Hough3DGrouping>`，它基于Hough Voting过程。 
	//请注意，此算法需要为属于云的每个关键点关联本地参考帧（LRF），这些关键点作为参数传递！ 
	//在此示例中，我们在调用聚类算法之前使用：pcl：`BOARDLocalReferenceFrameEstimation <pcl::BOARDLocalReferenceFrameEstimation>`估算器
	//显式计算LRF集。
	//
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;

	//Using Hough3D
		if (use_hough_)
		{
			//
			//  Compute (Keypoints) Reference Frames only for Hough
			//
			pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
			pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());

			pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
			rf_est.setFindHoles(true);
			rf_est.setRadiusSearch(rf_rad_);

			rf_est.setInputCloud(model_keypoints);
			rf_est.setInputNormals(model_normals);
			rf_est.setSearchSurface(model);
			rf_est.compute(*model_rf);

			rf_est.setInputCloud(scene_keypoints);
			rf_est.setInputNormals(scene_normals);
			rf_est.setSearchSurface(scene);
			rf_est.compute(*scene_rf);

			//	//  Clustering
			pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
			clusterer.setHoughBinSize(cg_size_);
			clusterer.setHoughThreshold(cg_thresh_);
			clusterer.setUseInterpolation(true);
			clusterer.setUseDistanceWeight(false);

			clusterer.setInputCloud(model_keypoints);
			clusterer.setInputRf(model_rf);
			clusterer.setSceneCloud(scene_keypoints);
			clusterer.setSceneRf(scene_rf);
			clusterer.setModelSceneCorrespondences(model_scene_corrs);

			//clusterer.cluster (clustered_corrs);
			clusterer.recognize(rototranslations, clustered_corrs);
		}
		else // Using GeometricConsistency
		{
			pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
			gc_clusterer.setGCSize(cg_size_);
			gc_clusterer.setGCThreshold(cg_thresh_);

			gc_clusterer.setInputCloud(model_keypoints);
			gc_clusterer.setSceneCloud(scene_keypoints);
			gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

			//gc_clusterer.cluster (clustered_corrs);

			gc_clusterer.recognize(rototranslations, clustered_corrs);
		}
	/*在调用聚类算法之前，没有必要显式计算LRF。 如果提取到聚类算法的云没有关联的一组LRF，
	Hough3DGrouping会在执行聚类之前自动计算它们。 特别是，在不设置LRF的情况下调用识别（或集群）方法时会发生这种情况：
	在这种情况下，您需要指定LRF的半径作为聚类算法的附加参数（使用setLocalRfSearchRadius方法）。
	*/
	/*
	作为Hough3DGrouping的替代方案，通过前面介绍的相应命令行开关，
	您可以选择使用：pcl：`GeometricConsistencyGrouping <pcl :: GeometricConsistencyGrouping>`算法。
	在这种情况下，不需要LRF计算，因此我们只是创建算法类的实例，传递正确的参数并调用识别方法。
	*/
	//
	//  Output results
	//
	std::cout << "Model instances found: " << rototranslations.size() << std::endl;
	for (size_t i = 0; i < rototranslations.size(); ++i)
	{
		std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
		std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;

		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
		Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);

		printf("\n");
		printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		printf("\n");
		printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
	}

	//
	//  Visualization
	//
	pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
	viewer.addPointCloud(scene, "scene_cloud");//可视化场景点云

	pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());


	if (show_correspondences_ || show_keypoints_)
	{
		//We are translating the model so that it doesn't end in the middle of the scene representation
		pcl::transformPointCloud(*model, *off_scene_model, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
		pcl::transformPointCloud(*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler(off_scene_model, 255, 255, 128);
		viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");
	}

	if (show_keypoints_)
	{
		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler(scene_keypoints, 0, 0, 255);
		viewer.addPointCloud(scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler(off_scene_model_keypoints, 0, 0, 255);
		viewer.addPointCloud(off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
	}

	for (size_t i = 0; i < rototranslations.size(); ++i)
	{
		pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
		pcl::transformPointCloud(*model, *rotated_model, rototranslations[i]);

		std::stringstream ss_cloud;
		ss_cloud << "instance" << i;

		pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 255, 0, 0);
		viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());

		if (show_correspondences_)
		{
			for (size_t j = 0; j < clustered_corrs[i].size(); ++j)
			{
				std::stringstream ss_line;
				ss_line << "correspondence_line" << i << "_" << j;
				PointType& model_point = off_scene_model_keypoints->at(clustered_corrs[i][j].index_query);
				PointType& scene_point = scene_keypoints->at(clustered_corrs[i][j].index_match);

				//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
				viewer.addLine<PointType, PointType>(model_point, scene_point, 0, 255, 0, ss_line.str());
			}
		}
	}
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	system("pause");
	return (0);
}