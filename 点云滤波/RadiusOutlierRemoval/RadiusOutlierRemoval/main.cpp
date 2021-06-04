#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <PointCloudFilters.h>
#include <fstream>//写入txt
#include<string>

int main(int argc, char** argv) {
	string inputFileName = "..\\..\\..\\..\\data\\gen\\lock_1_000.ply";
	string outputFileName = "..\\..\\..\\..\\data\\gen\\handled\\lock_1_000_pass.ply";
	PointCloud::Ptr inputcloud(new PointCloud);
	PointCloud::Ptr outputcloud(new PointCloud);
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(inputFileName, *inputcloud) == -1) {
		PCL_ERROR("Couldnot read file.\n");
		system("pause");
		return(-1);
	}

	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*inputcloud, minPt, maxPt);

	//statisticalOutlierRemovalFilter(inputFileName, outputFileName);
	pathThroughFilter(inputcloud, outputcloud, "x", -0.16, 0.16);
	pathThroughFilter(outputcloud, outputcloud, "y", maxPt.y - 0.06 - 0.3, maxPt.y - 0.06);
	pathThroughFilter(outputcloud, outputcloud, "z", 0.6, 0.9 - 0.15);
	
	statisticalOutlierRemovalFilter(outputcloud, outputcloud, 20, 0.2);

	pcl::io::savePLYFile(outputFileName, *outputcloud);
	return 0;
}
