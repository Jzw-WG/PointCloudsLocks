#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <PointCloudFilters.h>
#include <fstream>//写入txt
#include <string>

void getFileList(string strPath, vector<string>& fileList, vector<string>& ownname, string nameCondition, string extention) {
    //文件句柄
    intptr_t  hFile = 0;
    //文件信息
    struct _finddata_t fileinfo;
    string p;
    if ((hFile = _findfirst(p.assign(strPath).append("\\*" + nameCondition + extention).c_str(), &fileinfo)) != -1)
    {
        do
        {
            //如果是目录,迭代之
            //如果不是,加入列表
            if ((fileinfo.attrib & _A_SUBDIR))
            {
                continue;//不递归调用
                /*if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
                {
                    getFileList(p.assign(strPath).append("\\").append(fileinfo.name), fileList, ownname, nameCondition, extention);
                }*/
            }
            else
            {
                fileList.push_back(strPath + "\\" + fileinfo.name);
                ownname.push_back(fileinfo.name);
            }
        } while (_findnext(hFile, &fileinfo) == 0);
        _findclose(hFile);
    }
}

int pre_process(string inputFileName, string outputFileName) {
	PointCloud::Ptr inputcloud(new PointCloud);
	PointCloud::Ptr outputcloud(new PointCloud);
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(inputFileName, *inputcloud) == -1) {
		PCL_ERROR("Couldnot read file.\n");
		system("pause");
		return(-1);
	}

	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*inputcloud, minPt, maxPt);

	pathThroughFilter(inputcloud, outputcloud, "x", -0.16, 0.16);
	pathThroughFilter(outputcloud, outputcloud, "y", maxPt.y - 0.076 - 0.3, maxPt.y - 0.076);//maxPt.y - 0.06 - 0.3, maxPt.y - 0.06
	pathThroughFilter(outputcloud, outputcloud, "z", 0.55, 0.9 - 0.2);//0.6 0.9-0.15

	statisticalOutlierRemovalFilter(outputcloud, outputcloud, 20, 1.5);//20 0.2

	//voxelFilter(outputcloud, outputcloud, 0.001, 0.001, 0.001);
	//regionGrowingSimplify(inputcloud);
	pcl::io::savePLYFile(outputFileName, *outputcloud);
	return 0;
}

int main(int argc, char** argv) {
	//string inputFileName = "..\\..\\..\\..\\data\\gen\\raw32\\lock_3_348.ply";
	//string outputFileName = "..\\..\\..\\..\\data\\gen\\raw32\\handled\\lock_3_348_statistic.ply";
	string inputFileName = "..\\..\\..\\..\\data\\gen\\raw32";
	string outputFileName = "..\\..\\..\\..\\data\\gen\\raw32\\handled";
	//string inputFileName = "..\\..\\..\\..\\data\\bunny\\reconstruction\\bun_zipper.ply";
	//string outputFileName = "..\\..\\..\\..\\data\\stdmodel\\bunny_model.ply";
	//string inputFileName = "..\\..\\..\\..\\data\\gen\\raw16\\handled\\model\\lock_3_model.ply";
	//string outputFileName = "..\\..\\..\\..\\data\\gen\\raw16\\handled\\model\\lock_3_model_desnse.ply";

	string tag = "_statistic";
    if (inputFileName.find(".ply") != -1) {
		pre_process(inputFileName, outputFileName);
	}
	else {
		vector<string> filenamelist;
		vector<string> ownnamelist;
		string nameCondition = "lock*"; //例"asd*" 或 ""
		string extention = ".ply";
		getFileList(inputFileName, filenamelist, ownnamelist, nameCondition, extention);
		for (size_t i = 0; i < filenamelist.size(); i++)
		{
			string ownname = ownnamelist[i];
			pre_process(filenamelist[i], outputFileName + "\\" + ownname.substr(0, ownname.rfind('.')) + tag + extention);
		}
	}
	return 0;
}
