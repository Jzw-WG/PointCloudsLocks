#pragma once
#include<iostream>

class GConst
{
public:
	GConst();
	~GConst();
	
	static const std::string BUILDMODE;
	static const std::string RECOGMODE;
	static const std::string GENMODE;

	static const std::string kdtree_idx_file_name;
	static const std::string training_data_h5_file_name;
	static const std::string training_data_list_file_name;
private:

};
