#include <cvfh_const.h>

using namespace std;
const string GConst::BUILDMODE = "build";
const string GConst::RECOGMODE = "recognition";
const string GConst::GENMODE = "generate";

const string GConst::kdtree_idx_file_name = "kdtree.idx";
const string GConst::training_data_h5_file_name = "training_data.h5";
const string GConst::training_data_list_file_name = "training_data.list";

const string GConst::g_cvfh = "cvfh";
const string GConst::g_shot = "shot";
const string GConst::g_box = "box";

const int GConst::boxsizerange1 = 1000;
const int GConst::boxsizerange2 = 1800;
const int GConst::boxsizerange3 = 2500;

const float GConst::min_distance = 0.6f;