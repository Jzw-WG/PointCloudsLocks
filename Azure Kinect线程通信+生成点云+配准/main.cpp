// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#define NOMINMAX  //因为windows.h和标准库中的min max模板函数冲突，所以要加这一句
#include <Windows.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include<io.h>
#include <k4a/k4a.h>
#include <string>
#include <vector>

#include "transformation_helpers.h"
#include "registration3D.h"
#include "turbojpeg.h"

using namespace std;

HANDLE H_Mutex = NULL;
HANDLE H_Event = NULL;
int XL = -1, XH = -1, YL = -1, YH = -1, LockType = 1;
double ZL = -1, ZH = -1;

static bool point_cloud_depth_to_color(k4a_transformation_t transformation_handle,
    const k4a_image_t depth_image,
    const k4a_image_t color_image,
    std::string file_name,
    int xl, int xh, int yl, int yh, double zl, double zh)
{
    // transform color image into depth camera geometry
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
    k4a_image_t transformed_depth_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t),
        &transformed_depth_image))
    {
        printf("Failed to create transformed depth image\n");
        return false;
    }

    k4a_image_t point_cloud_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * 3 * (int)sizeof(int16_t),
        &point_cloud_image))
    {
        printf("Failed to create point cloud image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image))
    {
        printf("Failed to compute transformed depth image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
        transformed_depth_image,
        K4A_CALIBRATION_TYPE_COLOR,
        point_cloud_image))
    {
        printf("Failed to compute point cloud\n");
        return false;
    }

    if (xh == -1) xh = k4a_image_get_width_pixels(point_cloud_image) - 1;
    if (yh == -1) yh = k4a_image_get_height_pixels(color_image) - 1;
    if (zh == -1) zh = 10;

    tranformation_helpers_write_point_cloud(point_cloud_image, color_image, file_name.c_str(), xl, xh, yl, yh, zl, zh);

    k4a_image_release(transformed_depth_image);
    k4a_image_release(point_cloud_image);

    return true;
}

static int capture(std::string output_dir, uint8_t deviceId = K4A_DEVICE_DEFAULT)
{
    int returnCode = 1;
    k4a_device_t device = NULL;
    const int32_t TIMEOUT_IN_MS = 10000;
    k4a_transformation_t transformation = NULL;
    k4a_transformation_t transformation_color_downscaled = NULL;
    k4a_capture_t capture = NULL;
    std::string file_name = "";
    uint32_t device_count = 0;
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;
    k4a_image_t color_image_downscaled = NULL;

    device_count = k4a_device_get_installed_count();

    if (device_count == 0)
    {
        printf("No K4A devices found\n");
        return 0;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceId, &device))
    {
        printf("Failed to open device\n");
        goto Exit;
    }

    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_5;
    config.synchronized_images_only = true; // ensures that depth and color images are both available in the capture

    k4a_calibration_t calibration;
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        printf("Failed to get calibration\n");
        goto Exit;
    }

    transformation = k4a_transformation_create(&calibration);

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        printf("Failed to start cameras\n");
        goto Exit;
    }

    // Get a capture
    switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
    {
    case K4A_WAIT_RESULT_SUCCEEDED:
        break;
    case K4A_WAIT_RESULT_TIMEOUT:
        printf("Timed out waiting for a capture\n");
        goto Exit;
    case K4A_WAIT_RESULT_FAILED:
        printf("Failed to read a capture\n");
        goto Exit;
    }

    // Get a depth image
    depth_image = k4a_capture_get_depth_image(capture);
    if (depth_image == 0)
    {
        printf("Failed to get depth image from capture\n");
        goto Exit;
    }

    // Get a color image
    color_image = k4a_capture_get_color_image(capture);
    if (color_image == 0)
    {
        printf("Failed to get color image from capture\n");
        goto Exit;
    }

    // Compute color point cloud by warping depth image into color camera geometry
#ifdef _WIN32
    file_name = output_dir + "\\depth_to_color.ply";
#else
    file_name = output_dir + "/depth_to_color.ply";
#endif
    if (point_cloud_depth_to_color(transformation, depth_image, color_image, file_name.c_str(), XL, XH, YL, YH, ZL, ZH) == false)
    {
        goto Exit;
    }

    returnCode = 0;

Exit:
    if (depth_image != NULL)
    {
        k4a_image_release(depth_image);
    }
    if (color_image != NULL)
    {
        k4a_image_release(color_image);
    }
    if (capture != NULL)
    {
        k4a_capture_release(capture);
    }
    if (transformation != NULL)
    {
        k4a_transformation_destroy(transformation);
    }
    if (transformation_color_downscaled != NULL)
    {
        k4a_transformation_destroy(transformation_color_downscaled);
    }
    if (device != NULL)
    {
        k4a_device_close(device);
    }
    return returnCode;
}

//字符串分割函数
static std::vector<std::string> split(std::string str, std::string pattern)
{
    std::string::size_type pos;
    std::vector<std::string> result;
    str += pattern;//扩展字符串以方便操作
    int size = str.size();
    for (int i = 0; i < size; i++)
    {
        pos = str.find(pattern, i);
        if (pos < size)
        {
            std::string s = str.substr(i, pos - i);
            result.push_back(s);
            i = pos + pattern.size() - 1;
        }
    }
    return result;
}

static void getXYAxis(char buffer[], int& XL, int& XH, int& YL, int& YH, double& ZL, double& ZH, int& LockType) {
    string str = buffer;
    if (str == "-1") {
        XL = -1;
        XH = -1;
        YL = -1;
        YH = -1;
        ZL = -1;
        ZH = -1;
        LockType = 1;
    }
    else {
        std::vector<std::string> result = split(str, " ");
        XL = stoi(result[0]);
        XH = stoi(result[1]);
        YL = stoi(result[2]);
        YH = stoi(result[3]);
        ZL = stod(result[4]);
        ZH = stod(result[5]);
        LockType = stoi(result[6]);
    }
}

static void getFiles(string path, vector<string>& files) {
    //文件句柄
    intptr_t hFile = 0;
    //文件信息
    struct _finddata_t fileinfo;
    string p;
    if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
    {
        do
        {
            //如果是目录,迭代之；如果不是,加入列表
            if ((fileinfo.attrib & _A_SUBDIR))
            {
                if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
                    getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
            }
            else
            {
                files.push_back(p.assign(path).append("\\").append(fileinfo.name));
            }
        } while (_findnext(hFile, &fileinfo) == 0);
        _findclose(hFile);
    }
}

static void print_usage()
{
    printf("Usage1: transformation_example capture <output_directory>\n");
    printf("Usage2: transformation_example capture <output_directory> registrate <model_directory>\n");
}


int main(int argc, char** argv) {
    //步骤1：打开共享文件句柄
    HANDLE shared_file = OpenFileMapping(
        FILE_MAP_ALL_ACCESS,//访问模式:可读写
        FALSE,
        L"ShareMemory"  //共享内存名称
    );

    if (shared_file == NULL)
    {
        printf("Could not open file mapping object...");
        return -1;
    }

    //步骤2：映射缓存区视图，得到指向共享内存的指针
    LPVOID lpBUF = MapViewOfFile(
        shared_file, //已创建的文件映射对象句柄
        FILE_MAP_ALL_ACCESS,//访问模式:可读写
        0, //文件偏移的高32位
        0, //文件偏移的低32位
        0 //映射视图的大小,0表示从偏移量到文件映射的末尾，因为共享文件open端不知道其大小，所以写0
    );

    if (lpBUF == NULL)
    {
        printf("Could not create file mapping object...");
        CloseHandle(shared_file);
        return -1;
    }

    H_Mutex = OpenMutex(MUTEX_ALL_ACCESS, FALSE, L"sm_mutex");
    if (H_Mutex == NULL)
    {
        printf("open mutex failed...");
        return -1;
    }

    H_Event = OpenEvent(EVENT_ALL_ACCESS, FALSE, L"sm_event");
    if (H_Event == NULL)
    {
        printf("open mutex failed...");
        return -1;
    }

    char Buffer[97];

    //步骤3：操作共享内存
    while (1) {
        printf("Receive data from other process:\n");
        WaitForSingleObject(H_Event, INFINITE);
        WaitForSingleObject(H_Mutex, INFINITE); //使用互斥体加锁
        memcpy(Buffer, lpBUF, strlen((char*)lpBUF) + 1);
        ReleaseMutex(H_Mutex); //放锁
        printf(Buffer);//打印另一进程给的参数
        printf("\n");

        //获取XL XH YL YH ZL ZH参数
        getXYAxis(Buffer, XL, XH, YL, YH, ZL, ZH,LockType);

        //启动kinect相机
        int returnCode = 0;
        if (argc < 2)
        {
            print_usage();
            return 0;
        }
        else
        {
            std::string mode = std::string(argv[1]);
            if (mode == "capture")
            {
                if (argc == 3)
                {
                    //只采集点云
                    returnCode = capture(argv[2]);
                }
                else if (argc == 5 && std::string(argv[3]) == "registrate")
                {
                    //采集点云
                    returnCode = capture(argv[2]);
                    //获取model_directory路径下的文件
                    //vector<string> files;
                    //getFiles(argv[4], files);
                    //读取点云文件，点云配准
                    std::string filenameX = std::string(argv[4]) + "\\Lock" + std::to_string(LockType) + ".ply";
                    std::string filenameY = std::string(argv[2]) + "\\depth_to_color.ply";//Todo: 不应该写死
                    pcl::PointCloud<pcl::PointXYZ>::Ptr X(new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::PointCloud<pcl::PointXYZ>::Ptr Y(new pcl::PointCloud<pcl::PointXYZ>);

                    if (pcl::io::loadPLYFile<pcl::PointXYZ>(filenameY, *Y) == -1 || pcl::io::loadPLYFile<pcl::PointXYZ>(filenameX, *X) == -1)
                    {
                        PCL_ERROR("Couldn't read file \n");
                        return (-1);
                    }          
                    show_param_t showParam = reg3D(X, Y);
                    std::cout << filenameY + " 与 " + filenameX + " 配准结果为：" << std::endl;
                    showResult(showParam, X, Y);
                    std::cout << "\n" << std::endl;
                }
                else
                {
                    print_usage();
                    return 0;
                }
            }
            else
            {
                print_usage();
                return 0;
            }
        }
    }
    CloseHandle(H_Event);
    CloseHandle(H_Mutex);

    //步骤4：解除映射和关闭句柄
    UnmapViewOfFile(lpBUF);
    CloseHandle(shared_file);
    system("pause");
    return 0;//正常退出
}