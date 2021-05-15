// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#define NOMINMAX  //��Ϊwindows.h�ͱ�׼���е�min maxģ�庯����ͻ������Ҫ����һ��
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

//�ַ����ָ��
static std::vector<std::string> split(std::string str, std::string pattern)
{
    std::string::size_type pos;
    std::vector<std::string> result;
    str += pattern;//��չ�ַ����Է������
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
    //�ļ����
    intptr_t hFile = 0;
    //�ļ���Ϣ
    struct _finddata_t fileinfo;
    string p;
    if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
    {
        do
        {
            //�����Ŀ¼,����֮���������,�����б�
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
    //����1���򿪹����ļ����
    HANDLE shared_file = OpenFileMapping(
        FILE_MAP_ALL_ACCESS,//����ģʽ:�ɶ�д
        FALSE,
        L"ShareMemory"  //�����ڴ�����
    );

    if (shared_file == NULL)
    {
        printf("Could not open file mapping object...");
        return -1;
    }

    //����2��ӳ�仺������ͼ���õ�ָ�����ڴ��ָ��
    LPVOID lpBUF = MapViewOfFile(
        shared_file, //�Ѵ������ļ�ӳ�������
        FILE_MAP_ALL_ACCESS,//����ģʽ:�ɶ�д
        0, //�ļ�ƫ�Ƶĸ�32λ
        0, //�ļ�ƫ�Ƶĵ�32λ
        0 //ӳ����ͼ�Ĵ�С,0��ʾ��ƫ�������ļ�ӳ���ĩβ����Ϊ�����ļ�open�˲�֪�����С������д0
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

    //����3�����������ڴ�
    while (1) {
        printf("Receive data from other process:\n");
        WaitForSingleObject(H_Event, INFINITE);
        WaitForSingleObject(H_Mutex, INFINITE); //ʹ�û��������
        memcpy(Buffer, lpBUF, strlen((char*)lpBUF) + 1);
        ReleaseMutex(H_Mutex); //����
        printf(Buffer);//��ӡ��һ���̸��Ĳ���
        printf("\n");

        //��ȡXL XH YL YH ZL ZH����
        getXYAxis(Buffer, XL, XH, YL, YH, ZL, ZH,LockType);

        //����kinect���
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
                    //ֻ�ɼ�����
                    returnCode = capture(argv[2]);
                }
                else if (argc == 5 && std::string(argv[3]) == "registrate")
                {
                    //�ɼ�����
                    returnCode = capture(argv[2]);
                    //��ȡmodel_directory·���µ��ļ�
                    //vector<string> files;
                    //getFiles(argv[4], files);
                    //��ȡ�����ļ���������׼
                    std::string filenameX = std::string(argv[4]) + "\\Lock" + std::to_string(LockType) + ".ply";
                    std::string filenameY = std::string(argv[2]) + "\\depth_to_color.ply";//Todo: ��Ӧ��д��
                    pcl::PointCloud<pcl::PointXYZ>::Ptr X(new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::PointCloud<pcl::PointXYZ>::Ptr Y(new pcl::PointCloud<pcl::PointXYZ>);

                    if (pcl::io::loadPLYFile<pcl::PointXYZ>(filenameY, *Y) == -1 || pcl::io::loadPLYFile<pcl::PointXYZ>(filenameX, *X) == -1)
                    {
                        PCL_ERROR("Couldn't read file \n");
                        return (-1);
                    }          
                    show_param_t showParam = reg3D(X, Y);
                    std::cout << filenameY + " �� " + filenameX + " ��׼���Ϊ��" << std::endl;
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

    //����4�����ӳ��͹رվ��
    UnmapViewOfFile(lpBUF);
    CloseHandle(shared_file);
    system("pause");
    return 0;//�����˳�
}