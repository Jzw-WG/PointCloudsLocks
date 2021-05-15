// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "transformation_helpers.h"

#include <iostream>
#include <fstream>
#include <sstream>

#include <pcl/point_types.h>
#include <vector>
#include <pcl/common/eigen.h>

#define _USE_MATH_DEFINES //需要放在math前,之后才可以使用M_PI等match定义参数
#include <math.h>

struct color_point_t
{
    int16_t xyz[3];
    uint8_t rgb[3];
};

void tranformation_helpers_write_point_cloud(const k4a_image_t point_cloud_image,
                                             const k4a_image_t color_image,
                                             const char *file_name,
                                             int XL, int XH, int YL, int YH, double ZL, double ZH)
{
    std::vector<pcl::PointXYZ> points;

    int width = k4a_image_get_width_pixels(point_cloud_image);
    int height = k4a_image_get_height_pixels(color_image);

    int16_t *point_cloud_image_data = (int16_t *)(void *)k4a_image_get_buffer(point_cloud_image);
    uint8_t *color_image_data = k4a_image_get_buffer(color_image);

    //kinect坐标系 与 opengl显示所用的坐标系稍有不同，因此进行调整
    //详见链接https://my.oschina.net/u/4580379/blog/4532910
    Eigen::Matrix3f m;
    m = Eigen::AngleAxisf(-M_PI,Eigen::Vector3f::UnitZ());//绕Z轴顺时针旋转180度

    Eigen::Matrix3f m_;
    m_ = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY());//绕Y轴逆时针旋转180度

    for (int i = 0; i < width * height; i++)
    {
        int y = i / width;
        int x = i - y * width;
        if (x < XL || x > XH || y < YL || y > YH)
            continue;
  
        pcl::PointXYZ point;
        point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
        point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
        point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

        if (point.z == 0 || point.z < ZL || point.z > ZH)
        {
            continue;
        }
        point.getVector3fMap() = m * m_ * point.getVector3fMap();
        
        uint8_t b = color_image_data[4 * i + 0];
        uint8_t g = color_image_data[4 * i + 1];
        uint8_t r = color_image_data[4 * i + 2];
        
        uint8_t alpha = color_image_data[4 * i + 3];

        if (b == 0 && g == 0 && r == 0 && alpha == 0)
        {
            continue;
        }

        points.push_back(point);
    }

#define PLY_START_HEADER "ply"
#define PLY_END_HEADER "end_header"
#define PLY_ASCII "format ascii 1.0"
#define PLY_ELEMENT_VERTEX "element vertex"

    // save to the ply file
    std::ofstream ofs(file_name); // text mode first
    ofs << PLY_START_HEADER << std::endl;
    ofs << PLY_ASCII << std::endl;
    ofs << PLY_ELEMENT_VERTEX << " " << points.size() << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    /*
    ofs << "property uchar red" << std::endl;
    ofs << "property uchar green" << std::endl;
    ofs << "property uchar blue" << std::endl;
    */
    ofs << PLY_END_HEADER << std::endl;
    ofs.close();

    std::stringstream ss;
    for (size_t i = 0; i < points.size(); ++i)
    {
        // image data is BGR
        ss << (float)points[i].x << " " << (float)points[i].y << " " << (float)points[i].z;
        //ss << " " << (float)points[i].r << " " << (float)points[i].g << " " << (float)points[i].b;
        ss << std::endl;
    }
    std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

k4a_image_t downscale_image_2x2_binning(const k4a_image_t color_image)
{
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
    int color_image_downscaled_width_pixels = color_image_width_pixels / 2;
    int color_image_downscaled_height_pixels = color_image_height_pixels / 2;
    k4a_image_t color_image_downscaled = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                 color_image_downscaled_width_pixels,
                                                 color_image_downscaled_height_pixels,
                                                 color_image_downscaled_width_pixels * 4 * (int)sizeof(uint8_t),
                                                 &color_image_downscaled))
    {
        printf("Failed to create downscaled color image\n");
        return color_image_downscaled;
    }

    uint8_t *color_image_data = k4a_image_get_buffer(color_image);
    uint8_t *color_image_downscaled_data = k4a_image_get_buffer(color_image_downscaled);
    for (int j = 0; j < color_image_downscaled_height_pixels; j++)
    {
        for (int i = 0; i < color_image_downscaled_width_pixels; i++)
        {
            int index_downscaled = j * color_image_downscaled_width_pixels + i;
            int index_tl = (j * 2 + 0) * color_image_width_pixels + i * 2 + 0;
            int index_tr = (j * 2 + 0) * color_image_width_pixels + i * 2 + 1;
            int index_bl = (j * 2 + 1) * color_image_width_pixels + i * 2 + 0;
            int index_br = (j * 2 + 1) * color_image_width_pixels + i * 2 + 1;

            color_image_downscaled_data[4 * index_downscaled + 0] = (uint8_t)(
                (color_image_data[4 * index_tl + 0] + color_image_data[4 * index_tr + 0] +
                 color_image_data[4 * index_bl + 0] + color_image_data[4 * index_br + 0]) /
                4.0f);
            color_image_downscaled_data[4 * index_downscaled + 1] = (uint8_t)(
                (color_image_data[4 * index_tl + 1] + color_image_data[4 * index_tr + 1] +
                 color_image_data[4 * index_bl + 1] + color_image_data[4 * index_br + 1]) /
                4.0f);
            color_image_downscaled_data[4 * index_downscaled + 2] = (uint8_t)(
                (color_image_data[4 * index_tl + 2] + color_image_data[4 * index_tr + 2] +
                 color_image_data[4 * index_bl + 2] + color_image_data[4 * index_br + 2]) /
                4.0f);
            color_image_downscaled_data[4 * index_downscaled + 3] = (uint8_t)(
                (color_image_data[4 * index_tl + 3] + color_image_data[4 * index_tr + 3] +
                 color_image_data[4 * index_bl + 3] + color_image_data[4 * index_br + 3]) /
                4.0f);
        }
    }

    return color_image_downscaled;
}
