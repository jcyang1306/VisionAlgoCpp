#pragma once

#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#ifndef LOGGER_NAME
#define LOGGER_NAME (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#endif

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudColor;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudMono;

inline PointCloudColor::Ptr pclVoxelDownSample(PointCloudColor::Ptr pc_src, float voxel_size)
{
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    sor.setInputCloud(pc_src);
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    sor.filter(*cloud_filtered);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Voxel downsample points size: [%d]->[%d]", pc_src->size(), cloud_filtered->size());
    
    return cloud_filtered;
}

inline PointCloudColor::Ptr pclCropBox(PointCloudColor::Ptr cloud_src, 
                                       Eigen::Vector4f& min_point, Eigen::Vector4f& max_point)
{
    pcl::CropBox<pcl::PointXYZRGB> clipper;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    clipper.setInputCloud(cloud_src);
    clipper.setMin(min_point);
    clipper.setMax(max_point);
    clipper.filter(*cloud_filtered);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "CropBox points size: [%d]->[%d]", cloud_src->size(), cloud_filtered->size());

    return cloud_filtered;
}


PointCloudColor::Ptr pclLoadFromPLYFile(std::string file_name);


