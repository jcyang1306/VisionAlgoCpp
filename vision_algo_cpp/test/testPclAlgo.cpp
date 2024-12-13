#include "VisionAlgoCpp.h"

int main(int argc, char** argv)
{
    if (argc != 2) {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Please specify input data file");
        return 0;
    }

    // Load data
    char* ply_file = argv[1];
    std::string file_name_str = ply_file;
    RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "Load data from ply file: %s ...", file_name_str.c_str());
    PointCloudColor::Ptr cloud = pclLoadFromPLYFile(file_name_str);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Data loading done with point cloud size: [%d]", cloud->width * cloud->height);

    // Crop with cropbox
    Eigen::Vector4f min_point(-0.25f, -0.5f, 0.5f, 1.0f);
    Eigen::Vector4f max_point(0.75f, 0.5f, 1.5f, 1.0f);
    PointCloudColor::Ptr cloud_cropped = pclCropBox(cloud, min_point, max_point);
   
    // Down sample with 0.002 voxel size
    cloud_cropped = pclVoxelDownSample(cloud_cropped, 0.002);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), ">>> Test PCL Utils cpp done");

    return 1;
}