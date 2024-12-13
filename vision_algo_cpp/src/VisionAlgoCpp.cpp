#include "VisionAlgoCpp.h"

PointCloudColor::Ptr pclLoadFromPLYFile(std::string file_name)
{
    PointCloudColor::Ptr result = boost::make_shared<PointCloudColor>();

    if (pcl::io::loadPLYFile<pcl::PointXYZRGB> (file_name, *result) == -1) //* load the file
    {
        return result;
    }
    // std::cout << "Loaded "
    //             << result->width * result->height
    //             << " data points from test_pcd.pcd with the following fields: "
    //             << std::endl;
    // for (const auto& point: *result)
    //     std::cout << "    " << point.x
    //             << " "    << point.y
    //             << " "    << point.z << std::endl;

    return result;
}