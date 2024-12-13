cmake_minimum_required(VERSION 3.5)
project(vision_algo_cpp VERSION 0.0.1)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall )
endif()

#### Find dependencies ####
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL COMPONENTS common features io REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/VisionAlgoCpp.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/include/pcl-1.10
  /usr/include/eigen3
)

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

# Target dependencies 
ament_target_dependencies(${PROJECT_NAME}
  Eigen3
  OpenCV
  rclcpp
  PCL_COMMON
  PCL_FEATURES
  PCL_IO
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Test file
add_executable(testPclAlgo test/testPclAlgo.cpp)
target_link_libraries(testPclAlgo ${PROJECT_NAME})

# Install
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS testPclAlgo DESTINATION bin)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
