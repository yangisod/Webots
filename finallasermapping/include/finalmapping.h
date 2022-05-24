#include <ros/ros.h>
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>
#include <future>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid_label.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/range_image/range_image.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <octomap/octomap.h>
// #include <octomap/OcTree.h>
#include <octomap/OcTreeStamped.h>
#include <octomap/AbstractOccupancyOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <webots_ros/set_bool.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_float.h>

#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;
typedef pcl::PointXYZI PointXYZI;
typedef pcl::PointXYZINormal PointXYZINormal;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<pcl::PointXYZI> pclCloudXYZI;
typedef pcl::PointCloud<pcl::PointXYZ> pclCloudXYZ;
#define delaunay_color Scalar(0, 0, 0)