#include <iostream>
#include <GeographicLib/Geodesic.hpp>
#include <pcl/point_types.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"

pcl::PointXYZ lonlat2xyz(double lat1, double lon1, double hei1, double lat2, double lon2, tf2_ros::Buffer& tf_buffer);