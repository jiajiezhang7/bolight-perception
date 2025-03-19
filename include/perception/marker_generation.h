#ifndef _MARKER_GENERATION_H
#define _MARKER_GENERATION_H

#include <visualization_msgs/Marker.h>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
visualization_msgs::Marker cylinder_marker(std::string frame, std::string ns, int id, double x, double y, double z, double h, double d, float ox, float oy, float oz, float ow, float r, float g, float b, float a);
visualization_msgs::Marker pc_marker(std::string frame, std::string ns, int id, float sx, float sy, float r, float g, float b, float a, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
#endif