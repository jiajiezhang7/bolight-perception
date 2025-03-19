#include <perception/marker_generation.h>

visualization_msgs::Marker cylinder_marker(std::string frame, \
    std::string ns, int id, double x, double y, double z, double h, double d, float ox, float oy, float oz, float ow, float r, float g, float b, float a){

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp. The default values are fine for this example.
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker. This serves to create a unique ID.
    marker.ns = ns;
    marker.id = id;

    // Set the marker type. CUBE stands for a cube.
    marker.type = visualization_msgs::Marker::CYLINDER;

    // Set the marker action. Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;  // Set the X position of the marker
    marker.pose.position.y = y;  // Set the Y position of the marker
    marker.pose.position.z = z;  // Set the Z position of the marker
    marker.pose.orientation.x = ox;
    marker.pose.orientation.y = oy;
    marker.pose.orientation.z = oz;
    marker.pose.orientation.w = ow;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = d;  // Set the X scale of the marker
    marker.scale.y = d;  // Set the Y scale of the marker
    marker.scale.z = h;  // Set the Z scale of the marker

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    return marker;
}

visualization_msgs::Marker pc_marker(std::string frame, std::string ns, int id, float sx, float sy, float r, float g, float b, float a, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    // marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    // 设置Marker的尺寸
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;


    // 设置Marker的颜色和透明度
    marker.color.a = a;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    
    // 遍历点云数据，根据条件将点添加到不同的Marker中
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = cloud->points[i].x;
        p.y = cloud->points[i].y;
        p.z = cloud->points[i].z;
        marker.points.push_back(p);
    }
    return marker;
}