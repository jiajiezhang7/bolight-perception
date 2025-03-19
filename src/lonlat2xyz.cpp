#include <perception/lonlat2xyz.h>

pcl::PointXYZ lonlat2xyz(double lat1, double lon1, double hei1, double lat2, double lon2, tf2_ros::Buffer& tf_buffer){
    GeographicLib::Geodesic geod = GeographicLib::Geodesic::WGS84();

    double dis_west, dis_north;

    geod.Inverse(lat1, lon1, lat1, lon2, dis_west);
    geod.Inverse(lat1, lon1, lat2, lon1, dis_north);
    if (lat1 > lat2) dis_north = -dis_north;
    if (lon1 < lon2) dis_west = -dis_west; 

    // we set the frame of 4 stands at the central bottom.
    pcl::PointXYZ res(dis_north, dis_west, 0 - hei1);
    
    // pts under world -> pts under base_link
    geometry_msgs::PointStamped point_under_world, point_under_baselink;
    point_under_world.point.x = res.x;
    point_under_world.point.y = res.y;
    point_under_world.point.z = res.z;
    point_under_world.header.frame_id = "world";
    point_under_baselink = tf_buffer.transform(point_under_world, "base_link");
    res.x = point_under_baselink.point.x;
    res.y = point_under_baselink.point.y;
    res.z = point_under_baselink.point.z;
    return res;
}