#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include <string>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

geometry_msgs::TransformStamped set_tf(std::string header_frame_id, std::string child_frame_id, double x, double y, double z, double r_x, double r_y, double r_z, double r_w){
    geometry_msgs::TransformStamped ts;

    //---set header
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = header_frame_id;

    //---set child frame
    ts.child_frame_id = child_frame_id;

    //---set translation
    ts.transform.translation.x = x;
    ts.transform.translation.y = y;
    ts.transform.translation.z = z;
    //---set quaternion
    ts.transform.rotation.x = r_x;
    ts.transform.rotation.y = r_y;
    ts.transform.rotation.z = r_z;
    ts.transform.rotation.w = r_w;
    return ts;
};

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"static_brocast");
   
    tf2_ros::StaticTransformBroadcaster broadcaster;
    
    geometry_msgs::TransformStamped ts0,ts1,ts2,ts3;

    std::string header_id, child_id;

    double x,y,z,r_x,r_y,r_z,r_w;

    YAML::Node config = YAML::LoadFile(ros::package::getPath("perception")+"/config/lidars2baselink.yaml");

    // transformation from lidar1 to baselink
    header_id = config["lidar1"]["frame_id"].as<std::string>();
    child_id = config["lidar1"]["child_frame_id"].as<std::string>();
    x = config["lidar1"]["translation"]["x"].as<double>(); 
    y = config["lidar1"]["translation"]["y"].as<double>(); 
    z = config["lidar1"]["translation"]["z"].as<double>();
    r_x = config["lidar1"]["rotation"]["x"].as<double>(); 
    r_y = config["lidar1"]["rotation"]["y"].as<double>(); 
    r_z = config["lidar1"]["rotation"]["z"].as<double>();
    r_w = config["lidar1"]["rotation"]["w"].as<double>();
    ts1 = set_tf(header_id,child_id,x,y,z,r_x,r_y,r_z,r_w);

    // // transformation from lidar2 to baselink
    header_id = config["lidar2"]["frame_id"].as<std::string>();
    child_id = config["lidar2"]["child_frame_id"].as<std::string>();
    x = config["lidar2"]["translation"]["x"].as<double>(); 
    y = config["lidar2"]["translation"]["y"].as<double>(); 
    z = config["lidar2"]["translation"]["z"].as<double>();
    r_x = config["lidar2"]["rotation"]["x"].as<double>(); 
    r_y = config["lidar2"]["rotation"]["y"].as<double>(); 
    r_z = config["lidar2"]["rotation"]["z"].as<double>();
    r_w = config["lidar2"]["rotation"]["w"].as<double>();
    ts2 = set_tf(header_id,child_id,x,y,z,r_x,r_y,r_z,r_w);

    // // transformation from lidar0 to baselink
    // header_id = config["lidar0"]["frame_id"].as<std::string>();
    // child_id = config["lidar0"]["child_frame_id"].as<std::string>();
    // x = config["lidar0"]["translation"]["x"].as<double>(); 
    // y = config["lidar0"]["translation"]["y"].as<double>(); 
    // z = config["lidar0"]["translation"]["z"].as<double>();
    // r_x = config["lidar0"]["rotation"]["x"].as<double>(); 
    // r_y = config["lidar0"]["rotation"]["y"].as<double>(); 
    // r_z = config["lidar0"]["rotation"]["z"].as<double>();
    // r_w = config["lidar0"]["rotation"]["w"].as<double>();
    // ts0 = set_tf(header_id,child_id,x,y,z,r_x,r_y,r_z,r_w);
    
    // transformation from baselink to world
    header_id = config["megatron"]["frame_id"].as<std::string>();
    child_id = config["megatron"]["child_frame_id"].as<std::string>();
    x = config["megatron"]["translation"]["x"].as<double>(); 
    y = config["megatron"]["translation"]["y"].as<double>(); 
    z = config["megatron"]["translation"]["z"].as<double>();
    r_x = config["megatron"]["rotation"]["x"].as<double>(); 
    r_y = config["megatron"]["rotation"]["y"].as<double>(); 
    r_z = config["megatron"]["rotation"]["z"].as<double>();
    r_w = config["megatron"]["rotation"]["w"].as<double>();
    ts3 = set_tf(header_id,child_id,x,y,z,r_x,r_y,r_z,r_w);
    
    // Publish static transform
    broadcaster.sendTransform(ts1);
    broadcaster.sendTransform(ts2);
    // broadcaster.sendTransform(ts0);
    broadcaster.sendTransform(ts3);
    ros::spin();
    return 0;
}