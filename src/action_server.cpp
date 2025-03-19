#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/TransformStamped.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2_ros/buffer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <vector>
#include <chrono>
#include <perception/marker_generation.h>
#include <perception/lonlat2xyz.h>
#include <perception/stands_seg.h>
#include <yaml-cpp/yaml.h>
//action
#include "actionlib/server/simple_action_server.h"
#include "perception/detectStandsAction.h"
#include <pcl/registration/icp.h>

int frames = 0;

typedef actionlib::SimpleActionServer<perception::detectStandsAction> Server;
tf2_ros::Buffer tf_buffer;
const std::string target_frame = "base_link";

// store all pc from three lidar
pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud_copy(new pcl::PointCloud<pcl::PointXYZ>);

std::mutex cloud_mutex;
bool flag0, flag1, flag2;
ros::Publisher marker_pub;
Eigen::Quaternionf stands_plane_quaternion;


// set rough pass through config (value set by experience)
struct passThrough_config{
    float x1 = -20;
    float x2 = 0;
    float y1 = -10;
    float y2 = -0.5;
    float z1 = -3;
    float z2 = 3;
}ptc;

// set tf message
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

// compute predicted rack center according to yaml map
pcl::PointXYZ computePredictedRackCenter(const std::string& yamlFilePath, tf2_ros::Buffer& tf_buffer) {
    double lon_baselink, lat_baselink, hei_baselink, lon_center, lat_center, hei_center;
    
    YAML::Node yaml_info = YAML::LoadFile(yamlFilePath);
    
    for(const auto baselink : yaml_info["baselinks"]){
        int baselink_id = baselink["id"].as<int>();
        if (baselink_id == 1){
            lon_baselink = baselink["long"].as<double>();
            lat_baselink = baselink["lat"].as<double>();
            hei_baselink = baselink["height"].as<double>();
            break;
        }
    }
    
    for (const auto rack : yaml_info["racks"]){
        int id = rack["id"].as<int>();
        if (id == 1){
            lon_center = rack["long"].as<double>();
            lat_center = rack["lat"].as<double>();
            break;
        }
    }
    
    // Compute predicted rack center according to yaml map (under base_link)
    pcl::PointXYZ center = lonlat2xyz(lat_baselink, lon_baselink, hei_baselink, 
                                      lat_center, lon_center, tf_buffer);
    
    std::cout << "The predicted rack center according to yaml map is " << center.x << " " << center.y << " " << center.z << std::endl;
    
    return center;
}

// callback function(0,1,2) to: 
    // 1. transform pc from LiDAR to base_link coordinate
    // 2. use filter to eliminate noize pc
    // 3. downsample pc
    // 4. combine lidar pc(0,1,2) to global

void cb1(const sensor_msgs::PointCloud2ConstPtr& msg){
    // auto start_time = std::chrono::high_resolution_clock::now();
    flag1 = true;
    sensor_msgs::PointCloud2 temp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud(target_frame, *msg, temp, tf_buffer);
    pcl::fromROSMsg(temp,*cloud);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (ptc.x1, ptc.x2);
    pass.filter (*cloud);

    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (ptc.y1, ptc.y2);
    pass.filter (*cloud);

    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (ptc.z1, ptc.z2);
    pass.filter (*cloud);

    pcl::PCLPointCloud2::Ptr downsampled_cloud_input(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr downsampled_cloud_output(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*cloud, *downsampled_cloud_input);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(downsampled_cloud_input);
    sor.setLeafSize(0.08f, 0.08f, 0.08f);
    sor.filter(*downsampled_cloud_output);
    pcl::fromPCLPointCloud2(*downsampled_cloud_output, *cloud);
    // auto end_time = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time-start_time);
    // std::cout << duration.count() << std::endl;
    {
        std::lock_guard<std::mutex> lock(cloud_mutex);
        *global_cloud += *cloud;
        *global_cloud_copy += *cloud;
    }

    return;
}

void cb2(const sensor_msgs::PointCloud2ConstPtr& msg){
    flag2 = true;
    sensor_msgs::PointCloud2 temp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud(target_frame, *msg, temp, tf_buffer);
    pcl::fromROSMsg(temp,*cloud);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (ptc.x1, ptc.x2);
    pass.filter (*cloud);

    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (ptc.y1, ptc.y2);
    pass.filter (*cloud);

    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (ptc.z1, ptc.z2);
    pass.filter (*cloud);

    pcl::PCLPointCloud2::Ptr downsampled_cloud_input(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr downsampled_cloud_output(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*cloud, *downsampled_cloud_input);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(downsampled_cloud_input);
    sor.setLeafSize(0.08f, 0.08f, 0.08f);
    sor.filter(*downsampled_cloud_output);
    pcl::fromPCLPointCloud2(*downsampled_cloud_output, *cloud);
    {
        std::lock_guard<std::mutex> lock(cloud_mutex);
        *global_cloud += *cloud;
        *global_cloud_copy += *cloud;
    }

    return;
}

void msg_process(tf2_ros::Buffer& tf_buffer, Server& server, perception::detectStandsResult& result_, ros::NodeHandle& nh_){
    
    std::string frame, ns;
    visualization_msgs::Marker model_stands_marker, filtered_stands_marker, guess_stands_marker;

    // Read YAML map, compute predicted rack center
    std::string yamlFilePath = ros::package::getPath("perception")+"/config/yaml_map.yaml";
    pcl::PointXYZ center =  computePredictedRackCenter(yamlFilePath, tf_buffer); 

    // stands_segmentation
    pcl::PointCloud<pcl::PointXYZ>::Ptr stands_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    stands_cloud = stands_segmentation(global_cloud, center);

    // visualize stands pc in rviz marker
    frame = "base_link";
    ns = "filtered_stands";
    filtered_stands_marker = pc_marker(frame,ns, 0, 0.03, 0.03, 0.0, 1.0, 0.0, 1.0, stands_cloud);
    marker_pub.publish(filtered_stands_marker);

 //load model stands as target cloud
    YAML::Node file_path = YAML::LoadFile(ros::package::getPath("perception")+"/config/file_config.yaml");
    std::string pcd_file_path = file_path["pcd_file_path"].as<std::string>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud_target);


//----------------------------------------------------------------------------------------------
    // icp
    Eigen::Matrix4f translationMatrix = Eigen::Matrix4f::Identity();
    translationMatrix(0, 3) = center.x;
    translationMatrix(1, 3) = center.y;
    translationMatrix(2, 3) = center.z;

    //from source pc's perspective, frame of model pc is overlapped with baselink.
    //since initialguess will transform source to model, that's the reason why we inverse
    Eigen::Matrix4f initialGuess = (translationMatrix).inverse();

    int max_iterations = 8;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(80);
    icp.setInputSource(stands_cloud);
    icp.setInputTarget(cloud_target);

    int current_iteration = 0;
    double feedback_value = 0.0, threshhold = 0.40;

    while (current_iteration < max_iterations){
        icp.align(*Final, initialGuess);
        perception::detectStandsFeedback feedback;
        feedback_value = static_cast<double>(current_iteration + 1) / static_cast<double>(max_iterations);
        feedback.progress = feedback_value;
        server.publishFeedback(feedback);
        ROS_INFO("Feedback value: %.3f", feedback_value);

        if (icp.hasConverged() && (icp.getFitnessScore() < threshhold)) {
            feedback_value = 1;
            feedback.progress = feedback_value; //completed in advance
            server.publishFeedback(feedback);
            ROS_INFO("Feedback value(completed in advance): %.3f", feedback_value);
            std::cout << "final fitness score: " << icp.getFitnessScore() << std::endl;
            // source(predicted) to target(model)
            Eigen::Matrix4f tfm = icp.getFinalTransformation();

            // what we need: target(model) to source(predicted).
            Eigen::Matrix4f tfm_inv = tfm.inverse();
            std::cout << "tfm_inv matrix:\n" << tfm_inv.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]")) << std::endl;

            //action result: translation part
            Eigen::Vector3f translationVector = tfm_inv.block<3, 1>(0, 3);
            result_.success = true;
            result_.message = "Perception succeeded!";
            result_.pose.position.x = translationVector.x();
            result_.pose.position.y = translationVector.y();
            result_.pose.position.z = translationVector.z();

            //action result: rotation part
            Eigen::Quaternionf quaternion(tfm_inv.block<3, 3>(0, 0));
            Eigen::Quaternionf res_quaternion = quaternion * stands_plane_quaternion;

            result_.pose.orientation.x = res_quaternion.x();
            result_.pose.orientation.y = res_quaternion.y();
            result_.pose.orientation.z = res_quaternion.z();
            result_.pose.orientation.w = res_quaternion.w();
            std::cout << translationVector.z() << std::endl;
            // Measured from ground to arm center, approximately
		    result_.pose.position.z += 1.515; // actual height is 1.515
            //fitted result of model stands to real stands
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZ>);

            //we assume model stands will appear in "cloud_guess" place according to yaml map
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_guess (new pcl::PointCloud<pcl::PointXYZ>);
            for (auto point : *cloud_target){
                Eigen::Vector4f p(point.x,point.y,point.z,1);
                Eigen::Vector4f p1,p2;
                p1 = tfm_inv * p;
                p2 = translationMatrix * p;
                cloud_transformed->push_back(pcl::PointXYZ(p1[0],p1[1],p1[2]));
                cloud_guess->push_back(pcl::PointXYZ(p2[0],p2[1],p2[2]));
            }
            // publish pc visualization 
                //visualize transformed model_stands
            frame = "base_link";
            ns = "model_stands";
            model_stands_marker = pc_marker(frame,ns, 0, 0.03, 0.03, 1.0, 0.0, 0.0, 1.0, cloud_transformed);
            marker_pub.publish(model_stands_marker);

            // pub stands axis
            Eigen::Matrix3f rotation_matrix = res_quaternion.toRotationMatrix();
            Eigen::Vector3f stands_axis = rotation_matrix * Eigen::Vector3f(0.0,1.0,0.0);
            stands_axis.normalize();
            std::cout << "stands_axis: " << stands_axis << std::endl;
            std::cout << "rotation_matrix: " << rotation_matrix << std::endl; 
            std::vector<double> stands_axis_msg, stands_position_msg;
            stands_axis_msg.resize(3);
            stands_position_msg.resize(3);
            stands_axis_msg[0] = (double)stands_axis.x();
            stands_axis_msg[1] = (double)stands_axis.y();
            stands_axis_msg[2] = (double)stands_axis.z();
            stands_position_msg[0] = result_.pose.position.x;
            stands_position_msg[1] = result_.pose.position.y;
            stands_position_msg[2] = result_.pose.position.z;
            nh_.setParam("/perception/stands_axis", stands_axis_msg);
            nh_.setParam("/perception/stands_position", stands_position_msg);
            server.setSucceeded(result_);
            break;

        }
        current_iteration ++;
        icp.setInputSource(Final);
    }

    if (!result_.success){
            result_.success = false;
            result_.message = "Perception failed! Please drive to the correct place & set goal and run again!";
            server.setSucceeded(result_);
            std::cout << "final fitness score (failed): " << icp.getFitnessScore() << std::endl;
    }
    

    // reset global_cloud, to load next 10 frames.
    std::lock_guard<std::mutex> lock(cloud_mutex);
    global_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    return;
}

class LidarStandsDetectServer{

private:
    ros::NodeHandle nh_;
    Server server;
    perception::detectStandsResult result_;
    // ros::Publisher stands_axis_pub, stands_position_pub;

    friend void cb1(const sensor_msgs::PointCloud2ConstPtr& msg);
    friend void cb2(const sensor_msgs::PointCloud2ConstPtr& msg);
    friend void msg_process(tf2_ros::Buffer& tf_buffer, Server& server, perception::detectStandsResult& result_, ros::NodeHandle& nh_);


public:
    //initialization function, being called when create an instance
    LidarStandsDetectServer(): server(nh_, "lidar_stands_detect", boost::bind(&LidarStandsDetectServer::execute, this, _1), false){
        server.start();
        ROS_INFO("Not yet detected, waiting for goal...");
        // stands_axis_pub = nh_.advertise<geometry_msgs::Vector3>("/perception/stands_axis", 1);
        // stands_position_pub = nh_.advertise<geometry_msgs::Vector3>("/perception/stands_position", 1);
    }

    ~LidarStandsDetectServer() {} 

    void execute(const perception::detectStandsGoalConstPtr &goal){
        // Get goal from Client
        bool perception_requested = goal->ifDetect;
        if (perception_requested == true){
            frames = 0;
            flag0 = flag1 = flag2 = false;
            tf2_ros::TransformListener listener(tf_buffer);
            marker_pub = nh_.advertise<visualization_msgs::Marker>("lidar_marker", 10);

            
            // Subscribe to LiDAR point cloud, process them in callback function
            // ros::Subscriber sub0 = nh.subscribe("/Pandar0/hesailidar/pandar",1,cb0);
            ros::Subscriber sub1 = nh_.subscribe("/Pandar1/hesai/pandar",1,cb1);
            ros::Subscriber sub2 = nh_.subscribe("/Pandar2/hesai/pandar",1,cb2);
            ros::Duration(2.5).sleep();

            visualization_msgs::Marker fusion_pc_marker;

            ros::Rate rate(10);
            while (ros::ok && !server.isPreemptRequested() ){
                ros::spinOnce();
                    // {
                    //     std::lock_guard<std::mutex> lock(cloud_mutex);
                    //     *global_cloud_copy += *global_cloud;  // Accumulate points
                    // }

                //Publish global pc in rviz marker (in real-time, that's why create a copy of global_cloud, since icp need 10 frames together )
                std::string frame = "base_link", ns = "fusion_pc";
                fusion_pc_marker = pc_marker(frame, ns, 0, 0.03, 0.03, 1.0, 1.0, 1.0, 1.0, global_cloud_copy);
                marker_pub.publish(fusion_pc_marker);

                {
                    std::lock_guard<std::mutex> lock(cloud_mutex);
                    global_cloud_copy.reset(new pcl::PointCloud<pcl::PointXYZ>);                       
                }


                frames++;

                if (flag1 && flag2 && (frames % 15 == 0)){
                    if (server.isNewGoalAvailable()) {
                        perception::detectStandsGoalConstPtr new_goal = server.acceptNewGoal();
                        msg_process(tf_buffer, server, result_, nh_);
                    } else if (server.isActive()) {
                        // Continue processing current goal
                        msg_process(tf_buffer, server, result_, nh_);
                    } else {
                        std::lock_guard<std::mutex> lock(cloud_mutex);
                        global_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
                    }
                }

                flag1 = flag2 = false;
                rate.sleep();
            }
            
        }
    }       
};



int main(int argc, char* argv[]){
    ROS_INFO("Action Server implement");
    ros::init(argc,argv,"lidar_stands_detect_server");

    YAML::Node file_path = YAML::LoadFile(ros::package::getPath("perception")+"/config/stands_config.yaml");
    stands_plane_quaternion.x() = file_path["x"].as<float>();
    stands_plane_quaternion.y() = file_path["y"].as<float>();
    stands_plane_quaternion.z() = file_path["z"].as<float>();
    stands_plane_quaternion.w() = file_path["w"].as<float>();

    LidarStandsDetectServer lidarServer;
    ros::spin();

    return 0;
}