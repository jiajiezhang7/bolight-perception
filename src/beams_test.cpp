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
#include <pcl/registration/icp.h>
#include <pcl/segmentation/extract_clusters.h>

int frames;
tf2_ros::Buffer tf_buffer;
const std::string target_frame = "base_link";
pcl::PointCloud<pcl::PointXYZ>::Ptr lidar1_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr lidar2_cloud(new pcl::PointCloud<pcl::PointXYZ>);

std::mutex cloud_mutex;
bool flag1, flag2;
ros::Publisher marker_pub;

struct passThrough_config{
    double x1,x2,y1,y2,z1,z2;
}clouds_ptc, beam1_ptc, beam2_ptc, beam3_ptc, beam4_ptc;
Eigen::Quaternionf stands_plane_quaternion;
Eigen::Matrix3f R;
Eigen::Vector3f stands_axis;

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
    pass.setFilterLimits (clouds_ptc.x1, clouds_ptc.x2);
    pass.filter (*cloud);

    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (clouds_ptc.y1, clouds_ptc.y2);
    pass.filter (*cloud);

    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (clouds_ptc.z1, clouds_ptc.z2);
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
    std::lock_guard<std::mutex> lock(cloud_mutex);
    *lidar1_cloud += *cloud;
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
    pass.setFilterLimits (clouds_ptc.x1, clouds_ptc.x2);
    pass.filter (*cloud);

    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (clouds_ptc.y1, clouds_ptc.y2);
    pass.filter (*cloud);

    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (clouds_ptc.z1, clouds_ptc.z2);
    pass.filter (*cloud);

    pcl::PCLPointCloud2::Ptr downsampled_cloud_input(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr downsampled_cloud_output(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*cloud, *downsampled_cloud_input);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(downsampled_cloud_input);
    sor.setLeafSize(0.08f, 0.08f, 0.08f);
    sor.filter(*downsampled_cloud_output);
    pcl::fromPCLPointCloud2(*downsampled_cloud_output, *cloud);
    std::lock_guard<std::mutex> lock(cloud_mutex);
    *lidar2_cloud += *cloud;
    return;
}

void cylinder_detection(passThrough_config ptc, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr beam_cloud, pcl::ModelCoefficients::Ptr coefficients){


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // passthrough
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (ptc.x1, ptc.x2);
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (ptc.y1, ptc.y2);
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (ptc.z1, ptc.z2);
    pass.filter (*cloud_filtered);

    // std::cout << "filtered size: " << cloud_filtered->points.size() << std::endl;
    // compute normal
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);
    
    // segment cylinder
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (100000);
    seg.setDistanceThreshold (0.1);
    seg.setRadiusLimits (0.02, 0.2);
    seg.setAxis(stands_axis);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    seg.segment (*inliers, *coefficients);
    // std::cout << coefficients->values.size() << std::endl;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*beam_cloud);

    if (coefficients->values[5] < 0){
        coefficients->values[3] = -coefficients->values[3];
        coefficients->values[4] = -coefficients->values[4];
        coefficients->values[5] = -coefficients->values[5];
    }
    // std::cout << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << " " 
    //         << coefficients->values[4] << " " << coefficients->values[5] << " " << coefficients->values[6] << std::endl;
    return;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr get_pointclouds(passThrough_config ptc, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_panel(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr beams_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_without_panel_normals (new pcl::PointCloud<pcl::Normal>);

    // pass through
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (ptc.x1, ptc.x2);
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (ptc.y1, ptc.y2);
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (ptc.z1, ptc.z2);
    pass.filter (*cloud_filtered);

    // compute normal
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // segment plane
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.1);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    seg.segment (*inliers_plane, *coefficients_plane);

    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (true);
    extract.filter (*cloud_without_panel);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_without_panel_normals);
  
    for (const auto& point: cloud_without_panel->points){
        float x = point.x;
        float y = point.y;
        float z = point.z;
        float distance = -(coefficients_plane->values[0]*x + coefficients_plane->values[1]*y + coefficients_plane->values[2]*z + coefficients_plane->values[3]) 
                        / sqrt(coefficients_plane->values[0]*coefficients_plane->values[0] + coefficients_plane->values[1]*coefficients_plane->values[1] + coefficients_plane->values[2]*coefficients_plane->values[2]);
        if (distance < 0.30 && distance > 0.175) {
            beams_cloud->push_back(point);
        }
    }

    return beams_cloud;
}

pcl::PointXYZ projectPointToLine(const pcl::PointXYZ& point, const pcl::PointXYZ& line_point, const Eigen::Vector3f& line_dir){
    Eigen::Vector3f point_vec(point.x - line_point.x, point.y - line_point.y, point.z - line_point.z);
    Eigen::Vector3f line_dir_unit = line_dir.normalized();
    float t = point_vec.dot(line_dir_unit);
    Eigen::Vector3f projection = line_point.getVector3fMap() + t * line_dir_unit;
    pcl::PointXYZ result;
    result.x = projection.x();
    result.y = projection.y();
    result.z = projection.z();
    return result;
}

pcl::PointXYZ calculateCylinderCenter(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const pcl::ModelCoefficients::Ptr coefficients){

    pcl::PointCloud<pcl::PointXYZ>::Ptr projection_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    float x_min = INFINITY;
    float x_max = -INFINITY;

    // std::cout << "cloud size: " << cloud->points.size() << std::endl;
    // projection and count ymin ymax
    for (const auto &point: cloud->points){
        pcl::PointXYZ projection = projectPointToLine(point, pcl::PointXYZ(coefficients->values[0], coefficients->values[1], coefficients->values[2]), Eigen::Vector3f(coefficients->values[3], coefficients->values[4], coefficients->values[5]));
        projection_cloud->push_back(projection);
        if (projection.x < x_min) x_min = projection.x;
        if (projection.x > x_max) x_max = projection.x;
    }

    // // compute window length on y axis
    // float beam_window_length = 3.696;
    // Eigen::Vector3f window_end_point = Eigen::Vector3f(coefficients->values[3], coefficients->values[4], coefficients->values[5]) * beam_window_length / sqrt(coefficients->values[3]*coefficients->values[3] + coefficients->values[4]*coefficients->values[4] + coefficients->values[5]*coefficients->values[5]);
    // float beam_window_y_length = window_end_point.y();
    // // calculate bins capacity
    // std::vector<float> bins;
    // float bin_length = 0.005;
    // std::cout << "ymax: " << y_max << " ymin: " << y_min << std::endl;
    // bins.resize(int((y_max - y_min) / bin_length) + 1);
    // for (const auto &point: projection_cloud->points){
    //     bins[int((point.y - y_min) / bin_length)]++;
    // }

    // // find the max bin index
    // int max_bin_index = 0;
    // for (int i = 0; i < bins.size(); i++){
    //     if (bins[i] > bins[max_bin_index]) max_bin_index = i;
    // }

    // // calculate the center
    // float center_y = y_min + max_bin_index * bin_length + bin_length / 2;
    float center_x = (x_min + x_max) / 2;
    float l = (center_x - coefficients->values[0]) / coefficients->values[3];
    float center_y = coefficients->values[1] + l * coefficients->values[4];
    float center_z = coefficients->values[2] + l * coefficients->values[5];
    pcl::PointXYZ center;
    center.x = center_x;
    center.y = center_y;
    center.z = center_z;
    // std::cout << "center: " << center << std::endl;
    return center;
}

Eigen::Matrix3f rotationMaxtrixFromVectors(const Eigen::Vector3f& A, const Eigen::Vector3f& B){
    Eigen::Vector3f A_norm = A.normalized();
    Eigen::Vector3f B_norm = B.normalized();
    Eigen::Vector3f v = A_norm.cross(B_norm);
    v.normalize();
    float angle = acos(A_norm.dot(B_norm));
    Eigen::Matrix3f M_rot;
    Eigen::AngleAxisf rot(angle, v);
    M_rot = rot.toRotationMatrix();
    std::cout << "Anorm: " << A_norm << std::endl;
    std::cout << "Bnorm: " << B_norm << std::endl;
    std::cout << "angle: " << angle << std::endl;
    return M_rot;
}

void msg_process(){
    frames++;
    if (frames % 10 != 0) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr beams_left_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr beams_right_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr beam1_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr beam2_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr beam3_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr beam4_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::ModelCoefficients::Ptr coefficients1 (new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr coefficients3 (new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr coefficients4 (new pcl::ModelCoefficients);

    beams_left_cloud = get_pointclouds(clouds_ptc, lidar1_cloud);
    beams_right_cloud = get_pointclouds(clouds_ptc, lidar2_cloud);
    // std::cout << "left size: " << beams_left_cloud->points.size() << std::endl;
    // std::cout << "right size: " << beams_right_cloud->points.size() << std::endl;

    // pcl::io::savePCDFileASCII("/home/donglzh/pcd_files/left.pcd", *beams_left_cloud);
    // pcl::io::savePCDFileASCII("/home/donglzh/pcd_files/right.pcd", *beams_right_cloud);

    cylinder_detection(beam1_ptc, beams_left_cloud, beam1_cloud, coefficients1);
    cylinder_detection(beam2_ptc, beams_left_cloud, beam2_cloud, coefficients2);
    cylinder_detection(beam3_ptc, beams_right_cloud, beam3_cloud, coefficients3);
    cylinder_detection(beam4_ptc, beams_right_cloud, beam4_cloud, coefficients4);    
    // std::cout << "beam1 size: " << beam1_cloud->points.size() << std::endl;
    // std::cout << "beam2 size: " << beam2_cloud->points.size() << std::endl;
    // std::cout << "beam3 size: " << beam3_cloud->points.size() << std::endl;
    // std::cout << "beam4 size: " << beam4_cloud->points.size() << std::endl;
    if (beam1_cloud->points.size() == 0 || beam2_cloud->points.size() == 0 || beam3_cloud->points.size() == 0 || beam4_cloud->points.size() == 0){
        return;
    }

    // pcl::io::savePCDFileASCII("/home/donglzh/pcd_files/1.pcd", *beam1_cloud);
    // pcl::io::savePCDFileASCII("/home/donglzh/pcd_files/2.pcd", *beam2_cloud);
    // pcl::io::savePCDFileASCII("/home/donglzh/pcd_files/3.pcd", *beam3_cloud);
    // pcl::io::savePCDFileASCII("/home/donglzh/pcd_files/4.pcd", *beam4_cloud);

    pcl::PointXYZ center1 = calculateCylinderCenter(beam1_cloud, coefficients1);
    pcl::PointXYZ center2 = calculateCylinderCenter(beam2_cloud, coefficients2);
    pcl::PointXYZ center3 = calculateCylinderCenter(beam3_cloud, coefficients3);
    pcl::PointXYZ center4 = calculateCylinderCenter(beam4_cloud, coefficients4);

    // marker
    visualization_msgs::Marker beam1_marker, beam2_marker, beam3_marker, beam4_marker, lidar1_cloud_marker, lidar2_cloud_marker;
    std::string frame, ns;
    frame = "base_link";
    ns = "beam1";
    beam1_marker = pc_marker(frame,ns, 0, 0.03, 0.03, 1.0, 0.0, 0.0, 1.0, beam1_cloud);
    ns = "beam2";
    beam2_marker = pc_marker(frame,ns, 1, 0.03, 0.03, 1.0, 0.0, 0.0, 1.0, beam2_cloud);
    ns = "beam3";
    beam3_marker = pc_marker(frame,ns, 2, 0.03, 0.03, 1.0, 0.0, 0.0, 1.0, beam3_cloud);
    ns = "beam4";
    beam4_marker = pc_marker(frame,ns, 3, 0.03, 0.03, 1.0, 0.0, 0.0, 1.0, beam4_cloud);
    ns = "lidar1_cloud";
    lidar1_cloud_marker = pc_marker(frame,ns, 0, 0.03, 0.03, 1.0, 1.0, 1.0, 1.0, lidar1_cloud);
    ns = "lidar2_cloud";
    lidar2_cloud_marker = pc_marker(frame,ns, 1, 0.03, 0.03, 1.0, 1.0, 1.0, 1.0, lidar2_cloud);

    marker_pub.publish(beam1_marker);
    marker_pub.publish(beam2_marker);
    marker_pub.publish(beam3_marker);
    marker_pub.publish(beam4_marker);
    marker_pub.publish(lidar1_cloud_marker);
    marker_pub.publish(lidar2_cloud_marker);

    // avg to find final pose
    pcl::PointXYZ center;
    center.x = (center1.x + center2.x + center3.x + center4.x) / 4.0;
    center.y = (center1.y + center2.y + center3.y + center4.y) / 4.0;
    center.z = (center1.z + center2.z + center3.z + center4.z) / 4.0;
    Eigen::Vector3f orientation;
    orientation[0] = (coefficients1->values[3] + coefficients2->values[3] + coefficients3->values[3] + coefficients4->values[3]) / 4.0;
    orientation[1] = (coefficients1->values[4] + coefficients2->values[4] + coefficients3->values[4] + coefficients4->values[4]) / 4.0;
    orientation[2] = (coefficients1->values[5] + coefficients2->values[5] + coefficients3->values[5] + coefficients4->values[5]) / 4.0;
    orientation.normalize();
    Eigen::Matrix3f rotation_matrix = rotationMaxtrixFromVectors(orientation, stands_axis);
    Eigen::Quaternionf rotation_quaternion(rotation_matrix);
    rotation_quaternion.normalize();
    std::cout << "xyz offset: " << center << std::endl;
    std::cout << "quaternions: " << rotation_quaternion.coeffs() << std::endl;
    // reset
    lidar1_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    lidar2_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    return;
}



int main(int argc, char* argv[]){
    ROS_INFO("Beams detection implement");
    ros::init(argc,argv,"lidar_beams_detect_server");

    frames = 0;
    ros::NodeHandle nh;
    tf2_ros::TransformListener listener(tf_buffer);
    // set pass through config
    clouds_ptc.x1 = -18.1; clouds_ptc.x2 = -0.1; clouds_ptc.y1 = -10; clouds_ptc.y2 = -0.5; clouds_ptc.z1 = -2.2; clouds_ptc.z2 = 2.0;
    beam1_ptc.x1 = -4.6; beam1_ptc.x2 = -0.1; beam1_ptc.y1 = -10; beam1_ptc.y2 = -0.5; beam1_ptc.z1 = -2.1; beam1_ptc.z2 = 2.0;
    beam2_ptc.x1 = -9.1; beam2_ptc.x2 = -4.6; beam2_ptc.y1 = -10; beam2_ptc.y2 = -0.5; beam2_ptc.z1 = -2.1; beam2_ptc.z2 = 2.0;
    beam3_ptc.x1 = -13.6; beam3_ptc.x2 = -9.1; beam3_ptc.y1 = -10; beam3_ptc.y2 = -0.5; beam3_ptc.z1 = -2.1; beam3_ptc.z2 = 2.0;
    beam4_ptc.x1 = -18.1; beam4_ptc.x2 = -13.6; beam4_ptc.y1 = -10; beam4_ptc.y2 = -0.5; beam4_ptc.z1 = -2.1; beam4_ptc.z2 = 2.0;
    YAML::Node file_path = YAML::LoadFile(ros::package::getPath("perception")+"/config/stands_config.yaml");
    stands_plane_quaternion.x() = file_path["x"].as<float>();
    stands_plane_quaternion.y() = file_path["y"].as<float>();
    stands_plane_quaternion.z() = file_path["z"].as<float>();
    stands_plane_quaternion.w() = file_path["w"].as<float>();
    R = stands_plane_quaternion.toRotationMatrix();
    Eigen::Vector3f axis(0.0, 1.0, 0.0);
    stands_axis = R * axis;
    stands_axis.normalize();
    lidar1_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    lidar2_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    flag1 = flag2 = false;
    ros::Subscriber sub1 = nh.subscribe("/Pandar1/hesai/pandar",1,cb1);
    ros::Subscriber sub2 = nh.subscribe("/Pandar2/hesai/pandar",1,cb2);
    marker_pub = nh.advertise<visualization_msgs::Marker>("lidar_marker", 10);



    ros::Rate rate(10);
    while (ros::ok){
        ros::spinOnce();
        if (flag1 && flag2){
            msg_process();
        }
        flag1 = flag2 = false;
        rate.sleep();
    }

    return 0;
}