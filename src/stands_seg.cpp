//Last modified by Jiajie. 2024.1.29 
#include <perception/stands_seg.h>
// using bounding box (from predicted rack center) to crop 4 stands, as preprocessed data for icp
pcl::PointCloud<PointT>::Ptr stands_segmentation(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointXYZ center){
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::PCLPointCloud2 cloud_blob;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filter_without_ground (new pcl::PointCloud<PointT>);
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::ModelCoefficients::Ptr  coefficients_ground (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr  inliers_ground (new pcl::PointIndices);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);


    //based on predicted rack center, magic number determined based on experience :)
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (center.y-2, center.y+2);
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (center.x-8, center.x+8);
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (center.z+0.13, center.z+2.4);
    pass.filter (*cloud_filtered);

    //Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch(50);
    ne.compute (*cloud_normals);

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    

    seg.segment (*inliers_ground, *coefficients_ground);

    //Remove the inliers belong to the ground, extract the rest
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_ground);
    extract.setNegative (true);

    //cloud_filter_without_ground
    extract.filter (*cloud_filter_without_ground);
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices (inliers_ground);
    extract_normals.filter(*cloud_normals2);
    std::cerr << " PointCloud that represents predicted stands has: " << cloud_filter_without_ground->size() << " data points." << std::endl;
    return cloud_filter_without_ground;
}
