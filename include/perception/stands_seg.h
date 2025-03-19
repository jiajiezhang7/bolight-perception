#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/geometry.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <cmath>
#include <pcl/filters/conditional_removal.h>
typedef pcl::PointXYZ PointT;

pcl::PointCloud<PointT>::Ptr stands_segmentation(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointXYZ center);
