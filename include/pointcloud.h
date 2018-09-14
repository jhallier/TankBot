#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <vector>
#include <cmath>

pcl::PointCloud<pcl::PointXYZ>::Ptr convert_ranges_to_pcl(std::vector<float> ranges, std::vector<float> intensities);

pcl::PointXYZ convert_single_pointXYZ(float angle, float range);

boost::shared_ptr<pcl::visualization::PCLVisualizer> visualize_PointCloudXYZ(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ptr);

void update_PointCloudXYZ(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ptr, boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer);