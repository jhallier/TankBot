#include "pointcloud.h"
#include "stdio.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr convert_ranges_to_pcl(std::vector<float> ranges, std::vector<float> intensities) {
    
    auto ranges_len = ranges.size();
    float angle = 0;
    float angle_inc = 2 * M_PI / ranges_len;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    /* Convert each measurement point (angle, range) to a X,Y,Z point and add it to the point cloud */
    for (auto i = 0; i < ranges_len; i++) {

        pcl::PointXYZ single_point = convert_single_pointXYZ(angle, ranges[i]);
        //single_point.intensity = intensities[i];

        cloud_ptr->points.push_back(single_point);
        angle += angle_inc;
    }

    cloud_ptr->width = (int) cloud_ptr->points.size();
    cloud_ptr->height = 1;

    cout << "Cloud size: " << cloud_ptr->width << endl;

    return cloud_ptr;
}

pcl::PointXYZ convert_single_pointXYZ(float angle, float range) {
    pcl::PointXYZ point;
    point.x = sin(angle) * range;
    point.y = cos(angle) * range;
    point.z = 0;
    return point;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> visualize_PointCloudXYZ(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ptr)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud_ptr, "Lidar Point Cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Lidar Point Cloud");
  viewer->addCoordinateSystem (0.5);
  viewer->initCameraParameters ();
  return (viewer);
}

void update_PointCloudXYZ(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ptr, boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer) {

    viewer->removeAllPointClouds();
    viewer->addPointCloud<pcl::PointXYZ> (cloud_ptr, "Lidar Point Cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Lidar Point Cloud");
    viewer->spinOnce();
}
