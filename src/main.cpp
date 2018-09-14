#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <signal.h>
#include <pointcloud.h>

using namespace std;
using namespace ydlidar;

// Modify this if connected to a different port or baud rate
const std::string device_port = "/dev/ttyUSB0";
const int baud_rate = 128000;
const bool intensities = false;

CYdLidar laser;
static bool running = false;

static void Stop(int signo) {      
    cout << "Received exit signal\n";
    running = true;     
}  

int main() {

    signal(SIGINT, Stop);
    signal(SIGTERM, Stop);

    laser.setSerialPort(device_port);
    laser.setSerialBaudrate(baud_rate);
    laser.setIntensities(intensities);
    laser.initialize();

    // Initializes first point cloud pointer and initializes viewer
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = visualize_PointCloudXYZ(cloud_ptr);
    std::cout << "PCL viewer initialized." << std::endl;

    // Main loop, each run receives a full lidar scan
    while (!running) {
		bool hardError;
		LaserScan scan;

		if (laser.doProcessSimple(scan, hardError )) {
            auto len_scan_intens = scan.intensities.size();
            auto len_scan_ranges = scan.ranges.size();

			assert (len_scan_intens == len_scan_ranges);

            // Converts the angle/range lidar data to a XYZ point cloud
            cloud_ptr = convert_ranges_to_pcl(scan.ranges, scan.intensities);

            // Re-draws the point cloud in the viewer
            update_PointCloudXYZ(cloud_ptr, viewer);
		}
        usleep(50*1000); // Sleep for 50 ms
	}

    // Lidar shutdown
    laser.turnOff();
    laser.disconnecting();

    return 0;
}
