#include "preprocessing_library.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <chrono>

class SensorModel {
private:
    ros::NodeHandle nh;
    ros::Subscriber pointCloudSub;
    ros::Publisher FilteredOccupancyGridPub;

    PreprocessingLibrary preprocessing;// Instance of the preprocessing library

    int callbackCounter = 0;
    long long totalDuration = 0;

public:
    SensorModel() : nh("~"), preprocessing() {
        pointCloudSub = nh.subscribe("/ouster/points", 1, &SensorModel::pointCloudCallback, this);
        FilteredOccupancyGridPub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_preprocessed", 1);
        
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        auto start_time = std::chrono::high_resolution_clock::now(); // Start measuring time
        preprocessing.processPointCloud(cloud);
        auto end_time = std::chrono::high_resolution_clock::now(); // Stop measuring time
        
        // Calculate the duration in milliseconds
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        
        // Update the global variables
        totalDuration += duration;
        ++callbackCounter;

        if (callbackCounter == 100) {
        long long averageDuration = totalDuration / callbackCounter;
        std::cout << "Average time taken by preprocessing.processPointCloud over "
                  << callbackCounter << " iterations: " << averageDuration << " milliseconds" << std::endl;
        // Reset the counter and total duration for subsequent iterations
        callbackCounter = 0;
        totalDuration = 0;
    }

        // Update the header information
        cloud->header = pcl_conversions::toPCL(msg->header);

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);

        FilteredOccupancyGridPub.publish(cloud_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "preprocessing_node");
    SensorModel sensorModel;
    
    ros::Rate r(10);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}