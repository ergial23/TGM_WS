#include "preprocessing_library.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <chrono>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>


class PreprocessingNodelet : public nodelet::Nodelet {
private:
    virtual void onInit();
    
    ros::Subscriber pointCloudSub;
    ros::Publisher FilteredObstaclePointCloudPub;
    ros::Publisher FilteredGroundPointCloudPub;

    PreprocessingLibrary preprocessing; // Instance of the preprocessing library

    int callbackCounter = 0;
    long long totalDuration = 0;

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
};

void PreprocessingNodelet::onInit() {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    
    pointCloudSub = nh.subscribe("/ouster/points", 1, &PreprocessingNodelet::pointCloudCallback, this);
    FilteredObstaclePointCloudPub = private_nh.advertise<sensor_msgs::PointCloud2>("/obstacle_pointcloud", 1);
    FilteredGroundPointCloudPub = private_nh.advertise<sensor_msgs::PointCloud2>("/ground_pointcloud", 1);
}

void PreprocessingNodelet::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle(new pcl::PointCloud<pcl::PointXYZ>);
        auto start_time = std::chrono::high_resolution_clock::now(); // Start measuring time
        preprocessing.processPointCloud(cloud, ground_plane, obstacle);
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
        ground_plane->header = pcl_conversions::toPCL(msg->header);
        obstacle->header = pcl_conversions::toPCL(msg->header);
        
        sensor_msgs::PointCloud2 cloud_msg;
        
        pcl::toROSMsg(*ground_plane, cloud_msg);
        FilteredGroundPointCloudPub.publish(cloud_msg);
        
        pcl::toROSMsg(*obstacle, cloud_msg);
        FilteredObstaclePointCloudPub.publish(cloud_msg);
}

PLUGINLIB_EXPORT_CLASS(PreprocessingNodelet, nodelet::Nodelet)