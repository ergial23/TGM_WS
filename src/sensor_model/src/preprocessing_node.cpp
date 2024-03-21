#include "preprocessing_library.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <chrono>

class SensorModel {
private:
    ros::NodeHandle nh;
    ros::Subscriber pointCloudSub;
    ros::Publisher FilteredObstaclePointCloudPub;
    ros::Publisher FilteredGroundPointCloudPub;

    PreprocessingLibrary preprocessing;// Instance of the preprocessing library

    int callbackCounter = 0;
    long long totalDuration = 0;

public:
    SensorModel() : nh("~"), preprocessing() {
        pointCloudSub = nh.subscribe("/ouster/points", 1, &SensorModel::pointCloudCallback, this);
        FilteredObstaclePointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_pointcloud", 1);
        FilteredGroundPointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/ground_pointcloud", 1);
        
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        
        auto start_time = std::chrono::high_resolution_clock::now(); // Start measuring time
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle(new pcl::PointCloud<pcl::PointXYZ>);
        
        preprocessing.processPointCloud(cloud, ground_plane, obstacle);
        

        // Update the header information
        ground_plane->header = pcl_conversions::toPCL(msg->header);
        obstacle->header = pcl_conversions::toPCL(msg->header);
        
        sensor_msgs::PointCloud2 cloud_msg;
        #include "preprocessing_library.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <chrono>

class SensorModel {
private:
    ros::NodeHandle nh;
    ros::Subscriber pointCloudSub;
    ros::Publisher FilteredObstaclePointCloudPub;
    ros::Publisher FilteredGroundPointCloudPub;

    PreprocessingLibrary preprocessing;// Instance of the preprocessing library
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane;
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle;
    

    int callbackCounter = 0;
    long long totalDuration = 0;

public:
    SensorModel() : nh("~"), preprocessing(),
        cloud(new pcl::PointCloud<pcl::PointXYZ>),
        ground_plane(new pcl::PointCloud<pcl::PointXYZ>),
        obstacle(new pcl::PointCloud<pcl::PointXYZ>) {
        
        pointCloudSub = nh.subscribe("/ouster/points", 1, &SensorModel::pointCloudCallback, this);
        FilteredObstaclePointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_pointcloud", 1);
        FilteredGroundPointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/ground_pointcloud", 1);
         
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        
        auto start_time = std::chrono::high_resolution_clock::now(); // Start measuring time
        cloud->clear();
        ground_plane->clear();
        obstacle->clear();
        
        pcl::fromROSMsg(*msg, *cloud);

        preprocessing.processPointCloud(cloud, ground_plane, obstacle);
        

        // Update the header information
        ground_plane->header = pcl_conversions::toPCL(msg->header);
        obstacle->header = pcl_conversions::toPCL(msg->header);
        
        sensor_msgs::PointCloud2 cloud_msg;
        
        pcl::toROSMsg(*ground_plane, cloud_msg);
        FilteredGroundPointCloudPub.publish(cloud_msg);
        
        pcl::toROSMsg(*obstacle, cloud_msg);
        FilteredObstaclePointCloudPub.publish(cloud_msg);

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
        pcl::toROSMsg(*ground_plane, cloud_msg);
        FilteredGroundPointCloudPub.publish(cloud_msg);
        
        pcl::toROSMsg(*obstacle, cloud_msg);
        FilteredObstaclePointCloudPub.publish(cloud_msg);

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