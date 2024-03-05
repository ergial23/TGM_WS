#include "preprocessing_library.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class SensorModel {
private:
    ros::NodeHandle nh;
    ros::Subscriber pointCloudSub;
    ros::Publisher FilteredOccupancyGridPub;

    PreprocessingLibrary preprocessing;// Instance of the preprocessing library

public:
    SensorModel() : nh("~"), preprocessing(nh) {
        pointCloudSub = nh.subscribe("/points_raw", 10, &SensorModel::pointCloudCallback, this);
        FilteredOccupancyGridPub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_preprocessed", 1);
        
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        preprocessing.processPointCloud(cloud);

        // Update the header information
        cloud->header = pcl_conversions::toPCL(msg->header);

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);

        FilteredOccupancyGridPub.publish(cloud_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_model_node");
    SensorModel sensorModel;
    
    ros::Rate r(10);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}