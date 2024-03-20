#include "occupancy_grid_map_library.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <boost/thread/mutex.hpp>
#include <ros/timer.h>

class RayCastingNodelet : public nodelet::Nodelet {
private:
    ros::NodeHandle nh;
    ros::Subscriber obstaclePointCloudSub;
    ros::Subscriber groundPointCloudSub;
    ros::Publisher occupancyGridMapPub;
    boost::mutex mutex;
    ros::Timer timer;

    OccupancyGridMapLibrary occupancyGridMap; // Instance of the occupancy grid map library

    // Create Pointers to the Pointclouds
    sensor_msgs::PointCloud2::ConstPtr latestObstaclePointCloud;
    sensor_msgs::PointCloud2::ConstPtr latestGroundPointCloud;

    //Create Pointcloud bins
    std::vector<std::vector<BinInfo>> obstaclePointCloudAngleBins;
    std::vector<std::vector<BinInfo>> rawPointCloudAngleBins;

    //Create an OccupancyGrid message
    nav_msgs::OccupancyGrid occupancyGrid_;

    bool obstaclePointCloudUpdated = false;
    bool groundPointCloudUpdated = false;

    void obstaclePointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        //boost::mutex::scoped_lock lock(mutex);
        latestObstaclePointCloud = msg;
        obstaclePointCloudUpdated = true;
        ROS_WARN("Obstacle point cloud callback executed.");
    }

    void groundPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        //boost::mutex::scoped_lock lock(mutex);
        latestGroundPointCloud = msg;
        groundPointCloudUpdated = true;
        ROS_WARN("Ground point cloud callback executed.");
    }

    void publish_msg(nav_msgs::OccupancyGrid& occupancyGrid_) {
        occupancyGrid_.header.stamp = ros::Time::now();
        occupancyGrid_.header.frame_id = "map";
        occupancyGridMapPub.publish(occupancyGrid_);
    }

    void generateGridMap() {
        //boost::mutex::scoped_lock lock(mutex);
        if(obstaclePointCloudUpdated && groundPointCloudUpdated) {
            
            occupancyGridMap.processPointClouds( latestGroundPointCloud, latestObstaclePointCloud,
                                                 obstaclePointCloudAngleBins, rawPointCloudAngleBins);
            
            occupancyGridMap.initializeFreeSpace(obstaclePointCloudAngleBins, rawPointCloudAngleBins, occupancyGrid_);
            occupancyGridMap.fillUnknownCells(obstaclePointCloudAngleBins, rawPointCloudAngleBins, occupancyGrid_);
            
            occupancyGridMap.fillOccupiedCells(obstaclePointCloudAngleBins, rawPointCloudAngleBins, occupancyGrid_);
    
            
           
            
            publish_msg(occupancyGrid_);
            
            // Reset flags
            obstaclePointCloudUpdated = false;
            groundPointCloudUpdated = false;
            occupancyGridMap.clearOccupancyGrid(occupancyGrid_);
        }
    }
    void timerCallback(const ros::TimerEvent&) {
        generateGridMap();
        ROS_WARN("Timer callback executed. About to generate grid map.");
    }
protected:
    virtual void onInit() {
        nh = getNodeHandle();
        ros::NodeHandle& private_nh = getPrivateNodeHandle();

        obstaclePointCloudSub = nh.subscribe("/obstacle_pointcloud", 1, &RayCastingNodelet::obstaclePointCloudCallback, this);
        groundPointCloudSub = nh.subscribe("/ground_pointcloud", 1, &RayCastingNodelet::groundPointCloudCallback, this);
        occupancyGridMapPub = private_nh.advertise<nav_msgs::OccupancyGrid>("/grid_map", 1);
        
        // Initialize grid map.
        occupancyGridMap.initializeGrid(occupancyGrid_);
        // Initialize timer
        timer = private_nh.createTimer(ros::Duration(0.1), &RayCastingNodelet::timerCallback, this);

        
    }

public:
    RayCastingNodelet() {}
};

PLUGINLIB_EXPORT_CLASS(RayCastingNodelet, nodelet::Nodelet)