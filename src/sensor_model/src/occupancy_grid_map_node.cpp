#include "occupancy_grid_map_library.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>



class RayCasting {
private:
    ros::NodeHandle nh;
    ros::Subscriber obstaclePointCloudSub;
    ros::Subscriber rawPointCloudSub;
    ros::Publisher occupancyGridMapPub;

public:

    RayCasting() : nh("~") {
        obstaclePointCloudSub = nh.subscribe("/obstacle_pointcloud", 1, &RayCasting::obstaclePointCloudCallback, this);
        rawPointCloudSub = nh.subscribe("/ground_pointcloud", 1, &RayCasting::rawPointCloudCallback, this);
        occupancyGridMapPub = nh.advertise<nav_msgs::OccupancyGrid>("/grid_map", 1); //cambiar este publisher e introducir el tipo correcto.
    }
    
    OccupancyGridMapLibrary occupancyGridMap;// Instance of the preprocessing library
    
    // Create Pointers to the Pointclouds
    sensor_msgs::PointCloud2::ConstPtr latestRawPointCloud;
    sensor_msgs::PointCloud2::ConstPtr latestObstaclePointCloud;
        
    bool rawPointCloudUpdated = false;
    bool obstaclePointCloudUpdated = false;
    
    //Create Pointcloud bins
    std::vector<std::vector<BinInfo>> obstaclePointCloudAngleBins;
    std::vector<std::vector<BinInfo>> rawPointCloudAngleBins;
    
    //Create an OccupancyGrid message
    nav_msgs::OccupancyGrid occupancyGrid_;

    void obstaclePointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        latestObstaclePointCloud = msg;
        obstaclePointCloudUpdated = true;
    }

    void rawPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        latestRawPointCloud = msg;
        rawPointCloudUpdated = true;
    }

    void publish_msg(nav_msgs::OccupancyGrid& occupancyGrid_){
        occupancyGrid_.header.stamp = ros::Time::now();
        occupancyGrid_.header.frame_id = "map";
        occupancyGridMapPub.publish(occupancyGrid_);
    }
    void generateGridMap() {
    occupancyGridMap.processPointCloudsIfUpdated(rawPointCloudUpdated, obstaclePointCloudUpdated,
                                                 latestRawPointCloud, latestObstaclePointCloud,
                                                 obstaclePointCloudAngleBins, rawPointCloudAngleBins);

    occupancyGridMap.initializeFreeSpace(obstaclePointCloudAngleBins, rawPointCloudAngleBins, occupancyGrid_);
    occupancyGridMap.fillUnknownCells(obstaclePointCloudAngleBins, rawPointCloudAngleBins, occupancyGrid_);
    occupancyGridMap.fillOccupiedCells(obstaclePointCloudAngleBins, occupancyGrid_);
    //occupancyGridMap.smoothOccupancyGrid(occupancyGrid_);
    publish_msg(occupancyGrid_);
    
    occupancyGridMap.clearOccupancyGrid(occupancyGrid_);
    
    }

}; 

int main(int argc, char** argv) {
    ros::init(argc, argv, "occupancy_grid_map_node");
    RayCasting rayCasting;
    rayCasting.occupancyGridMap.initializeGrid(rayCasting.occupancyGrid_);

    ros::Rate r(10);
    while (ros::ok()) {
        
        rayCasting.generateGridMap();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
