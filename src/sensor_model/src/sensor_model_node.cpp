// Necessary ROS and C++ headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


// Outlier removal
#include <pcl_ros/filters/radius_outlier_removal.h>
#include <pcl_ros/filters/statistical_outlier_removal.h>
#include <pcl_ros/filters/passthrough.h>

//Downsampling 
#include <pcl_ros/filters/voxel_grid.h>

class SensorModel {
private:
    // ROS node handle
    ros::NodeHandle nh;

    // ROS subscribers 
    ros::Subscriber pointCloudSub;
    
    // Other member variables as needed
    double leaf_size;
    double upperlim_voxel;
    double lowerlim_voxel;

public:
    ros::Publisher FilteredOccupancyGridPub;

    // Constructor
    SensorModel(): nh("~"){
        
        // Setup subscribers, publishers, etc.
        pointCloudSub = nh.subscribe("/ouster/points", 10, &SensorModel::pointCloudCallback, this);
        FilteredOccupancyGridPub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_preprocessed", 1);

        // Load parameters
        loadParameters();
    }

    // Functions
    void printParameters(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void voxelDownsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void loadParameters();

    // Callback function for the point cloud subscriber
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        
        // Apply the filters
        voxelDownsampling(cloud);

        // Update the header information
        cloud->header = pcl_conversions::toPCL(msg->header);

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);

        FilteredOccupancyGridPub.publish(cloud_msg);
    }
};


//FUNCTIONS 
void SensorModel::loadParameters(){
    if (!nh.getParam("leaf_size", leaf_size)) {
            ROS_ERROR("Failed to get parameter 'leaf_size'. Using default value.");
            leaf_size = 0.2;  // Set a default value
        }
        if (!nh.getParam("upperlim_voxel", upperlim_voxel)) {
            ROS_ERROR("Failed to get parameter 'upperlim_voxel'. Using default value.");
            upperlim_voxel = 5.0;  // Set a default value
        }
        if (!nh.getParam("lowerlim_voxel", lowerlim_voxel)) {
            ROS_ERROR("Failed to get parameter 'lowerlim_voxel'. Using default value.");
            lowerlim_voxel = -5.0;  // Set a default value
        }

}                            
void SensorModel::voxelDownsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(cloud);
    voxel.setFilterFieldName("z");
    voxel.setFilterLimits(lowerlim_voxel,upperlim_voxel);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.setFilterLimitsNegative(false);
    voxel.filter(*cloud);
}


void SensorModel::printParameters(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Accessing and printing the fields
    std::cout << "Height: " << msg->height << std::endl;
    std::cout << "Width: " << msg->width << std::endl;

    // Printing information about fields
    std::cout << "Fields: " << std::endl;
    for (const auto& field : msg->fields) {
        std::cout << "  Name: " << field.name << std::endl;
        std::cout << "  Offset: " << field.offset << std::endl;
        std::cout << "  Datatype: " << field.datatype << std::endl;
        std::cout << "  Count: " << field.count << std::endl;
    }
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "sensor_model_node");
    // Create an instance of your sensor model class
    SensorModel sensorModel;

    ros::Rate r(10);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
