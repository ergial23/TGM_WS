// Necessary ROS and C++ headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


// Outlier removal
#include <pcl_ros/filters/radius_outlier_removal.h>
#include <pcl_ros/filters/statistical_outlier_removal.h>
#include <pcl_ros/filters/crop_box.h>
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
    void statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void radiousOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void cropBoxFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void loadParameters();

    // Callback function for the point cloud subscriber
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        // Apply the filters
        //statisticalOutlierRemoval(cloud);
        
        cropBoxFilter(cloud);
        //radiousOutlierRemoval(cloud);
        //voxelDownsampling(cloud);
        

        // Update the header information
        //cloud->header = pcl_conversions::toPCL(msg->header);

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
void SensorModel::cropBoxFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    // Create the filter
    pcl::CropBox<pcl::PointXYZRGB> crop_box;

    // Set input cloud
    crop_box.setInputCloud(cloud);

    // Set the region of interest (ROI) 
    Eigen::Vector4f min_pt(-30.0, -30.0, -2.0, 1.0);  // Minimum point (x, y, z, 1.0 for homogeneous coordinates)
    Eigen::Vector4f max_pt(30.0, 30.0, 2.0, 1.0);    // Maximum point (x, y, z, 1.0 for homogeneous coordinates)
    crop_box.setMin(min_pt);
    crop_box.setMax(max_pt);
    // Filter the cloud
    crop_box.filter(*cloud);

    Eigen::Vector4f min_pt2(-2.0, -1.0, -0.5, 1.0);  // Minimum point (x, y, z, 1.0 for homogeneous coordinates)
    Eigen::Vector4f max_pt2(0.0, 1.0, 0, 1.0);      // Maximum point (x, y, z, 1.0 for homogeneous coordinates)
    crop_box.setMin(min_pt2);
    crop_box.setMax(max_pt2);
    crop_box.setNegative(true);
    
    crop_box.filter(*cloud);
}  

          
void SensorModel::voxelDownsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(cloud);
    voxel.setFilterFieldName("z");
    voxel.setFilterLimits(lowerlim_voxel,upperlim_voxel);
    // size of the voxels after downsampling
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.setFilterLimitsNegative(false);
    voxel.filter(*cloud);
}

void SensorModel::statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlier_filter;
    outlier_filter.setInputCloud(cloud);
    // Set the number of nearest neighbors to use for mean distance estimation
    outlier_filter.setMeanK(50);
    // standard deviation multiplier for the distance threshold calculation.
    // The distance threshold will be equal to: mean + stddev_mult * stddev.
    // points will be classified as inlier or outlier if their average neighbor distance is below or above this threshold respectively.
    outlier_filter.setStddevMulThresh(1.0);
    outlier_filter.filter(*cloud);
    
}
void SensorModel::radiousOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // Create the radius outlier removal object and set the parameters
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outlier_filter;
    outlier_filter.setInputCloud(cloud);
    outlier_filter.setRadiusSearch(2);  // Adjust the radius based on your specific requirements
    outlier_filter.setMinNeighborsInRadius(4);  // Adjust as needed
    outlier_filter.filter(*cloud);
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
