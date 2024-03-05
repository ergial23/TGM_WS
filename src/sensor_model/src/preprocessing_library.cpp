#include "preprocessing_library.h"


PreprocessingLibrary::PreprocessingLibrary(ros::NodeHandle& nh) {
    PreprocessingLibrary::loadParameters();
}

//FUNCTIONS 
void PreprocessingLibrary::processPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    cropBoxFilter(cloud);
    voxelDownsampling(cloud);
    //radiusOutlierRemoval(cloud);
    //statisticalOutlierRemoval(cloud);
    groundRemoval(cloud);

}
void PreprocessingLibrary::loadParameters(){
    if (!nh.getParam("sensor_model_node/leaf_size", leaf_size)) {
        ROS_ERROR("Failed to get parameter 'leaf_size'. Using default value.");
        leaf_size = 0.2;  // Set a default value
    }
    if (!nh.getParam("sensor_model_node/upperlim_height", upperlim_height)) {
        ROS_ERROR("Failed to get parameter 'upperlim_height'. Using default value.");
        upperlim_height = 2;  // Set a default value
    }
    if (!nh.getParam("sensor_model_node/lowerlim_height", lowerlim_height)) {
        ROS_ERROR("Failed to get parameter 'lowerlim_height'. Using default value.");
        lowerlim_height = -2.5;  // Set a default value
    }
    if (!nh.getParam("sensor_model_node/cropbox_size", cropbox_size)) {
        ROS_ERROR("Failed to get parameter 'cropbox_size'. Using default value.");
        cropbox_size = 30;  // Set a default value
    }
    if (!nh.getParam("sensor_model_node/StddevMulThresh", StddevMulThresh)) {
        ROS_ERROR("Failed to get parameter 'StddevMulThresh'. Using default value.");
        StddevMulThresh = 1;  // Set a default value
    }
    if (!nh.getParam("sensor_model_node/meanK", meanK)) {
        ROS_ERROR("Failed to get parameter 'meanK'. Using default value.");
        meanK = 50;  // Set a default value
    }
    if (!nh.getParam("sensor_model_node/RadiusSearch", RadiusSearch)) {
        ROS_ERROR("Failed to get parameter 'RadiusSearch'. Using default value.");
        RadiusSearch = 2;  // Set a default value
    }
    if (!nh.getParam("sensor_model_node/NeighborsInRadius", NeighborsInRadius)) {
        ROS_ERROR("Failed to get parameter 'NeighborsInRadius'. Using default value.");
        NeighborsInRadius = 4;  // Set a default value
    }
    if (!nh.getParam("sensor_model_node/DistanceThreshold", DistanceThreshold)) {
        ROS_ERROR("Failed to get parameter 'DistanceThreshold'. Using default value.");
        DistanceThreshold = 0.2;  // Set a default value
    }
    if (!nh.getParam("sensor_model_node/EpsAngle", EpsAngle)) {
        ROS_ERROR("Failed to get parameter 'EpsAngle'. Using default value.");
        EpsAngle = 0.025;  // Set a default value
    }
    if (!nh.getParam("sensor_model_node/NormalDistanceWeight", NormalDistanceWeight)) {
        ROS_ERROR("Failed to get parameter 'NeighborsInRadius'. Using default value.");
        NormalDistanceWeight = 0.2;  // Set a default value
    }
    if (!nh.getParam("sensor_model_node/MaxIterations", MaxIterations)) {
        ROS_ERROR("Failed to get parameter 'MaxIterations'. Using default value.");
        MaxIterations = 500;  // Set a default value
    }

}      
void PreprocessingLibrary::cropBoxFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    // Create the filter
    pcl::CropBox<pcl::PointXYZ> crop_box;
   
    // Set input cloud
    crop_box.setInputCloud(cloud);

    // Set the region of interest (ROI) 
    Eigen::Vector4f min_pt(-cropbox_size, -cropbox_size, lowerlim_height, 1.0);  // Minimum point (x, y, z, 1.0 for homogeneous coordinates)
    Eigen::Vector4f max_pt(cropbox_size, cropbox_size, upperlim_height, 1.0);    // Maximum point (x, y, z, 1.0 for homogeneous coordinates)
    crop_box.setMin(min_pt);
    crop_box.setMax(max_pt);
    crop_box.setKeepOrganized(true);
    // Filter the cloud
    crop_box.filter(*cloud);

    Eigen::Vector4f min_pt2(-2.0, -1.0, -0.5, 1.0);  // Minimum point (x, y, z, 1.0 for homogeneous coordinates)
    Eigen::Vector4f max_pt2(0.0, 1.0, 0, 1.0);      // Maximum point (x, y, z, 1.0 for homogeneous coordinates)
    crop_box.setMin(min_pt2);
    crop_box.setMax(max_pt2);
    crop_box.setNegative(true);
    crop_box.setKeepOrganized(true);
    crop_box.filter(*cloud);
}  

          
void PreprocessingLibrary::voxelDownsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    // size of the voxels after downsampling
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.setFilterLimitsNegative(false);
    voxel.filter(*cloud);
}


void PreprocessingLibrary::radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // Create KDTree for efficient neighbor search
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // Vector to store the number of neighbors for each point
    std::vector<int> point_neighbors(cloud->size(), 0);

    // Iterate through each point in the input cloud
    for (size_t i = 0; i < cloud->size(); ++i) {
        // Find the indices of the neighbors within the radius
        std::vector<int> indices;
        std::vector<float> squared_distances;
        kdtree.radiusSearch(cloud->points[i], RadiusSearch, indices, squared_distances);

        // Count the number of neighbors
        point_neighbors[i] = static_cast<int>(indices.size());
    }

    // Create a new point cloud to store the filtered points
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Iterate through each point and keep it if it has enough neighbors
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (point_neighbors[i] >= NeighborsInRadius) {
            filtered_cloud->push_back(cloud->points[i]);
        }
    }

    // Swap the original cloud with the filtered points
    cloud->swap(*filtered_cloud);
}


void PreprocessingLibrary::statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // Create KDTree for efficient neighbor search
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // First pass: Compute mean and standard deviation of distances
    std::vector<double> distances;
    for (const auto& point : cloud->points) {
        // Find the indices of the k nearest neighbors
        std::vector<int> neighbor_indices;
        std::vector<float> squared_distances;
        kdtree.nearestKSearch(point, meanK, neighbor_indices, squared_distances);

        // Compute the average distance
        double avg_distance = 0.0;
        for (const auto& distance : squared_distances) {
            avg_distance += std::sqrt(distance);
        }
        avg_distance /= meanK;

        distances.push_back(avg_distance);
    }

    // Compute mean and standard deviation of distances
    double sum_distances = 0.0;
    for (const auto& distance : distances) {
        sum_distances += distance;
    }
    double mean_distance = sum_distances / distances.size();

    double sum_squared_diff = 0.0;
    for (const auto& distance : distances) {
        sum_squared_diff += (distance - mean_distance) * (distance - mean_distance);
    }
    double stddev_distance = std::sqrt(sum_squared_diff / distances.size());

    // Second pass: Eliminate outliers
    double threshold = mean_distance + StddevMulThresh * stddev_distance;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        if (distances[i] <= threshold) {
            filtered_cloud->points.push_back(cloud->points[i]);
        }
    }

    // Update the original cloud with the filtered points
    cloud->swap(*filtered_cloud);
}
void PreprocessingLibrary::computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointIndices::Ptr roi_indices,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi)
{
    // Define the Region of Interest (Bounding Box)
    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;
    minPoint << -30, -30, -3, 1.0;
    maxPoint << 30, 30, -1.5, 1.0;

    // Crop the input cloud based on the defined bounding box (ROI)
    pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> crop_box_filter;
    crop_box_filter.setInputCloud(cloud);
    crop_box_filter.setMin(minPoint);
    crop_box_filter.setMax(maxPoint);
    crop_box_filter.filter(roi_indices->indices);

    // Extract the ROI points using the computed indices
    
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(roi_indices);
    extract_indices.filter(*cloud_roi);

    // Compute normals for the ROI cloud
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_roi);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);
}

void PreprocessingLibrary::groundRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{   std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, nan_indices);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointIndices::Ptr roi_indices(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);

    // Compute normals for the ROI cloud and get original indices
    computeNormals(cloud, cloud_normals, roi_indices,cloud_roi);

    // Find Plane
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> segmentor;
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);
    segmentor.setAxis(axis);
    segmentor.setMaxIterations(MaxIterations);
    segmentor.setDistanceThreshold(DistanceThreshold);
    segmentor.setEpsAngle(EpsAngle);
    segmentor.setNormalDistanceWeight(NormalDistanceWeight);
    segmentor.setInputCloud(cloud_roi);  // Use the ROI cloud for ground removal
    segmentor.setInputNormals(cloud_normals);

    // Output plane
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    segmentor.segment(*inliers_plane, *coefficients_plane);

    // Extract the planar inliers from the ROI cloud using original indices
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(roi_indices);
    extract_indices.setNegative(true); 
    extract_indices.filter(*cloud);
}


