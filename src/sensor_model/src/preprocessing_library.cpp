#include "preprocessing_library.h"


PreprocessingLibrary::PreprocessingLibrary() {
    PreprocessingLibrary::loadParameters(parameters_file_name);
    
}

//FUNCTIONS

void PreprocessingLibrary::processPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    cropBoxFilter(cloud);
    statisticalOutlierRemoval(cloud);
    //radiusOutlierRemoval(cloud);
    voxelDownsampling(cloud);
    //groundRemovalNormalSeeds(cloud);
    groundRemovalRandomSeeds(cloud);
    
}
void PreprocessingLibrary::loadParameters(const std::string& filename) {
    // Open the YAML file
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening YAML file: " << filename << std::endl;
        return;
    }

    // Parse the YAML file
    YAML::Node config = YAML::Load(file);

    // Access parameters from the YAML file
    leaf_size = config["leaf_size"].as<double>();
    upperlim_height = config["upperlim_height"].as<double>();
    lowerlim_height = config["lowerlim_height"].as<double>();
    cropbox_size = config["cropbox_size"].as<double>();
    StddevMulThresh = config["StddevMulThresh"].as<double>();
    meanK = config["meanK"].as<double>();
    RadiusSearch = config["RadiusSearch"].as<double>();
    NeighborsInRadius = config["NeighborsInRadius"].as<double>();
    DistanceThreshold = config["DistanceThreshold"].as<double>();
    EpsAngle = config["EpsAngle"].as<double>();
    NormalDistanceWeight = config["NormalDistanceWeight"].as<double>();
    MaxIterations = config["MaxIterations"].as<double>();
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
    crop_box.setKeepOrganized(false);
    // Filter the cloud
    crop_box.filter(*cloud);

    Eigen::Vector4f min_pt2(-2.0, -1.0, -0.5, 1.0);  // Minimum point (x, y, z, 1.0 for homogeneous coordinates)
    Eigen::Vector4f max_pt2(0.0, 1.0, 0, 1.0);      // Maximum point (x, y, z, 1.0 for homogeneous coordinates)
    crop_box.setMin(min_pt2);
    crop_box.setMax(max_pt2);
    crop_box.setNegative(true);
    crop_box.setKeepOrganized(false);
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
void PreprocessingLibrary::computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi)
{
    // Define the Region of Interest (Bounding Box)
    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;
    minPoint << -40, -40, -4, 1.0;
    maxPoint << 40, 40, -1, 1.0;

    // Crop the input cloud based on the defined bounding box (ROI)
    pcl::CropBox<pcl::PointXYZ> crop_box_filter;
    crop_box_filter.setInputCloud(cloud);
    crop_box_filter.setMin(minPoint);
    crop_box_filter.setMax(maxPoint);
    crop_box_filter.setKeepOrganized(true);
    crop_box_filter.filter(*cloud_roi);

    // Compute normals for the ROI cloud
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_roi);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);
}

void PreprocessingLibrary::groundRemovalNormalSeeds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{   
    
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);

    // Compute normals for the ROI cloud and get original indices
    computeNormals(cloud, cloud_normals,cloud_roi);

    // Find Plane
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> segmentor;
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
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
    extract_indices.setIndices(inliers_plane);
    extract_indices.setNegative(true); 
    extract_indices.filter(*cloud);
}


void PreprocessingLibrary::groundRemovalRandomSeeds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
    // Define the Region of Interest (Bounding Box)
    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;
    minPoint << -40, -40, -4, 1.0;
    maxPoint << 40, 40, -1.25, 1.0;

    // Crop the input cloud based on the defined bounding box (ROI)
    pcl::CropBox<pcl::PointXYZ> crop_box_filter;
    crop_box_filter.setInputCloud(cloud);
    crop_box_filter.setMin(minPoint);
    crop_box_filter.setMax(maxPoint);
    crop_box_filter.setKeepOrganized(true);
    crop_box_filter.filter(*cloud_roi);

    // Find Plane with Random Seeds
    pcl::SACSegmentation<pcl::PointXYZ> segmentor;
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    segmentor.setAxis(Eigen::Vector3f(0,0,1));
    segmentor.setEpsAngle(EpsAngle);
    segmentor.setMaxIterations(MaxIterations);
    segmentor.setDistanceThreshold(DistanceThreshold);
    segmentor.setInputCloud(cloud_roi);  

     // Output plane
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    segmentor.segment(*inliers_plane, *coefficients_plane);

    // Extract the planar inliers from the entire cloud using original indices
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(inliers_plane);
    extract_indices.setNegative(true);
    extract_indices.filter(*cloud);
}


