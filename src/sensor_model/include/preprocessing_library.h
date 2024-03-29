#pragma once

#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

class PreprocessingLibrary {
    private:
        std::string parameters_file_name = "/home/ericga/MASTER_THESIS/TGM_WS/src/sensor_model/params/preprocessing.yaml";
        double leaf_size;
        double upperlim_height;
        double lowerlim_height;
        double cropbox_size;
        double StddevMulThresh;
        double meanK;
        double RadiusSearch;
        double NeighborsInRadius;
        double DistanceThreshold;
        double EpsAngle;
        double NormalDistanceWeight;
        double MaxIterations;
        double NumSeeds;
        ros::NodeHandle nh;
    
    public:
        PreprocessingLibrary();

        void loadParameters(const std::string& filename);
        void processPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void voxelDownsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void cropBoxFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void groundRemovalNormalSeeds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void groundRemovalRandomSeeds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi);
    
};