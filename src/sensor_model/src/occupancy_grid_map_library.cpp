#include "occupancy_grid_map_library.h"


OccupancyGridMapLibrary::OccupancyGridMapLibrary(){
    OccupancyGridMapLibrary::loadParametersOccupancy(parameters_file_name);
}

void OccupancyGridMapLibrary::loadParametersOccupancy(const std::string& filename){
    // Open the YAML file
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening YAML file: " << filename << std::endl;
        return;
    }

    // Parse the YAML file
    YAML::Node config = YAML::Load(file);

    // Access parameters from the YAML file
    
    distance_margin_ = config["distance_margin"].as<float>();
    gridmap_size_ = config["gridmap_size"].as<float>();
    resolution_ = config["resolution"].as<float>();
    angular_resolution = config["angular_resolution"].as<double>();
    
    UNKNOWN = config["UNKNOWN"].as<double>();
    FREE_SPACE = config["FREE_SPACE"].as<double>();
    OBSTACLE = config["OBSTACLE"].as<double>();
    
    width_ = gridmap_size_ / resolution_;
    height_ = gridmap_size_ / resolution_;
    origin_x_ = - gridmap_size_ / 2;
    origin_y_ = - gridmap_size_ / 2;
    
}
void OccupancyGridMapLibrary::initializeGrid(nav_msgs::OccupancyGrid& occupancyGrid){
    occupancyGrid.info.resolution = resolution_; // Example resolution
    occupancyGrid.info.width = width_; // Example width
    occupancyGrid.info.height = height_; // Example height
    occupancyGrid.info.origin.position.x = origin_x_; // Example origin
    occupancyGrid.info.origin.position.y = origin_y_; // Assuming square grid
    occupancyGrid.data.resize(occupancyGrid.info.width * occupancyGrid.info.height, UNKNOWN);
}

std::pair<int, int> OccupancyGridMapLibrary::mapToGrid(double mx, double my){
    int gx = static_cast<int>((mx - origin_x_) / resolution_);
    int gy = static_cast<int>((my - origin_y_) / resolution_);
    
    return {gx, gy};
}

void OccupancyGridMapLibrary::raytrace(double x0, double y0, double x1, double y1, int value, nav_msgs::OccupancyGrid& occupancyGrid_) {
    auto [gx0, gy0] = mapToGrid(x0, y0);
    auto [gx1, gy1] = mapToGrid(x1, y1);
    int dx = std::abs(gx1 - gx0);
    int dy = std::abs(gy1 - gy0);
    int sx = gx0 < gx1 ? 1 : -1;
    int sy = gy0 < gy1 ? 1 : -1;
    int err = dx - dy;
    while (true) {
        if (gx0 >= 0 && gx0 < occupancyGrid_.info.width && gy0 >= 0 && gy0 < occupancyGrid_.info.height) {
            occupancyGrid_.data[gy0 * occupancyGrid_.info.width + gx0] = value;
        }
        if (gx0 == gx1 && gy0 == gy1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; gx0 += sx; }
        if (e2 < dx) { err += dx; gy0 += sy; }
    }
}

void OccupancyGridMapLibrary::setCellValue(double wx, double wy, unsigned char value, nav_msgs::OccupancyGrid& occupancyGrid_) {
    auto [gx, gy] = mapToGrid(wx, wy);

    if (gx >= 0 && gx < occupancyGrid_.info.width && gy >= 0 && gy < occupancyGrid_.info.height) {
        int index = gy * occupancyGrid_.info.width + gx;
        // Ensure that UNKNOWN values are correctly handled.
        occupancyGrid_.data[index] = (value == UNKNOWN) ? -1 : value;
    } else {
        ROS_WARN("Attempted to set cell value outside of occupancy grid bounds.");
    }
}

 void OccupancyGridMapLibrary::processPointCloudsIfUpdated( bool rawPointCloudUpdated, 
                                                            bool obstaclePointCloudUpdated,
                                                            sensor_msgs::PointCloud2::ConstPtr latestRawPointCloud,
                                                            sensor_msgs::PointCloud2::ConstPtr latestObstaclePointCloud,
                                                            std::vector<std::vector<BinInfo>> &obstaclePointCloudAngleBins,
                                                            std::vector<std::vector<BinInfo>> &rawPointCloudAngleBins){
    
    if (rawPointCloudUpdated && obstaclePointCloudUpdated) {
        updateWithPointCloud(latestRawPointCloud, latestObstaclePointCloud, robot_pose_, scan_origin_, obstaclePointCloudAngleBins, rawPointCloudAngleBins);
        rawPointCloudUpdated = false;
        obstaclePointCloudUpdated = false;
    }
 }
 void OccupancyGridMapLibrary::updateWithPointCloud(
        const sensor_msgs::PointCloud2::ConstPtr& rawPointCloud,
        const sensor_msgs::PointCloud2::ConstPtr& obstaclePointCloud,
        const geometry_msgs::Pose &robotPose,
        const geometry_msgs::Pose &scanOrigin,
        std::vector<std::vector<BinInfo>> &obstaclePointCloudAngleBins,
        std::vector<std::vector<BinInfo>> &rawPointCloudAngleBins
        ){
        
        double angle_increment = angular_resolution * M_PI / 180; // Example: 1 degree resolution
        double min_angle = -M_PI;
        double max_angle = M_PI;
        int angle_bin_size = static_cast<int>((max_angle - min_angle) / angle_increment);

        // Initialize bins
        obstaclePointCloudAngleBins.clear();
        rawPointCloudAngleBins.clear();
        obstaclePointCloudAngleBins.resize(angle_bin_size);
        rawPointCloudAngleBins.resize(angle_bin_size);

        // Convert to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr rawCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacleCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*rawPointCloud, *rawCloud);
        pcl::fromROSMsg(*obstaclePointCloud, *obstacleCloud);

        // Lambda to process a cloud and fill bins
        auto processCloud = [this, min_angle, max_angle, angle_increment](const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<std::vector<BinInfo>> &bins) {
            for (const auto &point : *cloud) {
                const double angle = atan2(point.y, point.x);
                if (angle >= min_angle && angle < max_angle) {
                    int angle_bin_index = static_cast<int>((angle - min_angle) / angle_increment);
                    bins[angle_bin_index].emplace_back(std::hypot(point.x, point.y), point.x, point.y);
                }
            }

            // Sort bins by range
            for (auto &bin : bins) {
                std::sort(bin.begin(), bin.end(), [](const BinInfo &a, const BinInfo &b) {
                    return a.range < b.range;
                });
            }
        };

        // Process both clouds
        processCloud(rawCloud, rawPointCloudAngleBins);
        processCloud(obstacleCloud, obstaclePointCloudAngleBins);

}

void OccupancyGridMapLibrary::initializeFreeSpace( std::vector<std::vector<BinInfo>> &obstaclePointCloudAngleBins,
                                                   std::vector<std::vector<BinInfo>> &rawPointCloudAngleBins,
                                                   nav_msgs::OccupancyGrid& occupancyGrid_){
    for (size_t bin_index = 0; bin_index < obstaclePointCloudAngleBins.size(); ++bin_index) {
            auto &obstacleBin = obstaclePointCloudAngleBins.at(bin_index);
            auto &rawBin = rawPointCloudAngleBins.at(bin_index);

            BinInfo end_distance;
            if (rawBin.empty() && obstacleBin.empty()) {
                continue;
            } else if (rawBin.empty()) {
                end_distance = obstacleBin.back();
            } else if (obstacleBin.empty()) {
                end_distance = rawBin.back();
            } else {
                end_distance = (obstacleBin.back().range + distance_margin_ < rawBin.back().range) ? rawBin.back() : obstacleBin.back();
            }

            raytrace(scan_origin_.position.x, scan_origin_.position.y, end_distance.wx, end_distance.wy, FREE_SPACE,occupancyGrid_);
        }
}

void OccupancyGridMapLibrary::fillUnknownCells(std::vector<std::vector<BinInfo>> &obstaclePointCloudAngleBins,
                                               std::vector<std::vector<BinInfo>> &rawPointCloudAngleBins,
                                               nav_msgs::OccupancyGrid& occupancyGrid_){
    for (size_t bin_index = 0; bin_index < obstaclePointCloudAngleBins.size(); ++bin_index) {
        auto &obstacleBin = obstaclePointCloudAngleBins[bin_index];
        auto &rawBin = rawPointCloudAngleBins[bin_index];
        auto rawDistanceIter = rawBin.begin();
        for (size_t dist_index = 0; dist_index < obstacleBin.size(); ++dist_index) {
            // Find the next raw point that is farther than the current obstacle point plus the margin
            while (rawDistanceIter != rawBin.end() && rawDistanceIter->range <= obstacleBin[dist_index].range + distance_margin_) {
                ++rawDistanceIter;
            }
            // Determine if there's no free space point beyond the current obstacle point
            bool noFreespacePoint = (rawDistanceIter == rawBin.end());
            // Process the last obstacle point separately
            if (dist_index + 1 == obstacleBin.size()) {
                if (!noFreespacePoint) {
                    // If there's a raw point beyond the last obstacle, mark the region between them as unknown
                    const auto &source = obstacleBin[dist_index];
                    const auto &target = *rawDistanceIter;
                    raytrace(source.wx, source.wy, target.wx, target.wy,UNKNOWN, occupancyGrid_);
                }
                // No further processing needed for the last obstacle point
                continue;
            }
            const auto &source = obstacleBin[dist_index];
            auto &nextObstacle = obstacleBin[dist_index + 1];
            // Calculate the distance to the next obstacle point
            double nextObstacleDistance = nextObstacle.range - source.range;
            // If there's a significant gap to the next obstacle, or there's no raw point beyond the current obstacle,
            // mark the region as unknown
            if (nextObstacleDistance > distance_margin_ || noFreespacePoint) {
                auto target = noFreespacePoint ? nextObstacle : *rawDistanceIter;
                raytrace(source.wx, source.wy, target.wx, target.wy, UNKNOWN,occupancyGrid_);
                if (!noFreespacePoint && target.wx == rawDistanceIter->wx && target.wy == rawDistanceIter->wy) {
                    // Optionally mark the target raw point as free space
                }
            }
        }
    }
}
void OccupancyGridMapLibrary::fillOccupiedCells(std::vector<std::vector<BinInfo>> &obstaclePointCloudAngleBins,
                                               std::vector<std::vector<BinInfo>> &rawPointCloudAngleBins,
                                               nav_msgs::OccupancyGrid& occupancyGrid_){
    for (size_t bin_index = 0; bin_index < obstaclePointCloudAngleBins.size(); ++bin_index) {
        auto &obstacleBin = obstaclePointCloudAngleBins[bin_index];
        for (size_t dist_index = 0; dist_index < obstacleBin.size(); ++dist_index) {
            const auto &source = obstacleBin[dist_index];
            // Mark the current obstacle point as LETHAL_OBSTACLE/OCCUPIED
            setCellValue(source.wx, source.wy, OBSTACLE, occupancyGrid_);

            // Check if there's a next obstacle point in the same bin
            if (dist_index + 1 < obstacleBin.size()) {
                const auto &nextSource = obstacleBin[dist_index + 1];
                // Calculate the distance between the current and the next obstacle point
                double obstacleDistance = std::hypot(nextSource.wx - source.wx, nextSource.wy - source.wy);

                // If the distance is less than or equal to the margin, fill the interval with LETHAL_OBSTACLE/OCCUPIED
                if (obstacleDistance <= distance_margin_) {
                    raytrace(source.wx, source.wy, nextSource.wx, nextSource.wy, OBSTACLE, occupancyGrid_);
                }
            }
        }
    }
}
void OccupancyGridMapLibrary::clearOccupancyGrid(nav_msgs::OccupancyGrid& occupancyGrid_){
    std::fill(occupancyGrid_.data.begin(), occupancyGrid_.data.end(), UNKNOWN);

}