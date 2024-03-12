#include "occupancy_grid_map_library.h"
#include "nav_msgs/OccupancyGrid.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <vector>
#include <cmath>

namespace occupancy_cost_value
{
    static const unsigned char UNKNOWN = -1;  
    static const unsigned char OBSTACLE = 100;
    static const unsigned char FREE_SPACE = 0;
}
struct BinInfo
{   
    
    double range;
    double wx;
    double wy;

    BinInfo() = default;
    BinInfo(double range, double wx, double wy) : range(range), wx(wx), wy(wy) {}
};

class RayCasting {
private:
    ros::NodeHandle nh;
    ros::Subscriber obstaclePointCloudSub;
    ros::Subscriber rawPointCloudSub;
    ros::Publisher occupancyGridMapPub;

    sensor_msgs::PointCloud2::ConstPtr latestRawPointCloud;
    sensor_msgs::PointCloud2::ConstPtr latestObstaclePointCloud;
    
    //Create an OccupancyGrid message
    nav_msgs::OccupancyGrid occupancyGrid_;    
    bool rawPointCloudUpdated = false;
    bool obstaclePointCloudUpdated = false;

    std::vector<std::vector<BinInfo>> obstaclePointCloudAngleBins;
    std::vector<std::vector<BinInfo>> rawPointCloudAngleBins;

public:
    //variables
    double distance_margin_ = 0.5;
    float origin_x_ = -25;
    float origin_y_ = -25;
    float resolution_ = 0.5;
    int width_= 100;
    int height_ = 100;
    

    RayCasting() : nh("~") {
        obstaclePointCloudSub = nh.subscribe("/pointcloud_preprocessed", 1, &RayCasting::obstaclePointCloudCallback, this);
        rawPointCloudSub = nh.subscribe("/ouster/points", 1, &RayCasting::rawPointCloudCallback, this);
        occupancyGridMapPub = nh.advertise<nav_msgs::OccupancyGrid>("/grid_map", 1); //cambiar este publisher e introducir el tipo correcto.
        occupancyGrid_.info.resolution = resolution_; // Example resolution
        occupancyGrid_.info.width = width_; // Example width
        occupancyGrid_.info.height = height_; // Example height
        occupancyGrid_.info.origin.position.x = origin_x_; // Example origin
        occupancyGrid_.info.origin.position.y = origin_y_; // Assuming square grid
        occupancyGrid_.data.resize(occupancyGrid_.info.width * occupancyGrid_.info.height, occupancy_cost_value::UNKNOWN);
    }

    void obstaclePointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        latestObstaclePointCloud = msg;
        obstaclePointCloudUpdated = true;
    }

    void rawPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        latestRawPointCloud = msg;
        rawPointCloudUpdated = true;
    }
    // Function to convert map coordinates to grid coordinates
    std::pair<int, int> mapToGrid(double mx, double my) {
        int gx = static_cast<int>((mx - origin_x_) / resolution_);
        int gy = static_cast<int>((my - origin_y_) / resolution_);
        return {gx, gy};
    }

    // Bresenham's line algorithm for raytracing
    void raytrace(double x0, double y0, double x1, double y1, int value) {
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
    void initializeFreeSpace(const geometry_msgs::Pose &scan_origin) {
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

            raytrace(scan_origin.position.x, scan_origin.position.y, end_distance.wx, end_distance.wy, occupancy_cost_value::FREE_SPACE);
        }
    }
    void setCellValue(double wx, double wy, unsigned char value) {
    auto [gx, gy] = mapToGrid(wx, wy);

    if (gx >= 0 && gx < occupancyGrid_.info.width && gy >= 0 && gy < occupancyGrid_.info.height) {
        int index = gy * occupancyGrid_.info.width + gx;
        // Ensure that UNKNOWN values are correctly handled.
        occupancyGrid_.data[index] = (value == occupancy_cost_value::UNKNOWN) ? -1 : value;
    } else {
        ROS_WARN("Attempted to set cell value outside of occupancy grid bounds.");
    }
}
    void fillUnknownCells() {
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
                        raytrace(source.wx, source.wy, target.wx, target.wy, occupancy_cost_value::UNKNOWN);
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
                    raytrace(source.wx, source.wy, target.wx, target.wy, occupancy_cost_value::UNKNOWN);
                    if (!noFreespacePoint && target.wx == rawDistanceIter->wx && target.wy == rawDistanceIter->wy) {
                        // Optionally mark the target raw point as free space
                    }
                }
            }
        }
    }
    void fillOccupiedCells() {
    for (size_t bin_index = 0; bin_index < obstaclePointCloudAngleBins.size(); ++bin_index) {
        auto &obstacleBin = obstaclePointCloudAngleBins[bin_index];
        for (size_t dist_index = 0; dist_index < obstacleBin.size(); ++dist_index) {
            const auto &source = obstacleBin[dist_index];
            // Mark the current obstacle point as LETHAL_OBSTACLE/OCCUPIED
            setCellValue(source.wx, source.wy, occupancy_cost_value::OBSTACLE);

            // Check if there's a next obstacle point in the same bin
            if (dist_index + 1 < obstacleBin.size()) {
                const auto &nextSource = obstacleBin[dist_index + 1];
                // Calculate the distance between the current and the next obstacle point
                double obstacleDistance = std::hypot(nextSource.wx - source.wx, nextSource.wy - source.wy);

                // If the distance is less than or equal to the margin, fill the interval with LETHAL_OBSTACLE/OCCUPIED
                if (obstacleDistance <= distance_margin_) {
                    raytrace(source.wx, source.wy, nextSource.wx, nextSource.wy, occupancy_cost_value::OBSTACLE);
                }
            }
        }
    }
}
void clearOccupancyGrid() {
    std::fill(occupancyGrid_.data.begin(), occupancyGrid_.data.end(), occupancy_cost_value::UNKNOWN);
    
}

    void updateWithPointCloud(
        
        const sensor_msgs::PointCloud2::ConstPtr& rawPointCloud,
        const sensor_msgs::PointCloud2::ConstPtr& obstaclePointCloud,
        const geometry_msgs::Pose &robotPose,
        const geometry_msgs::Pose &scanOrigin) {

        
        double angle_increment = 0.25 * M_PI / 180; // Example: 1 degree resolution
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

        //initialize FreeSpace
        initializeFreeSpace(scanOrigin);
        fillUnknownCells();
        fillOccupiedCells();

        //set headers and publish
        occupancyGrid_.header.stamp = ros::Time::now();
        occupancyGrid_.header.frame_id = "map";
        occupancyGridMapPub.publish(occupancyGrid_);
        clearOccupancyGrid();
    }

    void processPointCloudsIfUpdated() {
        if (rawPointCloudUpdated && obstaclePointCloudUpdated) {
            geometry_msgs::Pose dummyPose; // Replace with actual robot pose
            geometry_msgs::Pose dummyScanOrigin; // Replace with actual scan origin
            updateWithPointCloud(latestRawPointCloud, latestObstaclePointCloud, dummyPose, dummyScanOrigin);
            rawPointCloudUpdated = false;
            obstaclePointCloudUpdated = false;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "occupancy_grid_map_node");
    RayCasting rayCasting;

    ros::Rate r(10);
    while (ros::ok()) {
        rayCasting.processPointCloudsIfUpdated();

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
