#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

const std::string SENSOR_FRAME = "os_sensor";

class MeshPublisher {
public:
    visualization_msgs::Marker marker;
    MeshPublisher() : nh_("~") {
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        loadMarker();
    }

    void loadMarker() {
        marker.header.frame_id = SENSOR_FRAME;
        marker.header.stamp = ros::Time();
        marker.ns = "mesh";
        marker.id = 0;
        marker.type = 10;
        marker.mesh_resource = "package://rviz_visualizer/objects/car.stl";
        marker.action = 0;
        marker.pose.position.x = 3.15;  
        marker.pose.position.y = 1.375;
        marker.pose.position.z = -2.3;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 1.0;
        marker.pose.orientation.w = 0.0;
        marker.scale.x = 0.09;
        marker.scale.y = 0.09;
        marker.scale.z = 0.09;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
    }

    void publishMarker() {
        marker_pub_.publish(marker);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mesh_publisher_node");
    MeshPublisher meshPublisher;

    ros::Rate r(10); 
    while (ros::ok()) {
        meshPublisher.publishMarker();
        r.sleep();
    }

    return 0;
}
