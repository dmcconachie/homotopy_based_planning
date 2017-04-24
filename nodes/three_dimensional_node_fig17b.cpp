#include <Eigen/StdVector>
#include <ros/ros.h>
#include <arc_utilities/ros_helpers.hpp>
#include "homotopy_based_planning/three_dimensional_environment.h"
#include "homotopy_based_planning/h_signature_astar.hpp"

using namespace hbp;

int main(int argc, char* argv[])
{
    // Read in all ROS parameters
    ros::init(argc, argv, "three_dimensional_node");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    Eigen::IOFormat one_line(Eigen::FullPrecision, 0, " ", " ");

    const size_t num_paths = ROSHelpers::GetParam(ph, "num_paths", 4);
    const bool visualize = ROSHelpers::GetParam(ph, "visualize", true);

    const bool latch = false;
    const uint32_t queue_size = 1000;
    ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", queue_size, latch);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", queue_size, latch);

    std::cout << "Creating map\n";
    auto map = ThreeDimensionalEnvironment::CreateBhattacharyaExampleFig17b();
    marker_array_pub.publish(map.getCollisionMapMarkers());
    ros::spinOnce();
    sleep(1);

    std::cout << "Running planner\n";
    const auto results = HSignatureAStar<ThreeDimensionalEnvironment>::Plan(map, map.getStart(), map.getGoal(), num_paths, marker_pub, visualize);
    marker_pub.publish(map.getPathMarker(results.paths_[0], "paths", 10));
    map.clearBlacklist();

    ros::Rate rate(10);
    while (ros::ok())
    {
        marker_array_pub.publish(map.getCollisionMapMarkers());
        rate.sleep();
    }

    return -1;
}
