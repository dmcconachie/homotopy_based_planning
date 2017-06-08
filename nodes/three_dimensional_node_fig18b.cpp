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
//    ros::NodeHandle ph("~");

//    Eigen::IOFormat one_line(Eigen::FullPrecision, 0, " ", " ");

//    const size_t num_paths = ROSHelpers::GetParam(ph, "num_paths", 1);
//    const bool visualize = ROSHelpers::GetParam(ph, "visualize", true);

    const bool latch = false;
    const uint32_t queue_size = 1;
    ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", queue_size, latch);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", queue_size, latch);

    std::cout << "Creating map\n";
    auto map = ThreeDimensionalEnvironment::CreateBhattacharyaExampleFig18();
    marker_array_pub.publish(map.getCollisionMapMarkers());
    ros::spinOnce();
    sleep(1);

    std::cout << "Running planner\n";
    const auto initial_results = HSignatureAStar<ThreeDimensionalEnvironment>::Plan(map, map.getStart(), map.getGoal(), 1, marker_pub, false);
    marker_pub.publish(map.getPathMarker(initial_results.paths_[0], "paths", 10));
    map.clearBlacklist();

    map.appendToWhitelist(-1.0 * initial_results.hsignatures_[0]);
    const auto complementary_results = HSignatureAStar<ThreeDimensionalEnvironment>::Plan(map, map.getStart(), map.getGoal(), 1, marker_pub, true);
    marker_pub.publish(map.getPathMarker(complementary_results.paths_[0], "paths", 1));

    ros::Rate rate(10);
    while (ros::ok())
    {
        marker_array_pub.publish(map.getCollisionMapMarkers());
        rate.sleep();
    }

    return -1;
}
