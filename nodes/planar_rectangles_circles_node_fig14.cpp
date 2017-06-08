#include <Eigen/StdVector>
#include <ros/ros.h>
#include <arc_utilities/ros_helpers.hpp>
#include "homotopy_based_planning/planar_rectangles_circles_environment.h"
#include "homotopy_based_planning/h_signature_astar.hpp"

using namespace hbp;

int main(int argc, char* argv[])
{
    // Read in all ROS parameters
    ros::init(argc, argv, "planar_rectangles_circles_node");
    ros::NodeHandle nh;
//    ros::NodeHandle ph("~");

//    Eigen::IOFormat one_line(Eigen::FullPrecision, 0, " ", " ");

//    const size_t num_paths = ROSHelpers::GetParam(ph, "num_paths", 1);
//    const bool visualize = ROSHelpers::GetParam(ph, "visualize", true);
    std::cout << std::flush;


    const bool latch = false;
    const uint32_t queue_size = 1000;
    ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", queue_size, latch);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", queue_size, latch);

    std::cout << "Creating map\n";
    auto map = PlanarRectangesCircles::CreateBhattacharyaExampleFig14();
    marker_array_pub.publish(map.getCollisionMapMarkers());
    ros::spinOnce();
    sleep(1);

    std::cout << "Running planner\n";
    const auto first_robot_results = HSignatureAStar<PlanarRectangesCircles>::Plan(map, map.getStart(), map.getGoal(), 1, marker_pub, false);
    marker_pub.publish(map.getPathMarker(first_robot_results.paths_[0], "paths", 10));
    map.clearBlacklist();

    const auto second_robot_results = HSignatureAStar<PlanarRectangesCircles>::Plan(map, map.getStart2ndRobot(), map.getGoal2ndRobot(), 1, marker_pub, false);
    marker_pub.publish(map.getPathMarker(second_robot_results.paths_[0], "paths", 1));
    map.clearBlacklist();

    const auto second_robot_results_connect_to_first = HSignatureAStar<PlanarRectangesCircles>::Plan(map, map.getStart2ndRobot(), map.getStart(), 1, marker_pub, false);
    marker_pub.publish(map.getPathMarker(second_robot_results_connect_to_first.paths_[0], "paths", 6));
    map.clearBlacklist();

    const auto second_robot_results_connect_to_goal = HSignatureAStar<PlanarRectangesCircles>::Plan(map, map.getGoal(), map.getGoal2ndRobot(), 1, marker_pub, false);
    marker_pub.publish(map.getPathMarker(second_robot_results_connect_to_goal.paths_[0], "paths", 4));
    map.clearBlacklist();

    std::cout << "Creating combined H-Signature constraint\n";
    map.appendToWhitelist(second_robot_results_connect_to_first.hsignatures_[0] + first_robot_results.hsignatures_[0] + second_robot_results_connect_to_goal.hsignatures_[0]);
    const auto second_robot_results_constrained = HSignatureAStar<PlanarRectangesCircles>::Plan(map, map.getStart2ndRobot(), map.getGoal2ndRobot(), 1, marker_pub, false);
    marker_pub.publish(map.getPathMarker(second_robot_results_constrained.paths_[0], "paths", 1));


    ros::Rate rate(10);
    while (ros::ok())
    {
        marker_array_pub.publish(map.getCollisionMapMarkers());
        rate.sleep();
    }

    return -1;
}
