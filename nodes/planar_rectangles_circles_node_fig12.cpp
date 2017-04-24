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
    ros::NodeHandle ph("~");

    const size_t num_paths = ROSHelpers::GetParam(ph, "num_paths", 20);
    const bool visualize = ROSHelpers::GetParam(ph, "visualize", false);
    std::cout << std::flush;


    const bool latch = false;
    const uint32_t queue_size = 1000;
    ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", queue_size, latch);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", queue_size, latch);

    // Clear previous markers
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "mocap_world";
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::DELETEALL;
        marker.ns = "explored_states";
        marker.header.stamp = ros::Time::now();
        marker_pub.publish(marker);
        ros::spinOnce();
        sleep(1);
    }

    std::cout << "Creating map\n";
    auto map = PlanarRectangesCircles::CreateBhattacharyaExampleFig12();
    marker_array_pub.publish(map.getCollisionMapMarkers());
    ros::spinOnce();
    sleep(1);

    std::cout << "Running planner\n";
    const auto results = HSignatureAStar<PlanarRectangesCircles>::Plan(map, map.getStart(), map.getGoal(), num_paths, marker_pub, visualize);
    std::cout << std::endl << "Finished planning:\n" << results;
    for (size_t path_ind = 0; path_ind < results.paths_.size(); path_ind++)
    {
        marker_pub.publish(map.getPathMarker(results.paths_[path_ind], "paths", (int32_t)(path_ind + 1)));
    }

    ros::Rate rate(10);
    while (ros::ok())
    {
        marker_array_pub.publish(map.getCollisionMapMarkers());
        rate.sleep();
    }

    return -1;
}
