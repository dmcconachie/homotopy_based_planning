#include "homotopy_based_planning/three_dimensional_environment.h"

#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <gsl/gsl_integration.h>
#include <gsl/gsl_errno.h>
#include <chrono>

#include <omp.h>

using namespace hbp;
using namespace EigenHelpers;
using namespace EigenHelpersConversions;


enum StopwatchControl {RESET, READ};

inline static double stopwatch(const StopwatchControl control = READ)
{
    static std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

    const std::chrono::high_resolution_clock::time_point end_time = std::chrono::high_resolution_clock::now();
    if (control == RESET)
    {
        start_time = end_time;
    }

    return std::chrono::duration<double>(end_time - start_time).count();
}


static visualization_msgs::Marker makeWaypointMarker(const ThreeDimensionalEnvironment::ConfigType& waypoint, const std::string& ns, const int id, const double scale, const double line_length)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "mocap_world";
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = scale;
    marker.color = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(1, 0, 0, 1);

    marker.points.push_back(EigenHelpersConversions::EigenVector3dToGeometryPoint(Eigen::Vector3d(waypoint.x() - line_length, waypoint.y(), waypoint.z())));
    marker.points.push_back(EigenHelpersConversions::EigenVector3dToGeometryPoint(Eigen::Vector3d(waypoint.x() + line_length, waypoint.y(), waypoint.z())));
    marker.points.push_back(EigenHelpersConversions::EigenVector3dToGeometryPoint(Eigen::Vector3d(waypoint.x(), waypoint.y() - line_length, waypoint.z())));
    marker.points.push_back(EigenHelpersConversions::EigenVector3dToGeometryPoint(Eigen::Vector3d(waypoint.x(), waypoint.y() + line_length, waypoint.z())));
    marker.points.push_back(EigenHelpersConversions::EigenVector3dToGeometryPoint(Eigen::Vector3d(waypoint.x(), waypoint.y(), waypoint.z() - line_length)));
    marker.points.push_back(EigenHelpersConversions::EigenVector3dToGeometryPoint(Eigen::Vector3d(waypoint.x(), waypoint.y(), waypoint.z() + line_length)));

    return marker;
}

static void addLineSegmentToCollisionMapGrid(sdf_tools::CollisionMapGrid& map, const Eigen::Vector3d& start, const Eigen::Vector3d& end, const double radius)
{
    const Eigen::Vector3d z_axis = (end - start).normalized();
    const Eigen::Vector3d x_axis = (z_axis.y() != 0.0 ? VectorRejection(z_axis , Eigen::Vector3d(1.5, 0, 0)) : VectorRejection(z_axis , Eigen::Vector3d(0, 1.5, 0))).normalized();
    const Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

    Eigen::Matrix3d rot;
    rot.col(0) = x_axis;
    rot.col(1) = y_axis;
    rot.col(2) = z_axis;

    for (double x = -radius; x <= radius; x += map.GetResolution() / 2.0)
    {
        for (double y = -radius; y <= radius; y += map.GetResolution() / 2.0)
        {
            for (double z = 0.0; z <= (end - start).norm(); z += map.GetResolution() / 2.0)
            {
                const bool in_circle = Eigen::Vector2d(x, y).squaredNorm() <= radius * radius;
                if (in_circle)
                {
                    const Eigen::Vector3d loc = rot * Eigen::Vector3d(x, y, z) + start;
                    map.Set3d(loc, sdf_tools::COLLISION_CELL(1.0));
                }
            }
        }
    }
}

static void addCylindricalLineStripToCollisionMapGrid(sdf_tools::CollisionMapGrid& map, const std::vector<Eigen::Vector3d>& line_strip, const double radius)
{
    for (size_t ind = 0; ind + 1 < line_strip.size(); ++ind)
    {
        addLineSegmentToCollisionMapGrid(map, line_strip[ind], line_strip[ind + 1], radius);
    }
}

static void addRectangularSolidToCollisionMapGrid(sdf_tools::CollisionMapGrid& map, const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    for (double x = a.x(); x <= b.x(); x += map.GetResolution())
    {
        for (double y = a.y(); y <= b.y(); y += map.GetResolution())
        {
            for (double z = a.z(); z <= b.z(); z += map.GetResolution())
            {
                map.Set(x, y, z, sdf_tools::COLLISION_CELL(1.0));
            }
        }
    }
}
/*
static void addRectangularLineStripToCollisionMapGrid(sdf_tools::CollisionMapGrid& map, const std::vector<Eigen::Vector3d>& line_strip, const double radius)
{
    for (size_t ind = 0; ind + 1 < line_strip.size(); ++ind)
    {
        const Eigen::Vector3d a = line_strip[ind] + Eigen::Vector3d(radius, radius, radius);
        const Eigen::Vector3d b = line_strip[ind] - Eigen::Vector3d(radius, radius, radius);
        addRectangularSolidToCollisionMapGrid(map, a, b);
    }
}
*/
// Section 6.2.1
ThreeDimensionalEnvironment ThreeDimensionalEnvironment::CreateBhattacharyaExampleFig17a()
{
    // Setup the world itself
    ThreeDimensionalEnvironment env;
    {
        const double res = 0.25;
        const double x_size = 20.0;
        const double y_size = 20.0;
        const double z_size = 18.0;

        env.marker_scale_ = 0.5;
        env.start_end_line_length_ = 1.0;

        auto grid_offset = Eigen::Isometry3d::Identity();
        grid_offset(0, 3) = 0.5;
        grid_offset(1, 3) = 0.5;

        env.collision_map_grid_ = sdf_tools::CollisionMapGrid(grid_offset, "mocap_world", res, x_size, y_size, z_size, sdf_tools::COLLISION_CELL(0.0), sdf_tools::COLLISION_CELL(1.0));

        assert(env.collision_map_grid_.GetNumXCells() == (ssize_t)x_size * 4);
        assert(env.collision_map_grid_.GetNumYCells() == (ssize_t)y_size * 4);
        assert(env.collision_map_grid_.GetNumZCells() == (ssize_t)z_size * 4);
    }

    // Setup obstacles
    {
        std::vector<ConfigType> obstacle;
        obstacle.push_back(ConfigType(8.0, 18.0, 4.0));
        obstacle.push_back(ConfigType(8.0, 18.0, 12.0));
        obstacle.push_back(ConfigType(5.0, 10.0, 16.0));
        obstacle.push_back(ConfigType(3.0, 6.0, 10.0));
        obstacle.push_back(ConfigType(3.0, 10.0, 6.0));
        obstacle.push_back(ConfigType(8.0, 18.0, 4.0));
        addCylindricalLineStripToCollisionMapGrid(env.collision_map_grid_, obstacle, 1.0);

        env.obstacles_.push_back(obstacle);
    }
    {
        std::vector<ConfigType> obstacle;
        obstacle.push_back(ConfigType(14.0, 2.0, 17.0));
        obstacle.push_back(ConfigType(18.0, 10.0, 17.0));
        obstacle.push_back(ConfigType(10.0, 16.0, 15.0));
        obstacle.push_back(ConfigType(10.0, 14.0, 12.0));
        obstacle.push_back(ConfigType(10.0, 8.0, 10.0));
        obstacle.push_back(ConfigType(12.0, 4.0, 14.0));
        obstacle.push_back(ConfigType(14.0, 2.0, 17.0));
        addCylindricalLineStripToCollisionMapGrid(env.collision_map_grid_, obstacle, 1.0);

        env.obstacles_.push_back(obstacle);
    }

    // Setup start and goal
    {
        env.start_ = ConfigType(2.0, 18.0, 6.0);
        env.goal_ = ConfigType(18.0, 2.0, 15.0);

        assert(env.collision_map_grid_.Get(env.start_.x(), env.start_.y(), env.start_.z()).first.occupancy == 0.0);
        assert(env.collision_map_grid_.Get(env.goal_.x(), env.goal_.y(), env.goal_.z()).first.occupancy == 0.0);
    }

    // Pre-generate markers for exporting
    {
        env.collision_map_marker_array_.markers.push_back(env.collision_map_grid_.ExportForDisplay(
                    arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(140.0f/255.0f, 153.0f/255.0f, 140.0f/255.0f, 1.0f),
                    arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(0.0f, 1.0f, 0.0f, 0.0f),
                    arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(0.0f, 0.0f, 1.0f, 0.1f)));

        env.collision_map_marker_array_.markers.push_back(makeWaypointMarker(env.start_, "start", 1, env.marker_scale_, env.start_end_line_length_));
        env.collision_map_marker_array_.markers.push_back(makeWaypointMarker(env.goal_, "goal", 1, env.marker_scale_, env.start_end_line_length_));
    }

    env.use_cached_h_vals_ = false;
    omp_set_num_threads(2);

    return env;
}

// Section 6.2.1
ThreeDimensionalEnvironment ThreeDimensionalEnvironment::CreateBhattacharyaExampleFig17b()
{
    // Setup the world itself
    ThreeDimensionalEnvironment env;
    {
        const double res = 1.0;
        const double x_size = 20.0;
        const double y_size = 20.0;
        const double z_size = 20.0;

        env.marker_scale_ = 0.5;
        env.start_end_line_length_ = 1.0;

        auto grid_offset = Eigen::Isometry3d::Identity();
        grid_offset(0, 3) = 0.5;
        grid_offset(1, 3) = 0.5;

        env.collision_map_grid_ = sdf_tools::CollisionMapGrid(grid_offset, "mocap_world", res, x_size, y_size, z_size, sdf_tools::COLLISION_CELL(0.0), sdf_tools::COLLISION_CELL(1.0));

        assert(env.collision_map_grid_.GetNumXCells() == (ssize_t)x_size);
        assert(env.collision_map_grid_.GetNumYCells() == (ssize_t)y_size);
        assert(env.collision_map_grid_.GetNumZCells() == (ssize_t)z_size);
    }

    // Setup obstacles
    {
        std::vector<ConfigType> obstacle;
        obstacle.push_back(ConfigType(4.0, 4.0, 4.0));
        obstacle.push_back(ConfigType(4.0, 4.0, 16.0));
        obstacle.push_back(ConfigType(4.0, 16.0, 16.0));
        obstacle.push_back(ConfigType(4.0, 16.0, 4.0));
        obstacle.push_back(ConfigType(4.0, 4.0, 4.0));
        addCylindricalLineStripToCollisionMapGrid(env.collision_map_grid_, obstacle, 2.0);

        env.obstacles_.push_back(obstacle);
    }
    {
        std::vector<ConfigType> obstacle;
        obstacle.push_back(ConfigType(4.0, 4.0, 4.0));
        obstacle.push_back(ConfigType(4.0, 4.0, 16.0));
        obstacle.push_back(ConfigType(16.0, 4.0, 16.0));
        obstacle.push_back(ConfigType(16.0, 4.0, 4.0));
        obstacle.push_back(ConfigType(4.0, 4.0, 4.0));
        addCylindricalLineStripToCollisionMapGrid(env.collision_map_grid_, obstacle, 2.0);

        env.obstacles_.push_back(obstacle);
    }
    {
        std::vector<ConfigType> obstacle;
        obstacle.push_back(ConfigType(4.0, 16.0, 4.0));
        obstacle.push_back(ConfigType(4.0, 16.0, 16.0));
        obstacle.push_back(ConfigType(16.0, 16.0, 16.0));
        obstacle.push_back(ConfigType(16.0, 16.0, 4.0));
        obstacle.push_back(ConfigType(4.0, 16.0, 4.0));
        addCylindricalLineStripToCollisionMapGrid(env.collision_map_grid_, obstacle, 2.0);

        env.obstacles_.push_back(obstacle);
    }
    {
        std::vector<ConfigType> obstacle;
        obstacle.push_back(ConfigType(16.0, 4.0, 4.0));
        obstacle.push_back(ConfigType(16.0, 4.0, 16.0));
        obstacle.push_back(ConfigType(16.0, 16.0, 16.0));
        obstacle.push_back(ConfigType(16.0, 16.0, 4.0));
        obstacle.push_back(ConfigType(16.0, 4.0, 4.0));
        addCylindricalLineStripToCollisionMapGrid(env.collision_map_grid_, obstacle, 2.0);

        env.obstacles_.push_back(obstacle);
    }
    {
        addRectangularSolidToCollisionMapGrid(env.collision_map_grid_, Eigen::Vector3d(2.0, 2.0, 2.0), Eigen::Vector3d(18.0, 18.0, 4.0));
        addRectangularSolidToCollisionMapGrid(env.collision_map_grid_, Eigen::Vector3d(2.0, 2.0, 16.0), Eigen::Vector3d(18.0, 18.0, 18.0));
    }

    // Setup start and goal
    {
        env.start_ = ConfigType(0.0, 1.0, 6.0);
//        env.start_ = ConfigType(0.0, 19.0, 6.0);
        env.goal_ = ConfigType(19.0, 1.0, 15.0);

        assert(env.collision_map_grid_.Get(env.start_.x(), env.start_.y(), env.start_.z()).first.occupancy == 0.0);
        assert(env.collision_map_grid_.Get(env.goal_.x(), env.goal_.y(), env.goal_.z()).first.occupancy == 0.0);
    }

    // Pre-generate markers for exporting
    {
        env.collision_map_marker_array_.markers.push_back(env.collision_map_grid_.ExportForDisplay(
                    arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(140.0f/255.0f, 153.0f/255.0f, 140.0f/255.0f, 1.0f),
                    arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(0.0f, 1.0f, 0.0f, 0.0f),
                    arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(0.0f, 0.0f, 1.0f, 0.1f)));

        env.collision_map_marker_array_.markers.push_back(makeWaypointMarker(env.start_, "start", 1, env.marker_scale_, env.start_end_line_length_));
        env.collision_map_marker_array_.markers.push_back(makeWaypointMarker(env.goal_, "goal", 1, env.marker_scale_, env.start_end_line_length_));
    }

    env.use_cached_h_vals_ = false;
    omp_set_num_threads(4);

    return env;
}

// Section 6.2.2/6.2.4
ThreeDimensionalEnvironment ThreeDimensionalEnvironment::CreateBhattacharyaExampleFig18()
{
    // Setup the world itself
    ThreeDimensionalEnvironment env;
    {
        const double res = 1.0;
        const double x_size = 44.0;
        const double y_size = 44.0;
        const double z_size = 44.0;

        env.marker_scale_ = 0.5;
        env.start_end_line_length_ = 1.0;

        auto grid_offset = Eigen::Isometry3d::Identity();
        grid_offset(0, 3) = 0.5;
        grid_offset(1, 3) = 0.5;

        env.collision_map_grid_ = sdf_tools::CollisionMapGrid(grid_offset, "mocap_world", res, x_size, y_size, z_size, sdf_tools::COLLISION_CELL(0.0), sdf_tools::COLLISION_CELL(1.0));

        assert(env.collision_map_grid_.GetNumXCells() == (ssize_t)x_size);
        assert(env.collision_map_grid_.GetNumYCells() == (ssize_t)y_size);
        assert(env.collision_map_grid_.GetNumZCells() == (ssize_t)z_size);
    }

    // Setup obstacles
    {
        std::vector<ConfigType> obstacle;
        obstacle.push_back(ConfigType(20.0, 500.0, 20.0));
        obstacle.push_back(ConfigType(20.0, 10.0, 20.0));
        obstacle.push_back(ConfigType(20.0, 10.0, -500.0));
        addCylindricalLineStripToCollisionMapGrid(env.collision_map_grid_, obstacle, 1.0);

        env.obstacles_.push_back(obstacle);
    }
    {
        std::vector<ConfigType> obstacle;
        obstacle.push_back(ConfigType(25.0, 500.0, 20.0));
        obstacle.push_back(ConfigType(25.0, 25.0, 20.0));
        obstacle.push_back(ConfigType(25.0, 25.0, 500.0));
        addCylindricalLineStripToCollisionMapGrid(env.collision_map_grid_, obstacle, 1.0);

        env.obstacles_.push_back(obstacle);
    }
    {
        std::vector<ConfigType> obstacle;
        obstacle.push_back(ConfigType(-500.0, 17.0, 25.0));
        obstacle.push_back(ConfigType(500.0, 17.0, 25.0));
        addCylindricalLineStripToCollisionMapGrid(env.collision_map_grid_, obstacle, 1.0);

        env.obstacles_.push_back(obstacle);
    }
    {
        std::vector<ConfigType> obstacle;
        obstacle.push_back(ConfigType(-500.0, 10.0, 25.0));
        obstacle.push_back(ConfigType(35.0, 10.0, 25.0));
        obstacle.push_back(ConfigType(35.0, 10.0, 40.0));
        obstacle.push_back(ConfigType(35.0, -500.0, 40.0));
        addCylindricalLineStripToCollisionMapGrid(env.collision_map_grid_, obstacle, 1.0);

        env.obstacles_.push_back(obstacle);
    }
    {
        std::vector<ConfigType> obstacle;
        obstacle.push_back(ConfigType(15.0, 30.0, -500.0));
        obstacle.push_back(ConfigType(15.0, 30.0, 25.0));
        obstacle.push_back(ConfigType(35.0, 30.0, 25.0));
        obstacle.push_back(ConfigType(35.0, 30.0, -500.0));
        addCylindricalLineStripToCollisionMapGrid(env.collision_map_grid_, obstacle, 1.0);

        env.obstacles_.push_back(obstacle);
    }
    {
        std::vector<ConfigType> obstacle;
        obstacle.push_back(ConfigType(30.0, 35.0, 500.0));
        obstacle.push_back(ConfigType(30.0, 35.0, 30.0));
        obstacle.push_back(ConfigType(30.0, 22.0, 30.0));
        obstacle.push_back(ConfigType(500.0, 22.0, 30.0));
        addCylindricalLineStripToCollisionMapGrid(env.collision_map_grid_, obstacle, 1.0);

        env.obstacles_.push_back(obstacle);
    }
    {
        std::vector<ConfigType> obstacle;
        obstacle.push_back(ConfigType(40.0, 5.0, 500.0));
        obstacle.push_back(ConfigType(40.0, 5.0, 10.0));
        obstacle.push_back(ConfigType(10.0, 5.0, 10.0));
        obstacle.push_back(ConfigType(10.0, 18.0, 10.0));
        obstacle.push_back(ConfigType(-500.0, 18.0, 10.0));
        addCylindricalLineStripToCollisionMapGrid(env.collision_map_grid_, obstacle, 1.0);

        env.obstacles_.push_back(obstacle);
    }



    // Setup start and goal
    {
        env.start_ = ConfigType(5.0, 5.0, 5.0);
        env.goal_ = ConfigType(41.0, 41.0, 41.0);

        assert(env.collision_map_grid_.Get(env.start_.x(), env.start_.y(), env.start_.z()).first.occupancy == 0.0);
        assert(env.collision_map_grid_.Get(env.goal_.x(), env.goal_.y(), env.goal_.z()).first.occupancy == 0.0);
    }

    // Pre-generate markers for exporting
    {
        env.collision_map_marker_array_.markers.push_back(env.collision_map_grid_.ExportForDisplay(
                    arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(140.0f/255.0f, 153.0f/255.0f, 140.0f/255.0f, 1.0f),
                    arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(0.0f, 1.0f, 0.0f, 0.0f),
                    arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(0.0f, 0.0f, 1.0f, 0.1f)));

        env.collision_map_marker_array_.markers.push_back(makeWaypointMarker(env.start_, "start", 1, env.marker_scale_, env.start_end_line_length_));
        env.collision_map_marker_array_.markers.push_back(makeWaypointMarker(env.goal_, "goal", 1, env.marker_scale_, env.start_end_line_length_));
    }

    // Generate H value cache
    omp_set_num_threads(7);
    {
        env.use_cached_h_vals_ = false;
//        stopwatch(RESET);
//        env.cached_h_values_ =  VoxelGrid::VoxelGrid<HValueToNeighbours>(Eigen::Isometry3d::Identity(), env.collision_map_grid_.GetResolution(), env.collision_map_grid_.GetNumXCells(), env.collision_map_grid_.GetNumYCells(), env.collision_map_grid_.GetNumZCells(), HValueToNeighbours(Eigen::Vector3d(NAN, NAN, NAN)));
//        for (double x = 0.0; x < env.collision_map_grid_.GetNumXCells(); x += 1.0)
//        {
//            for (double y = 0.0; y < env.collision_map_grid_.GetNumYCells(); y += 1.0)
//            {
//                #pragma omp parallel for
////                for (double z = 0.0; z <= env.collision_map_grid_.GetNumZCells(); z += 1.0)
//                for (int z_int = 0; z_int < env.collision_map_grid_.GetNumZCells(); ++z_int)
//                {
//                    const double z = (double)z_int;
//                    const Eigen::Vector3d center(x, y, z);
//                    const auto h_edge_values = env.generateNeighbourHValues(center);
//                    env.cached_h_values_.SetValue3d(center, h_edge_values);
////                    const auto h_edge_values_retreived = env.cached_h_values_.GetImmutable3d(center).first;
////                    assert((h_edge_values_retreived.center_ - center).norm() < 1e-5);
//                }
//            }
//        }
//        env.use_cached_h_vals_ = true;
//        std::cout << "Calcing H values took " << stopwatch(READ) << " seconds\n";
    }

    return env;
}


void ThreeDimensionalEnvironment::assertIsGridAligned(const ConfigType& p)
{
    assert(std::ceil(p.x()) == p.x());
    assert(std::ceil(p.y()) == p.y());
    assert(std::ceil(p.z()) == p.z());
}

const visualization_msgs::MarkerArray& ThreeDimensionalEnvironment::getCollisionMapMarkers() const
{
    return collision_map_marker_array_;
}

visualization_msgs::Marker ThreeDimensionalEnvironment::getPathMarker(const std::vector<ConfigType>& path, const std::string& ns, const int32_t id) const
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "mocap_world";
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = marker_scale_;
    marker.color = arc_helpers::GenerateUniqueColor<std_msgs::ColorRGBA>(id);

    for (size_t ind = 0; ind < path.size(); ind++)
    {
        marker.points.push_back(EigenHelpersConversions::EigenVector3dToGeometryPoint(Eigen::Vector3d(path[ind].x(), path[ind].y(), path[ind].z())));
    }

    return marker;
}


const ThreeDimensionalEnvironment::ConfigType& ThreeDimensionalEnvironment::getStart() const
{
    return start_;
}

const ThreeDimensionalEnvironment::ConfigType& ThreeDimensionalEnvironment::getGoal() const
{
    return goal_;
}

EigenHelpers::VectorVector3d ThreeDimensionalEnvironment::getNeighbours(const ConfigType& node) const
{
    assertIsGridAligned(node);

    EigenHelpers::VectorVector3d neighbours;

    for (double dx = -1.0; dx <= 1.0; dx += 1.0)
    {
        for (double dy = -1.0; dy <= 1.0; dy += 1.0)
        {
            for (double dz = -1.0; dz <= 1.0; dz += 1.0)
            {
                if (dx != 0.0 || dy != 0.0 || dz != 0)
                {
                    const ConfigType neighbour = node + ConfigType(dx, dy, dz);
                    assertIsGridAligned(neighbour);

                    if (collision_map_grid_.Get(neighbour[0], neighbour[1], neighbour[2]).first.occupancy < 0.5)
                    {
                        neighbours.push_back(neighbour);
                    }
                }
            }
        }
    }

    return neighbours;
}

double ThreeDimensionalEnvironment::distance(const ConfigType& a, const ConfigType& b) const
{
    return (a - b).norm();
}

double ThreeDimensionalEnvironment::heuristicDistance(const ConfigType& node) const
{
    return distance(node, goal_);
}


struct MagneticFieldCalcData
{
    public:
        MagneticFieldCalcData(
                const std::vector<Eigen::Vector3d>& obstacle,
                const Eigen::Vector3d& start,
                const Eigen::Vector3d& end)
            : obstacle_(obstacle)
            , start_(start)
            , end_(end)
        {}

        const std::vector<Eigen::Vector3d>& obstacle_;
        const Eigen::Vector3d& start_;
        const Eigen::Vector3d& end_;
};

static void GSLErrorHandler(const char * reason, const char * file, int line, int gsl_errno)
{
    std::cout << "Reason: " << reason << std::endl;
    std::cout << "File:   " << file << ":" << line << std::endl;
    std::cout << "Errno:  " << gsl_errno << std::endl;
    assert(false);
}

static double MagneticFieldAtPoint(double lambda, void* params)
{
    const MagneticFieldCalcData* const calc_data = (MagneticFieldCalcData *)(params);
    const Eigen::Vector3d r =  Interpolate(calc_data->start_, calc_data->end_, lambda);

    double mag_field_sum = 0;
    for (size_t j = 0; j + 1 < calc_data->obstacle_.size(); j++)
    {
        const size_t j_prime = j + 1;
        const Eigen::Vector3d& s_j = calc_data->obstacle_[j];
        const Eigen::Vector3d& s_j_prime = calc_data->obstacle_[j_prime];
        const Eigen::Vector3d p = s_j - r;
        const Eigen::Vector3d p_prime = s_j_prime - r;
        const Eigen::Vector3d d = (s_j_prime - s_j).cross(p.cross(p_prime)) / (s_j_prime - s_j).squaredNorm();

        if (d.norm() > 1e-10)
        {
            mag_field_sum += ((d.cross(p_prime) / p_prime.norm() - d.cross(p) / p.norm()) / d.squaredNorm())(0);
        }

        if (std::isnan(mag_field_sum) || std::isinf(mag_field_sum))// || calc_data->start_.norm() > 100 || calc_data->end_.norm() < 100)
        {
            std::cout << "lambda: " << lambda << std::endl;
            std::cout << "start: " << calc_data->start_.transpose() << std::endl;
            std::cout << "end:   " << calc_data->end_.transpose() << std::endl;

            std::cout << "r:  " << r.transpose() << std::endl;
            std::cout << "j:  " << j << " s_j  : " << s_j.transpose() << std::endl;
            std::cout << "j': " << j_prime << " s_j' : " << s_j_prime.transpose() << std::endl;
            std::cout << "p:  " << p.transpose() << std::endl;
            std::cout << "p': " << p_prime.transpose() << std::endl;
            std::cout << "d:  " << d.transpose() << std::endl;
            std::cout << std::endl;
            assert(false && "Found NaN or Inf");
        }
    }

    return mag_field_sum / (4.0 * M_PI);
}

static double ComputeLineSegmentHSignature(const std::vector<Eigen::Vector3d>& obstacle, const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    const MagneticFieldCalcData calc_data(obstacle, a, b);

    const auto old_handler = gsl_set_error_handler(&GSLErrorHandler);
    gsl_integration_workspace* gsl_workspace = gsl_integration_workspace_alloc(5000);

    double result, error;
    size_t neval;
    gsl_function F;
    F.function = &MagneticFieldAtPoint;
    F.params = (void *)(const_cast<MagneticFieldCalcData *>(&calc_data));

    const double parameterized_start = 0.0;
    const double parameterized_end = 1.0;
//    gsl_integration_qag(&F, parameterized_start, parameterized_end, 1e-7, 1e-7, 2000, GSL_INTEG_GAUSS61, gsl_workspace, &result, &error);
    gsl_integration_qng(&F, parameterized_start, parameterized_end, 1e-7, 1e-7, &result, &error, &neval);

    gsl_integration_workspace_free(gsl_workspace);
    gsl_set_error_handler(old_handler);

    return result;
}


ThreeDimensionalEnvironment::HValueToNeighbours ThreeDimensionalEnvironment::generateNeighbourHValues(const Eigen::Vector3d& center) const
{
    HValueToNeighbours cache(center);
    const auto neighbours = getNeighbours(center);
    for (const auto& neighbour : neighbours)
    {
        const ThreeDimensionalEnvironment::HSignatureType h_val = getLineSegmentHSignature(center, neighbour);
        cache.setHValue(neighbour, h_val);
    }
    return cache;
}


ThreeDimensionalEnvironment::HSignatureType ThreeDimensionalEnvironment::getLineSegmentHSignature(const ConfigType& a, const ConfigType& b) const
{
    assertIsGridAligned(a);
    assertIsGridAligned(b);

    if (use_cached_h_vals_)
    {
        const auto& cached_vals = cached_h_values_.GetImmutable3d(a);
        if ((cached_vals.first.center_ - a).norm() > 1e-5)
        {
            std::cout << "Cached center:    " << cached_vals.first.center_.transpose() << std::endl;
            std::cout << "Requested center: " << a.transpose() << std::endl;
            assert(false);
        }
        return cached_vals.first.getHValue(b);
    }
    else
    {
        // Section 4.3 - integration along line segement
        HSignatureType h_vals = getZeroHSignature();

        #pragma omp parallel for
        for (size_t ind = 0; ind < obstacles_.size(); ind++)
        {
            h_vals[ind] = ComputeLineSegmentHSignature(obstacles_[ind], a, b);
        }

        return h_vals;
    }
}

ThreeDimensionalEnvironment::HSignatureType ThreeDimensionalEnvironment::getZeroHSignature() const
{
    return HSignatureType::Zero(obstacles_.size());
}

ThreeDimensionalEnvironment::HSignatureType ThreeDimensionalEnvironment::RoundHSignature(HSignatureType h_signature)
{
    for (ssize_t idx = 0; idx < h_signature.size(); ++idx)
    {
//        double rounded = h_signature(idx) * 1e1;
//        rounded = std::round(rounded);
//        rounded /= 1e1;
//        h_signature(idx) = rounded;

        if (h_signature(idx) < 0.0)
        {
            h_signature(idx) = -1.0;
        }
        else if (h_signature(idx) > 0.0)
        {
            h_signature(idx) = 1.0;
        }
    }

    return h_signature;
}

bool ThreeDimensionalEnvironment::hSignatureInBlacklist(const HSignatureType& h_signature) const
{
    if ((h_signature.array().abs() > 1).any())
    {
        return false;
    }

    for (const auto h_sig : blacklisted_h_signatures_)
    {
        if (RoundHSignature(h_sig).isApprox(RoundHSignature(h_signature), 1e-1))
        {
            return true;
        }

        if ((h_sig - h_signature).norm() < 1e-1)
        {
            return true;
        }
    }

    return false;
}

bool ThreeDimensionalEnvironment::hSignatureInWhitelist(const HSignatureType& h_signature) const
{
    if (whitelisted_h_signatures_.size() == 0)
    {
        return true;
    }

    std::cout << "Test sig:\n";
    std::cout << h_signature.transpose() << std::endl;
    std::cout << RoundHSignature(h_signature).transpose() << std::endl;

    for (const auto h_sig : whitelisted_h_signatures_)
    {
        std::cout << "Internal sig:\n";
        std::cout << h_sig.transpose() << std::endl;
        std::cout << RoundHSignature(h_sig).transpose() << std::endl;

        if (RoundHSignature(h_sig).isApprox(RoundHSignature(h_signature), 1e-1))
        {
            return true;
        }

        if ((h_sig - h_signature).norm() < 1e-1)
        {
            return true;
        }
    }
    std::cout << std::endl;

    return false;
}

void ThreeDimensionalEnvironment::appendToBlacklist(const HSignatureType& h_signature)
{
    blacklisted_h_signatures_.push_back(h_signature);
}

void ThreeDimensionalEnvironment::appendToWhitelist(const HSignatureType& h_signature)
{
    whitelisted_h_signatures_.push_back(h_signature);
}

void ThreeDimensionalEnvironment::clearBlacklist()
{
    blacklisted_h_signatures_.clear();
}

void ThreeDimensionalEnvironment::clearWhitelist()
{
    whitelisted_h_signatures_.clear();
}


std::vector<ThreeDimensionalEnvironment::ConfigType> ThreeDimensionalEnvironment::waypointsToPath(const std::vector<ConfigType>& waypoints) const
{
    (void)waypoints;
    std::vector<ConfigType> path;

    assert(false && "waypointsToPath not implemented");

    return path;
}

ThreeDimensionalEnvironment::HSignatureType ThreeDimensionalEnvironment::getPathHSignature(const std::vector<ConfigType>& path) const
{
    assert(false && "This function is broken somehow");
    HSignatureType h_signature = getZeroHSignature();

    for (size_t start_ind = 0; start_ind < path.size() - 1; ++start_ind)
    {
        h_signature += getLineSegmentHSignature(path[start_ind], path[start_ind + 1]);
    }

    return h_signature;
}

