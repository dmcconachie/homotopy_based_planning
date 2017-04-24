#include "homotopy_based_planning/three_dimensional_environment.h"

#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <gsl/gsl_integration.h>
#include <gsl/gsl_errno.h>

#include <omp.h>

using namespace hbp;
using namespace EigenHelpers;
using namespace EigenHelpersConversions;

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

static void addLineStripToCollisionMapGrid(sdf_tools::CollisionMapGrid& map, const std::vector<Eigen::Vector3d>& line_strip, const double radius)
{
    std::cout << "Line strip size: " << line_strip.size() << std::endl;

    for (size_t ind = 0; ind + 1 < line_strip.size(); ++ind)
    {
        std::cout << "adding linestrip\n";
        addLineSegmentToCollisionMapGrid(map, line_strip[ind], line_strip[ind + 1], radius);
    }
}

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

        auto grid_offset = Eigen::Affine3d::Identity();
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
        addLineStripToCollisionMapGrid(env.collision_map_grid_, obstacle, 1.0);

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
        addLineStripToCollisionMapGrid(env.collision_map_grid_, obstacle, 1.0);

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
}

static double MagneticFieldAtPoint(double lambda, void* params)
{
    const MagneticFieldCalcData* const calc_data = reinterpret_cast<MagneticFieldCalcData*>(params);

    const Eigen::Vector3d r = lambda * calc_data->start_ + (1.0 - lambda) * calc_data->end_;
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

//        if (std::isnan(mag_field_sum) || std::isinf(mag_field_sum))
//        {
//            std::cout << "r:  " << r.transpose() << std::endl;
//            std::cout << "j:  " << j << "s_j  : " << s_j.transpose() << std::endl;
//            std::cout << "j': " << j_prime << "s_j' : " << s_j_prime.transpose() << std::endl;
//            std::cout << "p:  " << p.transpose() << std::endl;
//            std::cout << "p': " << p_prime.transpose() << std::endl;
//            std::cout << "d:  " << d.transpose() << std::endl;
//            std::cout << std::endl;
//            assert(false && "Found NaN or Inf");
//        }

//        std::cout << mag_field_sum << std::endl;
    }

    return mag_field_sum / (4.0 * M_PI);
}

static double ComputeLineSegmentHSignature(const std::vector<Eigen::Vector3d>& obstacle, const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    const auto old_handler = gsl_set_error_handler(&GSLErrorHandler);
    gsl_integration_workspace* gsl_workspace = gsl_integration_workspace_alloc(5000);

    double result, error;

    const MagneticFieldCalcData calc_data(obstacle, a, b);

    gsl_function F;
    F.function = &MagneticFieldAtPoint;
    F.params = reinterpret_cast<void *>(const_cast<MagneticFieldCalcData *>(&calc_data));

    const double start = 0.0;
    const double end = 1.0;
    gsl_integration_qag(&F, start, end, 1e-10, 0, 2000, GSL_INTEG_GAUSS15, gsl_workspace, &result, &error);

    gsl_integration_workspace_free(gsl_workspace);
    gsl_set_error_handler(old_handler);

    return result;
}

ThreeDimensionalEnvironment::HSignatureType ThreeDimensionalEnvironment::getLineSegmentHSignature(const ConfigType& a, const ConfigType& b) const
{
    assertIsGridAligned(a);
    assertIsGridAligned(b);

    // Section 4.3 - integration along line segement
    HSignatureType h_vals = getZeroHSignature();

    for (size_t ind = 0; ind < obstacles_.size(); ind++)
    {
        h_vals[ind] = ComputeLineSegmentHSignature(obstacles_[ind], a, b);
    }

    return h_vals;
}

ThreeDimensionalEnvironment::HSignatureType ThreeDimensionalEnvironment::getZeroHSignature() const
{
    return HSignatureType::Zero(obstacles_.size());
}

ThreeDimensionalEnvironment::HSignatureType ThreeDimensionalEnvironment::RoundHSignature(HSignatureType h_signature)
{
    for (ssize_t idx = 0; idx < h_signature.size(); ++idx)
    {
        double rounded = h_signature(idx) * 1e1;
        rounded = std::round(rounded);
        rounded /= 1e1;

//        h_signature(idx) = h_signature(idx) < 0 ? -1 : 1;
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
        if (h_sig.isApprox(h_signature, 1e-1))
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

    for (const auto h_sig : whitelisted_h_signatures_)
    {
        if (h_sig.isApprox(h_signature, 1e-1))
        {
            return true;
        }
    }

    return false;
}

void ThreeDimensionalEnvironment::appendToBlacklist(const HSignatureType& h_signature)
{
    blacklisted_h_signatures_.push_back(h_signature);

    std::cout << PrettyPrint::PrettyPrint(blacklisted_h_signatures_, true, "\n") << std::endl;
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

