#include "homotopy_based_planning/planar_rectangles_circles_environment.h"

#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/pretty_print.hpp>

using namespace hbp;

static void addCircleToCollisionMapGrid(sdf_tools::CollisionMapGrid& map, const Circle& circle)
{
    const double x_min = circle.center_.x() - circle.radius_;
    const double x_max = circle.center_.x() + circle.radius_;
    const double y_min = circle.center_.y() - circle.radius_;
    const double y_max = circle.center_.y() + circle.radius_;
    const double z = map.GetOriginTransform().translation().x();

    for (double x = x_min; x <= x_max; x += map.GetResolution())
    {
        for (double y = y_min; y <= y_max; y += map.GetResolution())
        {
            const bool in_circle = (Eigen::Vector2d(x, y) - circle.center_).squaredNorm() <= circle.radius_ * circle.radius_;
            if (in_circle)
            {
                map.Set(x, y, z, sdf_tools::COLLISION_CELL(1.0));
            }
        }
    }
}

static void addRectangleToCollisionMapGrid(sdf_tools::CollisionMapGrid& map, const Rectangle& rectangle)
{
    const double z = map.GetOriginTransform().translation().x();

    for (double x = rectangle.bottom_left_corner_.x(); x <= rectangle.upper_right_corner_.x(); x += map.GetResolution())
    {
        for (double y = rectangle.bottom_left_corner_.y(); y <= rectangle.upper_right_corner_.y(); y += map.GetResolution())
        {
            map.Set(x, y, z, sdf_tools::COLLISION_CELL(1.0));
        }
    }
}

static visualization_msgs::Marker makeWaypointMarker(const Eigen::Vector2d& waypoint, const std::string& ns, const int id, const double scale)
{
    const double length = 10;

    visualization_msgs::Marker marker;

    marker.header.frame_id = "mocap_world";
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = scale;
    marker.color = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(1, 0, 0, 1);

    marker.points.push_back(EigenHelpersConversions::EigenVector3dToGeometryPoint(Eigen::Vector3d(waypoint.x() - length, waypoint.y(), -1.0)));
    marker.points.push_back(EigenHelpersConversions::EigenVector3dToGeometryPoint(Eigen::Vector3d(waypoint.x() + length, waypoint.y(), -1.0)));
    marker.points.push_back(EigenHelpersConversions::EigenVector3dToGeometryPoint(Eigen::Vector3d(waypoint.x(), waypoint.y() - length, -1.0)));
    marker.points.push_back(EigenHelpersConversions::EigenVector3dToGeometryPoint(Eigen::Vector3d(waypoint.x(), waypoint.y() + length, -1.0)));

    return marker;
}

static std::complex<double> internalNaturalLogHelper(const std::complex<double>& z_pole, const std::complex<double>& z_a, const std::complex<double>& z_b)
{
    // Calculate some basic differences
    const std::complex<double> z_diff_a = z_a - z_pole;
    const std::complex<double> z_diff_b = z_b - z_pole;

    // Calcuate the real portion of the result
    const double real = std::log(std::abs(z_diff_b)) - std::log(std::abs(z_diff_a));

    // Deal with wraparound on angles
    const double arg = std::arg(z_diff_b) - std::arg(z_diff_a);
    const double arg_minus1 = std::arg(z_diff_b) - std::arg(z_diff_a) - 2.0 * M_PI;
    const double arg_plus1 = std::arg(z_diff_b) - std::arg(z_diff_a) + 2.0 * M_PI;
    const double arg_minus2 = std::arg(z_diff_b) - std::arg(z_diff_a) - 4.0 * M_PI;
    const double arg_plus2 = std::arg(z_diff_b) - std::arg(z_diff_a) + 4.0 * M_PI;
    const double arg_minus3 = std::arg(z_diff_b) - std::arg(z_diff_a) - 6.0 * M_PI;
    const double arg_plus3 = std::arg(z_diff_b) - std::arg(z_diff_a) + 6.0 * M_PI;

    // Do arg absmin over 7 elements
    const double* arg_min = &arg;
    if (std::abs(arg_minus1) < std::abs(*arg_min))
    {
        arg_min = &arg_minus1;
    }
    if (std::abs(arg_plus1) < std::abs(*arg_min))
    {
        arg_min = &arg_plus1;
    }
    if (std::abs(arg_minus2) < std::abs(*arg_min))
    {
        arg_min = &arg_minus2;
    }
    if (std::abs(arg_plus2) < std::abs(*arg_min))
    {
        arg_min = &arg_plus2;
    }
    if (std::abs(arg_minus3) < std::abs(*arg_min))
    {
        arg_min = &arg_minus3;
    }
    if (std::abs(arg_plus3) < std::abs(*arg_min))
    {
        arg_min = &arg_plus3;
    }
    const double imag = *arg_min;

    assert(std::abs(imag) <= M_PI);

    return std::complex<double>(real, imag);
}



// Section 6.1.1
PlanarRectangesCircles PlanarRectangesCircles::CreateBhattacharyaExampleFig12()
{
    PlanarRectangesCircles env;

    const double res = 1.0;
    const double x_size = 1000.0;
    const double y_size = 1000.0;
    const double z_size = 1.0;

    env.marker_scale_ = 3.0;

    auto grid_offset = Eigen::Affine3d::Identity();
    grid_offset(0, 3) = 0.5;
    grid_offset(1, 3) = 0.5;

    env.collision_map_grid_ = sdf_tools::CollisionMapGrid(grid_offset, "mocap_world", res, x_size, y_size, z_size, sdf_tools::COLLISION_CELL(0.0), sdf_tools::COLLISION_CELL(1.0));

    assert(env.collision_map_grid_.GetNumXCells() == (ssize_t)x_size);
    assert(env.collision_map_grid_.GetNumYCells() == (ssize_t)y_size);
    assert(env.collision_map_grid_.GetNumZCells() == 1);

    env.circles_.push_back(Circle(Eigen::Vector2d(750.0, 100.0), 90.0));
    env.circles_.push_back(Circle(Eigen::Vector2d(190.0, 320.0), 160.0));
    env.circles_.push_back(Circle(Eigen::Vector2d(550.0, 460.0), 200.0));
    env.circles_.push_back(Circle(Eigen::Vector2d(850.0, 380.0), 100.0));
    env.circles_.push_back(Circle(Eigen::Vector2d(500.0, 850.0), 140.0));

    env.rectangles_.push_back(Rectangle(Eigen::Vector2d( 10.0,   0.0), Eigen::Vector2d(300.0, 150.0)));
    env.rectangles_.push_back(Rectangle(Eigen::Vector2d(450.0, 100.0), Eigen::Vector2d(620.0, 250.0)));
    env.rectangles_.push_back(Rectangle(Eigen::Vector2d( 20.0, 620.0), Eigen::Vector2d(160.0, 990.0)));
    env.rectangles_.push_back(Rectangle(Eigen::Vector2d(210.0, 660.0), Eigen::Vector2d(320.0, 820.0)));
    env.rectangles_.push_back(Rectangle(Eigen::Vector2d(680.0, 660.0), Eigen::Vector2d(900.0, 850.0)));

//    env.rectangles_.push_back(Rectangle(Eigen::Vector2d(4.0, 2.0), Eigen::Vector2d(6.0, 8.0)));

    for (const auto& circle : env.circles_)
    {
        addCircleToCollisionMapGrid(env.collision_map_grid_, circle);
    }

    for (const auto& rectangle : env.rectangles_)
    {
        addRectangleToCollisionMapGrid(env.collision_map_grid_, rectangle);
    }

    env.start_ = Eigen::Vector2d(20.0, 200.0);
    env.goal_ = Eigen::Vector2d(950.0, 900.0);

//    env.start_ = Eigen::Vector2d(1.0, 1.0);
//    env.goal_ = Eigen::Vector2d(9.0, 7.0);

    assert(env.collision_map_grid_.Get(env.start_.x(), env.start_.y(), 0.0).first.occupancy == 0.0);
    assert(env.collision_map_grid_.Get(env.goal_.x(), env.goal_.y(), 0.0).first.occupancy == 0.0);

    env.collision_map_marker_array_.markers.push_back(env.collision_map_grid_.ExportForDisplay(
                arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(140.0f/255.0f, 153.0f/255.0f, 140.0f/255.0f, 1.0f),
                arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(0.0f, 1.0f, 0.0f, 0.0f),
                arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(0.0f, 0.0f, 1.0f, 0.1f)));

    env.collision_map_marker_array_.markers.push_back(makeWaypointMarker(env.start_, "start", 1, env.marker_scale_));
    env.collision_map_marker_array_.markers.push_back(makeWaypointMarker(env.goal_, "goal", 1, env.marker_scale_));

    return env;
}

// Section 6.1.2
PlanarRectangesCircles PlanarRectangesCircles::CreateBhattacharyaExampleFig13()
{
    PlanarRectangesCircles env;

    const double res = 1.0;
    const double x_size = 50.0;
    const double y_size = 50.0;
    const double z_size = 1.0;

    env.marker_scale_ = 1.0;

    auto grid_offset = Eigen::Affine3d::Identity();
    grid_offset(0, 3) = 0.5;
    grid_offset(1, 3) = 0.5;

    env.collision_map_grid_ = sdf_tools::CollisionMapGrid(grid_offset, "mocap_world", res, x_size, y_size, z_size, sdf_tools::COLLISION_CELL(0.0), sdf_tools::COLLISION_CELL(1.0));

    assert(env.collision_map_grid_.GetNumXCells() == (ssize_t)x_size);
    assert(env.collision_map_grid_.GetNumYCells() == (ssize_t)y_size);
    assert(env.collision_map_grid_.GetNumZCells() == 1);

    env.rectangles_.push_back(Rectangle(Eigen::Vector2d( 4.0,  5.0), Eigen::Vector2d(45.0, 10.0)));
    env.rectangles_.push_back(Rectangle(Eigen::Vector2d( 7.0, 15.0), Eigen::Vector2d(22.0, 20.0)));
    env.rectangles_.push_back(Rectangle(Eigen::Vector2d(25.0, 17.0), Eigen::Vector2d(47.0, 25.0)));
    env.rectangles_.push_back(Rectangle(Eigen::Vector2d( 3.0, 30.0), Eigen::Vector2d(47.0, 38.0)));
    env.rectangles_.push_back(Rectangle(Eigen::Vector2d(30.0, 41.0), Eigen::Vector2d(42.0, 48.0)));

    for (const auto& circle : env.circles_)
    {
        addCircleToCollisionMapGrid(env.collision_map_grid_, circle);
    }

    for (const auto& rectangle : env.rectangles_)
    {
        addRectangleToCollisionMapGrid(env.collision_map_grid_, rectangle);
    }

    env.start_ = Eigen::Vector2d(1.0, 1.0);
    env.goal_ = Eigen::Vector2d(49.0, 49.0);

//    env.start_ = Eigen::Vector2d(1.0, 1.0);
//    env.goal_ = Eigen::Vector2d(9.0, 7.0);

    assert(env.collision_map_grid_.Get(env.start_.x(), env.start_.y(), 0.0).first.occupancy == 0.0);
    assert(env.collision_map_grid_.Get(env.goal_.x(), env.goal_.y(), 0.0).first.occupancy == 0.0);

    env.collision_map_marker_array_.markers.push_back(env.collision_map_grid_.ExportForDisplay(
                arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(140.0f/255.0f, 153.0f/255.0f, 140.0f/255.0f, 1.0f),
                arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(0.0f, 1.0f, 0.0f, 0.0f),
                arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(0.0f, 0.0f, 1.0f, 0.1f)));

    env.collision_map_marker_array_.markers.push_back(makeWaypointMarker(env.start_, "start", 1, env.marker_scale_));
    env.collision_map_marker_array_.markers.push_back(makeWaypointMarker(env.goal_, "goal", 1, env.marker_scale_));

    return env;
}



void PlanarRectangesCircles::assertIsGridAligned(const Eigen::Vector2d& p)
{
    assert(std::ceil(p.x()) == p.x());
    assert(std::ceil(p.y()) == p.y());
}

const visualization_msgs::MarkerArray& PlanarRectangesCircles::getCollisionMapMarkers() const
{
    return collision_map_marker_array_;
}

visualization_msgs::Marker PlanarRectangesCircles::getPathMarker(const std::vector<Eigen::Vector2d>& path, const std::string& ns, const int32_t id) const
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
        marker.points.push_back(EigenHelpersConversions::EigenVector3dToGeometryPoint(Eigen::Vector3d(path[ind].x(), path[ind].y(), 1.0)));
    }

    return marker;
}


const Eigen::Vector2d& PlanarRectangesCircles::getStart() const
{
    return start_;
}

const Eigen::Vector2d& PlanarRectangesCircles::getGoal() const
{
    return goal_;
}

EigenHelpers::VectorVector2d PlanarRectangesCircles::getNeighbours(const Eigen::Vector2d& node) const
{
    assertIsGridAligned(node);

    EigenHelpers::VectorVector2d neighbours;

    for (double dx = -1.0; dx <= 1.0; dx += 1.0)
    {
        for (double dy = -1.0; dy <= 1.0; dy += 1.0)
        {
            if (dx != 0.0 || dy != 0.0)
            {
                const Eigen::Vector2d neighbour = node + Eigen::Vector2d(dx, dy);
                assertIsGridAligned(neighbour);

                if (collision_map_grid_.Get(neighbour[0], neighbour[1], 0.0).first.occupancy < 0.5)
                {
                    neighbours.push_back(neighbour);
                }
            }
        }
    }

    return neighbours;
}

bool PlanarRectangesCircles::goalReached(const Eigen::Vector2d& test_node) const
{
    return (test_node - goal_).norm() < 1e-5;
}

double PlanarRectangesCircles::distance(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const
{
    return (a - b).norm();
}

double PlanarRectangesCircles::heuristicDistance(const Eigen::Vector2d& node) const
{
    const double abs_dx = std::abs(goal_[0] - node[0]);
    const double abs_dy = std::abs(goal_[1] - node[1]);

    const double diagonal_delta = std::min(abs_dx, abs_dy);
    const double aligned_delta = std::max(abs_dx, abs_dy) - diagonal_delta;

    return diagonal_delta * std::sqrt(2.0) + aligned_delta;
}



Eigen::VectorXcd PlanarRectangesCircles::getLineSegmentHSignature(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const
{
//    return getZeroHSignature();

    assertIsGridAligned(a);
    assertIsGridAligned(b);
    assert(b.x() - a.x() == 0 || std::abs(b.x() - a.x()) == collision_map_grid_.GetResolution());
    assert(b.y() - a.y() == 0 || std::abs(b.y() - a.y()) == collision_map_grid_.GetResolution());

    const std::complex<double> z_a(a(0), a(1));
    const std::complex<double> z_b(b(0), b(1));

    // Section 3.3 - approximate integration along line segement
    Eigen::VectorXcd h_vals(circles_.size() + rectangles_.size());

    for (size_t circle_ind = 0; circle_ind < circles_.size(); ++circle_ind)
    {
        const std::complex<double> z_pole(circles_[circle_ind].center_.x(), circles_[circle_ind].center_.y());
        h_vals(circle_ind) = internalNaturalLogHelper(z_pole, z_a, z_b);
    }

    for (size_t rectangle_ind = 0; rectangle_ind < rectangles_.size(); ++rectangle_ind)
    {
        const std::complex<double> z_pole(rectangles_[rectangle_ind].center_.x(), rectangles_[rectangle_ind].center_.y());
        h_vals(circles_.size() + rectangle_ind) = internalNaturalLogHelper(z_pole, z_a, z_b);
    }

    return h_vals;
}

Eigen::VectorXcd PlanarRectangesCircles::getZeroHSignature() const
{
    return Eigen::VectorXcd::Zero(circles_.size() + rectangles_.size());
}

Eigen::VectorXcd PlanarRectangesCircles::RoundHSignature(Eigen::VectorXcd h_signature)
{
    for (ssize_t idx = 0; idx < h_signature.size(); ++idx)
    {
        const std::complex<double>& val = h_signature(idx);

        double real = val.real() * std::pow(10, 10);
        real = std::round(real);
        real /= std::pow(10, 10);

        double imag = val.imag() * std::pow(10, 10);
        imag = std::round(imag);
        imag /= std::pow(10, 10);

        h_signature(idx) = std::complex<double>(real, imag);
    }

    return h_signature;
}
