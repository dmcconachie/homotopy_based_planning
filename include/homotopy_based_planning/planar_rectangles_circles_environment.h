#include <Eigen/Dense>
#include <sdf_tools/collision_map.hpp>
#include <visualization_msgs/MarkerArray.h>

namespace hbp
{
    struct Circle
    {
        public:
            Circle(const Eigen::Vector2d& center, const double radius)
                : center_(center)
                , radius_(radius)
            {}

            const Eigen::Vector2d center_;
            const double radius_;
    };

    struct Rectangle
    {
        public:
            Rectangle(const Eigen::Vector2d& bottom_left_corner, const Eigen::Vector2d& upper_right_corner)
                : bottom_left_corner_(bottom_left_corner)
                , upper_right_corner_(upper_right_corner)
                , center_(bottom_left_corner_ + (upper_right_corner_ - bottom_left_corner_)/2.0)
            {
                assert(bottom_left_corner_.x() <= upper_right_corner_.x());
                assert(bottom_left_corner_.y() <= upper_right_corner_.y());
            }

            const Eigen::Vector2d bottom_left_corner_;
            const Eigen::Vector2d upper_right_corner_;
            const Eigen::Vector2d center_;
    };

    class PlanarRectangesCircles
    {
        public:
            typedef Eigen::Vector2d ConfigType;
            typedef Eigen::VectorXcd HSignatureType;



            static PlanarRectangesCircles CreateBhattacharyaExampleFig12();
            static PlanarRectangesCircles CreateBhattacharyaExampleFig13();
            static PlanarRectangesCircles CreateBhattacharyaExampleFig14();



            static void assertIsGridAligned(const Eigen::Vector2d& p);

            const visualization_msgs::MarkerArray& getCollisionMapMarkers() const;

            visualization_msgs::Marker getPathMarker(const std::vector<Eigen::Vector2d>& path, const std::string& ns, const int32_t id) const;



            const Eigen::Vector2d& getStart() const;

            const Eigen::Vector2d& getGoal() const;

            const Eigen::Vector2d& getStart2ndRobot() const;

            const Eigen::Vector2d& getGoal2ndRobot() const;

            EigenHelpers::VectorVector2d getNeighbours(const Eigen::Vector2d& node) const;

            double distance(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const;

            double heuristicDistance(const Eigen::Vector2d& node) const;



            Eigen::VectorXcd getLineSegmentHSignature(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const;

            Eigen::VectorXcd getZeroHSignature() const;

            static Eigen::VectorXcd RoundHSignature(Eigen::VectorXcd h_signature);


            bool hSignatureInBlacklist(const Eigen::VectorXcd& h_signature) const;

            bool hSignatureInWhitelist(const Eigen::VectorXcd& h_signature) const;

            void appendToBlacklist(const Eigen::VectorXcd& h_signature);

            void appendToWhitelist(const Eigen::VectorXcd& h_signature);

            void clearBlacklist();

            void clearWhitelist();


            std::vector<Eigen::Vector2d> waypointsToPath(const std::vector<Eigen::Vector2d>& waypoints) const;

            Eigen::VectorXcd getPathHSignature(const std::vector<Eigen::Vector2d>& path) const;

        private:
            std::vector<Circle> circles_;
            std::vector<Rectangle> rectangles_;

            sdf_tools::CollisionMapGrid collision_map_grid_;
            visualization_msgs::MarkerArray collision_map_marker_array_;

            Eigen::Vector2d start_;
            Eigen::Vector2d goal_;

            Eigen::Vector2d start_2nd_robot_;
            Eigen::Vector2d goal_2nd_robot_;

            double marker_scale_;
            double start_end_line_length_;

            std::vector<Eigen::VectorXcd> blacklisted_h_signatures_;
            std::vector<Eigen::VectorXcd> whitelisted_h_signatures_;
    };
}
