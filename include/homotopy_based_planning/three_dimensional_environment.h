#include <Eigen/Dense>
#include <sdf_tools/collision_map.hpp>
#include <visualization_msgs/MarkerArray.h>

namespace hbp
{
    class ThreeDimensionalEnvironment
    {
        public:
            typedef Eigen::Vector3d ConfigType;
            typedef Eigen::VectorXd HSignatureType;



            static ThreeDimensionalEnvironment CreateBhattacharyaExampleFig17a();

            static void assertIsGridAligned(const ConfigType& p);

            const visualization_msgs::MarkerArray& getCollisionMapMarkers() const;

            visualization_msgs::Marker getPathMarker(const std::vector<ConfigType>& path, const std::string& ns, const int32_t id) const;



            const ConfigType& getStart() const;

            const ConfigType& getGoal() const;

            EigenHelpers::VectorVector3d getNeighbours(const ConfigType& node) const;

            double distance(const ConfigType& a, const ConfigType& b) const;

            double heuristicDistance(const ConfigType& node) const;



            HSignatureType getLineSegmentHSignature(const ConfigType& a, const ConfigType& b) const;

            HSignatureType getZeroHSignature() const;

            static HSignatureType RoundHSignature(HSignatureType h_signature);


            bool hSignatureInBlacklist(const HSignatureType& h_signature) const;

            bool hSignatureInWhitelist(const HSignatureType& h_signature) const;

            void appendToBlacklist(const HSignatureType& h_signature);

            void appendToWhitelist(const HSignatureType& h_signature);

            void clearBlacklist();

            void clearWhitelist();


            std::vector<ConfigType> waypointsToPath(const std::vector<ConfigType>& waypoints) const;

            HSignatureType getPathHSignature(const std::vector<ConfigType>& path) const;

        private:
            sdf_tools::CollisionMapGrid collision_map_grid_;
            visualization_msgs::MarkerArray collision_map_marker_array_;

            std::vector<std::vector<ConfigType>> obstacles_;

            ConfigType start_;
            ConfigType goal_;

            double marker_scale_;
            double start_end_line_length_;

            std::vector<HSignatureType> blacklisted_h_signatures_;
            std::vector<HSignatureType> whitelisted_h_signatures_;
    };
}
