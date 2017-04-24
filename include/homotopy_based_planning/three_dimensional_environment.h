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
            static ThreeDimensionalEnvironment CreateBhattacharyaExampleFig17b();
            static ThreeDimensionalEnvironment CreateBhattacharyaExampleFig18();

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

            struct HValueToNeighbours
            {
                public:
                    HValueToNeighbours()
                        : center_(NAN, NAN, NAN)
                        , is_initialized_(false)
                    {}

                    HValueToNeighbours(const Eigen::Vector3d& loc)
                        : center_(loc)
//                        , values_(3, std::vector<std::vector<ThreeDimensionalEnvironment::HSignatureType>>(3, std::vector<ThreeDimensionalEnvironment::HSignatureType>(3)))
                        , is_initialized_(true)
                    {}

                    HValueToNeighbours(const HValueToNeighbours& other)
                        : center_(other.center_)
                        , values_(other.values_)
                        , is_initialized_(other.is_initialized_)
                    {}

                    void assertIsNeighbour(const Eigen::Vector3d& delta) const
                    {
                        assert(std::abs(delta.x()) <= 1.0);
                        assert(std::abs(delta.y()) <= 1.0);
                        assert(std::abs(delta.z()) <= 1.0);
                    }

                    ThreeDimensionalEnvironment::HSignatureType getHValue(const Eigen::Vector3d& loc) const
                    {
                        assert(is_initialized_);
                        const Eigen::Vector3d delta = loc - center_;
                        assertIsNeighbour(delta);
                        assert(values_[(size_t)delta.x() + 1][(size_t)delta.y() + 1][(size_t)delta.z() + 1].size() == 4);
                        return values_[(size_t)delta.x() + 1][(size_t)delta.y() + 1][(size_t)delta.z() + 1];
                    }

                    void setHValue(const Eigen::Vector3d& loc, const ThreeDimensionalEnvironment::HSignatureType val)
                    {
                        assert(is_initialized_);
                        const Eigen::Vector3d delta = loc - center_;
                        assertIsNeighbour(delta);
                        assert(delta.squaredNorm() >= 1.0);
                        values_[(size_t)delta.x() + 1][(size_t)delta.y() + 1][(size_t)delta.z() + 1] = val;
                    }

                    Eigen::Vector3d center_;
                    ThreeDimensionalEnvironment::HSignatureType values_[3][3][3];
//                    std::vector<std::vector<std::vector<ThreeDimensionalEnvironment::HSignatureType>>> values_;
                    bool is_initialized_;
            };

            bool use_cached_h_vals_;
            VoxelGrid::VoxelGrid<HValueToNeighbours> cached_h_values_;
            HValueToNeighbours generateNeighbourHValues(const Eigen::Vector3d& center) const;
    };
}
