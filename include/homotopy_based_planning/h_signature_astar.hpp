#ifndef H_SIGNATURE_ASTAR_HPP
#define H_SIGNATURE_ASTAR_HPP

#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <chrono>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/arc_helpers.hpp>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace hbp
{
    enum StopwatchControl {RESET, READ};

    inline double stopwatch(const StopwatchControl control = READ)
    {
        static std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

        const std::chrono::high_resolution_clock::time_point end_time = std::chrono::high_resolution_clock::now();
        if (control == RESET)
        {
            start_time = end_time;
        }

        return std::chrono::duration<double>(end_time - start_time).count();
    }

    template<class T, class Container = std::vector<T>, class Compare = std::less<typename Container::value_type>>
    std::string PrintPriorityQueue(std::priority_queue<T, Container, Compare> queue)
    {
        std::stringstream ss;
        while (queue.size() > 0)
        {
            const T current = queue.top();
            queue.pop();
            ss << PrettyPrint::PrettyPrint(current) << std::endl;
        }
        return ss.str();
    }

    template <typename GraphType>
    class HSignatureAStar
    {
        public:
            typedef typename GraphType::ConfigType BasicConfigType;
            typedef typename GraphType::HSignatureType HSignatureType;
            struct AugmentedConfigType
            {
                public:
                    AugmentedConfigType()
                        : basic_config_(NAN * BasicConfigType())
                    {}

                    AugmentedConfigType(const BasicConfigType& basic_config,
                                        const HSignatureType& h_signature,
                                        const HSignatureType& h_signature_rounded)
                        : basic_config_(basic_config)
                        , h_signature_(h_signature)
                        , h_signature_rounded_(h_signature_rounded)
                    {}

                    AugmentedConfigType(BasicConfigType&& basic_config,
                                        HSignatureType&& h_signature,
                                        HSignatureType&& h_signature_rounded)
                        : basic_config_(basic_config)
                        , h_signature_(h_signature)
                        , h_signature_rounded_(h_signature_rounded)
                    {}

                    AugmentedConfigType(const AugmentedConfigType& other)
                        : basic_config_(other.basic_config_)
                        , h_signature_(other.h_signature_)
                        , h_signature_rounded_(other.h_signature_rounded_)
                    {}

                    BasicConfigType basic_config_;
                    HSignatureType h_signature_;
                    HSignatureType h_signature_rounded_;

                    bool operator==(const AugmentedConfigType& other) const
                    {
                        assert(GraphType::RoundHSignature(h_signature_) == h_signature_rounded_);
                        assert(GraphType::RoundHSignature(other.h_signature_) == other.h_signature_rounded_);
                        return (basic_config_ == other.basic_config_ && h_signature_rounded_ == other.h_signature_rounded_);
                    }

                    AugmentedConfigType& operator=(const AugmentedConfigType& other)
                    {
                        assert(GraphType::RoundHSignature(other.h_signature_) == other.h_signature_rounded_);

                        basic_config_ = other.basic_config_;
                        h_signature_ = other.h_signature_;
                        h_signature_rounded_ = other.h_signature_rounded_;

                        return *this;
                    }

                    AugmentedConfigType& operator=(AugmentedConfigType&& other)
                    {
                        assert(GraphType::RoundHSignature(other.h_signature_) == other.h_signature_rounded_);

                        basic_config_ = other.basic_config_;
                        h_signature_ = other.h_signature_;
                        h_signature_rounded_ = other.h_signature_rounded_;

                        return *this;
                    }

                    friend std::ostream& operator<<(std::ostream& os, const AugmentedConfigType& config)
                    {
                        Eigen::IOFormat one_line(Eigen::FullPrecision, 0, " ", " ");
                        os << PrettyPrint::PrettyPrint(config.basic_config_) << "\nHSignature:\n" << PrettyPrint::PrettyPrint(config.h_signature_.format(one_line));
                        return os;
                    }
            };
            struct AugmentedConfigHasher
            {
                    std::size_t operator()(const AugmentedConfigType& config) const
                    {
                        std::size_t seed = 0;
                        std::hash_combine(seed, config.basic_config_);
                        std::hash_combine(seed, config.h_signature_rounded_);
                        return seed;
                    }
            };
            struct AugmentedConfigComparator
            {
                    bool operator()(const AugmentedConfigType& c1, const AugmentedConfigType& c2) const
                    {
                        for (ssize_t ind = 0; ind < c1.basic_config_.size(); ++ind)
                        {
                            if (c1.basic_config_[ind] < c2.basic_config_[ind])
                            {
                                return true;
                            }
                            if (c1.basic_config_[ind] > c2.basic_config_[ind])
                            {
                                return false;
                            }
                        }

                        for (ssize_t ind = 0; ind < c1.h_signature_rounded_.size(); ++ind)
                        {
                            if (c1.h_signature_rounded_[ind].real() < c2.h_signature_rounded_[ind].real())
                            {
                                return true;
                            }
                            if (c1.h_signature_rounded_[ind].real() > c2.h_signature_rounded_[ind].real())
                            {
                                return false;
                            }

                            if (c1.h_signature_rounded_[ind].imag() < c2.h_signature_rounded_[ind].imag())
                            {
                                return true;
                            }
                            if (c1.h_signature_rounded_[ind].imag() > c2.h_signature_rounded_[ind].imag())
                            {
                                return false;
                            }
                        }

                        return false;
                    }
            };

            struct PlanResults
            {
                    std::vector<std::vector<BasicConfigType>> paths_;
                    std::vector<HSignatureType> hsignatures_;
                    std::vector<double> time_;
                    std::vector<size_t> num_states_explored_;

                    friend std::ostream& operator<<(std::ostream& os, const PlanResults& results)
                    {
                        assert(results.paths_.size() == results.hsignatures_.size());
                        assert(results.paths_.size() == results.time_.size());
                        assert(results.paths_.size() == results.num_states_explored_.size());

                        os << "Number of paths found: " << results.paths_.size() << std::endl;
                        os << "Time:            " << PrettyPrint::PrettyPrint(results.time_, true, " ") << std::endl;
                        os << "States explored: " << PrettyPrint::PrettyPrint(results.num_states_explored_, true, " ") << std::endl;
                        return os;
                    }
            };

            typedef std::pair<AugmentedConfigType, double> AugmentedConfigAndDistType;
            struct AStarComparator
            {
                public:
                    AStarComparator(const GraphType& graph)
                        : graph_(graph)
                    {}

                    // Defines a "less" operation"; by using "greater" then the smallest element will appear at the top of the priority queue
                    bool operator()(const AugmentedConfigAndDistType& c1, const AugmentedConfigAndDistType& c2) const
                    {
                        // If both expected distances are the same, then we want to explore the one that has the smaller heuristic distance
                        if (std::abs(c1.second - c2.second) < 1e-10)
                        {
                            const double hdist_c1 = graph_.heuristicDistance(c1.first.basic_config_);
                            const double hdist_c2 = graph_.heuristicDistance(c2.first.basic_config_);
                            return (hdist_c1 > hdist_c2);
                        }
                        // If expected distances are different, we want to explore the one with the smaller expected distance
                        else
                        {
                            return (c1.second > c2.second);
                        }
                    }

                    const GraphType& graph_;
            };

            static PlanResults Plan(GraphType& graph, const BasicConfigType& start, const BasicConfigType& goal, const size_t num_paths, ros::Publisher& vis_pub, const bool visualization_enabled)
            {
                stopwatch(RESET);

                PlanResults results;

                visualization_msgs::Marker marker;
                marker.header.frame_id = "mocap_world";
                marker.type = visualization_msgs::Marker::POINTS;
                marker.action = visualization_msgs::Marker::ADD;
                marker.ns = "explored_states";
                marker.id = 1;
                marker.scale.x = 1.0;
                marker.scale.y = 1.0;
                marker.header.stamp = ros::Time::now();
                vis_pub.publish(marker);
                marker.color = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(0.0, 0.0, 1.0, 1.0);

                const AStarComparator astar_compare(graph);
                std::priority_queue<AugmentedConfigAndDistType, std::vector<AugmentedConfigAndDistType>, AStarComparator> frontier(astar_compare);
                std::unordered_set<AugmentedConfigType, AugmentedConfigHasher> explored;
                std::unordered_map<AugmentedConfigType, double, AugmentedConfigHasher> cost_to_come;
                std::unordered_map<AugmentedConfigType, AugmentedConfigType, AugmentedConfigHasher> backpointers;

                const AugmentedConfigType augmented_start(start, graph.getZeroHSignature(), graph.getZeroHSignature());
                frontier.push(AugmentedConfigAndDistType(augmented_start, graph.heuristicDistance(graph.getStart())));
                cost_to_come[augmented_start] = 0.0;

                bool done_planning = false;
                std::cout << "Entering explore loop\n\n";
                while (!done_planning && ros::ok() && frontier.size() > 0)
                {
                    const AugmentedConfigAndDistType current = frontier.top();
                    frontier.pop();
                    const AugmentedConfigType& current_augmented_node = current.first;
                    const BasicConfigType& current_basic_node = current_augmented_node.basic_config_;
                    const HSignatureType& current_hsignature = current_augmented_node.h_signature_;

                    // Visualization code
                    if (visualization_enabled)
                    {
                        geometry_msgs::Point p;
                        p.x = current_basic_node[0];
                        p.y = current_basic_node[1];
                        if (current_basic_node.size() >= 3)
                        {
                            p.z = current_basic_node[2];
                        }
                        ++marker.id;
                        marker.points.push_back(p);

                        if (marker.id % 1000 == 0)
                        {
                            marker.header.stamp = ros::Time::now();
                            vis_pub.publish(marker);
                            marker.points.clear();
                            usleep(10);
                        }
                    }

                    if (current_basic_node.isApprox(goal, 1e-10) && !graph.hSignatureInBlacklist(current_hsignature) && graph.hSignatureInWhitelist(current_hsignature))
                    {
                        if (visualization_enabled)
                        {
                            std::cout << "Reached goal!\n";
                            std::cout << PrettyPrint::PrettyPrint(current_augmented_node, true, " ") << std::endl << std::flush;
                            std::cout << std::endl;
                            marker.header.stamp = ros::Time::now();
                            vis_pub.publish(marker);
                        }

                        graph.appendToBlacklist(current_hsignature);

                        results.paths_.push_back(ExtractPathBasic(backpointers, current_augmented_node));
                        results.hsignatures_.push_back(current_hsignature);
                        results.num_states_explored_.push_back(explored.size());
                        results.time_.push_back(stopwatch(READ));
                        std::cout << results << std::endl;
                        vis_pub.publish(graph.getPathMarker(results.paths_.back(), "paths", (int32_t)results.paths_.size()));

                        if (results.paths_.size() == num_paths)
                        {
                            done_planning = true;
                        }
                    }
                    // Double check if we've already explored this node:
                    //    a single node can be inserted into the frontier multiple times at the same or different priorities
                    //    so we want to avoid the expense of re-exploring it, and just discard this one once we pop it
                    else if (explored.find(current_augmented_node) == explored.end())
                    {
//                        ++num_explored;
                        explored.insert(current_augmented_node);
                        const double current_cost_to_come = cost_to_come.at(current_augmented_node);

                        // Expand the node to find all neighbours, adding them to the frontier if we have not already explored them
                        const auto basic_neighbours = graph.getNeighbours(current_basic_node);
                        for (const auto basic_neighbour : basic_neighbours)
                        {
                            const HSignatureType h_signature_delta = graph.getLineSegmentHSignature(current_basic_node, basic_neighbour);
                            const AugmentedConfigType augmented_neighbour(basic_neighbour, current_hsignature + h_signature_delta, GraphType::RoundHSignature(current_hsignature + h_signature_delta));

                            // Check if we've already explored this neighbour
                            if (explored.find(augmented_neighbour) != explored.end())
                            {
                                continue;
                            }

                            // Do some sanity checks so that we can make assumptions later
                            const auto neighbour_cost_to_come_ittr = cost_to_come.find(augmented_neighbour);
                            const auto neighbour_backpointer_ittr = backpointers.find(augmented_neighbour);
                            if (neighbour_cost_to_come_ittr == cost_to_come.end())
                            {
                                assert(neighbour_backpointer_ittr == backpointers.end());
                            }
                            if (neighbour_backpointer_ittr == backpointers.end())
                            {
                                assert(neighbour_cost_to_come_ittr == cost_to_come.end());
                            }

                            // If we haven't already explored this neighbour, see if we've found a cheaper path
                            const double neighbour_new_cost_to_come = current_cost_to_come + graph.distance(current_basic_node, basic_neighbour);
                            if (neighbour_cost_to_come_ittr != cost_to_come.end() && neighbour_cost_to_come_ittr->second <= neighbour_new_cost_to_come)
                            {
                                continue;
                            }

                            frontier.push(AugmentedConfigAndDistType(augmented_neighbour, neighbour_new_cost_to_come + graph.heuristicDistance(basic_neighbour)));
                            cost_to_come[augmented_neighbour] = neighbour_new_cost_to_come;
                            backpointers[augmented_neighbour] = current_augmented_node;
                        }
                    }
                    else
                    {
//                        std::cout << "Already explored this node, skipping\n";
                    }
                }

                if (visualization_enabled)
                {
                    marker.header.stamp = ros::Time::now();
                    vis_pub.publish(marker);
                }

                return results;
            }

        private:
            HSignatureAStar() {}

            template<typename MapType>
            static std::vector<AugmentedConfigType> ExtractPathAugmented(const MapType& backpointers, AugmentedConfigType last_state)
            {
                std::vector<AugmentedConfigType> path;
                for (auto backpointer_ittr = backpointers.find(last_state); backpointer_ittr != backpointers.end(); backpointer_ittr = backpointers.find(last_state))
                {
                    path.push_back(last_state);
                    last_state = backpointer_ittr->second;
                }
                path.push_back(last_state);
                return path;
            }

            template<typename MapType>
            static std::vector<BasicConfigType> ExtractPathBasic(const MapType& backpointers, AugmentedConfigType last_state)
            {
                std::vector<BasicConfigType> path;
                for (auto backpointer_ittr = backpointers.find(last_state); backpointer_ittr != backpointers.end(); backpointer_ittr = backpointers.find(last_state))
                {
                    path.push_back(last_state.basic_config_);
                    last_state = backpointer_ittr->second;
                }
                path.push_back(last_state.basic_config_);
                return path;
            }
    };
}

#endif // H_SIGNATURE_ASTAR_HPP
