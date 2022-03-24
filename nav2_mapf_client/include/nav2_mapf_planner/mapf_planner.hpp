
#ifndef NAV2_MAPF_PLANNER__MAPF_PLANNER_HPP_
#define NAV2_MAPF_PLANNER__MAPF_PLANNER_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/executors.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "mapf_actions/action/mapf.hpp"
#include "mapf_actions/srv/mapf.hpp"

namespace nav2_mapf_planner
{

    class MapfPlanner : public nav2_core::GlobalPlanner
    {

    public:
        MapfPlanner();
        ~MapfPlanner() = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

        void cleanup() override;

        void activate() override;

        void deactivate() override;

        nav_msgs::msg::Path createPlan(
            const geometry_msgs::msg::PoseStamped &start,
            const geometry_msgs::msg::PoseStamped &goal) override;

        std::shared_ptr<tf2_ros::Buffer> tf_;
        nav2_util::LifecycleNode::SharedPtr node_;
        nav2_costmap_2d::Costmap2D *costmap_;
        std::string global_frame_, name_;
        std::shared_ptr<rclcpp::Node> node;
        rclcpp::Client<mapf_actions::srv::Mapf>::SharedPtr client_;
        int id;
    };
}
#endif
