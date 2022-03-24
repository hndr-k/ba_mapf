#include "mapf_actions/action/mapf.hpp"
#include "mapf_actions/srv/mapf.hpp"
#include <math.h>

#include "ccbs/cbs.h"
#include "ccbs/map.h"
#include "ccbs/structs.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "structs.h"

//#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace mapf_action_server {

//    class CCBS : public rclcpp::Node
//    class CCBS : public rclcpp_lifecycle::LifecycleNode
class CCBS : public nav2_util::LifecycleNode {

public:
  //        CCBS(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) :
  //        Node("mapf_action_server", options), map_(0.25, 3), cbs_() explicit
  //        CCBS(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
  //        rclcpp_lifecycle::LifecycleNode("mapf_action_server", options),
  //        map_(0.25, 3), cbs_() CCBS() :
  //        nav2_util::LifecycleNode("mapf_action_server", "", true), map_(0.25,
  //        3), cbs_()
  //        {
  //
  ////            occ_grid_sub_ =
  /// this->create_subscription<nav_msgs::msg::OccupancyGrid>("global_costmap/costmap",
  /// 10, std::bind(&CCBS::occ_grid_callback, this, std::placeholders::_1));
  //            service_ =
  //            this->create_service<mapf_actions::srv::Mapf>("/off_field/mapf_plan",
  //            std::bind(&CCBS::path_response, this, std::placeholders::_1,
  //            std::placeholders::_2)); first_grid = true;
  //
  //            config_.agent_size = 0.25;
  //            config_.precision = 0.0000001;
  //            config_.focal_weight = 1.0;
  //            config_.hlh_type = 2;
  //            config_.timelimit = 30;
  //            config_.use_cardinal = true;
  //            config_.use_disjoint_splitting = true;
  //            config_.connectdness = 3;
  //            RCLCPP_INFO(this->get_logger(), "MAPF server initialized");
  //        }
  ////
  //        explicit CCBS(const std:.string & node_name);
  explicit CCBS(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  // CCBS();
  ~CCBS();

  void init();

  void create_agent(geometry_msgs::msg::PoseStamped start,
                    geometry_msgs::msg::PoseStamped goal, int start_id,
                    int goal_id, int robotino_id);

  void update_agent(geometry_msgs::msg::PoseStamped start,
                    geometry_msgs::msg::PoseStamped goal, Agent &agent);

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D *costmap_;

  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;

  void
  path_response(const std::shared_ptr<mapf_actions::srv::Mapf::Request> request,
                std::shared_ptr<mapf_actions::srv::Mapf::Response> response);

  nav_msgs::msg::Path create_plan(geometry_msgs::msg::PoseStamped start,
                                  geometry_msgs::msg::PoseStamped goal);

  void occ_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr grid);

  void paths_from_solution(Solution &solution);

  void control_callback();

  std::vector<std::pair<int, nav_msgs::msg::Path>> current_paths_;

  std::mutex world_state_mutex;

  //        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
  //        occ_grid_sub_; nav_msgs::msg::OccupancyGrid current_grid_;
  rclcpp::Service<mapf_actions::srv::Mapf>::SharedPtr service_;
  geometry_msgs::msg::PoseStamped draw_point(double x_1, double y_1, double x_2,
                                             double y_2, double d,
                                             unsigned int i);
  // variables for ccbs
  Map map_;
  Task task_;
  CBS cbs_;
  Solution solution_;
  bool first_grid;
  bool needs_replan_;

  rclcpp::Clock::SharedPtr clock_;

  Config config_;

  std::vector<Agent> agents;
  // Variables for control loop
  rclcpp::TimerBase::SharedPtr timer_;

  // Create and execute the thread
  std::thread planning_thread; // foo is the function to execute, 10 is the
                               // argument to pass to it

  rclcpp::Rate loop_rate;
  tf2_ros::TransformListener *tfl_;
  tf2_ros::Buffer *tf_;

protected:
  /**
   * @brief Configure member variables and initializes planner
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &state) override;
  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &state) override;
  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &state) override;
  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &state) override;
};

} // namespace mapf_action_server
