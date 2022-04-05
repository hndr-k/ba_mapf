#include "mapf_server/mapf_server.hpp"

namespace mapf_action_server {
//    CCBS::CCBS(const rclcpp::NodeOptions& ) : Node("mapf_action_server"),
//    map_(0.25, 3), cbs_() CCBS::CCBS() :
//    rclcpp_lifecycle::LifecycleNode("mapf_action_server",
//    rclcpp::NodeOptions()), map_(0.25, 3), cbs_()
//
CCBS::CCBS(const rclcpp::NodeOptions &options)
    : nav2_util::LifecycleNode("mapf_action_server", "", true, options),
      map_(0.25, 3), cbs_(), loop_rate(20) {

  //        occ_grid_sub_ =
  //        this->create_subscription<nav_msgs::msg::OccupancyGrid>("global_costmap/costmap",
  //        10, std::bind(&CCBS::occ_grid_callback, this,
  //        std::placeholders::_1));
  service_ = this->create_service<mapf_actions::srv::Mapf>(
      "/off_field/mapf_plan",
      std::bind(&CCBS::path_response, this, std::placeholders::_1,
                std::placeholders::_2));
  first_grid = true;

  config_.agent_size = 0.5;
  config_.precision = 0.0000001;
  config_.focal_weight = 1.0;
  config_.hlh_type = 2;
  config_.timelimit = 30;
  config_.use_cardinal = true;
  config_.use_disjoint_splitting = true;
  config_.connectdness = 3;

  RCLCPP_INFO(this->get_logger(), "MAPF server initialized");
}
CCBS::~CCBS() {}

void CCBS::init() {
  //            occ_grid_sub_ =
  //            this->create_subscription<nav_msgs::msg::OccupancyGrid>("global_costmap/costmap",
  //            10, std::bind(&CCBS::occ_grid_callback, this,
  //            std::placeholders::_1));
  //        service_ =
  //        this->create_service<mapf_actions::srv::Mapf>("/off_field/mapf_plan",
  //        std::bind(&CCBS::path_response, this, std::placeholders::_1,
  //        std::placeholders::_2)); first_grid = true;
  //
  //        config_.agent_size = 0.25;
  //        config_.precision = 0.0000001;
  //        config_.focal_weight = 1.0;
  //        config_.hlh_type = 2;
  //        config_.timelimit = 30;
  //        config_.use_cardinal = true;
  //        config_.use_disjoint_splitting = true;
  //        config_.connectdness = 3;
  //        RCLCPP_INFO(this->get_logger(), "MAPF server initialized");

  //        tf_ = new tf2_ros::Buffer(this->get_clock());
  //        tfl_ = new tf2_ros::TransformListener(*tf_);
  //	costmap_ros_->on_configure(rclcpp_lifecycle::State::);

  //        costmap_ = costmap_ros_->getLayeredCostmap()->getCostmap();

  // Launch a thread to run the costmap node
}

void CCBS::control_callback() {
  //	while (rclcpp::ok())
  //	{
  RCLCPP_INFO(this->get_logger(), "%u", __LINE__);

  if (costmap_ros_ != nullptr && costmap_ != nullptr) {
    //               if (costmap_ros_->get_current_state().id() ==
    //               lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    cbs_ = CBS();
    map_ = Map(0.25, 3);
    if (map_.get_grid(costmap_)) {
      task_.agents = agents;
      task_.make_ids(map_.width);
      solution_ = cbs_.find_solution(map_, task_, config_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Couldn't extract grid from costmap");
    }
    //	       }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Tried to execute the planning callback "
                                     "while the costmap is not ready!");
  }
  //   	loop_rate.sleep();
  //	}
}

nav2_util::CallbackReturn
CCBS::on_configure(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(this->get_logger(), "Configuring: %s", state.label().c_str());
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "mapf_costmap", std::string(get_namespace()), "mapf_costmap");
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
  if (costmap_ros_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "costmap_ros is null!");
  }

  costmap_ros_->on_configure(state);
  costmap_ = costmap_ros_->getLayeredCostmap()->getCostmap();
  RCLCPP_INFO(this->get_logger(), "Done configuring!");

  //      planning_thread = std::thread(&CCBS::control_callback, this);
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CCBS::on_activate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Activating");

  //      plan_publisher_->on_activate();
  //      action_server_pose_->activate();
  //      action_server_poses_->activate();
  costmap_ros_->on_activate(state);
  //      std::chrono::duration<double> period;
  //      period = std::chrono::milliseconds(100);
  //      timer_ = this->create_wall_timer(period,
  //      std::bind(&mapf_action_server::CCBS::control_callback, this));
  //      planning_thread.detach();
  RCLCPP_INFO(this->get_logger(), "Done activating!");
  //
  //      PlannerMap::iterator it;
  //      for (it = planners_.begin(); it != planners_.end(); ++it) {
  //        it->second->activate();
  //      }

  // create bond connection
  createBond();

  map_ = Map(0.25, 3);
    if (map_.get_grid(costmap_)) {
    std::cout << "map created" << std::endl;
    }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CCBS::on_deactivate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  //      action_server_pose_->deactivate();
  //      action_server_poses_->deactivate();
  //      plan_publisher_->on_deactivate();
  costmap_ros_->on_deactivate(state);
  //
  //      PlannerMap::iterator it;
  //      for (it = planners_.begin(); it != planners_.end(); ++it) {
  //        it->second->deactivate();
  //      }
  //
  //      // destroy bond connection
  //      destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CCBS::on_cleanup(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  //      action_server_pose_.reset();
  //      action_server_poses_.reset();
  //      plan_publisher_.reset();
  //      tf_.reset();
  costmap_ros_->on_cleanup(state);
  costmap_ = nullptr;
  //
  //      PlannerMap::iterator it;
  //      for (it = planners_.begin(); it != planners_.end(); ++it) {
  //        it->second->cleanup();
  //      }
  //      planners_.clear();
  //      costmap_ = nullptr;

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CCBS::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

//    void CCBS::occ_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr
//    grid)
//    {
//
//        nav_msgs::msg::OccupancyGrid new_grid_;
//        new_grid_.data = grid->data;
//        new_grid_.header = grid->header;
//        new_grid_.info = grid->info;
//        if (!first_grid)
//        {
//            current_grid_ = new_grid_;
//
//            first_grid = false;
////            nav2_costmap_2d::Costmap2D costmap_new_(new_grid_);
//
//            RCLCPP_INFO(this->get_logger(), "%.5f",
//            costmap_new_.getOriginX()); if (!map_.get_grid(new_grid_)) {
//                RCLCPP_INFO(this->get_logger(), "Couldn't extract grid from
//                costmap");
//            }
//
//            costmap_ = costmap_new_;
//        }
//        /*else if (new_grid_ != current_grid_)
//            {
//                map_.get_grid(new_grid_);
//                nav2_costmap_2d::Costmap2D costmap_new_(new_grid_);
//                if(!map_.get_grid(new_grid_))
//            {
//                RCLCPP_INFO(this->get_logger(), "Couldn't extract grid from
//                costmap");
//            }
//                costmap_ = costmap_new_;
//            }*/
//    }

void CCBS::create_agent(geometry_msgs::msg::PoseStamped start,
                        geometry_msgs::msg::PoseStamped goal, int start_id,
                        int goal_id, int robotino_id) {
  /*int resolution = current_grid_.info.resolution;*/
  /*int pos_i = (start.pose.position.x - current_grid_.info.origin.position.x )
  / resolution; int pos_j = (start.pose.position.y -
  current_grid_.info.origin.position.y ) / resolution;

  int goal_i = (goal.pose.position.x - current_grid_.info.origin.position.x ) /
  resolution; int goal_j = (goal.pose.position.y -
  current_grid_.info.origin.position.y ) / resolution;*/
  const std::lock_guard<std::mutex> lock(world_state_mutex);
  uint start_x, start_y, goal_x, goal_y;
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x,
                       start_y);
  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x,
                       goal_y);
  // TODO dynamic parametrization
  Agent agent_ = Agent(0, 0, 0);
  agent_.goal_i = goal_x;
  agent_.goal_j = goal_y;
  agent_.start_i = start_x;
  agent_.start_j = start_y;
  agent_.id = robotino_id;
  agents.push_back(agent_);
  needs_replan_ = true;
}

void CCBS::update_agent(geometry_msgs::msg::PoseStamped start,
                        geometry_msgs::msg::PoseStamped goal, Agent &agent)

{
  const std::lock_guard<std::mutex> lock(world_state_mutex);
  uint start_x, start_y, goal_x, goal_y;
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x,
                       start_y);
  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x,
                       goal_y);

  agent.goal_i = goal_x;
  agent.goal_j = goal_y;
  agent.start_i = start_x;
  agent.start_j = start_y;
  std::cout << "update: " << agent.start_i << " " << agent.start_j << "  "
            << agent.goal_i << " " << agent.goal_j << "\n";
  needs_replan_ = true;
}

void CCBS::path_response(
    const std::shared_ptr<mapf_actions::srv::Mapf::Request> request,
    std::shared_ptr<mapf_actions::srv::Mapf::Response> response) {
  bool agent_missing = true;
  RCLCPP_INFO(this->get_logger(), "New request: %f %f -> %f %f!",
              request->start.pose.position.x, request->start.pose.position.y,
              request->goal.pose.position.x, request->goal.pose.position.y);
  for (auto &agent : agents) {
    if (agent.id == request->robotino_id) {
      RCLCPP_INFO(this->get_logger(), "Agent id %i update!", agent.id);
      update_agent(request->start, request->goal, agent);
      agent_missing = false;
    }
  }
  if (agent_missing) {
  //  RCLCPP_INFO(this->get_logger(), "Creating agent.");
    create_agent(request->start, request->goal, 0, 0, request->robotino_id);
  //  RCLCPP_INFO(this->get_logger(), "Agent created!");
  }

  nav_msgs::msg::Path path_;
  path_.header.frame_id = "map";
  path_.header.stamp = this->get_clock().get()->now();
  control_callback();

  //	while  (needs_replan_ == true) {
  //            loop_rate.sleep();
  //	    rclcpp::spin_some(this->get_node_base_interface());
  //
  //            RCLCPP_INFO(this->get_logger(), "Wait for path!");
  //	}
  for (auto &path : solution_.paths) {
    if (path.agentID == request->robotino_id) {
      for (auto &node : path.nodes) {
        geometry_msgs::msg::PoseStamped pose_;
        double x, y;
        costmap_->mapToWorld(map_.get_i(node.id), map_.get_j(node.id), x, y);
        pose_.pose.position.x = x;
        pose_.pose.position.y = y;
        pose_.pose.position.z = 0.0;
        pose_.pose.orientation.x = 0.0;
        pose_.pose.orientation.y = 0.0;
        pose_.pose.orientation.z = 0.0;
        pose_.pose.orientation.z = 1.0;
        pose_.header.frame_id = "map";
        pose_.header.stamp = this->get_clock().get()->now();
        path_.poses.push_back(pose_);
 //       std::cout << "x: " << x << " y: " << y << std::endl;
        ;
      }
      double x_last = path_.poses[path_.poses.size() - 1].pose.position.x;
      double y_last = path_.poses[path_.poses.size() - 1].pose.position.y;
      geometry_msgs::msg::PoseStamped pose_;
      for (int i = 1; i < 11; i++) {
        pose_ = draw_point(x_last, y_last, request->goal.pose.position.x,
                           request->goal.pose.position.y, 0.1, i);
        path_.poses.push_back(pose_);
      }

      nav_msgs::msg::Path path_res;
      if(path_.poses.size() < 10)
      {
      for(int i = 0; i < path_.poses.size(); i++)
      {
        geometry_msgs::msg::PoseStamped pose_res;
        pose_res = path_.poses[i];
        path_res.poses.push_back(pose_res);
      }
      pose_.pose = request->goal.pose;
      pose_.header.frame_id = "map";
      pose_.header.stamp = this->get_clock().get()->now();
      path_res.poses.push_back(pose_);
      }
      else{
      for(int i = 0; i < 10; i++)
      {
        geometry_msgs::msg::PoseStamped pose_res;
        pose_res = path_.poses[i];
        path_res.poses.push_back(pose_res);
      }
      pose_.pose = request->goal.pose;
      pose_.header.frame_id = "map";
      pose_.header.stamp = this->get_clock().get()->now();
      path_res.poses.push_back(pose_);
      }

//      RCLCPP_INFO(this->get_logger(), "%d size of path", path_.poses.size());
      path_res.header.frame_id = "map";
      path_res.header.stamp = this->get_clock().get()->now();
      response->path = path_res;
      RCLCPP_INFO(this->get_logger(), "%d size of path",
                  response->path.poses.size());
    }
  }
  RCLCPP_INFO(this->get_logger(), "Passing response!");
}
geometry_msgs::msg::PoseStamped CCBS::draw_point(double x_1, double y_1,
                                                 double x_2, double y_2,
                                                 double d, unsigned int i) {
  double dist_ = sqrt(pow(x_2 - x_1, 2.0) + pow(y_2 - y_1, 2.0));
  double angle_ = atan2(y_2 - y_1, x_2 - x_1);
  geometry_msgs::msg::PoseStamped pose_;
  pose_.pose.position.x = x_1 + (dist_ * i * d) * cos(angle_);
  pose_.pose.position.y = y_1 + (dist_ * i * d) * sin(angle_);
  pose_.pose.position.z = 0.0;
  pose_.pose.orientation.x = 0.0;
  pose_.pose.orientation.y = 0.0;
  pose_.pose.orientation.z = 0.0;
  pose_.pose.orientation.z = 1.0;
  pose_.header.frame_id = "map";
  pose_.header.stamp = this->get_clock().get()->now();

  return pose_;
}

} // namespace mapf_action_server

RCLCPP_COMPONENTS_REGISTER_NODE(mapf_action_server::CCBS)
