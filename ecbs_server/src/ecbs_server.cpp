#include <cstdio>

#include "ecbs_server/ecbs_server.hpp"


#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <ecbs_server/ecbs.hpp>
#include <ecbs_server/timer.hpp>

namespace ecbs_server {
//    ecbs_server::ecbs_server(const rclcpp::NodeOptions& ) : Node("mapf_action_server"),
//    map_(0.25, 3), cbs_() ecbs_server::ecbs_server() :
//    rclcpp_lifecycle::LifecycleNode("mapf_action_server",
//    rclcpp::NodeOptions()), map_(0.25, 3), cbs_()
//
ecbs_server::ecbs_server(const rclcpp::NodeOptions &options)
    : nav2_util::LifecycleNode("mapf_action_server", "", true, options), loop_rate(20), desc("Allowed options!") {

  //        occ_grid_sub_ =
  //        this->create_subscription<nav_msgs::msg::OccupancyGrid>("global_costmap/costmap",
  //        10, std::bind(&ecbs_server::occ_grid_callback, this,
  //        std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Constructing!");
  service_ = this->create_service<mapf_actions::srv::Mapf>(
      "/off_field/mapf_plan",
      std::bind(&ecbs_server::path_response, this, std::placeholders::_1,
                std::placeholders::_2));
  

    RCLCPP_INFO(this->get_logger(), "Init!");
  

 RCLCPP_INFO(this->get_logger(), "Parameters done!");

  RCLCPP_INFO(this->get_logger(), "MAPF server initialized");
}
ecbs_server::~ecbs_server() {}

void ecbs_server::init() {}

void ecbs_server::control_callback() {
  //	while (rclcpp::ok())
  //	{
  

  if (costmap_ros_ != nullptr && costmap_ != nullptr) {

    std::vector<Location> goals;
    std::vector<State> startStates;
    std::unordered_set<State> startStatesSet;

    for (const auto& agent : agents) {
    startStates.emplace_back(State(0, agent.start_x,agent.start_y));
    // std::cout << "s: " << startStates.back() << std::endl;
    goals.emplace_back(Location(agent.goal_x, agent.goal_y));
    }
    for (const auto& s : startStates) {
    startStatesSet.insert(s);
    }
    bool disappearAtGoal = false;
    double w = 1.3;
    Environment mapf(dimx, dimy, obstacles, goals, disappearAtGoal);
    ECBS<State, Action, int, Conflict, Constraints, Environment> ecbs(mapf, w);
    std::vector<PlanResult<State, Action, int> > solution;

    Timer timer;
    bool success = ecbs.search(startStates, solution);
    timer.stop();
    if (success) {
    std::cout << "Planning successful! " << std::endl;
    int cost = 0;
    int makespan = 0;
    for (const auto& s : solution) {
      cost += s.cost;
      makespan = std::max<int>(makespan, s.cost);
    }

    /*std::cout << "statistics:" << std::endl;
    std::cout << "  cost: " << cost << std::endl;
    std::cout << "  makespan: " << makespan << std::endl;
    std::cout << "  runtime: " << timer.elapsedSeconds() << std::endl;
    std::cout << "  highLevelExpanded: " << mapf.highLevelExpanded() << std::endl;
    std::cout << "  lowLevelExpanded: " << mapf.lowLevelExpanded() << std::endl;
    std::cout << "schedule:" << std::endl;*/
    paths.resize(solution.size());
    std::cout << "SIZE " << solution.size() << std::endl;
    for (size_t a = 0; a < solution.size(); ++a) {
      // std::cout << "Solution for: " << a << std::endl;
      // for (size_t i = 0; i < solution[a].actions.size(); ++i) {
      //   std::cout << solution[a].states[i].second << ": " <<
      //   solution[a].states[i].first << "->" << solution[a].actions[i].first
      //   << "(cost: " << solution[a].actions[i].second << ")" << std::endl;
      // }
      // std::cout << solution[a].states.back().second << ": " <<
      // solution[a].states.back().first << std::endl;

      std::cout << "  agent" << a << ":" << std::endl;
      paths[a].x_poses.clear();
      paths[a].y_poses.clear();
      paths[a].t_step.clear();
      paths[a].robot_id = int(a);

      //std::cout << "state size : " << solution[a].states.size() << std::endl;
      int i = 0;
      for (const auto& state : solution[a].states) {
        std::cout << "    - x: " << state.first.x << std::endl
            << "      y: " << state.first.y << std::endl
            << "      t: " << state.second << std::endl;
            paths[a].x_poses.push_back(state.first.x);
            paths[a].y_poses.push_back(state.first.y);
            paths[a].t_step.push_back(state.second);
            //std::cout << "iteration: " << i << std::endl;
            //std::cout << "poses path: " << paths[a].x_poses.size() << std::endl;
            ++i;
      }
     //std::cout << "SIZE IN LOOP " << paths[a].x_poses.size() << std::endl;
    }
    
  } else {
    std::cout << "Planning NOT successful!" << std::endl;
  }



    //               if (costmap_ros_->get_current_state().id() ==
    //               lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
  
    //	       }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Tried to execute the planning callback "
                                     "while the costmap is not ready!");
  }
  //   	loop_rate.sleep();
  //	}
}



nav2_util::CallbackReturn
ecbs_server::on_configure(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(this->get_logger(), "Configuring: %s", state.label().c_str());
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "mapf_costmap", std::string(get_namespace()), "mapf_costmap");
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
  if (costmap_ros_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "costmap_ros is null!");
  }

  costmap_ros_->on_configure(state);
  costmap_ = costmap_ros_->getLayeredCostmap()->getCostmap();


  std::cout << "test" << std::endl;
  RCLCPP_INFO(this->get_logger(), "Done configuring!");
  

  //      planning_thread = std::thread(&ecbs_server::control_callback, this);
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ecbs_server::on_activate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Activating");

  //      plan_publisher_->on_activate();
  //      action_server_pose_->activate();
  //      action_server_poses_->activate();
  
  costmap_ros_->on_activate(state);
  //      std::chrono::duration<double> period;
  //      period = std::chrono::milliseconds(100);
  //      timer_ = this->create_wall_timer(period,
  //      std::bind(&mapf_action_server::ecbs_server::control_callback, this));
  //      planning_thread.detach();
  dimx = costmap_->getSizeInCellsX();
  dimy = costmap_->getSizeInCellsY();

  for(int i = 0; i < dimx; i++)
  {
    for(int j = 0; j < dimy; j++)
    {
      //std::cout << (unsigned int)costmap_->getCost(i, j) << std::endl;
      if(costmap_->getCost(i, j) > 0)
      {
        //std::cout << "x " << i << " < y " << j << std::endl;
        //std::cout << (unsigned int)costmap_->getCost(i, j) << std::endl;
        obstacles.insert(Location(i, j));
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "Done activating!");
  //
  //      PlannerMap::iterator it;
  //      for (it = planners_.begin(); it != planners_.end(); ++it) {
  //        it->second->activate();
  //      }

  // create bond connection
    int agent_size = 3;
    int iterator = 0;
  for(int i = 0; i < agent_size; i++)
  {
    std::cout << "i " << i << std::endl;
    geometry_msgs::msg::PoseStamped start;
    start.pose.position.x = 5.0;
    start.pose.position.y = 1.0 + i * 0.5;

    geometry_msgs::msg::PoseStamped goal;
    goal.pose.position.x = 6.0;
    goal.pose.position.y = 0.0 + i * 0.5;
    create_agent(start, goal, i);
    
  }
	
  createBond();
  

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ecbs_server::on_deactivate(const rclcpp_lifecycle::State &state) {
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
ecbs_server::on_cleanup(const rclcpp_lifecycle::State &state) {
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

nav2_util::CallbackReturn ecbs_server::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool ecbs_server::check_bounds(double start_x, double start_y, double goal_x, double goal_y)
{
  if((start_x > costmap_->getSizeInCellsX()) || (start_y > costmap_->getSizeInCellsY()))
  {
    return false;
  }
  if((goal_x > costmap_->getSizeInCellsX()) || (goal_y > costmap_->getSizeInCellsY()))
  {
    return false;
  }

  return true;
}
bool ecbs_server::check_obstacle(double start_x, double start_y, double goal_x, double goal_y)
{
  if(((unsigned int)costmap_->getCost(start_x, start_y) > 0) || ((unsigned int)costmap_->getCost(goal_x, goal_y) > 0))
  {
    return false;
  }
  return true;
}
bool ecbs_server::create_agent(geometry_msgs::msg::PoseStamped start,
                        geometry_msgs::msg::PoseStamped goal, int robotino_id) {


  uint start_x, start_y, goal_x, goal_y;
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x,
                       start_y);
  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x,
                       goal_y);
  // TODO dynamic parametrization
  rosAgent agent_;
  agent_.goal_x = goal_x;
  agent_.goal_y = goal_y;
  agent_.start_x = start_x;
  agent_.start_y = start_y;
  agent_.id = robotino_id;
  int curr_agent_size = agents.size();
  for(int i = 0; i < curr_agent_size; i++)
  {
    if(agent_.id != agents[i].id)
    {
      if((agent_.start_x == agents[i].start_x) && (agent_.start_y == agents[i].start_y))
      {
        return false;
      }
      if((agent_.goal_x == agents[i].goal_x) && (agent_.goal_y == agents[i].goal_y))
      {
        return false;
      }

    }
  }
  if(!check_obstacle(agent_.start_x, agent_.start_y, agent_.goal_x, agent_.goal_y))
  {
    return false;
  }
  if(!check_bounds(agent_.start_x, agent_.start_y, agent_.goal_x, agent_.goal_y))
  {
    return false;
  }
  RCLCPP_INFO(get_logger(), "Creating: %i", agent_.id);
  agents.push_back(agent_);
  return true;

}

bool ecbs_server::update_agent(geometry_msgs::msg::PoseStamped start,
                        geometry_msgs::msg::PoseStamped goal, rosAgent &agent)

{
  
  uint start_x, start_y, goal_x, goal_y;
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x,
                       start_y);
  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x,
                       goal_y);
  int curr_agent_size = agents.size();
  for(int i = 0; i < curr_agent_size; i++)
  {
    if(agent.id != agents[i].id)
    {
      if((start_x == agents[i].start_x) && (start_y == agents[i].start_y))
      {
        return false;
      }
      if((goal_x == agents[i].goal_x) && (goal_y == agents[i].goal_y))
      {
        return false;
      }

    }
  }
  if(!check_obstacle(start_x, start_y, goal_x, goal_y))
  {
    return false;
  }
  if(!check_bounds(start_x, start_y, goal_x, goal_y))
  {
    return false;
  }
  agent.goal_x = goal_x;
  agent.goal_y = goal_y;
  agent.start_x = start_x;
  agent.start_y = start_y;
  std::cout << "update: " << agent.start_x << " " << agent.start_y << "  "
            << agent.goal_x << " " << agent.goal_y << "\n";
  return true;

}


void ecbs_server::path_response(
    const std::shared_ptr<mapf_actions::srv::Mapf::Request> request,
    std::shared_ptr<mapf_actions::srv::Mapf::Response> response) {
  
  bool agent_missing = true;
  bool valid_points = false;
  

  RCLCPP_INFO(this->get_logger(), "New request: %f %f -> %f %f!",
              request->start.pose.position.x, request->start.pose.position.y,
              request->goal.pose.position.x, request->goal.pose.position.y);

    
  for (auto &agent : agents) {

    if (agent.id == request->robotino_id) {

      RCLCPP_INFO(this->get_logger(), "Agent id %i update!", agent.id);

      if(!update_agent(request->start, request->goal, agent))
      {
      nav_msgs::msg::Path path_;
      path_.header.frame_id = "map";

      path_.header.stamp = this->get_clock().get()->now(); 
      response->path = path_;
      RCLCPP_INFO(this->get_logger(), "Either the goal pose or start pose did overlap with other agents!");
      }

      agent_missing = false;
      valid_points = true;
    }
  }
    
  if (agent_missing) {
  //  RCLCPP_INFO(this->get_logger(), "Creating agent.");
    if(!create_agent(request->start, request->goal, request->robotino_id))
    {
      nav_msgs::msg::Path path_;
      path_.header.frame_id = "map";

      path_.header.stamp = this->get_clock().get()->now(); 
      response->path = path_;
      RCLCPP_INFO(this->get_logger(), "Either the goal pose or start pose did overlap with other agents!");
    }
    valid_points = true;
  //  RCLCPP_INFO(this->get_logger(), "Agent created!");
  }
    


  if(valid_points)
  {
  control_callback();

  nav_msgs::msg::Path path_;
  path_.header.frame_id = "map";

  path_.header.stamp = this->get_clock().get()->now();



  double x_old, y_old;
  for (auto &path : paths) {
    if (path.robot_id == request->robotino_id) {
      for(int i = 0; i < path.x_poses.size(); i++)
      {
        //std::cout << "SIZE PATH " << path.x_poses.size() << std::endl;
        //std::cout << "robotino ID " << path.robot_id << std::endl;
        geometry_msgs::msg::PoseStamped pose_;
        double x, y;
        costmap_->mapToWorld(path.x_poses[i], path.y_poses[i], x, y);
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
        
      }

    
      geometry_msgs::msg::PoseStamped pose_;      
      pose_.pose = request->goal.pose;
      pose_.header.frame_id = "map";
      pose_.header.stamp = this->get_clock().get()->now();
      path_.poses.push_back(pose_);
      path_.header.frame_id = "map";
      path_.header.stamp = this->get_clock().get()->now();
      for(auto pose : path_.poses)
      {
        //std::cout << "x: " << pose.pose.position.x << "y : " << pose.pose.position.y << std::endl;
      }
      response->path = path_;
      RCLCPP_INFO(this->get_logger(), "%d size of path",
                  response->path.poses.size());
    }
    }
  
  RCLCPP_INFO(this->get_logger(), "Passing response!");
  }
    }
  geometry_msgs::msg::PoseStamped ecbs_server::draw_point(double x_1, double y_1,
                                                 double x_2, double y_2,
                                                 double d, unsigned int i) {
  //double dist_ = sqrt(pow(x_2 - x_1, 2.0) + pow(y_2 - y_1, 2.0));
  //double angle_ = atan2(y_2 - y_1, x_2 - x_1);
  double diff_x = x_2 - x_1;
  double diff_y = y_2 - y_1;

  geometry_msgs::msg::PoseStamped pose_;
  //pose_.pose.position.x = x_1 + (dist_ * i * d) * cos(angle_);
  //pose_.pose.position.y = y_1 + (dist_ * i * d) * sin(angle_);
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

RCLCPP_COMPONENTS_REGISTER_NODE(ecbs_server::ecbs_server)
