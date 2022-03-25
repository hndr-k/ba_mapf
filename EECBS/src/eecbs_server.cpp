#include "eecbs_server.h"

namespace eecbs_server {
//    eecbs_server::eecbs_server(const rclcpp::NodeOptions& ) : Node("mapf_action_server"),
//    map_(0.25, 3), cbs_() eecbs_server::eecbs_server() :
//    rclcpp_lifecycle::LifecycleNode("mapf_action_server",
//    rclcpp::NodeOptions()), map_(0.25, 3), cbs_()
//
eecbs_server::eecbs_server(const rclcpp::NodeOptions &options)
    : nav2_util::LifecycleNode("mapf_action_server", "", true, options), loop_rate(20), desc("Allowed options!") {

  //        occ_grid_sub_ =
  //        this->create_subscription<nav_msgs::msg::OccupancyGrid>("global_costmap/costmap",
  //        10, std::bind(&eecbs_server::occ_grid_callback, this,
  //        std::placeholders::_1));
  service_ = this->create_service<mapf_actions::srv::Mapf>(
      "/off_field/mapf_plan",
      std::bind(&eecbs_server::path_response, this, std::placeholders::_1,
                std::placeholders::_2));
  


  RCLCPP_INFO(this->get_logger(), "MAPF server initialized");
}
eecbs_server::~eecbs_server() {}

void eecbs_server::init() {
  //            occ_grid_sub_ =
  //            this->create_subscription<nav_msgs::msg::OccupancyGrid>("global_costmap/costmap",
  //            10, std::bind(&eecbs_server::occ_grid_callback, this,
  //            std::placeholders::_1));
  //        service_ =
  //        this->create_service<mapf_actions::srv::Mapf>("/off_field/mapf_plan",
  //        std::bind(&eecbs_server::path_response, this, std::placeholders::_1,
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

	// Declare the supported options.

	desc.add_options()
		("help", "produce help message")

		// params for the input instance and experiment settings
		("map,m", boost::program_options::value<string>()->required(), "input file for map")
		("agents,a", boost::program_options::value<string>()->required(), "input file for agents")
		("output,o", boost::program_options::value<string>(), "output file for statistics")
		("outputPaths", boost::program_options::value<string>(), "output file for paths")
		("agentNum,k", boost::program_options::value<int>()->default_value(0), "number of agents")
		("cutoffTime,t", boost::program_options::value<double>()->default_value(7200), "cutoff time (seconds)")
		("screen,s", boost::program_options::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
		("stats", boost::program_options::value<bool>()->default_value(false), "write to files some detailed statistics")

		// params for CBS node selection strategies
		("highLevelSolver", boost::program_options::value<string>()->default_value("EES"), "the high-level solver (A*, A*eps, EES, NEW)")
		("lowLevelSolver", boost::program_options::value<bool>()->default_value(true), "using suboptimal solver in the low level")
		("inadmissibleH", boost::program_options::value<string>()->default_value("Global"), "inadmissible heuristics (Zero, Global, Path, Local, Conflict)")
		("suboptimality", boost::program_options::value<double>()->default_value(1.2), "suboptimality bound")

		// params for CBS improvement
		("heuristics", boost::program_options::value<string>()->default_value("WDG"), "admissible heuristics for the high-level search (Zero, CG,DG, WDG)")
		("prioritizingConflicts", boost::program_options::value<bool>()->default_value(true), "conflict prioirtization. If true, conflictSelection is used as a tie-breaking rule.")
		("bypass", boost::program_options::value<bool>()->default_value(true), "Bypass1")
		("disjointSplitting", boost::program_options::value<bool>()->default_value(false), "disjoint splitting")
		("rectangleReasoning", boost::program_options::value<bool>()->default_value(true), "rectangle reasoning")
		("corridorReasoning", boost::program_options::value<bool>()->default_value(true), "corridor reasoning")
		("targetReasoning", boost::program_options::value<bool>()->default_value(true), "target reasoning")
		("restart", boost::program_options::value<int>()->default_value(0), "rapid random restart times")
		;
 
	if (vm.count("help")) {
		cout << desc << endl;
		
	} 
 char* test[] = {
   NULL
 };
  boost::program_options::store(boost::program_options::parse_command_line(0, test, desc), vm);
	boost::program_options::notify(vm);
	if (vm["suboptimality"].as<double>() < 1)
	{
		cerr << "Suboptimal bound should be at least 1!" << endl;

	}

	
	if (vm["highLevelSolver"].as<string>() == "A*")
		s = high_level_solver_type::ASTAR;
	else if (vm["highLevelSolver"].as<string>() == "A*eps")
		s = high_level_solver_type::ASTAREPS;
	else if (vm["highLevelSolver"].as<string>() == "EES")
		s = high_level_solver_type::EES;
	else if (vm["highLevelSolver"].as<string>() == "NEW")
		s = high_level_solver_type::NEW;
	else
	{
		cout << "WRONG high level solver!" << endl;
		
	}

	if (s == high_level_solver_type::ASTAR && vm["suboptimality"].as<double>() > 1)
	{
		cerr << "A* cannot perform suboptimal search!" << endl;
		
	}


	if (vm["heuristics"].as<string>() == "Zero")
		h = heuristics_type::ZERO;
	else if (vm["heuristics"].as<string>() == "CG")
		h = heuristics_type::CG;
	else if (vm["heuristics"].as<string>() == "DG")
		h = heuristics_type::DG;
	else if (vm["heuristics"].as<string>() == "WDG")
		h = heuristics_type::WDG;
	else
	{
		cout << "WRONG heuristics strategy!" << endl;
		
	}

    if ((h == heuristics_type::CG || h == heuristics_type::DG) && vm["lowLevelSolver"].as<bool>())
    {
        cerr << "CG or DG heuristics do not work with low level of suboptimal search!" << endl;
        
    }


	if (s == high_level_solver_type::ASTAR ||
	    s == high_level_solver_type::ASTAREPS ||
	    vm["inadmissibleH"].as<string>() == "Zero")
		h_hat = heuristics_type::ZERO;
	else if (vm["inadmissibleH"].as<string>() == "Global")
		h_hat = heuristics_type::GLOBAL;
	else if (vm["inadmissibleH"].as<string>() == "Path")
		h_hat = heuristics_type::PATH;
	else if (vm["inadmissibleH"].as<string>() == "Local")
		h_hat = heuristics_type::LOCAL;
	else if (vm["inadmissibleH"].as<string>() == "Conflict")
		h_hat = heuristics_type::CONFLICT;
	else
	{
		cout << "WRONG inadmissible heuristics strategy!" << endl;
	
	}

	conflict = conflict_selection::EARLIEST;
	n = node_selection::NODE_CONFLICTPAIRS;

}

void eecbs_server::control_callback() {
  //	while (rclcpp::ok())
  //	{
  RCLCPP_INFO(this->get_logger(), "%u", __LINE__);

  if (costmap_ros_ != nullptr && costmap_ != nullptr) {
    //               if (costmap_ros_->get_current_state().id() ==
    //               lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
  Instance instance(costmap_, agents);

	srand(0);
	int runs = 1 + vm["restart"].as<int>();
	//////////////////////////////////////////////////////////////////////
    // initialize the solver
	if (vm["lowLevelSolver"].as<bool>())
    {
        ECBS ecbs(instance, false, vm["screen"].as<int>());
        ecbs.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
        ecbs.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
        ecbs.setBypass(vm["bypass"].as<bool>());
        ecbs.setRectangleReasoning(vm["rectangleReasoning"].as<bool>());
        ecbs.setCorridorReasoning(vm["corridorReasoning"].as<bool>());
        ecbs.setHeuristicType(h, h_hat);
        ecbs.setTargetReasoning(vm["targetReasoning"].as<bool>());
        ecbs.setMutexReasoning(false);
        ecbs.setConflictSelectionRule(conflict);
        ecbs.setNodeSelectionRule(n);
        ecbs.setSavingStats(vm["stats"].as<bool>());
        ecbs.setHighLevelSolver(s, vm["suboptimality"].as<double>());
        //////////////////////////////////////////////////////////////////////
        // run
        double runtime = 0;
        int lowerbound = 0;
        for (int i = 0; i < runs; i++)
        {
            ecbs.clear();
            ecbs.solve(vm["cutoffTime"].as<double>() / runs, lowerbound);
            runtime += ecbs.runtime;
            if (ecbs.solution_found)
                break;
            lowerbound = ecbs.getLowerBound();
            ecbs.randomRoot = true;
            cout << "Failed to find solutions in Run " << i << endl;
        }
        ecbs.runtime = runtime;
        //if (vm.count("output"))
        //    ecbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
        if (ecbs.solution_found && vm.count("outputPaths"))
            paths = ecbs.savePaths();
        /*size_t pos = vm["output"].as<string>().rfind('.');      // position of the file extension
        string output_name = vm["output"].as<string>().substr(0, pos);     // get the name without extension
        cbs.saveCT(output_name); // for debug*/
        //if (vm["stats"].as<bool>())
        //    ecbs.saveStats(vm["output"].as<string>(), vm["agents"].as<string>());
        ecbs.clearSearchEngines();
    }
    else
    {
        CBS cbs(instance, false, vm["screen"].as<int>());
        cbs.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
        cbs.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
        cbs.setBypass(vm["bypass"].as<bool>());
        cbs.setRectangleReasoning(vm["rectangleReasoning"].as<bool>());
        cbs.setCorridorReasoning(vm["corridorReasoning"].as<bool>());
        cbs.setHeuristicType(h, h_hat);
        cbs.setTargetReasoning(vm["targetReasoning"].as<bool>());
        cbs.setMutexReasoning(false);
        cbs.setConflictSelectionRule(conflict);
        cbs.setNodeSelectionRule(n);
        cbs.setSavingStats(vm["stats"].as<bool>());
        cbs.setHighLevelSolver(s, vm["suboptimality"].as<double>());
        //////////////////////////////////////////////////////////////////////
        // run
        double runtime = 0;
        int lowerbound = 0;
        for (int i = 0; i < runs; i++)
        {
            cbs.clear();
            cbs.solve(vm["cutoffTime"].as<double>() / runs, lowerbound);
            runtime += cbs.runtime;
            if (cbs.solution_found)
                break;
            lowerbound = cbs.getLowerBound();
            cbs.randomRoot = true;
            cout << "Failed to find solutions in Run " << i << endl;
        }
        cbs.runtime = runtime;
        
        if (cbs.solution_found && vm.count("outputPaths"))
            paths = cbs.savePaths();
        cbs.clearSearchEngines();
    }


    if (true) {

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
eecbs_server::on_configure(const rclcpp_lifecycle::State &state) {
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

  //      planning_thread = std::thread(&eecbs_server::control_callback, this);
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
eecbs_server::on_activate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Activating");

  //      plan_publisher_->on_activate();
  //      action_server_pose_->activate();
  //      action_server_poses_->activate();
  costmap_ros_->on_activate(state);
  //      std::chrono::duration<double> period;
  //      period = std::chrono::milliseconds(100);
  //      timer_ = this->create_wall_timer(period,
  //      std::bind(&mapf_action_server::eecbs_server::control_callback, this));
  //      planning_thread.detach();
  RCLCPP_INFO(this->get_logger(), "Done activating!");
  //
  //      PlannerMap::iterator it;
  //      for (it = planners_.begin(); it != planners_.end(); ++it) {
  //        it->second->activate();
  //      }

  // create bond connection

  createBond();


  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
eecbs_server::on_deactivate(const rclcpp_lifecycle::State &state) {
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
eecbs_server::on_cleanup(const rclcpp_lifecycle::State &state) {
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

nav2_util::CallbackReturn eecbs_server::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}



void eecbs_server::create_agent(geometry_msgs::msg::PoseStamped start,
                        geometry_msgs::msg::PoseStamped goal, int start_id,
                        int goal_id, int robotino_id) {
  /*int resolution = current_grid_.info.resolution;*/
  /*int pos_i = (start.pose.position.x - current_grid_.info.origin.position.x )
  / resolution; int pos_j = (start.pose.position.y -
  current_grid_.info.origin.position.y ) / resolution;

  int goal_i = (goal.pose.position.x - current_grid_.info.origin.position.x ) /
  resolution; int goal_j = (goal.pose.position.y -
  current_grid_.info.origin.position.y ) / resolution;*/

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
  agents.push_back(agent_);

}

void eecbs_server::update_agent(geometry_msgs::msg::PoseStamped start,
                        geometry_msgs::msg::PoseStamped goal, rosAgent &agent)

{

  uint start_x, start_y, goal_x, goal_y;
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x,
                       start_y);
  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x,
                       goal_y);

  agent.goal_x = goal_x;
  agent.goal_y = goal_y;
  agent.start_x = start_x;
  agent.start_y = start_y;
  std::cout << "update: " << agent.start_x << " " << agent.start_y << "  "
            << agent.goal_x << " " << agent.goal_y << "\n";

}


void eecbs_server::path_response(
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



  control_callback();

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
  for (auto &path : paths) {
    if (path.robot_id == request->robotino_id) {
      for(int i = 0; i < path.x_poses.size(); i++)
      {
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
 //       std::cout << "x: " << x << " y: " << y << std::endl;
        
      }
    }
      geometry_msgs::msg::PoseStamped pose_;
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
  
  RCLCPP_INFO(this->get_logger(), "Passing response!");

    }
} // namespace mapf_action_server

RCLCPP_COMPONENTS_REGISTER_NODE(eecbs_server::eecbs_server)
