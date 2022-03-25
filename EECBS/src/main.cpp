#include <iostream>
#include <fstream>
#include "rclcpp/rclcpp.hpp"

#include "eecbs_server.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, const char **argv)
{

    rclcpp::init(argc, argv);
    auto EECBSNode = std::make_shared<eecbs_server::eecbs_server>();
    //    CCBSNode->init();
    rclcpp::spin(EECBSNode->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
