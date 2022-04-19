
#include "rclcpp/rclcpp.hpp"

#include "ecbs_server/ecbs_server.hpp"

int main(int argc, const char **argv)
{

    rclcpp::init(argc, argv);
    auto ECBSNode = std::make_shared<ecbs_server::ecbs_server>();
    //    CCBSNode->init();
    rclcpp::spin(ECBSNode->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
