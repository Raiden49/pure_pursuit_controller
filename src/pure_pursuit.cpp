#include "pure_pursuit_controller/pure_pursuit_node.hpp"

int main(int argc, char** argv) {

    ros::init(argc, argv, "pure_pursuit_controller");
    ros::NodeHandle n;
    auto node = new pure_pursuit_controller::PurePursuitNode(n);
    node->PurePursuitRun();

    return 0;
}