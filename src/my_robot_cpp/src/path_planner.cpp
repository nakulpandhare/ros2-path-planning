#include "rclcpp/rclcpp.hpp"

class PathPlannerNode : public rclcpp::Node {
public:
    PathPlannerNode() : Node("path_planner_cpp") {
        RCLCPP_INFO(this->get_logger(), "C++ Path Planner started!");
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PathPlannerNode::plan_path, this)
        );
    }

private:
    void plan_path() {
        RCLCPP_INFO(this->get_logger(), "Planning a path...");
    }
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}