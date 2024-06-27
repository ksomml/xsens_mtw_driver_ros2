#include "rclcpp/rclcpp.hpp"

#include "../include/xsens_mtw_driver_ros2/xsens_mtw_visualization.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<xsens_mtw_visualization::XsensVisualization>("xsens_mtw_visualization"));
    rclcpp::shutdown();
    return 0;
}