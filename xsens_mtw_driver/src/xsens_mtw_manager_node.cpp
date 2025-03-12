#include "rclcpp/rclcpp.hpp"

#include "../include/xsens_mtw_driver/xsens_mtw_manager.hpp"


bool isShuttingDown = false;

void signalHandler(int signum)
{
    (void)signum;
    if (!isShuttingDown)
    {
        isShuttingDown = true;
        RCLCPP_FATAL(rclcpp::get_logger("xsens_mtw_manager"), "Exiting...");
        rclcpp::shutdown();
    }
}

int main(int argc, char* argv[])
{
    signal(SIGINT, signalHandler);

    rclcpp::init(argc, argv);

    std::shared_ptr<xsens_mtw_manager::XsensManager> node;

    try
    {
        node = std::make_shared<xsens_mtw_manager::XsensManager>("xsens_mtw_manager");
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("xsens_mtw_manager"), "%s", e.what());
        std::cout << "****ABORT****" << std::endl;
    }
    catch (...)
    {
        RCLCPP_ERROR(rclcpp::get_logger("xsens_mtw_manager"), "An unknown fatal error has occured.");
        std::cout << "****ABORT****" << std::endl;
    }

    rclcpp::shutdown();

    return 0;
}