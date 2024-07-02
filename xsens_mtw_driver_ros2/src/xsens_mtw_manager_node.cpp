#include "rclcpp/rclcpp.hpp"

#include "../include/xsens_mtw_driver_ros2/xsens_mtw_manager.hpp"


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<xsens_mtw_manager::XsensManager> node;

    rclcpp::Context context;
    context.add_pre_shutdown_callback(std::bind(&xsens_mtw_manager::XsensManager::cleanupAndShutdown, node));

    try
    {
        node = std::make_shared<xsens_mtw_manager::XsensManager>("xsens_mtw_manager");
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    }
    catch (const std::exception& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl;
        std::cout << "****ABORT****" << std::endl;
    }
    catch (...)
    {
        std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
        std::cout << "****ABORT****" << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}