#include<rclcpp/rclcpp.hpp>
#include<memory>
#include<traverse_layer/PointcloudToGridmap.hpp>

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<traverse_layer::PointcloudToGridmap>());
    rclcpp::shutdown();
    return 0;
}