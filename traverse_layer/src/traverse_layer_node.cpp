#include<rclcpp/rclcpp.hpp>
#include<memory>
#include<traverse_layer/TraverseLayer.hpp>

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<traverse_layer::TraverseLayer>());
    rclcpp::shutdown();
    return 0;
}