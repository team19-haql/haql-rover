#include <memory>
#include <string>
#include <utility>

#include <traverse_layer/TraverseLayer.hpp>

namespace traverse_layer
{

TraverseLayer::TraverseLayer() : Node("traverse_layer"), filter_chain_("grid_map::GridMap")
{
    if (!readParameters()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read parameters.");
        rclcpp::shutdown();
        return;
    }

    subscriber_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
        input_topic_, 1,
        std::bind(&TraverseLayer::callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        output_topic_, rclcpp::QoS(1).transient_local());

    // setup filter chain
    if (filter_chain_.configure(
        filter_chain_parameter_name_, this->get_node_logging_interface(),
        this->get_node_parameters_interface()))
    {
        RCLCPP_INFO(this->get_logger(), "Filter chain has been configured.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to configure filter chain.");
        rclcpp::shutdown();
    }
}

TraverseLayer::~TraverseLayer()
{
    RCLCPP_INFO(this->get_logger(), "TraverseLayer node has been stopped.");
}


bool TraverseLayer::readParameters() {
    this->declare_parameter<std::string>("input_topic", "input_topic");
    this->declare_parameter<std::string>("output_topic", "output_topic");
    this->declare_parameter<std::string>("filter_chain_parameter_name", "filters");

    if (!this->get_parameter("input_topic", input_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get input_topic.");
        return false;
    }

    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("filter_chain_parameter_name", filter_chain_parameter_name_);

    RCLCPP_INFO(this->get_logger(), "input_topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "filter_chain_parameter_name: %s", filter_chain_parameter_name_.c_str());

    return true;
}

void TraverseLayer::callback(const grid_map_msgs::msg::GridMap::SharedPtr msg) {
}


} // namespace traverse_layer