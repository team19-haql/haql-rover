#include <memory>
#include <string>
#include <utility>
#include <chrono>

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

    costmap_publisher_ = this->create_publisher<nav2_msgs::msg::Costmap>(
        costmap_topic_, rclcpp::QoS(1).transient_local());

    latency_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
        "traverse_layer/latency", rclcpp::QoS(1).transient_local());

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

    this->declare_parameter<std::string>("costmap.topic", "costmap_topic");
    this->declare_parameter<std::string>("costmap.layer", "traversability");
    this->declare_parameter<double>("costmap.min", 0.0);
    this->declare_parameter<double>("costmap.max", 1.0);
    this->declare_parameter<bool>("publish_latency", false);

    if (!this->get_parameter("input_topic", input_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get input_topic.");
        return false;
    }

    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("filter_chain_parameter_name", filter_chain_parameter_name_);
    this->get_parameter("costmap.topic", costmap_topic_);
    this->get_parameter("costmap.layer", costmap_layer_);
    this->get_parameter("costmap.min", costmap_min_);
    this->get_parameter("costmap.max", costmap_max_);
    this->get_parameter("publish_latency", publish_latency_);

    RCLCPP_INFO(this->get_logger(), "input_topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "filter_chain_parameter_name: %s", filter_chain_parameter_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "costmap_topic: %s", costmap_topic_.c_str());

    return true;
}

void TraverseLayer::callback(const grid_map_msgs::msg::GridMap::SharedPtr msg) {
    grid_map::GridMap inputMap;
    grid_map::GridMapRosConverter::fromMessage(*msg, inputMap);

    auto start = std::chrono::high_resolution_clock::now();

    // Apply chain filter
    grid_map::GridMap outputMap;
    if (!filter_chain_.update(inputMap, outputMap)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to update filter chain.");
        return;
    }

    // RCLCPP_INFO(this->get_logger(), "Filter chain has been updated.");
    // Publish output map
    std::unique_ptr<grid_map_msgs::msg::GridMap> outputMessage;
    outputMessage = grid_map::GridMapRosConverter::toMessage(outputMap);
    auto end = std::chrono::high_resolution_clock::now();

    publisher_->publish(std::move(outputMessage));

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    double latency = duration.count() / 1000.0;
    if (publish_latency_) {
        std_msgs::msg::Float64 latencyMessage;
        latencyMessage.data = latency;
        latency_publisher_->publish(latencyMessage);
    }

    // Publish costmap
    // nav2_msgs::msg::Costmap costmapMessage;
    // grid_map::GridMapRosConverter::toCostmap(
    //     outputMap, costmap_layer_, costmap_min_, costmap_max_, costmapMessage);
    // costmap_publisher_->publish(costmapMessage);
}


} // namespace traverse_layer