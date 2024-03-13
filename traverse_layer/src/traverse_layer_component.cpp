#include <memory>
#include <string>
#include <utility>
#include <chrono>

#include <traverse_layer/traverse_layer_component.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

namespace traverse_layer
{

TraverseLayer::TraverseLayer(const rclcpp::NodeOptions & options) : 
    Node("traverse_layer", options),
    map_({"elevation", "traversability"}),
    downscaled_map_({"elevation", "traversability"}),
    filter_chain_("grid_map::GridMap")
{
    if (!readParameters()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read parameters.");
        rclcpp::shutdown();
        return;
    }

    // Setup map
    map_initialized_ = false;

    subscriber_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
        input_topic_, 1,
        std::bind(&TraverseLayer::callback, this, std::placeholders::_1));

    local_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        local_map_topic_, 5);

    global_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        global_map_topic_, 5);

    costmap_publisher_ = this->create_publisher<nav2_msgs::msg::Costmap>(
        costmap_topic_, 5);

    latency_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
        "traverse_layer/latency", 10);

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
    this->declare_parameter<std::string>("local_map_topic", "local_map_topic");
    this->declare_parameter<std::string>("global_map_topic", "global_map_topic");
    this->declare_parameter<std::string>("filter_chain_parameter_name", "filters");

    this->declare_parameter<std::string>("costmap.topic", "costmap_topic");
    this->declare_parameter<std::string>("costmap.layer", "traversability");
    this->declare_parameter<double>("costmap.min", 0.0);
    this->declare_parameter<double>("costmap.max", 1.0);
    this->declare_parameter<bool>("costmap.enabled", false);
    this->declare_parameter<bool>("publish_latency", false);

    // map parameters
    this->declare_parameter<double>("map.resolution", 0.20);
    this->declare_parameter<double>("map.local_length", 20.0);
    this->declare_parameter<double>("map.movement_update_threshold", 5.0);
    this->declare_parameter<double>("map.downscaled_resolution", 1.0);
    this->declare_parameter<std::string>("map.frame_id", "map");

    if (!this->get_parameter("input_topic", input_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get input_topic.");
        return false;
    }

    this->get_parameter("local_map_topic", local_map_topic_);
    this->get_parameter("global_map_topic", global_map_topic_);
    this->get_parameter("filter_chain_parameter_name", filter_chain_parameter_name_);
    this->get_parameter("costmap.topic", costmap_topic_);
    this->get_parameter("costmap.layer", costmap_layer_);
    this->get_parameter("costmap.min", costmap_min_);
    this->get_parameter("costmap.max", costmap_max_);
    this->get_parameter("costmap.enabled", publish_costmap_);
    this->get_parameter("publish_latency", publish_latency_);

    // map parameters
    this->get_parameter("map.resolution", map_resolution_);
    this->get_parameter("map.local_length", map_local_length_);
    this->get_parameter("map.movement_update_threshold", movement_update_threshold_);
    this->get_parameter("map.downscaled_resolution", downscaled_map_resolution_);
    this->get_parameter("map.frame_id", map_frame_id_);

    RCLCPP_INFO(this->get_logger(), "input_topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "local_map_topic: %s", local_map_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "global_map_topic: %s", global_map_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "map_frame_id: %s", map_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "filter_chain_parameter_name: %s", filter_chain_parameter_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "costmap_topic: %s", costmap_topic_.c_str());

    return true;
}

void TraverseLayer::update_downscaled_map(grid_map::GridMap& input_map) {
    downscaled_map_.extendToInclude(input_map);

    grid_map::GridMap downscaled_local_map;
    grid_map::GridMapCvProcessing::changeResolution(
        input_map,
        downscaled_local_map,
        downscaled_map_resolution_
    );

    downscaled_map_.addDataFrom(downscaled_local_map, true, true, false, {"elevation", "traversability"});

    std::unique_ptr<grid_map_msgs::msg::GridMap> outputMessage;
    outputMessage = grid_map::GridMapRosConverter::toMessage(downscaled_map_);
    global_publisher_->publish(std::move(outputMessage));
}

void TraverseLayer::update_map(grid_map::GridMap& input_map) {
    if (!map_initialized_) {
        map_initialized_ = true;
        grid_map::Length mapLength(map_local_length_, map_local_length_);

        map_.setGeometry(mapLength, map_resolution_, input_map.getPosition());
        RCLCPP_INFO(
            this->get_logger(),
            "Created map with size %f x %f m (%i x %i cells).",
            map_.getLength().x(), map_.getLength().y(),
            map_.getSize()(0), map_.getSize()(1));

        downscaled_map_.setGeometry(mapLength, downscaled_map_resolution_, input_map.getPosition());
        RCLCPP_INFO(
            this->get_logger(),
            "Created downscaled map with size %f x %f m (%i x %i cells).",
            downscaled_map_.getLength().x(), downscaled_map_.getLength().y(),
            downscaled_map_.getSize()(0), downscaled_map_.getSize()(1));

        last_position_ = input_map.getPosition();

        map_.setFrameId(map_frame_id_);
        downscaled_map_.setFrameId(map_frame_id_);
    }
    // Fuse maps
    // addDataFrame(other, extendMap, overwriteData, copyAllLayers, layers)
    map_.addDataFrom(input_map, true, true, false, {"elevation", "traversability"});

    grid_map::Length mapLength(map_local_length_, map_local_length_);
    bool success;
    grid_map::GridMap local_map = map_.getSubmap(input_map.getPosition(), mapLength, success);
    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get local submap.");
        return;
    }

    int nan_count_traversability = 0;
    int nan_count_elevation = 0;

    for (grid_map::GridMapIterator iterator(local_map); !iterator.isPastEnd(); ++iterator) {
        if (std::isnan(local_map.at("traversability", *iterator))) {
            ++nan_count_traversability;
        }
        if (std::isnan(local_map.at("elevation", *iterator))) {
            ++nan_count_elevation;
        }
    }

    RCLCPP_DEBUG(this->get_logger(), "Number of NAN values in traversability: %d", nan_count_traversability);
    RCLCPP_DEBUG(this->get_logger(), "Number of NAN values in elevation: %d", nan_count_elevation);

    // update downscaled map
    double dx = last_position_.x() - input_map.getPosition().x();
    double dy = last_position_.y() - input_map.getPosition().y();
    
    if (dx * dx + dy * dy > movement_update_threshold_ * movement_update_threshold_) {
        RCLCPP_INFO(this->get_logger(), "Updating downscaled map.");
        update_downscaled_map(local_map);
        last_position_ = input_map.getPosition();
    }

    std::unique_ptr<grid_map_msgs::msg::GridMap> outputMessage;
    outputMessage = grid_map::GridMapRosConverter::toMessage(local_map);
    local_publisher_->publish(std::move(outputMessage));
}

static void clear_footprint(grid_map::GridMap& map, const double radius) {
    for (grid_map::CircleIterator it(map, map.getPosition(), radius); !it.isPastEnd(); ++it) {
        map.at("traversability", *it) = 0.0;
    }
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

    clear_footprint(outputMap, 1.0);
    // RCLCPP_INFO(this->get_logger(), "Filter chain has been updated.");

    update_map(outputMap);

    auto end = std::chrono::high_resolution_clock::now();

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

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(traverse_layer::TraverseLayer)