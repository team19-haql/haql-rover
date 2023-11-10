#include <memory>
#include <string>
#include <utility>
#include <string>

#include <traverse_layer/PointcloudToGridmap.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point_stamped.h>
#ifdef TF2_CPP_HEADERS
  #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
  #include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#else
  #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
  #include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#endif

namespace traverse_layer
{

PointcloudToGridmap::PointcloudToGridmap() : Node("pointcloud_to_gridmap")
{
    if (!read_parameters()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read parameters.");
        rclcpp::shutdown();
        return;
    }

    grid_map::Length mapLength(world_length_, world_width_);
    grid_map::Position mapPosition(0.0, 0.0);

    map_.setGeometry(mapLength, resolution_, mapPosition);
    map_.setFrameId(map_frame_id_);
    map_.add("elevation");

    RCLCPP_INFO(
        this->get_logger(),
        "Created map with size %f x %f m (%i x %i cells).",
        map_.getLength().x(), map_.getLength().y(),
        map_.getSize()(0), map_.getSize()(1));


    std::chrono::duration<int> buffer_timeout(1);
    // TF Buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    // Create time interface before call to wait for transform
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subscriber_.subscribe(this, input_topic_);
    tf_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        subscriber_, *tf_buffer_, map_frame_id_, 100,
        this->get_node_logging_interface(),
        this->get_node_clock_interface(), buffer_timeout);

    tf_filter_->registerCallback(&PointcloudToGridmap::callback, this);

    // subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     input_topic_, rclcpp::SensorDataQoS(),
    //     std::bind(&PointcloudToGridmap::callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        output_topic_, rclcpp::QoS(1).transient_local());

    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
        std::bind(&PointcloudToGridmap::timer_callback, this));
}

PointcloudToGridmap::~PointcloudToGridmap()
{
    RCLCPP_INFO(this->get_logger(), "PointcloudToGridmap node has been stopped.");
}


bool PointcloudToGridmap::read_parameters() {
    this->declare_parameter<std::string>("input_topic", "input_topic");
    this->declare_parameter<std::string>("output_topic", "output_topic");
    this->declare_parameter<std::string>("map_frame_id", "map");
    this->declare_parameter<double>("resolution", 0.03);
    this->declare_parameter<double>("world_size.length", 10.0);
    this->declare_parameter<double>("world_size.width", 10.0);
    this->declare_parameter<double>("publish_rate", 1.0);

    if (!this->get_parameter("input_topic", input_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get input_topic.");
        return false;
    }
    this->get_parameter("output_topic", output_topic_);

    RCLCPP_INFO(this->get_logger(), "input_topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic_.c_str());

    if (!this->get_parameter("map_frame_id", map_frame_id_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get map_frame_id.");
        return false;
    }

    if (!this->get_parameter("resolution", resolution_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get resolution.");
        return false;
    }

    if (!this->get_parameter("world_size.length", world_length_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get world_size.length.");
        return false;
    }

    if (!this->get_parameter("world_size.width", world_width_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get world_size.width.");
        return false;
    }

    if (!this->get_parameter("publish_rate", publish_rate_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get publish_rate.");
        return false;
    }

    return true;
}

void PointcloudToGridmap::timer_callback() {
    RCLCPP_INFO(this->get_logger(), "Publishing grid map.");
    std::unique_ptr<grid_map_msgs::msg::GridMap> message;
    message = grid_map::GridMapRosConverter::toMessage(map_);
    publisher_->publish(std::move(message));
}

void PointcloudToGridmap::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received point cloud message.");

    // if (!tf_buffer_->canTransform(
    //         msg->header.frame_id, map_.getFrameId(),
    //         tf2_ros::fromMsg(msg->header.stamp),
    //         tf2::TimePointZero)) {
    //     RCLCPP_WARN(
    //         this->get_logger(),
    //         "Transform from %s to %s not available, skipping point cloud.",
    //         msg->header.frame_id.c_str(), map_.getFrameId().c_str());
    // }

    // Convert point cloud to grid map frame
    sensor_msgs::msg::PointCloud2 cloud;

    try {
        tf_buffer_->transform(*msg, cloud, map_.getFrameId());
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(
            this->get_logger(),
            "Could not transform point cloud from frame %s to %s: %s",
            msg->header.frame_id.c_str(), map_.getFrameId().c_str(), ex.what());
        return;
    }

    // create cloudpoint2 iterator
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    grid_map::Index index;

    // iterate through point cloud
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        double px = *iter_x, py = *iter_y, pz = *iter_z;

        if (map_.getIndex(grid_map::Position(px, py), index)) {
            float & elevation = map_.at("elevation", index);
            // set point if elevation is nan float
            // set point if elevation is higher than previous
            if (std::isnan(elevation) || pz > elevation) {
                elevation = pz;
            }
        }
    }
}


} // namespace traverse_layer