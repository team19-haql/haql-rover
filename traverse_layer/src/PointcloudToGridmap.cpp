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
#include <chrono>

namespace traverse_layer
{

PointcloudToGridmap::PointcloudToGridmap() : Node("pointcloud_to_gridmap"),
    map_({"elevation", "cov"})
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

    latency_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
        "point_to_grid/latency", rclcpp::QoS(1).transient_local());
}

PointcloudToGridmap::~PointcloudToGridmap()
{
    RCLCPP_INFO(this->get_logger(), "PointcloudToGridmap node has been stopped.");
}


bool PointcloudToGridmap::read_parameters() {
    this->declare_parameter<std::string>("input_topic", "input_topic");
    this->declare_parameter<std::string>("output_topic", "output_topic");
    this->declare_parameter<std::string>("map_frame_id", "map");
    this->declare_parameter<std::string>("center_frame_id", "base_link");
    this->declare_parameter<double>("resolution", 0.03);
    this->declare_parameter<double>("world_size.length", 10.0);
    this->declare_parameter<double>("world_size.width", 10.0);
    this->declare_parameter<double>("publish_rate", 1.0);
    this->declare_parameter<double>("process_noise", 0.01);
    this->declare_parameter<bool>("publish_latency", false);

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

    if (!this->get_parameter("process_noise", process_noise_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get process_noise.");
        return false;
    }

    if (!this->get_parameter("publish_latency", publish_latency_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get publish_latency.");
        return false;
    }

    if (!this->get_parameter("center_frame_id", center_frame_id_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get center_frame_id.");
        return false;
    }

    return true;
}

void PointcloudToGridmap::timer_callback() {
    // RCLCPP_INFO(this->get_logger(), "Publishing grid map.");
    std::unique_ptr<grid_map_msgs::msg::GridMap> message;
    message = grid_map::GridMapRosConverter::toMessage(map_);
    publisher_->publish(std::move(message));
    try {
        geometry_msgs::msg::TransformStamped transform;
        transform = tf_buffer_->lookupTransform(
            map_.getFrameId(), center_frame_id_,
            tf2::TimePointZero
        );
        grid_map::Position newCenter(
            transform.transform.translation.x,
            transform.transform.translation.y
        );
        map_.move(newCenter);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(
            this->get_logger(),
            "Failed to center map. Could not find transform from %s to %s: %s",
            map_.getFrameId().c_str(), center_frame_id_.c_str(), ex.what());
        return;
    }
}

void PointcloudToGridmap::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convert point cloud to grid map frame
    sensor_msgs::msg::PointCloud2 cloud;
    geometry_msgs::msg::TransformStamped transform;

    auto start = std::chrono::high_resolution_clock::now();

    try {
        tf_buffer_->transform(*msg, cloud, map_.getFrameId());

        transform = tf_buffer_->lookupTransform(
            map_.getFrameId(), msg->header.frame_id,
            tf2::TimePointZero);

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(
            this->get_logger(),
            "Could not transform point cloud from frame %s to %s: %s",
            msg->header.frame_id.c_str(), map_.getFrameId().c_str(), ex.what());
        return;
    }

    // RCLCPP_INFO(
    //     this->get_logger(),
    //     "Translation: %f, %f, %f", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);

    // create cloudpoint2 iterator
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    grid_map::Index index;

    // iterate through point cloud
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        double px = *iter_x, py = *iter_y, pz = *iter_z;

        double dx = px - transform.transform.translation.x;
        double dy = py - transform.transform.translation.y;
        double dz = pz - transform.transform.translation.z;

        double distance = sqrt(dx * dx + dy * dy + dz * dz);
        double R = distance / 4.0; // sensor covariance

        if (map_.getIndex(grid_map::Position(px, py), index)) {
            float & x = map_.at("elevation", index);
            float & P = map_.at("cov", index);
            // set point if elevation is nan float
            // set point if elevation is higher than previous
            if (std::isnan(x)) {
                x = pz;
                P = R;
            } else {
                // simple kalman filter
                P = P + process_noise_;
                double S = P + R;
                double K = P / S;
                x = x + K * (pz - x);
                P = (1.0 - K) * P;
            }
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    if (publish_latency_) {
        std_msgs::msg::Float64 latency;
        latency.data = duration.count() / 1000.0;
        latency_publisher_->publish(latency);
    }
}


} // namespace traverse_layer