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
    raw_map_({"elevation", "variance","time",
              "lowest_scan_point"}),
    map_({"elevation", "lower_bound", "upper_bound"}),
    min_variance_(0.000009),
    max_variance_(0.0009),
    mahalanobis_distance_threshold_(2.5),
    multi_height_noise_(0.000009),
    scanning_duration_(0.05)
{
    if (!read_parameters()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read parameters.");
        rclcpp::shutdown();
        return;
    }

    initial_time_ = this->get_clock()->now();

    grid_map::Length mapLength(world_length_, world_width_);
    grid_map::Position mapPosition(0.0, 0.0);

    raw_map_.setBasicLayers({"elevation", "variance"});
    raw_map_.setGeometry(mapLength, resolution_, mapPosition);
    raw_map_.setFrameId(map_frame_id_);
    map_.setBasicLayers({"elevation", "lower_bound", "upper_bound"});
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
    for (grid_map::GridMapIterator iterator(raw_map_); !iterator.isPastEnd(); ++iterator) {
        grid_map::Index index;
        grid_map::Position position;

        if (!raw_map_.getPosition(*iterator, position)) {
            continue; // skip this point outside of map
        }

        if (!map_.getIndex(position, index)) {
            continue;
        }

        double elevation = raw_map_.at("elevation", *iterator);
        double variance = raw_map_.at("variance", *iterator);
        double sigma = sqrt(variance);
        map_.at("elevation", index) = elevation;
        map_.at("upper_bound", index) = elevation + 2 * sigma;
        map_.at("lower_bound", index) = elevation - 2 * sigma;
    }

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
        raw_map_.move(newCenter);
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

    // transform pointcloud to map frame
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

    const rclcpp::Time current_time = this->get_clock()->now();
    const double time_since_start = (current_time - initial_time_).seconds();

    auto& elevation_layer = raw_map_["elevation"];
    auto& variance_layer = raw_map_["variance"];
    auto& time_layer = raw_map_["time"];
    auto& lowest_scan_point_layer = raw_map_["lowest_scan_point"];

    // create cloudpoint2 iterator
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    // iterate through point cloud
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        double px = *iter_x, py = *iter_y, pz = *iter_z;

        double dx = px - transform.transform.translation.x;
        double dy = py - transform.transform.translation.y;
        double dz = pz - transform.transform.translation.z;

        grid_map::Index index;
        grid_map::Position position(px, py);

        if (!raw_map_.getIndex(position, index)) {
            continue; // skip this point outside of map
        }

        double distance = sqrt(dx * dx + dy * dy + dz * dz);
        double R = distance / 40.0; // sensor covariance
        R = R * R; // variance

        auto& elevation = elevation_layer(index(0), index(1));
        auto& variance = variance_layer(index(0), index(1));
        auto& time = time_layer(index(0), index(1));
        auto& lowest_scan_point = lowest_scan_point_layer(index(0), index(1));

        if (std::isnan(elevation)) {
            elevation = pz;
            variance = R;
            time = time_since_start;
            lowest_scan_point = pz;
            continue;
        }

        const double mahalanobis_distance = std::abs(pz - elevation) / sqrt(variance);
        if (mahalanobis_distance > mahalanobis_distance_threshold_) {
            if (time_since_start - time <= scanning_duration_) {
                if (pz > elevation) {
                    // set point to heigher elevation.
                    elevation = pz;
                    variance = R;
                }
            } else {
                variance += multi_height_noise_;
            }
            continue;
        }

        // point + 3 sigma
        const double point_height_plus_uncertainty = pz + 3.0 * sqrt(variance);
        if (point_height_plus_uncertainty < lowest_scan_point) {
            lowest_scan_point = point_height_plus_uncertainty;
        }
    }


    // clean map
    for (grid_map::GridMapIterator iterator(raw_map_); !iterator.isPastEnd(); ++iterator) {
        auto& variance = raw_map_.at("variance", *iterator);
        if (!std::isnan(variance)) {
            variance = variance < min_variance_ ? min_variance_ : variance;
            variance = variance > max_variance_ ? max_variance_ : variance;
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