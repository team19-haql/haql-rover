#ifndef POINTCLOUD_TO_GRIDMAP_HPP_
#define POINTCLOUD_TO_GRIDMAP_HPP_

#include <grid_map_ros/grid_map_ros.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <std_msgs/msg/float64.hpp>
#include <message_filters/subscriber.h>
#include <mutex>

namespace traverse_layer
{

class PointcloudToGridmap : public rclcpp::Node
{
public:
    PointcloudToGridmap(const rclcpp::NodeOptions & options);
    virtual ~PointcloudToGridmap();

    bool read_parameters();

    void add_sensor_data(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void visibility_cleanup();
    void update_map_from_raw();
    void update_map_center();

    void publish_pointcloud();

private:
    // Topic names
    std::string input_topic_;
    std::string output_topic_;

    // frames
    std::string map_frame_id_;
    std::string center_frame_id_;

    bool publish_latency_;

    // resolution
    double resolution_;
    double world_length_;
    double world_width_;

    // publish timer
    double publish_rate_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr visibility_cleanup_timer_;

    rclcpp::Time initial_time_;

    // grid map
    grid_map::GridMap raw_map_;
    grid_map::GridMap map_;
    std::mutex map_mutex_;
    std::mutex raw_map_mutex_;

    // parameters
    double min_variance_;
    double max_variance_;
    double mahalanobis_distance_threshold_;
    double multi_height_noise_;
    double scanning_duration_;
    bool enable_visibility_cleanup_;
    double visibility_cleanup_rate_;


    // Pub/Sub handles
    // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;

    // latency
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_publisher_;

    // TF buffer
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subscriber_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> tf_filter_;
};

} // namespace traverse_layer

#endif