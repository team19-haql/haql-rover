#ifndef POINTCLOUD_TO_GRIDMAP_HPP_
#define POINTCLOUD_TO_GRIDMAP_HPP_

#include <grid_map_ros/grid_map_ros.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>

namespace traverse_layer
{

class PointcloudToGridmap : public rclcpp::Node
{
public:
    PointcloudToGridmap();
    virtual ~PointcloudToGridmap();

    bool read_parameters();

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void timer_callback();

private:
    // Topic names
    std::string input_topic_;
    std::string output_topic_;

    // frames
    std::string map_frame_id_;

    double process_noise_;

    // resolution
    double resolution_;
    double world_length_;
    double world_width_;

    // publish timer
    double publish_rate_;
    rclcpp::TimerBase::SharedPtr publish_timer_;


    // grid map
    grid_map::GridMap map_;


    // Pub/Sub handles
    // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;

    // TF buffer
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subscriber_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> tf_filter_;
};

} // namespace traverse_layer

#endif