#ifndef TRAVERSE_LAYER_HPP_
#define TRAVERSE_LAYER_HPP_

#include <grid_map_ros/grid_map_ros.hpp>
#include <filters/filter_chain.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace traverse_layer
{

class TraverseLayer : public rclcpp::Node
{
public:
    TraverseLayer();
    virtual ~TraverseLayer();

    bool readParameters();

    void callback(const grid_map_msgs::msg::GridMap::SharedPtr msg);

private:
    // Topic names
    std::string input_topic_;
    std::string output_topic_;

    // Pub/Sub handles
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscriber_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;

    // Filter chain
    filters::FilterChain<grid_map::GridMap> filter_chain_;

    // Filter chain parameters name.
    std::string filter_chain_parameter_name_;
};

} // namespace traverse_layer

#endif