#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <unistd.h>

namespace video_enc_node
{

class VideoEncNode : public rclcpp::Node
{
public:

  VideoEncNode()
  : Node("video_enc_node")
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    auto callback =
      [this](const std::shared_ptr<sensor_msgs::msg::Image> image) -> void
      {
        RCLCPP_INFO(this->get_logger(), "image->header.frame_id: [%s]", image->header.frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "image->encoding: [%s]", image->encoding.c_str());
        RCLCPP_INFO(this->get_logger(), "image->height: [%d]", image->height);
        RCLCPP_INFO(this->get_logger(), "image->width: [%d]", image->width);
      };
    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.
    m_sub = create_subscription<sensor_msgs::msg::Image>("/carla/ego_vehicle/rgb_view/image", rclcpp::SensorDataQoS(), callback);
  }

private:
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> m_sub;
};

}


int main(int argc, char **argv)
{
    //init client
    rclcpp::init(argc, argv);
    //create new node

    rclcpp::spin(std::make_shared<video_enc_node::VideoEncNode>());

    //shutdown
    rclcpp::shutdown();

    return 0;
}
