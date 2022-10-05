#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <queue>
#include <thread>

#include <unistd.h>

namespace video_enc_node
{

class VideoEncNode : public rclcpp::Node
{
public:

  VideoEncNode()
  : Node("video_enc_node"), m_stopSignal(false)
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    m_pub = create_publisher<sensor_msgs::msg::Image>("/carla/video_enc/image_h264", 10);
    m_pubThread = std::make_unique<std::thread>([this]() { nodeProcess(); });

    auto callback =
      [this](const std::shared_ptr<sensor_msgs::msg::Image> image) -> void
      {
        RCLCPP_INFO(this->get_logger(), "image->header.frame_id: [%s]", image->header.frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "image->encoding: [%s]", image->encoding.c_str());
        RCLCPP_INFO(this->get_logger(), "image->height: [%d]", image->height);
        RCLCPP_INFO(this->get_logger(), "image->width: [%d]", image->width);
        m_buffer.push(image);
      };
    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.
    m_sub = create_subscription<sensor_msgs::msg::Image>("/carla/ego_vehicle/rgb_view/image", 10, callback);
  }

  ~VideoEncNode();

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_sub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub;

  std::queue<std::shared_ptr<sensor_msgs::msg::Image>> m_buffer;
  std::mutex m_bufferMutex;

  std::unique_ptr<std::thread> m_pubThread;
  std::atomic<bool> m_stopSignal;
  void nodeProcess();
};

void VideoEncNode::nodeProcess()
{
  RCLCPP_INFO(this->get_logger(), "video encode process thread started");

  while (!m_stopSignal && rclcpp::ok())
  {
    if(m_buffer.empty())
      continue;

    auto tmp_image = m_buffer.front();
    auto image_msg = std::make_unique<sensor_msgs::msg::Image>(*tmp_image);
    image_msg->header.frame_id = "video_enc/image_h264";
    image_msg->encoding = "h264";
    m_pub->publish(std::move(image_msg));
    m_buffer.pop();
  }

}

VideoEncNode::~VideoEncNode()
{
  if (m_pubThread->joinable())
  {
    m_stopSignal = true;
    m_pubThread->join();
  }
}

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
