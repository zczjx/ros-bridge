#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "video_encoder.hpp"

#include <queue>
#include <thread>

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
    std::string codec_name("h264_nvenc");
    m_encoder = std::make_shared<VideoEncoder>(codec_name);
    m_pub = create_publisher<sensor_msgs::msg::Image>("/carla/video_enc/image_h264", 10);
    m_encodeThread = std::make_unique<std::thread>([this]() { doEncode(); });
    m_pubThread = std::make_unique<std::thread>([this]() { doPublish(); });

    auto callback = [this](const std::shared_ptr<sensor_msgs::msg::Image> image){ subCallback(image); };
    m_sub = create_subscription<sensor_msgs::msg::Image>("/carla/ego_vehicle/rgb_view/image", 10, callback);
  }

  ~VideoEncNode();

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_sub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub;
  std::shared_ptr<VideoEncoder> m_encoder;

  std::queue<std::shared_ptr<sensor_msgs::msg::Image>> m_buffer;
  std::mutex m_bufferMutex;

  std::queue<std::shared_ptr<sensor_msgs::msg::Image>> m_encodeBuffer;
  std::mutex m_encodeBufferMutex;

  std::unique_ptr<std::thread> m_pubThread;
  std::unique_ptr<std::thread> m_encodeThread;
  std::atomic<bool> m_stopSignal;
  void doPublish();
  void doEncode();
  void subCallback(const std::shared_ptr<sensor_msgs::msg::Image> image);
};

void VideoEncNode::doEncode()
{
  RCLCPP_INFO(this->get_logger(), "video doEncode thread started");
  int pts_idx = 0;

  while (!m_stopSignal && rclcpp::ok())
  {
    std::shared_ptr<sensor_msgs::msg::Image> tmp_image;
    if(m_buffer.empty())
        continue;
    {
      std::lock_guard<std::mutex> lock(m_bufferMutex);
      // RCLCPP_INFO(this->get_logger(), "m_buffer.size: [%d]", m_buffer.size());
      tmp_image = m_buffer.front();
      m_buffer.pop();
    }
    auto image_msg = std::make_shared<sensor_msgs::msg::Image>(*tmp_image);
    // RCLCPP_INFO(this->get_logger(), "image_msg->data.size(): [%d]", image_msg->data.size());
    auto bgra_frame = m_encoder->fillinFrame(image_msg, pts_idx);
    m_encoder->encode(bgra_frame, m_encodeBuffer);
    pts_idx++;
  }

  //TODO: flush encode

}

void VideoEncNode::doPublish()
{
  RCLCPP_INFO(this->get_logger(), "video doPublish thread started");

  while (!m_stopSignal && rclcpp::ok())
  {
    if(m_encodeBuffer.empty())
      continue;

    auto h264_msg = m_encodeBuffer.front();
    m_encodeBuffer.pop();
    m_pub->publish(std::move(*h264_msg));
  }
}

void VideoEncNode::subCallback(const std::shared_ptr<sensor_msgs::msg::Image> image)
{
  std::lock_guard<std::mutex> lock(m_bufferMutex);
  m_buffer.push(image);
}

VideoEncNode::~VideoEncNode()
{
  if (m_encodeThread->joinable())
  {
    RCLCPP_INFO(this->get_logger(), "join m_encodeThread");
    m_stopSignal = true;
    m_encodeThread->join();
  }

  if (m_pubThread->joinable())
  {
    RCLCPP_INFO(this->get_logger(), "join m_pubThread");
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
