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
    m_pubThread = std::make_unique<std::thread>([this]() { nodeProcess(); });

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

  std::unique_ptr<std::thread> m_pubThread;
  std::atomic<bool> m_stopSignal;
  void nodeProcess();
  void subCallback(const std::shared_ptr<sensor_msgs::msg::Image> image);
};

void VideoEncNode::nodeProcess()
{
  RCLCPP_INFO(this->get_logger(), "video encode process thread started");

  int pts_idx = 0;
  std::string filename("carla_image.h264");
  std::shared_ptr<FILE> file_hdl(fopen(filename.c_str(), "wb"));

  if (nullptr == file_hdl)
  {
        std::cerr << "Could not open " << filename << std::endl;
        exit(1);
  }


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
    // image_msg->header.frame_id = "video_enc/image_h264";
    // image_msg->encoding = "h264";
    // m_pub->publish(std::move(image_msg));
    if (pts_idx < 1000)
    {
      auto bgra_frame = m_encoder->fillinFrame(image_msg, pts_idx);
      m_encoder->encode(bgra_frame, file_hdl);
      pts_idx++;
    }
    else if(1000 == pts_idx)
    {
      m_encoder->flushEncode(file_hdl);
      pts_idx++;
    }
  }

}

void VideoEncNode::subCallback(const std::shared_ptr<sensor_msgs::msg::Image> image)
{
  /*
  RCLCPP_INFO(this->get_logger(), "image->header.frame_id: [%s]", image->header.frame_id.c_str());
  RCLCPP_INFO(this->get_logger(), "image->encoding: [%s]", image->encoding.c_str());
  RCLCPP_INFO(this->get_logger(), "image->height: [%d]", image->height);
  RCLCPP_INFO(this->get_logger(), "image->width: [%d]", image->width);
  */
  std::lock_guard<std::mutex> lock(m_bufferMutex);
  m_buffer.push(image);
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
