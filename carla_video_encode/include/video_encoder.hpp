#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <thread>

#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace video_enc_node
{

class VideoEncoder
{
public:
    VideoEncoder();

    int encode(std::shared_ptr<AVCodecContext> enc_ctx, std::shared_ptr<AVFrame> frame,
            std::shared_ptr<AVPacket> pkt);

    int encode(std::shared_ptr<AVCodecContext> enc_ctx, std::shared_ptr<AVFrame> frame,
            std::shared_ptr<AVPacket> pkt, std::shared_ptr<FILE> outfile);

    ~VideoEncoder();

private:
    const std::shared_ptr<AVCodec> m_codec;
    std::string m_codec_name;
    std::shared_ptr<AVCodecContext> m_ctx{nullptr};
    std::shared_ptr<AVFrame> m_frame;
    std::shared_ptr<AVPacket> m_pkt;
};

}