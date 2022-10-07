#include "video_encode.hpp"

namespace video_enc_node
{

VideoEncoder::VideoEncoder()
{
    ;
}

int VideoEncoder::encode(std::shared_ptr<AVCodecContext> enc_ctx, std::shared_ptr<AVFrame> frame,
        std::shared_ptr<AVPacket> pkt)
{

}

int VideoEncoder::encode(std::shared_ptr<AVCodecContext> enc_ctx, std::shared_ptr<AVFrame> frame,
            std::shared_ptr<AVPacket> pkt, std::shared_ptr<FILE> outfile)
{

}

VideoEncoder::~VideoEncoder()
{
}

}