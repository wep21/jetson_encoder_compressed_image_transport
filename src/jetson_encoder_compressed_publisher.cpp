// Copyright 2022 Daisuke Nishimatsu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "nppi_color_conversion.h"

#include "jetson_encoder_compressed_image_transport/jetson_encoder_compression_common.hpp"
#include "jetson_encoder_compressed_image_transport/jetson_encoder_compressed_publisher.hpp"
#include "jetson_encoder_compressed_image_transport/color_space.hpp"


#include <sensor_msgs/image_encodings.hpp>

#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/parameter_client.hpp>

#include <sstream>
#include <vector>

namespace jetson_encoder_compressed_image_transport
{

namespace enc = sensor_msgs::image_encodings;

void JetsonEncoderCompressedPublisher::advertiseImpl(
  rclcpp::Node * node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos)
{
  node_ = node;
  using Base = image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage>;
  Base::advertiseImpl(node, base_topic, custom_qos);

  uint ns_len = node->get_effective_namespace().length();
  std::string param_base_name = base_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');

  jpeg_quality_param_name_ = param_base_name + ".jpeg_quality";
  rcl_interfaces::msg::ParameterDescriptor jpeg_quality_description;
  jpeg_quality_description.name = "jpeg_quality";
  jpeg_quality_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  jpeg_quality_description.description = "Image quality for JPEG format";
  jpeg_quality_description.read_only = false;
  rcl_interfaces::msg::IntegerRange jpeg_range;
  jpeg_range.from_value = 1;
  jpeg_range.to_value = 100;
  jpeg_range.step = 1;
  jpeg_quality_description.integer_range.push_back(jpeg_range);
  try {
    config_.jpeg_quality = node->declare_parameter(
      jpeg_quality_param_name_, DEFAULT_JPEG_QUALITY, jpeg_quality_description);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", jpeg_quality_param_name_.c_str());
    config_.jpeg_quality = node->get_parameter(jpeg_quality_param_name_).get_value<int64_t>();
  }
}

void JetsonEncoderCompressedPublisher::publish(
  const sensor_msgs::msg::Image & message,
  const PublishFn & publish_fn) const
{
  // Compressed image message
  sensor_msgs::msg::CompressedImage compressed;
  compressed.header = message.header;
  compressed.format = message.encoding;

  // Bit depth of image encoding
  int bitDepth = enc::bitDepth(message.encoding);

  // Update ros message format header
  compressed.format += "; jpeg compressed ";

  // Check input format

  if ((bitDepth == 8) || (bitDepth == 16)) {
    if (message.encoding != "rgb8" && message.encoding != "bgr8") {
      RCLCPP_ERROR(
        logger_,
        "%s is invalid encording format! Not supportted encording type.",
        message.encoding.c_str()
      );
      return;
    } else {
      compressed.format += "bgr8";
    }

    if (image_size_ < message.data.size()) {
      dev_image_ = cuda::memory::device::make_unique<uint8_t[]>(message.data.size());
      yuv_size_ = message.width * message.height +
        (static_cast<size_t>(message.width / 2) * static_cast<size_t>(message.height / 2)) * 2;
      host_yuv_ = cuda::memory::host::make_unique<uint8_t[]>(yuv_size_);
      dev_yuv_ = cuda::memory::device::make_unique<uint8_t[]>(yuv_size_);
      image_size_ = message.data.size();
    }

    cuda::memory::copy(dev_image_.get(), &message.data[0], message.data.size());

    if (message.encoding == "rgb8") {
      if (cudaRGB8ToBGR8(
          dev_image_.get(), message.width, message.height, message.step) != cudaSuccess)
      {
        RCLCPP_ERROR(logger_, "failed to convert rgb8 to bgr8");
        return;
      }
    }

    if (cudaBGRToYUV420(
      dev_image_.get(), dev_yuv_.get(), message.width, message.height) != cudaSuccess)
    {
      RCLCPP_ERROR(logger_, "failed to convert bgr8 to yuv420");
      return;
    }

    cuda::memory::copy(host_yuv_.get(), dev_yuv_.get(), yuv_size_);

    NvBuffer buffer(V4L2_PIX_FMT_YUV420M, message.width, message.height, 0);
    auto ret = buffer.allocateMemory();
    if (ret != 0){
      RCLCPP_ERROR(logger_, "NvBuffer allocation failed");
      return;
    }

    auto image_data = reinterpret_cast<int8_t *>(host_yuv_.get());
    for (uint32_t i = 0; i < buffer.n_planes; ++i)
    {
        NvBuffer::NvBufferPlane & plane = buffer.planes[i];
        plane.bytesused = plane.fmt.stride * plane.fmt.height;
        memcpy(plane.data, image_data, plane.bytesused);
        image_data += plane.bytesused;
    }

    uint64_t out_buf_size = message.width * message.height * 3 / 2;
    compressed.data.resize(out_buf_size);
    auto out_data = compressed.data.data();
    ret = encoder_->encodeFromBuffer(
      buffer, JCS_YCbCr, &out_data, out_buf_size, config_.jpeg_quality
    );
    if (ret != 0){
      RCLCPP_ERROR(logger_, "NvJpeg Encoder Error");
      return;
    }
    buffer.deallocateMemory();

    // Publish message
    publish_fn(compressed);
  } else {
    RCLCPP_ERROR(
      logger_,
      "Compressed Image Transport - JPEG compression requires 8/16-bit color format (input format is: %s)",
      message.encoding.c_str());
  }
}
} // namespace jetson_encoder_compressed_image_transport
