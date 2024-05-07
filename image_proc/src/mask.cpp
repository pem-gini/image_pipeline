// Copyright 2017, 2019 Kentaro Wada, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <functional>
#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "tracetools_image_pipeline/tracetools.h"

#include <image_proc/mask.hpp>
#include <image_proc/utils.hpp>

#include <image_transport/image_transport.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace image_proc {

MaskNode::MaskNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("MaskNode", options)
{
    auto qos_profile = getTopicQosProfile(this, "image/image_raw");
    // Create image pub
    pub_image_ = image_transport::create_camera_publisher(
        this, "out/image_raw", qos_profile);
    // Create image sub
    sub_mask_ = image_transport::create_camera_subscription(
        this,
        "mask/image_raw",
        std::bind(
            &MaskNode::maskCb,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        "raw",
        qos_profile);
    sub_image_ = image_transport::create_camera_subscription(
        this,
        "image/image_raw",
        std::bind(
            &MaskNode::imageCb,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        "raw",
        qos_profile);
}

void MaskNode::imageCb(
    sensor_msgs::msg::Image::ConstSharedPtr image_msg,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
{
    if (!mask_image_received_)
        return;

    sensor_msgs::msg::Image msg = *image_msg; 
    cv_bridge::CvImagePtr cv_input_image;
    try {
        if (msg.encoding.compare("16UC1") == 0) {
            msg.encoding = "mono16";
            cv_input_image
                = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        } else {
            cv_input_image
                = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
        }
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception 2: %s", e.what());
        return;
    }

    cv::Mat input_image_8bit = cv_input_image->image;
    // RCLCPP_ERROR(
    //     this->get_logger(),
    //     "AAAAA:       %d ::: %d",
    //     input_image_8bit.depth(),
    //     mask_image_.depth());

    /// resize mask image
    cv::Mat resizedMask;
    cv::resize(mask_image_, resizedMask, cv::Size(input_image_8bit.cols, input_image_8bit.rows));

    /// do masking
    cv::Mat masked_image;
    cv::bitwise_and(input_image_8bit, resizedMask, masked_image);

    // Publish masked image
    cv::Mat mono16_image;
    masked_image.convertTo(mono16_image, CV_16U, 65535.0 / 255.0); // Scaling from [0, 255] to [0, 65535]
    cv_bridge::CvImage new_cv_image(std_msgs::msg::Header(), "mono16", mono16_image); // "mono16" is the encoding for MONO16 images
    sensor_msgs::msg::Image::SharedPtr masked_msg = new_cv_image.toImageMsg();
    masked_msg->encoding = "16UC1";

    sensor_msgs::msg::CameraInfo::SharedPtr dst_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(*info_msg);
    masked_msg->header = dst_info_msg->header;

    pub_image_.publish(*masked_msg, *dst_info_msg);
}

void MaskNode::maskCb(
    const sensor_msgs::msg::Image::ConstSharedPtr image_msg,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr /*info_msg*/)
{
    cv_bridge::CvImagePtr cv_mask_image;
    try {
        cv_mask_image = cv_bridge::toCvCopy(
            image_msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception 1: %s", e.what());
        return;
    }

    /// cv_mask_image is a gray image now, make it full black / white for perfect masking
    cv::Mat black_and_white_image;
    cv::threshold(cv_mask_image->image, black_and_white_image, 100, 255, cv::THRESH_BINARY);


    mask_image_ = black_and_white_image;
    mask_image_received_ = true;
}

} // namespace image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(image_proc::MaskNode)
