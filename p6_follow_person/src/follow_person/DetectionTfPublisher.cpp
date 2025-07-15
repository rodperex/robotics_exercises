// Copyright 2024 Intelligent Robotics Lab
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

#include "follow_person/DetectionTfPublisher.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace follow_person
{

DetectionTfPublisher::DetectionTfPublisher()
: Node("detection_tf_publisher"),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  detection3d_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
    "output_detection_3d",
    rclcpp::SensorDataQoS().reliable(),
    std::bind(&DetectionTfPublisher::detection_callback, this, _1));

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  odom2target_msg_.header.frame_id = "odom";
  odom2target_msg_.child_frame_id = "target";
}


void
DetectionTfPublisher::publish_odom2target(geometry_msgs::msg::TransformStamped bf2target_msg)
{
  tf2::Stamped<tf2::Transform> odom2bf;
  tf2::Stamped<tf2::Transform> bf2target;
  tf2::Stamped<tf2::Transform> odom2target;

  std::string error;
  if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {

    auto odom2bf_msg = tf_buffer_.lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);

    tf2::fromMsg(odom2bf_msg, odom2bf);
    tf2::fromMsg(bf2target_msg, bf2target);
    auto odom2target = odom2bf * bf2target;

    odom2target_msg_.header.stamp = now();
    odom2target_msg_.transform = tf2::toMsg(odom2target);
    tf_broadcaster_->sendTransform(odom2target_msg_);

    RCLCPP_DEBUG(
      get_logger(),
      "target at: x = %f, y = %f, z = %f",
      odom2target_msg_.transform.translation.x,
      odom2target_msg_.transform.translation.y,
      odom2target_msg_.transform.translation.z);

  } else {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Error in TF odom -> base_footprint [<< " << error << "]");
  }
}


void
DetectionTfPublisher::detection_callback(
  const vision_msgs::msg::Detection3DArray::SharedPtr detections)
{
  double x_pos, y_pos, z_pos;
  bool tf_exists = false;

  for (const auto & detection : detections->detections) {
    if (detection.results[0].hypothesis.class_id == "person") {
      x_pos = detection.bbox.center.position.x;
      y_pos = detection.bbox.center.position.y;
      z_pos = detection.bbox.center.position.z;
      tf_exists = true;
    }
  }

  if (tf_exists) {
    geometry_msgs::msg::TransformStamped bf2target;
    bf2target.header.stamp = now();

    bf2target.transform.translation.x = z_pos;
    bf2target.transform.translation.y = -x_pos;
    bf2target.transform.translation.z = y_pos;

    publish_odom2target(bf2target);
  }
}

}  //  namespace follow_person
