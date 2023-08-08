/**
 * Aruco Detector node implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * August 7, 2023
 */

/**
 * This is free software.
 * You can redistribute it and/or modify this file under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 3 of the License, or (at your option) any later
 * version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this file; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <string>
#include <stdexcept>

#include <aruco_detector/aruco_detector.hpp>

namespace ArucoDetector
{

/**
 * @brief Builds a new Aruco Detector node.
 *
 * @param opts Node options.
 *
 * @throws RuntimeError
 */
ArucoDetectorNode::ArucoDetectorNode(const rclcpp::NodeOptions & node_options)
: NodeBase("aruco_detector", node_options, true)
{
  // Initialize callback groups
  init_cgroups();

  // Initialize node parameters
  init_parameters();

  // Initialize topic publishers
  init_publishers();

  // Initialize topic subscriptions
  init_subscriptions();

  // Initialize services
  init_services();

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Finalizes node operation.
 */
ArucoDetectorNode::~ArucoDetectorNode()
{
  // Unsubscribe from image topics
  if (is_on_) {
    camera_sub_.shutdown();
    is_on_ = false;
  }
  target_img_pub_.shutdown();
}

/**
 * @brief Routine to initialize callback groups.
 */
void ArucoDetectorNode::init_cgroups()
{
  pose_cgroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  enable_cgroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

/**
 * @brief Routine to initialize topic subscriptions.
 */
void ArucoDetectorNode::init_subscriptions()
{
  // Drone pose
  // auto pose_sub_opts = rclcpp::SubscriptionOptions();
  // pose_sub_opts.callback_group = pose_cgroup_;
  // pose_sub_ = this->create_subscription<Pose>(
  //   "/flight_control/pose",
  //   rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(FlightControl::fc_pose_qos_profile)),
  //   std::bind(
  //     &ArucoDetectorNode::pose_callback,
  //     this,
  //     std::placeholders::_1),
  //   pose_sub_opts);
}

/**
 * @brief Routine to initialize topic publishers.
 */
void ArucoDetectorNode::init_publishers()
{
  // Camera rate
  camera_rate_pub_ = this->create_publisher<Empty>(
    "~/camera_rate",
    DUAQoS::get_datum_qos());

  // Target data
  // target_pub_ = this->create_publisher<Target>(
  //   "/targets",
  //   rmw_qos_profile_sensor_data);

  // Target images
  target_img_pub_ = image_transport::create_publisher(
    this,
    "~/targets/image_rect_color",
    rmw_qos_profile_sensor_data); // FIXME: use DUAQoS::get_datum_qos() instead
}

/**
 * @brief Routine to initialize service servers.
 */
void ArucoDetectorNode::init_services()
{
  // Enable
  enable_server_ = this->create_service<SetBool>(
    "~/enable",
    std::bind(
      &ArucoDetectorNode::enable_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    enable_cgroup_);
}

} // namespace ArucoDetector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ArucoDetector::ArucoDetectorNode)
