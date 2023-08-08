/**
 * Aruco Detector node definition.
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

#ifndef ARUCO_DETECTOR_HPP
#define ARUCO_DETECTOR_HPP

#include <memory>
#include <opencv2/aruco.hpp>
// #include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <pthread.h>
#include <vector>

#include <dua_node/dua_node.hpp>
#include <dua_qos/dua_qos.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>

// #include <stanis_interfaces/msg/pose.hpp>
// #include <stanis_interfaces/msg/target.hpp>

using namespace rcl_interfaces::msg;
using namespace sensor_msgs::msg;
using namespace std_msgs::msg;
using namespace std_srvs::srv;

namespace ArucoDetector
{

/**
 * Main target detection node.
 */
class ArucoDetectorNode : public DUANode::NodeBase
{
public:
  ArucoDetectorNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  ~ArucoDetectorNode();

private:
  /* Node initialization routines */
  void init_cgroups();
  void init_parameters();
  void init_publishers();
  void init_services();
  void init_subscriptions();

  /* Topic subscriptions callback groups */
  rclcpp::CallbackGroup::SharedPtr pose_cgroup_;

  /* Topic subscriptions */
  // rclcpp::Subscription<Pose>::SharedPtr pose_sub_;

  /* image_transport subscriptions */
  image_transport::Subscriber camera_sub_;

  /* Topic subscriptions callbacks */
  void camera_callback(const Image::ConstSharedPtr & msg);
  // void pose_callback(const Pose::SharedPtr msg);

  /* Topic publishers */
  rclcpp::Publisher<Empty>::SharedPtr camera_rate_pub_;
  // rclcpp::Publisher<Target>::SharedPtr target_pub_;

  /* image_transport publishers */
  image_transport::Publisher target_img_pub_;

  /* Service servers callback groups */
  rclcpp::CallbackGroup::SharedPtr enable_cgroup_;

  /* Service servers */
  rclcpp::Service<SetBool>::SharedPtr enable_server_;

  /* Service callbacks */
  void enable_callback(
    SetBool::Request::SharedPtr req,
    SetBool::Response::SharedPtr resp);

  /* Data buffers */
  cv::Mat camera_frame_;
  std::vector<cv::Point> aruco_centers_;

  /* Internal state variables */
  uint8_t camera_id_ = 0;
  bool is_on_ = false;

  /* Node parameters */
  double aruco_side = 0.0;
  std::string base_frame = "";
  bool best_effort_sub_qos = false;
  double camera_offset = 0.0;
  std::string camera_topic = "";
  int64_t centering_width = 0;
  bool compute_position = false;
  double error_min = 0.0;
  int64_t focal_length = 0;
  int64_t image_sub_depth = 0;
  std::vector<std::string> input_topics = {""};
  std::string node_namespace = "";
  std::string output_topic = "";
  bool rotate_image = false;
  std::string transport = "";

  /* Synchronization primitives for internal update operations */
  std::mutex pose_lock_;

  /* Utility routines */
  Image::SharedPtr frame_to_msg(cv::Mat & frame);
  float round_angle(float num, float prec);
  float round_space(float num, float prec);
};

} // namespace ArucoDetector

#endif // ARUCO_DETECTOR_HPP