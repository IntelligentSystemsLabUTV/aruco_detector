/**
 * Aruco Detector node definition.
 *
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Roberto Masocco <robmasocco@gmail.com>
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

#include <vector>

#include <dua_node/dua_node.hpp>
#include <dua_qos/dua_qos.hpp>
#include <dua_interfaces/msg/target.hpp>
#include <dua_interfaces/msg/target_array.hpp>
#include <dua_interfaces/msg/target_id.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <theora_wrappers/publisher.hpp>

using namespace dua_interfaces::msg;
using namespace geometry_msgs::msg;
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

  /* image_transport subscriptions */
  image_transport::CameraSubscriber camera_sub_;

  /* Topic subscriptions callbacks */
  void camera_callback(const Image::ConstSharedPtr & msg,
                       const CameraInfo::ConstSharedPtr & camera_info_msg);

  /* Topic publishers */
  rclcpp::Publisher<Empty>::SharedPtr camera_rate_pub_;
  rclcpp::Publisher<TargetArray>::SharedPtr target_array_pub_;

  /* image_transport publishers */
  image_transport::Publisher target_img_pub_;

  /* Theora stream publishers. */
  std::shared_ptr<TheoraWrappers::Publisher> stream_pub_;

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
  bool is_on_ = false;
  bool get_calibration_params_ = true;
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;

  /* Node parameters */
  double aruco_side = 0.0;
  bool autostart = false;
  bool best_effort_sub_qos = false;
  int64_t centering_width = 0;
  int64_t image_sub_depth = 0;
  std::string input_topic = "";
  std::string output_topic = "";
  std::string stream_topic = "";
  std::string transport = "";

  /* Synchronization primitives for internal update operations */
  std::mutex pose_lock_;
  std::mutex camera_info_lock_;

  /* Utility routines */
  Image::SharedPtr frame_to_msg(cv::Mat & frame);
  float round_angle(float num, float prec);
  float round_space(float num, float prec);
  void rodrToQuat(cv::Vec3d r, Pose & target_pose);
};

} // namespace ArucoDetector

#endif // ARUCO_DETECTOR_HPP
