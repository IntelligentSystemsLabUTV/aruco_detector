/**
 * Aruco Detector node implementation.
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
  // Initialize atomic members
  init_atomics();

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
  if (running_.load(std::memory_order_acquire))
  {
    // Deactivate thread
    running_.store(false, std::memory_order_release);
    sem_post(&sem1_);
    sem_post(&sem2_);
    worker_.join();

    // Shutdown camera subscriber
    camera_sub_->shutdown();
    camera_sub_.reset();

    // Destroy semaphores
    sem_destroy(&sem1_);
    sem_destroy(&sem2_);

  }
  target_img_pub_->shutdown();
  target_img_pub_.reset();
  stream_pub_.reset();
}

/**
 * @brief Routine to initialize atomic members.
 */
void ArucoDetectorNode::init_atomics()
{
  running_.store(false, std::memory_order_release);
}

/**
 * @brief Routine to initialize topic subscriptions.
 */
void ArucoDetectorNode::init_subscriptions()
{
  if (autostart)
  {
    // Initialize semaphores
    sem_init(&sem1_, 0, 1);
    sem_init(&sem2_, 0, 0);

    // Spawn camera thread
    running_.store(true, std::memory_order_release);
    worker_ = std::thread(
      &ArucoDetectorNode::worker_thread_routine,
      this);
    if (worker_cpu != -1) {
      cpu_set_t worker_cpu_set;
      CPU_ZERO(&worker_cpu_set);
      CPU_SET(worker_cpu, &worker_cpu_set);
      if (pthread_setaffinity_np(
          worker_.native_handle(),
          sizeof(cpu_set_t),
          &worker_cpu_set))
      {
        char err_msg_buf[100] = {};
        char * err_msg = strerror_r(errno, err_msg_buf, 100);
        throw std::runtime_error(
                "ArucoDetectorNode::init_subscriptions: Failed to configure worker thread: " +
                std::string(err_msg));
      }
    }

    // Subscribe to image topic
    camera_sub_ = std::make_shared<image_transport::CameraSubscriber>(
      image_transport::create_camera_subscription(
        this,
        input_topic,
        std::bind(
          &ArucoDetectorNode::camera_callback,
          this,
          std::placeholders::_1,
          std::placeholders::_2),
        transport,
        best_effort_sub_qos ?
          DUAQoS::Visualization::get_image_qos(image_sub_depth).get_rmw_qos_profile() :
          DUAQoS::get_image_qos(image_sub_depth).get_rmw_qos_profile()));
  }
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

  // Targets data
  target_array_pub_ = this->create_publisher<TargetArray>(
    output_topic,
    DUAQoS::get_datum_qos());

  // Theora publisher
  stream_pub_ = std::make_shared<TheoraWrappers::Publisher>(
    this,
    stream_topic,
    DUAQoS::Visualization::get_image_qos(image_sub_depth).get_rmw_qos_profile());
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
    rmw_qos_profile_services_default);
}

} // namespace ArucoDetector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ArucoDetector::ArucoDetectorNode)
