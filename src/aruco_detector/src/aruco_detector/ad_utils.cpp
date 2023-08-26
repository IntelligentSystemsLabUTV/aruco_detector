/**
 * Aruco Detector node auxiliary functions.
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

#include <aruco_detector/aruco_detector.hpp>

namespace ArucoDetector
{

/**
 * @brief Converts a frame into an Image message.
 *
 * @param frame cv::Mat storing the frame.
 * @return Shared pointer to a new Image message.
 */
Image::SharedPtr ArucoDetectorNode::frame_to_msg(cv::Mat & frame)
{
  auto ros_image = std::make_shared<Image>();

  // Set frame-relevant image contents
  ros_image->set__width(frame.cols);
  ros_image->set__height(frame.rows);
  ros_image->set__encoding(sensor_msgs::image_encodings::BGR8);
  ros_image->set__step(frame.cols * frame.elemSize());

  // Check data endianness
  int num = 1;
  ros_image->set__is_bigendian(!(*(char *)&num == 1));

  // Copy frame data (this avoids the obsolete cv_bridge)
  size_t size = ros_image->step * frame.rows;
  ros_image->data.resize(size);
  memcpy(ros_image->data.data(), frame.data, size);

  return ros_image;
}

/**
 * @brief Function to round spatial measure to a fixed precision.
 *
 * @param num Value to round.
 * @param prec 100 -> centimeter, 1000 -> mm
 * @return Rounded value.
 */
float ArucoDetectorNode::round_space(float num, float prec)
{
  num *= prec;
  num = floor(num);
  num /= prec;

  return num;
}

/**
 * @brief Function to round angular measure to a fixed precision.
 *
 * @param num Value to round.
 * @param prec 10 -> tenth of a degree
 * @return Rounded value.
 */
float ArucoDetectorNode::round_angle(float num, float prec)
{
  num = num * 180.0 / M_PIf32;

  num *= prec;
  num = floor(num);
  num /= prec;

  return num * M_PIf32 / 180.0;
}

/**
 * @brief Function to convert Rodrigues' vector to quaternion.
 *
 * @param r Vector in Rodrigues' form (axis-angle).
 * @param target_pose pose msg to fill
 */
void ArucoDetectorNode::rodrToQuat(cv::Vec3d r, Pose & target_pose)
{
    double w, x, y, z;
    double angle = cv::norm(r);

    if (angle < 1e-8) {
        w = 1.0;
        x = y = z = 0.0;
    }
    else
    {
      double c = std::cos(angle / 2.0);
      double s = std::sin(angle / 2.0);

      w = c;
      x = s * r[0] / angle;
      y = s * r[1] / angle;
      z = s * r[2] / angle;
    }

    target_pose.orientation.set__w(w);
    target_pose.orientation.set__x(x);
    target_pose.orientation.set__y(y);
    target_pose.orientation.set__z(z);
}

/**
 * @brief Function to compute 2D and 3D square center.
 *
 * @param corners Vector with the four square vertices coordinates.
 * @param aruco_centers Vector to append new center point to.
 */
void ArucoDetectorNode::square_center_2d(std::vector<cv::Point2f> corners,
                                         std::vector<cv::Point>& aruco_centers)
{
  double x1 = corners[0].x;
  double y1 = corners[0].y;

  double x2 = corners[1].x;
  double y2 = corners[1].y;

  double x3 = corners[2].x;
  double y3 = corners[2].y;

  double x4 = corners[3].x;
  double y4 = corners[3].y;

  double xc_den = (-((x2 - x4) * (y1 - y3)) + (x1 - x3) * (y2 - y4));
  double yc_den = (-((x2 - x4) * (y1 - y3)) + (x1 - x3) * (y2 - y4));

  if ((abs(xc_den) < 1e-5) || (abs(yc_den) < 1e-5))
  {
    // Do not divide by zero! Just discard this sample
    return;
  }

  int xc =
    (x3 * x4 * (y1 - y2) + x1 * x4 * (y2 - y3) + x1 * x2 * (y3 - y4) + x2 * x3 * (-y1 + y4)) /
    xc_den;
  int yc =
    (x4 * y2 * (y1 - y3) + x1 * y2 * y3 - x2 * y1 * y4 - x1 * y3 * y4 + x2 * y3 * y4 + x3 * y1 *
    (-y2 + y4)) / yc_den;

  aruco_centers.push_back(cv::Point(xc, yc));
}

/**
 * @brief Thread routine.
 */
void ArucoDetectorNode::worker_thread_routine()
{
  while (true)
  {
    std_msgs::msg::Header header_;
    cv::Mat image_{};
    sem_wait(&sem2_);
    if (!running_.load(std::memory_order_acquire))
      break;
    image_ = new_frame_.clone();
    header_ = last_header_;
    sem_post(&sem1_);

    // Detect targets
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    // TODO: add other dictionaries
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

    // Set detector parameters
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    detectorParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    detector.detectMarkers(image_, markerCorners, markerIds);

    // Remove markers with IDs in the exclusion list
    for (int64_t id : excluded_ids)
    {
      auto iterId = std::remove(markerIds.begin(), markerIds.end(), id);
      auto iterCorn = markerCorners.begin() + std::distance(markerIds.begin(), iterId);

      // Remove elements from both arrays
      markerIds.erase(iterId, markerIds.end());
      markerCorners.erase(iterCorn, markerCorners.end());
    }

    // Return if no target is detected
    if (markerIds.size() == 0) continue;

    std::vector<cv::Vec3d> rvecs(markerIds.size()), tvecs(markerIds.size());

    // Publish information about detected targets
    aruco_centers_.clear();
    TargetArray target_array_msg{};

    for (int k = 0; k < int(markerIds.size()); k++)
    {
      // Compute Aruco center
      square_center_2d(markerCorners[k], aruco_centers_);

      // Calculate pose for each marker
      solvePnP(objPoints, markerCorners[k], cameraMatrix, distCoeffs, rvecs[k], tvecs[k]);

      // Prepare messages to be published
      // Populate TargetID
      TargetID target_id{};
      target_id.set__int_id(markerIds[k]);
      target_id.set__str_id(header_.frame_id);

      Pose target_pose{};
      target_pose.position.set__x(tvecs[k][0]);
      target_pose.position.set__y(tvecs[k][1]);
      target_pose.position.set__z(tvecs[k][2]);
      rodrToQuat(rvecs[k], target_pose);

      // Populate Target message
      Target target{};
      target.set__header(header_);
      target.set__target_id(target_id);
      target.set__pose(target_pose);

      target_array_msg.targets.push_back(target);
    }
    target_array_pub_->publish(target_array_msg);

    // Draw search output, ROI and HUD in another image
    cv::aruco::drawDetectedMarkers(image_, markerCorners, markerIds);

    // Draw axis for each marker
    for(int i = 0; i < int(markerIds.size()); i++)
      cv::drawFrameAxes(image_, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

    camera_frame_ = image_; // Doesn't copy image data, but sets data type...

    // cv::Point rect_p1(
    //   (camera_frame_.size().width / 2) - (centering_width / 2),
    //   (camera_frame_.size().height / 2) - (centering_width / 2));
    // cv::Point rect_p2(
    //   (camera_frame_.size().width / 2) + (centering_width / 2),
    //   (camera_frame_.size().height / 2) + (centering_width / 2));

    // cv::Point crosshair_p(
    //   camera_frame_.size().width / 2,
    //   camera_frame_.size().height / 2);

    // cv::rectangle(
    //   camera_frame_,
    //   rect_p1,
    //   rect_p2,
    //   cv::Scalar(0, 255, 0),
    //   5);
    // cv::drawMarker(
    //   camera_frame_,
    //   crosshair_p,
    //   cv::Scalar(0, 255, 0),
    //   cv::MARKER_CROSS,
    //   15,
    //   3);

    // Publish rate message and processed image
    Empty rate_msg{};
    camera_rate_pub_->publish(rate_msg);

    Image::SharedPtr processed_image_msg = frame_to_msg(camera_frame_);
    processed_image_msg->set__header(header_);
    stream_pub_->publish(processed_image_msg);
  }

  RCLCPP_WARN(this->get_logger(), "Thread stopped");
}


} // namespace ArucoDetector
