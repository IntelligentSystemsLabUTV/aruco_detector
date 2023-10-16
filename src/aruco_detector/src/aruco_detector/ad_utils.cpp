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
 * @brief Function to convert Rodrigues' vector to quaternion.
 *
 * @param r Vector in Rodrigues' form (axis-angle).
 * @param target_pose Pose message to fill.
 */
void ArucoDetectorNode::rodrToQuat(cv::Vec3d r, Pose & target_pose)
{
  double w, x, y, z;
  double angle = cv::norm(r);

  if (angle < 1e-8) {
    w = 1.0;
    x = y = z = 0.0;
  } else {
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
void ArucoDetectorNode::square_center_2d(
  std::vector<cv::Point2f> corners,
  std::vector<cv::Point> & aruco_centers)
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

  if ((abs(xc_den) < 1e-5) || (abs(yc_den) < 1e-5)) {
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

} // namespace ArucoDetector
