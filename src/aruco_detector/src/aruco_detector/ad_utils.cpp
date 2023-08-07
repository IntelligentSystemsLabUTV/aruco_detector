/**
 * Aruco Detector node auxiliary functions.
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

} // namespace ArucoDetector
