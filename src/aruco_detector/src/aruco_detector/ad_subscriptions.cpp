/**
 * Aruco Detector node topic subscription callbacks.
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
 * @brief Searches targets in a new image.
 *
 * @param msg Image message to parse.
 */
void ArucoDetectorNode::camera_callback(const Image::ConstSharedPtr & msg,
                                        const CameraInfo::ConstSharedPtr & camera_info_msg)
{
  // Get camera parameters
  if (get_calibration_params_)
  {
    cameraMatrix = cv::Mat(3, 3, cv::DataType<double>::type);
    distCoeffs = cv::Mat(1, 5, cv::DataType<double>::type);

    for (size_t i = 0; i < 3; i++)
      for (size_t j = 0; j < 3; j++)
        cameraMatrix.at<double>(i, j) = camera_info_msg->k[i*3+j];

    for (size_t i = 0; i < 5; i++)
      distCoeffs.at<double>(0, i) = camera_info_msg->d[i];

    // Set coordinate system
    objPoints = cv::Mat(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-aruco_side/2.f,  aruco_side/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f( aruco_side/2.f,  aruco_side/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f( aruco_side/2.f, -aruco_side/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-aruco_side/2.f, -aruco_side/2.f, 0);

    get_calibration_params_ = false;
  }

  // Convert msg to OpenCV image
  cv::Mat new_frame(
    msg->height,
    msg->width,
    CV_8UC3,
    (void *)(msg->data.data()));

  // Detect targets
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners;
  // TODO: add other dictionaries
  cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
  cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
  cv::aruco::ArucoDetector detector(dictionary, detectorParams);
  detector.detectMarkers(new_frame, markerCorners, markerIds);

  // Remove markers with IDs in the exclusion list
  for (int64_t id : excluded_ids)
  {
    auto iterId = std::remove(markerIds.begin(), markerIds.end(), id);
    auto iterCorn = markerCorners.begin() + std::distance(markerIds.begin(), iterId);

    // Rimuovi il valore da A e il corrispondente da B
    markerIds.erase(iterId, markerIds.end());
    markerCorners.erase(iterCorn, markerCorners.end());
  }

  // Return if no target is detected
  if (markerIds.size() == 0) return;

  std::vector<cv::Vec3d> rvecs(markerIds.size()), tvecs(markerIds.size());

  // Publish information about detected targets
  aruco_centers_.clear();
  TargetArray target_array_msg{};

  for (int k = 0; k < int(markerIds.size()); k++)
  {
    // Calculate pose for each marker
    solvePnP(objPoints, markerCorners.at(k), cameraMatrix, distCoeffs, rvecs.at(k), tvecs.at(k));

    // Populate TargetID
    TargetID target_id{};
    target_id.set__int_id(markerIds[k]);
    target_id.set__str_id(msg->header.frame_id);

    Pose target_pose{};
    target_pose.position.set__x(tvecs[k][0]);
    target_pose.position.set__y(tvecs[k][1]);
    target_pose.position.set__z(tvecs[k][2]);
    rodrToQuat(rvecs[k], target_pose);

    // Populate Target message
    Target target{};
    target.set__header(msg->header);
    target.set__target_id(target_id);
    target.set__pose(target_pose);

    target_array_msg.targets.push_back(target);

    // Detect target center
    double x1 = markerCorners[k][0].x;
    double y1 = markerCorners[k][0].y;

    double x2 = markerCorners[k][1].x;
    double y2 = markerCorners[k][1].y;

    double x3 = markerCorners[k][2].x;
    double y3 = markerCorners[k][2].y;

    double x4 = markerCorners[k][3].x;
    double y4 = markerCorners[k][3].y;

    double xc_den = (-((x2 - x4) * (y1 - y3)) + (x1 - x3) * (y2 - y4));
    double yc_den = (-((x2 - x4) * (y1 - y3)) + (x1 - x3) * (y2 - y4));

    if ((abs(xc_den) < 1e-5) || (abs(yc_den) < 1e-5))
    {
      // Do not divide by zero! Just discard this sample
      continue;
    }

    int xc =
      (x3 * x4 * (y1 - y2) + x1 * x4 * (y2 - y3) + x1 * x2 * (y3 - y4) + x2 * x3 * (-y1 + y4)) /
      xc_den;
    int yc =
      (x4 * y2 * (y1 - y3) + x1 * y2 * y3 - x2 * y1 * y4 - x1 * y3 * y4 + x2 * y3 * y4 + x3 * y1 *
      (-y2 + y4)) / yc_den;

    aruco_centers_.push_back(cv::Point(xc, yc));
  }
  target_array_pub_->publish(target_array_msg);

  // Draw search output, ROI and HUD in another image
  cv::aruco::drawDetectedMarkers(new_frame, markerCorners, markerIds);

  // Draw axis for each marker
  for(int i = 0; i < int(markerIds.size()); i++)
    cv::drawFrameAxes(new_frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

  camera_frame_ = new_frame; // Doesn't copy image data, but sets data type...

  cv::Point rect_p1(
    (camera_frame_.size().width / 2) - (centering_width / 2),
    (camera_frame_.size().height / 2) - (centering_width / 2));
  cv::Point rect_p2(
    (camera_frame_.size().width / 2) + (centering_width / 2),
    (camera_frame_.size().height / 2) + (centering_width / 2));

  cv::Point crosshair_p(
    camera_frame_.size().width / 2,
    camera_frame_.size().height / 2);

  cv::rectangle(
    camera_frame_,
    rect_p1,
    rect_p2,
    cv::Scalar(0, 255, 0),
    5);
  cv::drawMarker(
    camera_frame_,
    crosshair_p,
    cv::Scalar(0, 255, 0),
    cv::MARKER_CROSS,
    15,
    3);

  // Publish rate message and processed image
  Empty rate_msg{};
  camera_rate_pub_->publish(rate_msg);

  Image::SharedPtr processed_image_msg = frame_to_msg(camera_frame_);
  processed_image_msg->set__header(msg->header);
  stream_pub_->publish(processed_image_msg);
}

} // namespace ArucoDetector
