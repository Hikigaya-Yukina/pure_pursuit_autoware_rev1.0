// Copyright 2020 Tier IV, Inc.
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
/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pure_pursuit/pure_pursuit.hpp"

#include "pure_pursuit/util/planning_utils.hpp"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace pure_pursuit
{
bool PurePursuit::isDataReady()
{
  if (!curr_wps_ptr_) {
    return false;
  }
  if (!curr_pose_ptr_) {
    return false;
  }
  return true;
}

std::pair<bool, double> PurePursuit::run()
{
  if (!isDataReady()) {//检查是否有waypoint和pose的数据
    return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());// pair相当于结构体。pair.first和pair.second.
  }//缺少就返回false和 无效值（标准化生成NaN）

  auto closest_pair = planning_utils::findClosestIdxWithDistAngThr(
    *curr_wps_ptr_, *curr_pose_ptr_, closest_thr_dist_, closest_thr_ang_);//已经在头文件内设置
  //搜索在阈值内最近（还是最相似的？）的姿态pair 这里重复计算了吧。设计很脑残啊

  if (!closest_pair.first) {
    RCLCPP_WARN(
      logger, "cannot find, curr_bool: %d, closest_idx: %d", closest_pair.first,
      closest_pair.second);
    return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
  }

  int32_t next_wp_idx = findNextPointIdx(closest_pair.second);//把当前的目标传入，搜索下一个点？
  if (next_wp_idx == -1) {
    RCLCPP_WARN(logger, "lost next waypoint");
    return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
  }

  loc_next_wp_ = curr_wps_ptr_->at(next_wp_idx).position;//找到预瞄点

  geometry_msgs::msg::Point next_tgt_pos;
  // if next waypoint is first 第一个点就是预瞄点
  if (next_wp_idx == 0) {
    next_tgt_pos = curr_wps_ptr_->at(next_wp_idx).position;
  } else {
    // linear interpolation
    std::pair<bool, geometry_msgs::msg::Point> lerp_pair = lerpNextTarget(next_wp_idx);
    //生成预瞄路径上的点进行跟踪。
    if (!lerp_pair.first) {
      RCLCPP_WARN(logger, "lost target! ");
      return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
    }

    next_tgt_pos = lerp_pair.second;
  }
  loc_next_tgt_ = next_tgt_pos;

  double kappa = planning_utils::calcCurvature(next_tgt_pos, *curr_pose_ptr_);
/*double calcCurvature(
  const geometry_msgs::msg::Point & target, const geometry_msgs::msg::Pose & current_pose)
{
  constexpr double KAPPA_MAX = 1e9;
  const double radius = calcRadius(target, current_pose);

  if (fabs(radius) > 0) {
    return 1 / radius;
  } else {
    return KAPPA_MAX;
  }
}*/
  return std::make_pair(true, kappa);//返回一个油门开度
}

// linear interpolation of next target
std::pair<bool, geometry_msgs::msg::Point> PurePursuit::lerpNextTarget(int32_t next_wp_idx)
{ //通过线性差值计算下一目标点，确保其位于当前车辆位姿的正前方，从而符合Pure Pursuit算法的约束。自己计算跟踪轨迹？
  constexpr double ERROR2 = 1e-5;  // 0.00001
  const geometry_msgs::msg::Point & vec_end = curr_wps_ptr_->at(next_wp_idx).position;
  const geometry_msgs::msg::Point & vec_start = curr_wps_ptr_->at(next_wp_idx - 1).position;
  const geometry_msgs::msg::Pose & curr_pose = *curr_pose_ptr_;

  Eigen::Vector3d vec_a(
    (vec_end.x - vec_start.x), (vec_end.y - vec_start.y), (vec_end.z - vec_start.z));

  if (vec_a.norm() < ERROR2) {
    RCLCPP_ERROR(logger, "waypoint interval is almost 0");
    return std::make_pair(false, geometry_msgs::msg::Point());
  }

  const double lateral_error =
    planning_utils::calcLateralError2D(vec_start, vec_end, curr_pose.position);

  if (fabs(lateral_error) > lookahead_distance_) {
    RCLCPP_ERROR(logger, "lateral error is larger than lookahead distance");
    RCLCPP_ERROR(
      logger, "lateral error: %lf, lookahead distance: %lf", lateral_error, lookahead_distance_);
    return std::make_pair(false, geometry_msgs::msg::Point());
  }

  /* calculate the position of the foot of a perpendicular line */
  Eigen::Vector2d uva2d(vec_a.x(), vec_a.y());
  uva2d.normalize();
  Eigen::Rotation2Dd rot =
    (lateral_error > 0) ? Eigen::Rotation2Dd(-M_PI / 2.0) : Eigen::Rotation2Dd(M_PI / 2.0);
  Eigen::Vector2d uva2d_rot = rot * uva2d;

  geometry_msgs::msg::Point h;
  h.x = curr_pose.position.x + fabs(lateral_error) * uva2d_rot.x();
  h.y = curr_pose.position.y + fabs(lateral_error) * uva2d_rot.y();
  h.z = curr_pose.position.z;

  // if there is a intersection
  if (fabs(fabs(lateral_error) - lookahead_distance_) < ERROR2) {
    return std::make_pair(true, h);
  } else {
    // if there are two intersection
    // get intersection in front of vehicle
    const double s = sqrt(pow(lookahead_distance_, 2) - pow(lateral_error, 2));
    geometry_msgs::msg::Point res;
    res.x = h.x + s * uva2d.x();
    res.y = h.y + s * uva2d.y();
    res.z = curr_pose.position.z;
    return std::make_pair(true, res);
  }
}

int32_t PurePursuit::findNextPointIdx(int32_t search_start_idx)//搜索开始的idx。为什么要从目标点开始搜索？
{
  // if waypoints are not given, do nothing.
  if (curr_wps_ptr_->empty() || search_start_idx == -1) {
    return -1;
  }//空的就不管

  // look for the next waypoint.
  for (int32_t i = search_start_idx; i < (int32_t)curr_wps_ptr_->size(); i++) {
    // if search waypoint is the last
    if (i == ((int32_t)curr_wps_ptr_->size() - 1)) {
      return i;
    }//搜索完也没有，则返回末尾i。

    // if waypoint direction is forward
    const auto gld = planning_utils::getLaneDirection(*curr_wps_ptr_, 0.05);
/*int8_t getLaneDirection(const std::vector<geometry_msgs::msg::Pose> & poses, double th_dist)
{ th_dist 用于过滤较小移动或噪声的距离阈值
  if (poses.size() < 2) {
    RCLCPP_ERROR(rclcpp::get_logger(PLANNING_UTILS_LOGGER), "size of waypoints is smaller than 2");
    return 2;
  }

  for (uint32_t i = 0; i < poses.size(); i++) {
    geometry_msgs::msg::Pose prev;
    geometry_msgs::msg::Pose next;

    if (i == (poses.size() - 1)) {
      prev = poses.at(i - 1);
      next = poses.at(i);
    } else {
      prev = poses.at(i);
      next = poses.at(i + 1);
    }
    //只有阈值大于0.05时，才判断是否是转弯。
    if (planning_utils::calcDistSquared2D(prev.position, next.position) > th_dist * th_dist) {
      const auto rel_p = transformToRelativeCoordinate2D(next.position, prev);
      return (rel_p.x > 0.0) ? 0 : 1; || 使用transformToreLativeCoordinate2D函数，将next转换到prev的相对坐标系中。
      如果转换后相对坐标的x为正数，则返回0，车道方向在前（？），否则返回1，车道在后面。这个是用于辨别转弯吗？
    }
  }

  RCLCPP_ERROR(rclcpp::get_logger(PLANNING_UTILS_LOGGER), "lane is something wrong");
  return 2;
}*/
    if (gld == 0) {
      // if waypoint is not in front of ego, skip
      auto ret = planning_utils::transformToRelativeCoordinate2D(
        curr_wps_ptr_->at(i).position, *curr_pose_ptr_);
/*geometry_msgs::msg::Point transformToRelativeCoordinate2D(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & origin)
{
  // translation
  geometry_msgs::msg::Point trans_p;
  trans_p.x = point.x - origin.position.x;
  trans_p.y = point.y - origin.position.y;//计算距离差

  // rotation (use inverse matrix of rotation)
  double yaw = tf2::getYaw(origin.orientation);//获取原点的方向角

  geometry_msgs::msg::Point res;
  res.x = (cos(yaw) * trans_p.x) + (sin(yaw) * trans_p.y);
  res.y = ((-1) * sin(yaw) * trans_p.x) + (cos(yaw) * trans_p.y);
  res.z = origin.position.z;

  return res;
}*/
      if (ret.x < 0) {//遍历点在 currentpose 的后面 舍弃
        continue;
      }
    } else if (gld == 1) {// 路径中有折角超过90度
      // waypoint direction is backward

      // if waypoint is in front of ego, skip
      auto ret = planning_utils::transformToRelativeCoordinate2D(
        curr_wps_ptr_->at(i).position, *curr_pose_ptr_);
      if (ret.x > 0) {//现在那个点在 currentpose 的前面，舍弃
        continue;
      }
    } else {
      return -1;
    }

    const geometry_msgs::msg::Point & curr_motion_point = curr_wps_ptr_->at(i).position;
    const geometry_msgs::msg::Point & curr_pose_point = curr_pose_ptr_->position;
    // if there exists an effective waypoint
    const double ds = planning_utils::calcDistSquared2D(curr_motion_point, curr_pose_point);
    if (ds > std::pow(lookahead_distance_, 2)) {
      return i;//如果距离平方大于前视距离，则返回当前这个点的序号。这就是前视距离跟踪的对象。
    }
  }

  // if this program reaches here , it means we lost the waypoint!
  return -1;
}

void PurePursuit::setCurrentPose(const geometry_msgs::msg::Pose & msg)
{
  curr_pose_ptr_ = std::make_shared<geometry_msgs::msg::Pose>();
  *curr_pose_ptr_ = msg;
}

void PurePursuit::setWaypoints(const std::vector<geometry_msgs::msg::Pose> & msg)
{
  curr_wps_ptr_ = std::make_shared<std::vector<geometry_msgs::msg::Pose>>();
  *curr_wps_ptr_ = msg;
}

}  // namespace pure_pursuit
