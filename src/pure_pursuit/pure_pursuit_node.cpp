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

#include "pure_pursuit/pure_pursuit_node.hpp"

#include "pure_pursuit/pure_pursuit_viz.hpp"
#include "pure_pursuit/util/planning_utils.hpp"
#include "pure_pursuit/util/tf_utils.hpp"

#include <vehicle_info_util/vehicle_info_util.hpp>//获取车辆参数

#include <algorithm>
#include <memory>
#include <utility>

namespace
{
double calcLookaheadDistance(//计算前视距离函数 不小于最小值
  const double velocity, const double lookahead_distance_ratio, const double min_lookahead_distance)
{
  const double lookahead_distance = lookahead_distance_ratio * std::abs(velocity);
  return std::max(lookahead_distance, min_lookahead_distance);
}

}  // namespace

namespace pure_pursuit
{//初始化 pure_pursuit节点
PurePursuitNode::PurePursuitNode(const rclcpp::NodeOptions & node_options)//这里创建了PurePursuitNODE的对象
: Node("pure_pursuit", node_options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  pure_pursuit_ = std::make_unique<PurePursuit>();//创建了PurePursuit对象（类的实现）

  // Vehicle Parameters
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  param_.wheel_base = vehicle_info.wheel_base_m;//获取轴距

  // Node Parameters
  param_.ctrl_period = this->declare_parameter<double>("control_period");//加载控制周期

  // Algorithm Parameters
  param_.lookahead_distance_ratio = this->declare_parameter<double>("lookahead_distance_ratio");
  param_.min_lookahead_distance = this->declare_parameter<double>("min_lookahead_distance");
  param_.reverse_min_lookahead_distance =
    this->declare_parameter<double>("reverse_min_lookahead_distance");//前视 最小 逆前视距离参数加载

  // Subscribers
  using std::placeholders::_1;
  sub_trajectory_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "input/reference_trajectory", 1, std::bind(&PurePursuitNode::onTrajectory, this, _1));
  sub_current_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "input/current_odometry", 1, std::bind(&PurePursuitNode::onCurrentOdometry, this, _1));
  //接收参考轨迹 和 里程计数据
  // Publishers
  pub_ctrl_cmd_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannLateralCommand>(
    "output/control_raw", 1);
  //发布控制指令
  // Debug Publishers
  pub_debug_marker_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/markers", 0);
  //发布调试用的可视化标记

  //曲率 Publishers
  pub_curvature_ = this->create_publisher<std_msgs::msg::Float32>("/extra_cur",1);
  // Timer
  {
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(param_.ctrl_period));
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&PurePursuitNode::onTimer, this));
  }//定时器初始化，根据ctrl_period周期调用onTimer

  //  Wait for first current pose
  tf_utils::waitForTransform(tf_buffer_, "map", "base_link");//等待map和base_link的坐标变换数据
}

bool PurePursuitNode::isDataReady()//检查里程计 轨迹 姿态数据。发出节流日志警告
{
  if (!current_odometry_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for current_odometry...");
    return false;
  }

  if (!trajectory_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for trajectory...");
    return false;
  }

  if (!current_pose_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for current_pose...");
    return false;
  }

  return true;
}

void PurePursuitNode::onCurrentOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  current_odometry_ = msg;//回调函数，接收里程计信息
}

void PurePursuitNode::onTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  trajectory_ = msg;//同上
}

void PurePursuitNode::onTimer()//通过定时器触发。所以是不断触发姿态获取。定时器是核心函数
{
  current_pose_ = self_pose_listener_.getCurrentPose();//tf的功能。获取最新位姿。

  if (!isDataReady()) {
    return;//如果没收到三个数据直接退出
  }

  const auto target_curvature = calcTargetCurvature();//计算目标曲率 

  if (target_curvature) {
    publishCommand(*target_curvature);
    publishCurvature(*target_curvature);
    publishDebugMarker();
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "failed to solve pure_pursuit");
    publishCommand({0.0});
    publishCurvature(0.0);
  }


}

void PurePursuitNode::publishCurvature(const double target_curvature);
{
  std_msgs::msg::Float32 msg;
  msg.data = target_curvature;
  pub_curvature_->publish(msg);
}


void PurePursuitNode::publishCommand(const double target_curvature)
{
  autoware_auto_control_msgs::msg::AckermannLateralCommand cmd;
  cmd.stamp = get_clock()->now();
  cmd.steering_tire_angle = //就在这里计算出目标角度。现在改成车辆的角速度
    planning_utils::convertCurvatureToSteeringAngle(param_.wheel_base, target_curvature);
/*double convertCurvatureToSteeringAngle(double wheel_base, double kappa)
{
  return atan(wheel_base * kappa);
}*/
  pub_ctrl_cmd_->publish(cmd);
}

void PurePursuitNode::publishDebugMarker() const
{
  visualization_msgs::msg::MarkerArray marker_array;

  marker_array.markers.push_back(createNextTargetMarker(debug_data_.next_target));
  marker_array.markers.push_back(
    createTrajectoryCircleMarker(debug_data_.next_target, current_pose_->pose));

  pub_debug_marker_->publish(marker_array);
}

boost::optional<double> PurePursuitNode::calcTargetCurvature()
{
  // Ignore invalid trajectory
  if (trajectory_->points.size() < 3) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "received path size is < 3, ignored");
    return {};//小于三个点，不计算曲率？
  }

  // Calculate target point for velocity/acceleration
  const auto target_point = calcTargetPoint();//搜索最近点，返回轨迹点
  if (!target_point) {//目标点为空，直接返回
    return {};
  }

  const double target_vel = target_point->longitudinal_velocity_mps;//目标点的速度 对应位置的速度（轨迹生成时就带有的？）

  // Calculate lookahead distance
  const bool is_reverse = (target_vel < 0);//是否反向？
  const double min_lookahead_distance =
    is_reverse ? param_.reverse_min_lookahead_distance : param_.min_lookahead_distance;//取得最小前视距离。要判断是否反向。
  const double lookahead_distance = calcLookaheadDistance(
    current_odometry_->twist.twist.linear.x, param_.lookahead_distance_ratio,
    min_lookahead_distance);//计算前视距离

  // Set PurePursuit data
  pure_pursuit_->setCurrentPose(current_pose_->pose);//把现在的位姿传给 curr_pose_ptr
  pure_pursuit_->setWaypoints(planning_utils::extractPoses(*trajectory_));
  
  /*std::vector<geometry_msgs::msg::Pose> extractPoses(
  const autoware_auto_planning_msgs::msg::Trajectory & trajectory)//提取轨迹
{
  std::vector<geometry_msgs::msg::Pose> poses;

  for (const auto & p : trajectory.points) { ||const auto &直接引用原始对象避免拷贝。
    poses.push_back(p.pose);
  }

  return poses;
}*/

  pure_pursuit_->setLookaheadDistance(lookahead_distance);//同理

  // Run PurePursuit
  const auto pure_pursuit_result = pure_pursuit_->run();//结果等于 run？在pure pursuit里
  if (!pure_pursuit_result.first) {
    return {};
  }

  const auto kappa = pure_pursuit_result.second;//曲率

  // Set debug data
  debug_data_.next_target = pure_pursuit_->getLocationOfNextTarget();

  return kappa;
}

boost::optional<autoware_auto_planning_msgs::msg::TrajectoryPoint>
PurePursuitNode::calcTargetPoint() const //计算目标点
{
  const auto closest_idx_result = planning_utils::findClosestIdxWithDistAngThr(
    planning_utils::extractPoses(*trajectory_), current_pose_->pose, 3.0, M_PI_4);//这个值和头文件里的一样
  //——————————————————————————————————————————————————————————————————————————————关于这个搜索函数
  /*std::pair<bool, int32_t> findClosestIdxWithDistAngThr(
  const std::vector<geometry_msgs::msg::Pose> & poses, ||poses应该是把这个轨迹上的所有pose都提取了
  const geometry_msgs::msg::Pose & current_pose, double th_dist, double th_yaw)
{
  double dist_squared_min = std::numeric_limits<double>::max(); 最小距离平方值
  int32_t idx_min = -1; -1就表示没找到

  for (size_t i = 0; i < poses.size(); ++i) {
    const double ds = calcDistSquared2D(poses.at(i).position, current_pose.position);
    if (ds > th_dist * th_dist) {
      continue; //比阈值平方大就不要了
    }

    const double yaw_pose = tf2::getYaw(current_pose.orientation);
    const double yaw_ps = tf2::getYaw(poses.at(i).orientation);
    const double yaw_diff = normalizeEulerAngle(yaw_pose - yaw_ps); 计算偏航角并归一化
    if (fabs(yaw_diff) > th_yaw) {
      continue;//偏航角差过大时放弃
    }

    if (ds < dist_squared_min) {
      dist_squared_min = ds;
      idx_min = i; //更新最小距离和索引
    }
  }

  return (idx_min >= 0) ? std::make_pair(true, idx_min) : std::make_pair(false, idx_min);
  //满足 大于 0（找到），返回序号和真值。要不就没有返回值。（这个时候车辆应该会停止不动）
}*/
//—————————————————————————————————————————————————————————————————————————————————
  //找最近的点？
  if (!closest_idx_result.first) {//如果返回false
    RCLCPP_ERROR(get_logger(), "cannot find closest waypoint");
    return {};//找不到则返回空
  }

  return trajectory_->points.at(closest_idx_result.second);//返回轨迹中对应的最近点
}
}  // namespace pure_pursuit

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pure_pursuit::PurePursuitNode)
