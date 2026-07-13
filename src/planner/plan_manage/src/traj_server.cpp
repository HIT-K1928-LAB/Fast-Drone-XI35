#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "traj_utils/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <algorithm>
#include <cmath>

ros::Publisher pos_cmd_pub;

quadrotor_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};
std::string command_frame_id_;
using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

void bsplineCallback(traj_utils::BsplineConstPtr msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  //UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}
double wrapToPi(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
std::pair<double, double> calculate_yaw(
    double t_cur,
    Eigen::Vector3d &pos,
    ros::Time &time_now,
    ros::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  constexpr double MIN_DT = 1e-6;
  constexpr double MIN_HORIZONTAL_DIR = 0.1;

  /*
   * 防止历史NaN继续污染后续结果。
   */
  if (!std::isfinite(last_yaw_))
  {
    ROS_ERROR_THROTTLE(
        1.0,
        "[Traj server]: last_yaw is invalid, reset to zero.");

    last_yaw_ = 0.0;
  }

  if (!std::isfinite(last_yaw_dot_))
  {
    ROS_ERROR_THROTTLE(
        1.0,
        "[Traj server]: last_yaw_dot is invalid, reset to zero.");

    last_yaw_dot_ = 0.0;
  }

  const double dt = (time_now - time_last).toSec();

  /*
   * 时间没有推进时，不能用航向角差除以dt。
   */
  if (!std::isfinite(dt) || dt <= MIN_DT)
  {
    last_yaw_dot_ = 0.0;

    return std::make_pair(
        last_yaw_,
        0.0);
  }

  /*
   * 取前视点。只使用水平方向判断航向，
   * 避免只有高度变化时atan2(0, 0)导致航向跳到0。
   */
  const double t_forward =
      std::min(
          traj_duration_,
          std::max(0.0, t_cur + time_forward_));

  const Eigen::Vector3d forward_pos =
      traj_[0].evaluateDeBoorT(t_forward);

  const Eigen::Vector2d horizontal_dir(
      forward_pos.x() - pos.x(),
      forward_pos.y() - pos.y());

  double target_yaw = last_yaw_;

  if (horizontal_dir.allFinite() &&
      horizontal_dir.norm() > MIN_HORIZONTAL_DIR)
  {
    target_yaw =
        std::atan2(
            horizontal_dir.y(),
            horizontal_dir.x());
  }

  if (!std::isfinite(target_yaw))
  {
    target_yaw = last_yaw_;
  }

  /*
   * 计算[-pi, pi]范围内的最短航向角误差。
   */
  const double yaw_error =
      wrapToPi(target_yaw - last_yaw_);

  /*
   * 根据误差和dt计算期望航向角速度。
   */
  double target_yaw_dot = yaw_error / dt;

  target_yaw_dot =
      std::max(
          -YAW_DOT_MAX_PER_SEC,
          std::min(
              YAW_DOT_MAX_PER_SEC,
              target_yaw_dot));

  /*
   * 对航向角速度执行一阶低通滤波。
   */
  double yaw_dot =
      0.5 * last_yaw_dot_ +
      0.5 * target_yaw_dot;

  if (!std::isfinite(yaw_dot))
  {
    yaw_dot = 0.0;
  }

  yaw_dot =
      std::max(
          -YAW_DOT_MAX_PER_SEC,
          std::min(
              YAW_DOT_MAX_PER_SEC,
              yaw_dot));

  /*
   * 由有限的yaw_dot积分得到本次航向角变化。
   * 同时防止滤波后的历史角速度造成过冲。
   */
  double yaw_step = yaw_dot * dt;

  if (std::fabs(yaw_step) > std::fabs(yaw_error))
  {
    yaw_step = yaw_error;
    yaw_dot = yaw_step / dt;
  }

  double yaw =
      wrapToPi(last_yaw_ + yaw_step);

  if (!std::isfinite(yaw))
  {
    yaw = last_yaw_;
    yaw_dot = 0.0;
  }

  last_yaw_ = yaw;
  last_yaw_dot_ = yaw_dot;

  return std::make_pair(
      yaw,
      yaw_dot);
}

void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
 else if (t_cur >= traj_duration_)
{
  /*
   * 轨迹结束后持续发布悬停指令。
   */
  pos = traj_[0].evaluateDeBoorT(traj_duration_);
  vel.setZero();
  acc.setZero();

  yaw_yawdot.first =
      std::isfinite(last_yaw_) ?
      last_yaw_ : 0.0;

  yaw_yawdot.second = 0.0;

  last_yaw_dot_ = 0.0;

  pos_f = pos;

  // 这里不能return，否则不会发布悬停指令
}
else
{
  ROS_WARN_THROTTLE(
      1.0,
      "[Traj server]: trajectory start time is in the future.");

  /*
   * 更新时间，防止下一次回调出现过大的dt。
   */
  time_last = time_now;
  return;
}

time_last = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = command_frame_id_;
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
cmd.yaw_dot = yaw_yawdot.second;

/*
 * yaw异常时使用最近一次有效值。
 */
if (!std::isfinite(cmd.yaw))
{
  ROS_ERROR_THROTTLE(
      1.0,
      "[Traj server]: invalid yaw before publishing.");

  cmd.yaw =
      std::isfinite(last_yaw_) ?
      last_yaw_ : 0.0;
}

/*
 * yaw_dot可以安全退化为0，但绝不能把NaN发给控制器。
 */
if (!std::isfinite(cmd.yaw_dot))
{
  ROS_ERROR_THROTTLE(
      1.0,
      "[Traj server]: invalid yaw_dot before publishing, reset to zero.");

  cmd.yaw_dot = 0.0;
  last_yaw_dot_ = 0.0;
}

/*
 * 位置、速度或加速度出现NaN时不能继续发布。
 */
const bool translational_command_valid =
    std::isfinite(cmd.position.x) &&
    std::isfinite(cmd.position.y) &&
    std::isfinite(cmd.position.z) &&
    std::isfinite(cmd.velocity.x) &&
    std::isfinite(cmd.velocity.y) &&
    std::isfinite(cmd.velocity.z) &&
    std::isfinite(cmd.acceleration.x) &&
    std::isfinite(cmd.acceleration.y) &&
    std::isfinite(cmd.acceleration.z);

if (!translational_command_valid)
{
  ROS_ERROR_THROTTLE(
      1.0,
      "[Traj server]: invalid position/velocity/acceleration, "
      "command is not published.");

  return;
}

last_yaw_ = cmd.yaw;

pos_cmd_pub.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = nh.subscribe("planning/bspline", 10, bsplineCallback);

  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  nh.param<std::string>(
    "traj_server/frame_id",
    command_frame_id_,
    "map");
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}