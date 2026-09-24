#include "yopo_minco_planner/yopo_planner.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace yopo_minco_planner {
constexpr double pi = 3.14159265358979323846;
// 球坐标系到直角坐标系
static Eigen::Vector3d spherical(double yaw, double pitch, double r) {
    return {
        std::cos(pitch) * std::cos(yaw) * r, std::cos(pitch) * std::sin(yaw) * r,
        std::sin(pitch) * r};
}
// 从ROS节点传入参数c并保存到配置
YopoPlanner::YopoPlanner(PlannerConfig c) : c_(c) {
    const double values[] = {
        c.velocity,         c.training_velocity, c.training_acceleration, c.tail_range,
        c.horizontal_fov,   c.vertical_fov,      c.horizontal_anchor_fov, c.vertical_anchor_fov,
        c.duration_min,     c.radius_lambda,     c.safe_radius,           c.max_speed,
        c.max_acceleration, c.max_jerk,          c.height_band,           c.test_radius,
        c.max_duration,     c.check_dt};
    // 检查参数是否合法
    for (double x : values)
        if (!std::isfinite(x) || x <= 0) throw std::invalid_argument("Invalid planner parameter");
    if (!std::isfinite(c.min_height) || !std::isfinite(c.max_height) ||
        c.min_height >= c.max_height || c.check_dt > 0.1)
        throw std::invalid_argument("Invalid trajectory check bounds");
    // 根据速度的训练与实机比例放缩加速度
    acc_scale_ = std::pow(c.velocity / c.training_velocity, 2) * c.training_acceleration;
    // 每段轨迹时间
    piece_time_ = c.tail_range / c.velocity / 2.;
    for (int i = 0; i < 15; ++i) {
        int lattice = 14 - i;
        double y    = (lattice % 5 - 2) * c.horizontal_fov * pi / 180. / 5.;
        double p    = (lattice / 5 - 1) * c.vertical_fov * pi / 180. / 3.;
        // 保存15个候选的yaw和pitch
        angles_[i]    = {y, p};
        rotations_[i] = (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(-p, Eigen::Vector3d::UnitY()))
                            .toRotationMatrix();
    }
}
// 对输入YOPO网络的无人机状态等Input进行预处理
Observation YopoPlanner::observation(const PlanningState& s) const {
    // g是目标向量
    Eigen::Vector3d g = s.goal - s.head.row(0).transpose();
    // 如果太远就把目标压缩到 tail_range 范围内
    if (g.norm() > c_.tail_range) {
        // clamp把一个值(这里是z)限制在指定区间内
        double z = std::clamp(g.z(), -c_.tail_range, c_.tail_range);
        // 处理完z后，处理水平方向，单位方向向量×允许的最大长度
        g.head<2>() = g.head<2>() / (g.head<2>().norm() + 1e-9) *
                      std::sqrt(std::max(0., c_.tail_range * c_.tail_range - z * z));
        g.z() = z;
    }
    Eigen::Matrix3d rows;
    // rows为3*3的v,a,g
    // s.head.row(0) = 位置 position      [x, y, z]
    // s.head.row(1) = 速度 velocity      [vx, vy, vz]
    // s.head.row(2) = 加速度 acceleration [ax, ay, az]
    // 世界系→相机系，并归一化
    rows.row(0) = (s.rotation_wc.transpose() * s.head.row(1).transpose() / c_.velocity).transpose();
    rows.row(1) = (s.rotation_wc.transpose() * s.head.row(2).transpose() / acc_scale_).transpose();
    g           = s.rotation_wc.transpose() * g;
    rows.row(2) = (g / std::max(g.norm(), c_.tail_range)).transpose();
    Observation out{};
    for (int i = 0; i < 15; ++i) {
        // 从相机系到基元系
        Eigen::Matrix3d v = rows * rotations_[i];
        // 排序
        for (int ch = 0; ch < 9; ++ch) out[ch * 15 + i] = static_cast<float>(v(ch / 3, ch % 3));
    }
    return out;
}
// decode函数，把神经网络对第 i 个候选基元输出的 14 个归一化数，转变成真实的轨迹参数。
Candidate YopoPlanner::decode(const NetworkOutput& out, int i) const {
    if (i < 0 || i >= 15) throw std::out_of_range("candidate index");
    // x是网络预测的输出
    // x(0~2)    → inner 的方向和距离参数
    // x(3~5)    → tail 的方向和距离参数
    // x(6~8)    → tail 速度
    // x(9~11)   → tail 加速度
    // x(12~13)  → 两段轨迹时间
    // inner：只预测位置
    // tail：预测位置 + 速度 + 加速度
    auto x = [&](int ch) { return double(out.endstate[ch * 15 + i]); };
    Candidate c;
    // Outputs already contain tanh: do not apply tanh a second time.
    // 偏差值+原yaw+pitch
    c.inner = spherical(
        angles_[i][0] + x(0) * c_.horizontal_anchor_fov * pi / 360.,
        angles_[i][1] + x(1) * c_.vertical_anchor_fov * pi / 360.,
        // 归一化距离→实际距离
        (x(2) + 1) * 0.5 * c_.tail_range);
    // 缩放得到tail 的 yaw、pitch
    c.tail.row(0) = spherical(
                        x(3) * c_.horizontal_fov * pi / 360., x(4) * c_.vertical_fov * pi / 360.,
                        (x(5) + 1) * 0.5 * c_.tail_range)
                        .transpose();
    for (int k = 0; k < 3; ++k) {
        c.tail(1, k) = x(6 + k) * c_.velocity;
        c.tail(2, k) = x(9 + k) * acc_scale_;
    }
    c.durations = {
        std::max(c_.duration_min, (x(12) + 1) * piece_time_),
        std::max(c_.duration_min, (x(13) + 1) * piece_time_)};
    return c;
}
// 检查生成后的轨迹采样点的运动状态和空间位置是否合法
bool YopoPlanner::check(
    const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& a,
    const Eigen::Vector3d& j, const Eigen::Vector3d& origin) const {
    return p.allFinite() && v.allFinite() && a.allFinite() && j.allFinite() &&
           v.norm() <= c_.max_speed && a.norm() <= c_.max_acceleration && j.norm() <= c_.max_jerk &&
           p.z() >= c_.min_height && p.z() <= c_.max_height &&
           std::abs(p.z() - origin.z()) <= c_.height_band &&
           (p - origin).head<2>().norm() <= c_.test_radius;
}
// 检查网络输入、输出是否合法
bool YopoPlanner::plan(
    const PlanningState& s, const NetworkOutput& out, Plan* result, std::string* reason) const {
    if (!result || !reason) return false;
    if (!s.head.allFinite() || !s.rotation_wc.allFinite() || !s.goal.allFinite() ||
        !s.origin.allFinite()) {
        *reason = "non-finite planning state";
        return false;
    }
    for (float x : out.endstate)
        if (!std::isfinite(x)) {
            *reason = "non-finite network output";
            return false;
        }
    for (float x : out.score)
        if (!std::isfinite(x)) {
            *reason = "non-finite network output";
            return false;
        }
    for (float x : out.radius)
        if (!std::isfinite(x)) {
            *reason = "non-finite network output";
            return false;
        }
    int best     = -1;
    double score = -std::numeric_limits<double>::infinity(), chosen_mu = 0;
    // 计算安全裕度并排除不安全的
    const double threshold = 1 - std::exp(-c_.safe_radius / c_.radius_lambda);
    double best_mu = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < 15; ++i) {
        double mu = std::numeric_limits<double>::infinity();
        for (int k = 0; k < 10; ++k)
            mu = std::min(mu, double(out.radius[k * 15 + i] - out.radius[(k + 10) * 15 + i]));
        best_mu = std::max(best_mu, mu);
        // 选出分数最高的
        if (mu >= threshold && std::isfinite(out.score[i]) && out.score[i] > score) {
            best      = i;
            score     = out.score[i];
            chosen_mu = mu;
        }
    }
    if (best < 0) {
        *reason = "no candidate passes predicted corridor threshold; best_mu=" +
                  std::to_string(best_mu) + " threshold=" + std::to_string(threshold);
        return false;
    }
    // 被选中的候选转变为真实轨迹参数
    Candidate c                 = decode(out, best);
    Eigen::Matrix3d tail        = c.tail * s.rotation_wc.transpose();
    const Eigen::Vector3d start = s.head.row(0).transpose();
    // 算tail位置
    tail.row(0) += start.transpose();
    // 调用 MincoSolver 生成轨迹，用起点、终点、中点、时间生成
    MincoSolver traj;
    if (c.durations.sum() > c_.max_duration ||
        !traj.solve(s.head, tail, s.rotation_wc * c.inner + start, c.durations)) {
        *reason = "MINCO solve/duration invalid";
        return false;
    }
    // 离散采样检查
    // 检查多少段traj.duration() / c_.check_dt是段数
    int n = std::max(1, int(std::ceil(traj.duration() / c_.check_dt)));
    for (int i = 0; i <= n; ++i) {
        double t = traj.duration() * i / n;
        if (!check(
                traj.evaluate(t, 0), traj.evaluate(t, 1), traj.evaluate(t, 2), traj.evaluate(t, 3),
                s.origin)) {
            *reason = "selected trajectory exceeds sampled motion/spatial limits";
            return false;
        }
    }
    result->trajectory  = traj;
    result->action      = best;
    result->score       = score;
    result->corridor_mu = chosen_mu;
    return true;
}
// 生成无人机的 yaw（航向角）和 yaw 角速度，让机头平滑地朝向“飞行方向 + 目标方向”
double YopoPlanner::wrap(double x) { return std::atan2(std::sin(x), std::cos(x)); }
std::pair<double, double> YopoPlanner::yaw(
    const Eigen::Vector3d& v, const Eigen::Vector3d& g, double last, double dt) {
    double delta        = wrap(std::atan2(g.y(), g.x()) - last);
    Eigen::Vector3d dir = v / (v.norm() + 1e-5) + 2 * std::abs(delta) / pi * g / (g.norm() + 1e-5);
    double desired      = g.norm() > 0.5 ? std::atan2(dir.y(), dir.x()) : last;
    double change       = std::clamp(wrap(desired - last), -0.5 * pi * dt, 0.5 * pi * dt);
    return {wrap(last + change), change / dt};
}
}  // namespace yopo_minco_planner
