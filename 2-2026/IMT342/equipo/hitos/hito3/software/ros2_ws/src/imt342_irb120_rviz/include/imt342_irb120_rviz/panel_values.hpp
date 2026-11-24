#ifndef IMT342_IRB120_RVIZ__PANEL_VALUES_HPP_
#define IMT342_IRB120_RVIZ__PANEL_VALUES_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace imt342_irb120_rviz
{
inline constexpr std::array<const char *, 6> joint_names{
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
inline constexpr std::array<double, 6> lower{
  -2.879793, -1.919862, -1.919862, -2.792527, -2.094395, -6.981317};
inline constexpr std::array<double, 6> upper{
  2.879793, 1.919862, 1.221730, 2.792527, 2.094395, 6.981317};
inline constexpr double deg_per_rad = 180.0 / 3.14159265358979323846;
inline constexpr int slider_max = 10000;
using Joints = std::array<double, 6>;

inline bool same_joint_origin(const Joints & plan, const Joints & actual, double tolerance = 1e-4)
{
  for (std::size_t i = 0; i < actual.size(); ++i) {
    if (!(std::abs(plan[i] - actual[i]) <= tolerance)) {
      return false;
    }
  }
  return true;
}

inline double slider_radians(std::size_t axis, int value)
{
  if (value <= 0) {
    return lower[axis];
  }
  if (value >= slider_max) {
    return upper[axis];
  }
  const double fraction = static_cast<double>(value) / slider_max;
  return lower[axis] + (upper[axis] - lower[axis]) * fraction;
}

inline int slider_value(std::size_t axis, double radians)
{
  const double clamped = std::clamp(radians, lower[axis], upper[axis]);
  return static_cast<int>(std::lround(
    (clamped - lower[axis]) / (upper[axis] - lower[axis]) * slider_max));
}

inline double editor_radians(
  std::size_t axis, double shown, double factor, double editor_minimum, double editor_maximum)
{
  if (shown <= editor_minimum) {
    return lower[axis];
  }
  if (shown >= editor_maximum) {
    return upper[axis];
  }
  return std::clamp(shown / factor, lower[axis], upper[axis]);
}

template<typename Names, typename Positions>
std::optional<Joints> ordered_joints(const Names & names, const Positions & positions)
{
  if (names.size() != joint_names.size() || positions.size() != joint_names.size()) {
    return std::nullopt;
  }
  Joints ordered{};
  std::array<bool, 6> seen{};
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto name = std::find(joint_names.begin(), joint_names.end(), names[i]);
    if (name == joint_names.end() || !std::isfinite(positions[i])) {
      return std::nullopt;
    }
    const auto axis = static_cast<std::size_t>(name - joint_names.begin());
    if (seen[axis]) {
      return std::nullopt;
    }
    seen[axis] = true;
    ordered[axis] = positions[i];
  }
  return ordered;
}
}  // namespace imt342_irb120_rviz

#endif
