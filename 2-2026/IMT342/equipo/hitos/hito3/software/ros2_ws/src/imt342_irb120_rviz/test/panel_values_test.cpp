#include "imt342_irb120_rviz/panel_values.hpp"

#include <limits>
#include <string>
#include <vector>

int main()
{
  using namespace imt342_irb120_rviz;
  for (std::size_t axis = 0; axis < joint_names.size(); ++axis) {
    if (std::abs(slider_radians(axis, 0) - lower[axis]) > 1e-12 ||
      std::abs(slider_radians(axis, slider_max) - upper[axis]) > 1e-12 ||
      slider_value(axis, slider_radians(axis, 1500)) != 1500 ||
      slider_value(axis, upper[axis] + 1.0) != slider_max)
    {
      return 1;
    }
  }
  std::vector<std::string> names{
    "joint_6", "joint_1", "joint_3", "joint_4", "joint_2", "joint_5"};
  std::vector<double> positions{6, 1, 3, 4, 2, 5};
  const auto ordered = ordered_joints(names, positions);
  if (!ordered || *ordered != Joints{1, 2, 3, 4, 5, 6}) {
    return 2;
  }
  positions[0] = std::numeric_limits<double>::infinity();
  if (ordered_joints(names, positions)) {
    return 3;
  }
  positions[0] = 6;
  names[0] = "joint_1";
  if (ordered_joints(names, positions)) {
    return 4;
  }
  positions.pop_back();
  if (ordered_joints(names, positions)) {
    return 5;
  }
  const Joints origin{};
  auto actual = origin;
  actual[4] = 1e-5;
  if (!same_joint_origin(origin, actual)) {
    return 6;
  }
  actual[4] = .02 / deg_per_rad;
  if (same_joint_origin(origin, actual) || !same_joint_origin(origin, actual, .05 / deg_per_rad)) {
    return 9;
  }
  actual[4] = 1e-3;
  if (same_joint_origin(origin, actual)) {
    return 7;
  }
  actual[4] = std::numeric_limits<double>::quiet_NaN();
  return same_joint_origin(origin, actual) ? 8 : 0;
}
