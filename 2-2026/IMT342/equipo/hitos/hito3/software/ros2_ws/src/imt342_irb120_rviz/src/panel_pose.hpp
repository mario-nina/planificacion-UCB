#pragma once

#include <array>
#include <cmath>
#include <optional>

#include <QJsonArray>
#include <QJsonValue>

namespace imt342_irb120_rviz
{
using PoseMatrix = std::array<std::array<double, 4>, 4>;

inline std::optional<PoseMatrix> read_pose_matrix(const QJsonArray & array)
{
  if (array.size() != 4) {
    return {};
  }
  PoseMatrix matrix{};
  for (int row = 0; row < 4; ++row) {
    const auto cells = array[row].toArray();
    if (cells.size() != 4) {
      return {};
    }
    for (int col = 0; col < 4; ++col) {
      if (!cells[col].isDouble() || !std::isfinite(cells[col].toDouble())) {
        return {};
      }
      matrix[row][col] = cells[col].toDouble();
    }
  }
  return matrix;
}

inline QJsonArray pose_json(const PoseMatrix & matrix)
{
  QJsonArray values;
  for (const auto & row : matrix) {
    values.append(QJsonArray{row[0], row[1], row[2], row[3]});
  }
  return values;
}

inline PoseMatrix multiply_pose(const PoseMatrix & left, const PoseMatrix & right)
{
  PoseMatrix result{};
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      for (int k = 0; k < 4; ++k) {
        result[row][col] += left[row][k] * right[k][col];
      }
    }
  }
  return result;
}

inline PoseMatrix rigid_inverse(const PoseMatrix & pose)
{
  PoseMatrix inverse{};
  inverse[3][3] = 1.0;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      inverse[row][col] = pose[col][row];
      inverse[row][3] -= pose[col][row] * pose[col][3];
    }
  }
  return inverse;
}

enum class PoseError {none, non_finite, last_row, orthogonality, determinant};

inline PoseError check_pose(const PoseMatrix & pose)
{
  constexpr double tolerance = 1e-7;
  for (const auto & row : pose) {
    for (const auto value : row) {
      if (!std::isfinite(value)) {
        return PoseError::non_finite;
      }
    }
  }
  if (std::abs(pose[3][3] - 1.0) > tolerance) {
    return PoseError::last_row;
  }
  for (int col = 0; col < 3; ++col) {
    if (std::abs(pose[3][col]) > tolerance) {
      return PoseError::last_row;
    }
  }
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      double product = 0.0;
      for (int axis = 0; axis < 3; ++axis) {
        product += pose[axis][row] * pose[axis][col];
      }
      if (std::abs(product - (row == col ? 1.0 : 0.0)) > tolerance) {
        return PoseError::orthogonality;
      }
    }
  }
  const double det =
    pose[0][0] * (pose[1][1] * pose[2][2] - pose[1][2] * pose[2][1]) -
    pose[0][1] * (pose[1][0] * pose[2][2] - pose[1][2] * pose[2][0]) +
    pose[0][2] * (pose[1][0] * pose[2][1] - pose[1][1] * pose[2][0]);
  return std::abs(det - 1.0) <= tolerance ? PoseError::none : PoseError::determinant;
}

inline bool is_rigid_pose(const PoseMatrix & pose)
{
  return check_pose(pose) == PoseError::none;
}
}  // namespace imt342_irb120_rviz
