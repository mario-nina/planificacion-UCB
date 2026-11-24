#include "../src/panel_pose.hpp"

#include <cstdlib>

using namespace imt342_irb120_rviz;

int main()
{
  const PoseMatrix flange{{{0, -1, 0, .3}, {1, 0, 0, .4}, {0, 0, 1, .5}, {0, 0, 0, 1}}};
  const PoseMatrix offset{{{1, 0, 0, .2}, {0, 0, -1, 0}, {0, 1, 0, .1}, {0, 0, 0, 1}}};
  const auto tcp = multiply_pose(flange, offset);
  if (!is_rigid_pose(tcp) || std::abs(tcp[0][3] - .3) > 1e-12 ||
    std::abs(tcp[1][3] - .6) > 1e-12 || std::abs(tcp[2][3] - .6) > 1e-12 ||
    read_pose_matrix(pose_json(tcp)) != tcp)
  {
    return EXIT_FAILURE;
  }
  const auto recovered = multiply_pose(tcp, rigid_inverse(offset));
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      if (std::abs(recovered[row][col] - flange[row][col]) > 1e-12) {
        return EXIT_FAILURE;
      }
    }
  }
  auto malformed = flange;
  malformed[2][2] = -1;
  return is_rigid_pose(malformed) || read_pose_matrix(QJsonArray{0, 0, 0, 1}) ?
    EXIT_FAILURE : EXIT_SUCCESS;
}
