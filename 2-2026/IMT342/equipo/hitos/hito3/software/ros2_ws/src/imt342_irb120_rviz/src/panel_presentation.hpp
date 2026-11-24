#pragma once

#include <QString>
#include <QStringList>

namespace imt342_irb120_rviz
{
inline QString branch_label(const QString & branch)
{
  const auto fields = branch.split('/');
  if (fields.size() != 4) {
    return branch;
  }
  const auto wrist = fields[2].startsWith(QStringLiteral("M0")) ?
    QStringLiteral("Muñeca singular %1").arg(fields[2]) : QStringLiteral("Muñeca %1").arg(fields[2]);
  return QStringLiteral("Solución %1 — Hombro %2 · Codo %3 · %4")
    .arg(fields[3], fields[0], fields[1], wrist);
}

inline QString compact_branch_label(const QString & branch)
{
  const auto fields = branch.split('/');
  return fields.size() == 4 ? QStringLiteral("Solución %1 · %2 · %3 · %4")
    .arg(fields[3], fields[0], fields[1], fields[2]) : branch;
}

inline double effective_joint_speed(double nominal, double global_percent, double axis_percent)
{
  return nominal * global_percent / 100.0 * axis_percent / 100.0;
}
}  // namespace imt342_irb120_rviz
