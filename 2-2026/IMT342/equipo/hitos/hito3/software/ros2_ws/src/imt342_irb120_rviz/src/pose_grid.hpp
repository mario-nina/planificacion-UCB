#pragma once

#include "panel_pose.hpp"

#include <functional>
#include <utility>

#include <QApplication>
#include <QClipboard>
#include <QFontDatabase>
#include <QKeyEvent>
#include <QGridLayout>
#include <QJsonDocument>
#include <QLineEdit>
#include <QRegularExpression>
#include <QWidget>

namespace imt342_irb120_rviz
{
class PoseGrid : public QWidget
{
public:
  explicit PoseGrid(QWidget * parent = nullptr, bool fixed_bottom = false, bool display_only = false)
  : QWidget(parent), fixed_bottom_(fixed_bottom), display_only_(display_only)
  {
    auto * layout = new QGridLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    const auto font = QFontDatabase::systemFont(QFontDatabase::FixedFont);
    for (int row = 0; row < 4; ++row) {
      for (int column = 0; column < 4; ++column) {
        auto * cell = new Cell([this]() {
          if (display_only_ || !paste(QApplication::clipboard()->text())) {
            return false;
          }
          if (changed_) {
            changed_();
          }
          return true;
        });
        cell->setAlignment(Qt::AlignRight);
        cell->setFont(font);
        cell->setMinimumWidth(48);
        cell->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
        cell->setAccessibleName(tr("Matriz, fila %1 columna %2").arg(row + 1).arg(column + 1));
        layout->addWidget(cell, row, column);
        cells_[row][column] = cell;
        cell->setReadOnly(display_only_ || (fixed_bottom_ && row == 3));
        connect(cell, &QLineEdit::textEdited, this, [this]() {if (changed_) {changed_();}});
      }
    }
    setMatrix({});
  }

  void onEdit(std::function<void()> changed) {changed_ = std::move(changed);}

  void setMatrix(const QJsonArray & values)
  {
    const auto pose = read_pose_matrix(values);
    for (int row = 0; row < 4; ++row) {
      for (int column = 0; column < 4; ++column) {
        cells_[row][column]->setText(
          fixed_bottom_ && row == 3 ? QString::number(column == 3 ? 1 : 0) :
          pose ? QString::number((*pose)[row][column], 'g', display_only_ ? 7 : 16) : QString());
      }
    }
  }

  QJsonArray matrix() const
  {
    PoseMatrix values{};
    for (int row = 0; row < 4; ++row) {
      for (int column = 0; column < 4; ++column) {
        bool ok = false;
        const auto value = cells_[row][column]->text().trimmed().toDouble(&ok);
        if (!ok || !std::isfinite(value)) {
          return {};
        }
        values[row][column] = value;
      }
    }
    return pose_json(values);
  }

  bool paste(const QString & text)
  {
    const auto json = QJsonDocument::fromJson(text.toUtf8());
    if (const auto pose = read_pose_matrix(json.array())) {
      if (!accept_bottom(*pose)) {
        return false;
      }
      setMatrix(json.array());
      return true;
    }
    const auto rows = text.trimmed().split('\n');
    if (rows.size() != 4) {
      return false;
    }
    PoseMatrix values{};
    const QRegularExpression spaces(QStringLiteral("[\\s,;]+"));
    for (int row = 0; row < 4; ++row) {
      const auto fields = rows[row].trimmed().split(spaces);
      if (fields.size() != 4) {
        return false;
      }
      for (int column = 0; column < 4; ++column) {
        bool ok = false;
        const auto value = fields[column].toDouble(&ok);
        if (!ok || !std::isfinite(value)) {
          return false;
        }
        values[row][column] = value;
      }
    }
    if (!accept_bottom(values)) {
      return false;
    }
    setMatrix(pose_json(values));
    return true;
  }

private:
  bool accept_bottom(const PoseMatrix & pose) const
  {
    return !fixed_bottom_ || pose[3] == std::array<double, 4>{0, 0, 0, 1};
  }

  class Cell : public QLineEdit
  {
  public:
    explicit Cell(std::function<bool()> paste_grid) : paste_grid_(std::move(paste_grid)) {}

  protected:
    void keyPressEvent(QKeyEvent * event) override
    {
      if (event->matches(QKeySequence::Paste) && paste_grid_()) {
        event->accept();
      } else {
        QLineEdit::keyPressEvent(event);
      }
    }

  private:
    std::function<bool()> paste_grid_;
  };

  std::array<std::array<QLineEdit *, 4>, 4> cells_{};
  std::function<void()> changed_;
  bool fixed_bottom_;
  bool display_only_;
};
}  // namespace imt342_irb120_rviz
