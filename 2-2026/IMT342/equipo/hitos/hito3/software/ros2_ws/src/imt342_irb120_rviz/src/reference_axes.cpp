#include <array>
#include <memory>
#include <string>

#include <QStringList>

#include <OgreSceneNode.h>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_rendering/objects/axes.hpp>

namespace imt342_irb120_rviz
{
namespace
{
struct ReferenceFrame
{
  const char * name;
  float length;
  float radius;
};

constexpr std::array<ReferenceFrame, 3> frames{{
  {"base_link", 0.14f, 0.008f},
  {"dh_frame_6", 0.095f, 0.006f},
  {"tcp_active", 0.145f, 0.004f},
}};
}  // namespace

class ReferenceAxes : public rviz_common::Display
{
public:
  void update(float, float) override
  {
    if (!isEnabled() || !axes_.front()) {
      return;
    }
    QStringList missing;
    for (std::size_t i = 0; i < frames.size(); ++i) {
      Ogre::Vector3 position;
      Ogre::Quaternion orientation;
      const auto & frame = frames[i];
      const bool valid = context_->getFrameManager()->getTransform(
        std::string(frame.name), position, orientation);
      auto & axes = *axes_[i];
      axes.getSceneNode()->setVisible(valid);
      if (valid) {
        axes.setPosition(position);
        axes.setOrientation(orientation);
      } else {
        missing.append(QString::fromLatin1(frame.name));
      }
    }
    if (missing.isEmpty()) {
      setStatus(rviz_common::properties::StatusProperty::Ok, tr("TF"), tr("Base, brida y TCP activo"));
    } else {
      setStatus(rviz_common::properties::StatusProperty::Warn, tr("TF"),
        tr("Transformaciones no disponibles: %1").arg(missing.join(QStringLiteral(", "))));
    }
  }

  void reset() override
  {
    rviz_common::Display::reset();
    hideAxes();
  }

protected:
  void onInitialize() override
  {
    for (std::size_t i = 0; i < frames.size(); ++i) {
      axes_[i] = std::make_unique<rviz_rendering::Axes>(
        scene_manager_, scene_node_, frames[i].length, frames[i].radius);
    }
    hideAxes();
  }

  void onDisable() override
  {
    hideAxes();
  }

private:
  void hideAxes()
  {
    for (const auto & axes : axes_) {
      if (axes) {
        axes->getSceneNode()->setVisible(false);
      }
    }
  }

  std::array<std::unique_ptr<rviz_rendering::Axes>, frames.size()> axes_;
};
}  // namespace imt342_irb120_rviz

PLUGINLIB_EXPORT_CLASS(imt342_irb120_rviz::ReferenceAxes, rviz_common::Display)
