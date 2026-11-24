#include "analysis_widget.hpp"

#include <algorithm>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <QDockWidget>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMainWindow>
#include <QTimer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>

namespace imt342_irb120_rviz
{
class AnalysisPanel : public rviz_common::Panel
{
public:
  explicit AnalysisPanel(QWidget * parent = nullptr)
  : rviz_common::Panel(parent), widget_(new AnalysisWidget(this))
  {
    widget_->setObjectName(QStringLiteral("imt342_analysis_widget"));
    auto * layout = new QVBoxLayout(this);
    layout->setContentsMargins(3, 3, 3, 3);
    layout->addWidget(widget_);
    auto * timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() {drain();});
    timer->start(100);
  }

  void onInitialize() override
  {
    const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    if (!abstraction) {
      return;
    }
    node_ = abstraction->get_raw_node();
    if (node_) {
      samples_ = subscribe("/imt342/analysis", false);
      routes_ = subscribe("/imt342/pose_plan_result", true);
    }
  }

  void load(const rviz_common::Config & config) override
  {
    rviz_common::Panel::load(config);
    bool align = false;
    if (config.mapGetBool("Align Bottom", &align) && align) {
      QTimer::singleShot(0, this, [this]() {
        auto * dock = qobject_cast<QDockWidget *>(parentWidget());
        auto * window = dock ? qobject_cast<QMainWindow *>(dock->window()) : nullptr;
        if (window) {
          window->setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
          window->setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);
          window->addDockWidget(Qt::BottomDockWidgetArea, dock);
          window->resizeDocks({dock}, {290}, Qt::Vertical);
        }
      });
    }
  }

private:
  using Text = std_msgs::msg::String;
  struct Inbox
  {
    std::mutex mutex;
    std::deque<std::string> samples;
    std::string routes;
  };

  rclcpp::Subscription<Text>::SharedPtr subscribe(const char * topic, bool routes)
  {
    const auto inbox = inbox_;
    return node_->create_subscription<Text>(
      topic, rclcpp::QoS(routes ? 1 : 30).transient_local(),
      [inbox, routes](Text::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(inbox->mutex);
        if (routes) {
          inbox->routes = msg->data;
        } else {
          if (inbox->samples.size() == 256) {
            inbox->samples.pop_front();
          }
          inbox->samples.push_back(msg->data);
        }
      });
  }

  void drain()
  {
    std::deque<std::string> pending;
    std::string routes;
    {
      std::lock_guard<std::mutex> lock(inbox_->mutex);
      pending.swap(inbox_->samples);
      routes.swap(inbox_->routes);
    }
    QWidget * host = this;
    while (host->parentWidget()) {
      host = host->parentWidget();
    }
    widget_->setRadians(host->property("imt342_radians").toBool());
    if (!routes.empty()) {
      widget_->ingestRoutes(QJsonDocument::fromJson(QByteArray::fromStdString(routes)).object());
    }
    for (const auto & sample : pending) {
      widget_->ingest(QJsonDocument::fromJson(QByteArray::fromStdString(sample)).object());
    }
  }

  AnalysisWidget * widget_;
  std::shared_ptr<Inbox> inbox_ = std::make_shared<Inbox>();
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<Text>::SharedPtr samples_;
  rclcpp::Subscription<Text>::SharedPtr routes_;
};
}  // namespace imt342_irb120_rviz

PLUGINLIB_EXPORT_CLASS(imt342_irb120_rviz::AnalysisPanel, rviz_common::Panel)
