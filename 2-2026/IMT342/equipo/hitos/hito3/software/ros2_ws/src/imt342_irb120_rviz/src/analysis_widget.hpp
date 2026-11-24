#ifndef IMT342_IRB120_RVIZ__ANALYSIS_WIDGET_HPP_
#define IMT342_IRB120_RVIZ__ANALYSIS_WIDGET_HPP_

#include <memory>

#include <QString>
#include <QWidget>

class QJsonObject;

namespace imt342_irb120_rviz
{
class AnalysisWidget : public QWidget
{
public:
  struct CaptureSummary
  {
    int samples{};
    int events{};
    QString trial_id;
    QString trial_type;
    QString trial_result;
    QString timestamp_ns;
    QString exported_directory;
  };

  explicit AnalysisWidget(QWidget * parent = nullptr);
  ~AnalysisWidget() override;

  void ingest(const QJsonObject & sample);
  void ingestRoutes(const QJsonObject & routes);
  void setRadians(bool enabled);
  void selectDiagnostics();
  void selectEvidence();
  CaptureSummary captureSummary() const;

  /// Exporta el búfer de captura o un único ensayo si se indica su ID.
  bool exportEvidence(
    const QString & directory, QString * error = nullptr, const QString & run_id = {});

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace imt342_irb120_rviz

#endif  // IMT342_IRB120_RVIZ__ANALYSIS_WIDGET_HPP_
