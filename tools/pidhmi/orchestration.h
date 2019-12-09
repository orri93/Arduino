#ifndef ORCHESTRATION_H_
#define ORCHESTRATION_H_

#include <memory>

#include <QTimer>
#include <QtCore/QObject>
#include <QtCharts/QAbstractSeries>
#include <QtQml/QQmlContext>

#include <configuration.h>
#include <pidparsource.h>

QT_BEGIN_NAMESPACE
class QQuickView;
QT_END_NAMESPACE

QT_CHARTS_USE_NAMESPACE

namespace gos {
namespace arduino {
namespace tools {
namespace pid {

class Orchestration : public QObject {
  Q_OBJECT
public:
  explicit Orchestration(QQuickView* appViewer, QObject* parent = nullptr);
  ~Orchestration();

  Q_PROPERTY(bool isTunningWithT READ isTunningWithT NOTIFY isTunningWithTChanged)
  Q_PROPERTY(bool isConnected READ isConnected NOTIFY isConnectedChanged)
  Q_PROPERTY(bool isLogging READ isLogging NOTIFY isLoggingChanged)
  Q_PROPERTY(QString statusString READ statusString NOTIFY statusStringChanged)
  Q_PROPERTY(QString lastErrorString READ lastErrorString NOTIFY lastErrorStringChanged)
  Q_PROPERTY(errno_t lastErrorNumber READ lastErrorNumber NOTIFY lastErrorNumberChanged)
  Q_PROPERTY(int refreshInterval READ refreshInterval NOTIFY refreshIntervalChanged)
  Q_PROPERTY(double refreshFrequency READ refreshFrequency WRITE setRefreshFrequency NOTIFY refreshFrequencyChanged)
  Q_PROPERTY(int manual READ manual WRITE setManual NOTIFY manualChanged)
  Q_PROPERTY(double setpoint READ setpoint WRITE setSetpoint NOTIFY setpointChanged)
  Q_PROPERTY(double kp READ kp WRITE setKp NOTIFY kpChanged)
  Q_PROPERTY(double ki READ ki WRITE setKi NOTIFY kiChanged)
  Q_PROPERTY(double kd READ kd WRITE setKd NOTIFY kdChanged)
  Q_PROPERTY(double ti READ ti WRITE setTi NOTIFY tiChanged)
  Q_PROPERTY(double td READ td WRITE setTd NOTIFY tdChanged)

  bool initialize(QQmlContext* context);

  Q_INVOKABLE bool connectDisconnect();
  Q_INVOKABLE bool startStopLogging();
  Q_INVOKABLE bool switchTuning();

  bool isTunningWithT();
  bool isConnected();
  bool isLogging();
  QString statusString();
  QString lastErrorString();
  errno_t lastErrorNumber();
  int refreshInterval();
  double refreshFrequency();
  int manual();
  double setpoint();
  double kp();
  double ki();
  double kd();
  double ti();
  double td();

signals:
  /* Communication configuration */
  void isTunningWithTChanged();
  void isConnectedChanged();
  void isLoggingChanged();
  void statusStringChanged();
  void lastErrorStringChanged();
  void lastErrorNumberChanged();
  void refreshIntervalChanged();
  void refreshFrequencyChanged();
  void manualChanged();
  void setpointChanged();
  void kpChanged();
  void kiChanged();
  void kdChanged();
  void tiChanged();
  void tdChanged();

Q_SIGNALS:
//  void quit();
//  void exit(int retCode);

public Q_SLOTS:
//  bool close();

public slots:
  void setRefreshFrequency(const double& value);
  void setManual(const int& value);
  void setSetpoint(const double& value);
  void setKp(const double& value);
  void setKi(const double& value);
  void setKd(const double& value);
  void setTi(const double& value);
  void setTd(const double& value);
  int update(
    QAbstractSeries* output,
    QAbstractSeries* temperature,
    QAbstractSeries* setpoints);

private:
  typedef QVector<QPointF> VectorList;
  typedef std::unique_ptr<Configuration> ConfigurationPointer;
  typedef std::unique_ptr<::gos::arduino::tools::pid::datasource::Parameters>
    ParametersPointer;

  void setIsConnected(const bool& value);
//void setIsLogging(const bool& value);
  void setStatusString(const QString& value);
  void setLastErrorString(const QString& value);
  void setLastErrorNumber(const errno_t& value);
  void setRefreshInterval(const int& value);

  QQuickView* appViewer_;
  VectorList setpoints_;
  VectorList temperature_;
  VectorList outputs_;
  int count_;

  ConfigurationPointer configuration_;
  ParametersPointer parameters_;

  bool isTunningWithT_;
  bool isConnected_;
  QString statusString_;
  QString lastErrorString_;
  errno_t lastErrorNumber_;
  int refreshInterval_;
  double refreshFrequency_;
  int manual_;
  double setpoint_;
  double kp_;
  double ki_;
  double kd_;
  double ti_;
  double td_;
};


}
}
}
}

#endif
