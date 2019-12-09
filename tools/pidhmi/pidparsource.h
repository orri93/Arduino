#ifndef PIDPARSOURCE_H
#define PIDPARSOURCE_H

#include <cstdint>

#include <QObject>
#include <QSettings>
#include <QMetaType>
#include <QDebug>

namespace gos {
namespace arduino {
namespace tools {
namespace pid {
namespace datasource {

class Parameters : public QObject {
  Q_OBJECT

  /* PID */
  Q_PROPERTY(uint16_t manual READ manual WRITE setManual NOTIFY manualChanged)
  Q_PROPERTY(float setpoint READ setpoint WRITE setSetpoint NOTIFY setpointChanged)

  Q_PROPERTY(float kp READ kp WRITE setKp NOTIFY kpChanged)
  Q_PROPERTY(float ki READ ki WRITE setKi NOTIFY kiChanged)
  Q_PROPERTY(float kd READ kd WRITE setKd NOTIFY kdChanged)
  Q_PROPERTY(float ti READ ti WRITE setTi NOTIFY tiChanged)
  Q_PROPERTY(float td READ td WRITE setTd NOTIFY tdChanged)

public:
  explicit Parameters(QObject* parent = nullptr);

  /* PID Parameters */
  const uint16_t& manual() const;
  const float& setpoint() const;

  const float& kp() const;
  const float& ki() const;
  const float& kd() const;
  const float& ti() const;
  const float& td() const;

  void setManual(const uint16_t& value);
  void setSetpoint(const float& value);
  void setKp(const float& value);
  void setKi(const float& value);
  void setKd(const float& value);
  void setTi(const float& value);
  void setTd(const float& value);

signals:
  void manualChanged();
  void setpointChanged();
  void kpChanged();
  void kiChanged();
  void kdChanged();
  void tiChanged();
  void tdChanged();

private:
  uint16_t manual_;
  float setpoint_;
  float kp_;
  float ki_;
  float kd_;
  float ti_;
  float td_;
};

}
}
}
}
}

#endif
