#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <memory>

#include <QObject>
#include <QSettings>
#include <QMetaType>
#include <QDebug>

#define GOS_CONFIGURATION_FILE_PATH "configuration.ini"

namespace gos {
namespace arduino {
namespace tools {
namespace pid {

class Configuration : public QObject {
  Q_OBJECT

  /* Communication */
  Q_PROPERTY(QString serialPort READ serialPort NOTIFY serialPortChanged)
  Q_PROPERTY(int serialBaud READ serialBaud NOTIFY serialBaudChanged)
  
  /* Modbus */
  Q_PROPERTY(int slaveId READ slaveId NOTIFY slaveIdChanged)

  /* Timers */
  Q_PROPERTY(int loopInterval READ loopInterval NOTIFY loopIntervalChanged)
  Q_PROPERTY(int refreshInterval READ refreshInterval NOTIFY refreshIntervalChanged)

  /* UI configuration */

  /* Chart*/
#ifdef GOS_NOT_YET_USED
    antialiasing

#endif

protected:
  const QSettings::Format SettingsFormat = QSettings::IniFormat;

public:
  explicit Configuration(const QString& filepath, QObject* parent = nullptr);
  explicit Configuration(QObject* parent = nullptr);

  virtual QSettings* read();
  virtual QSettings* write(const bool& sync);

  /*
   * Value access methods
   */

   /* Communication configuration */
  const QString& serialPort() const;
  const int& serialBaud() const;

  /* Modbus configuration */
  const int& slaveId() const;

  /* Timers configuration */
  const int& loopInterval() const;
  const int& refreshInterval() const;

  /* UI configuration */

signals:
  /* Communication configuration */
  void serialPortChanged();
  void serialBaudChanged();
  /* Modbus configuration */
  void slaveIdChanged();
  /* Timers configuration */
  void loopIntervalChanged();
  void refreshIntervalChanged();

  /* UI configuration */

private:
  typedef std::unique_ptr<QSettings> SettingsPointer;

  SettingsPointer settings_;

  bool create();

  /* Communication configuration */
  void setSerialPort(const QString& value);
  void setSerialBaud(const int& value);

  /* Modbus configuration */
  void setSlaveId(const int& value);

  /* Timers configuration */
  void setLoopInterval(const int& value);
  void setRefreshInterval(const int& value);

  /* UI configuration */

  QString filepath_;
  
  /* Communication configuration */
  QString serialPort_;
  int serialBaud_;
  /* Modbus configuration */
  int slaveId_;
  int holdingRegistryStartAddress_;
  /* Timers configuration */
  int loopInterval_;
  int refreshInterval_;
  /* UI configuration */
};

namespace initialize {
bool configuration(::gos::arduino::tools::pid::Configuration& configuration);
}

}
}
}
}

#endif
