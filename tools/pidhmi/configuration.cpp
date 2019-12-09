#include <QDir>

#include <configuration.h>

/* Communication configuration */
#define GROUP_COMMUNICATION "Communication"
#define KEY_SERIAL_PORT "SerialPort"
#define KEY_SERIAL_BAUD "SerialBaud"

#define DEFAULT_SERIAL_PORT "COM1"
#define DEFAULT_SERIAL_BAUD 9600

/* Modbus configuration */
#define GROUP_MODBUS "Modbus"
#define KEY_SLAVE_ID "SlaveId"

#define DEFAULT_SLAVE_ID 1

/* Timers configuration */
#define GROUP_TIMERS "Timers"

#define KEY_LOOP_INTERVAL "LoopInterval"
#define KEY_REFRESH_INTERVAL "RefreshInterval"

#define DEFAULT_LOOP_INTERVAL 250
#define DEFAULT_REFRESH_INTERVAL 100

  /* UI configuration */
#define GROUP_UI "Ui"

namespace gos {
namespace arduino {
namespace tools {
namespace pid {

Configuration::Configuration(
  const QString& filepath,
  QObject* parent)
  : QObject(parent), filepath_(filepath) {
}

Configuration::Configuration(QObject* parent) :
  QObject(parent),
  filepath_(GOS_CONFIGURATION_FILE_PATH) {
}

QSettings* Configuration::read() {
  if (!settings_) {
    if (!create()) {
      return nullptr;
    }
  }
  settings_->sync();

  QVariant value;
  //  ndb::type::


  /* Communication configuration */
  settings_->beginGroup(GROUP_COMMUNICATION);
  value = settings_->value(KEY_SERIAL_PORT, DEFAULT_SERIAL_PORT);
  setSerialPort(value.toString());
  value = settings_->value(KEY_SERIAL_BAUD, DEFAULT_SERIAL_BAUD);
  setSerialBaud(value.toInt());
  settings_->endGroup();

  /* Modbus configuration */
  settings_->beginGroup(GROUP_MODBUS);
  value = settings_->value(KEY_SLAVE_ID, DEFAULT_SLAVE_ID);
  setSlaveId(value.toInt());
  settings_->endGroup();

  /* Timers configuration */
  settings_->beginGroup(GROUP_TIMERS);
  value = settings_->value(KEY_LOOP_INTERVAL, DEFAULT_LOOP_INTERVAL);
  setLoopInterval(value.toInt());
  value = settings_->value(KEY_REFRESH_INTERVAL, DEFAULT_REFRESH_INTERVAL);
  setRefreshInterval(value.toInt());
  settings_->endGroup();

  /* Ui configuration */

  return settings_.get();
}

QSettings* Configuration::write(const bool& sync) {
  if (!settings_) {
    if (!create()) {
      return nullptr;
    }
  }

  /* Communication configuration */
  settings_->beginGroup(GROUP_COMMUNICATION);
  settings_->setValue(KEY_SERIAL_PORT, serialPort_);
  settings_->setValue(KEY_SERIAL_BAUD, serialBaud_);
  settings_->endGroup();

  /* Modbus configuration */
  settings_->beginGroup(GROUP_MODBUS);
  settings_->setValue(KEY_SLAVE_ID, slaveId_);
  settings_->endGroup();

  /* Timers configuration */
  settings_->beginGroup(GROUP_TIMERS);
  settings_->setValue(KEY_LOOP_INTERVAL, loopInterval_);
  settings_->setValue(KEY_REFRESH_INTERVAL, refreshInterval_);
  settings_->endGroup();

  /* UI configuration */

  if (sync) {
    settings_->sync();
  }
  return settings_.get();
}

bool Configuration::create() {
  //QString filepath = QDir::cleanPath(path_ + QDir::separator() + filename_);
  settings_.reset(new QSettings(filepath_, SettingsFormat));
  if (settings_) {
    return true;
  } else {
    qCritical() << "Out of memory when trying to create a Qt Setting";
    return false;
  }
}

/* Communication configuration */
const QString& Configuration::serialPort() const {
  return serialPort_;
}

const int& Configuration::serialBaud() const {
  return serialBaud_;
}

/* Modbus configuration */
const int& Configuration::slaveId() const {
  return slaveId_;
}

/* Timers configuration */
const int& Configuration::loopInterval() const {
  return loopInterval_;
}

const int& Configuration::refreshInterval() const {
  return refreshInterval_;
}

/* UI configuration */


/* Communication configuration */
void Configuration::setSerialPort(const QString& value) {
  if (serialPort_ != value) {
    serialPort_ = value;
    qDebug() << "Setting serial port to " << serialPort_;
    emit serialPortChanged();
  }
}

void Configuration::setSerialBaud(const int& value) {
  if (serialBaud_ != value) {
    serialBaud_ = value;
    qDebug() << "Setting serial Baud to " << serialBaud_;
    emit serialBaudChanged();
  }
}

/* Modbus configuration */
void Configuration::setSlaveId(const int& value) {
  if (slaveId_ != value) {
    slaveId_ = value;
    qDebug() << "Setting slave id to " << slaveId_;
    emit slaveIdChanged();
  }
}

/* Timers configuration */
void Configuration::setLoopInterval(const int& value) {
  if (loopInterval_ != value) {
    loopInterval_ = value;
    qDebug() << "Setting loop interval to " << loopInterval_;
    emit loopIntervalChanged();
  }
}

void Configuration::setRefreshInterval(const int& value) {
  if (refreshInterval_ != value) {
    refreshInterval_ = value;
    qDebug() << "Setting refresh interval to " << refreshInterval_;
    emit refreshIntervalChanged();
  }
}

/* UI configuration */


namespace initialize {
bool configuration(::gos::arduino::tools::pid::Configuration& configuration) {
  QSettings* settings = configuration.read();
  if (settings != nullptr) {
    qInfo() << "Read the configuration successfully";
    return true;
  } else {
    qCritical() << "Failed to read the configuration";
    return false;
  }
}
}

}
}
}
}
