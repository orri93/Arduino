#include <sstream>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <memory>
#include <chrono>

#include <QDebug>

#include <QtCharts/QXYSeries>
#include <QtCharts/QAreaSeries>
#include <QtQuick/QQuickView>
#include <QtQuick/QQuickItem>
#include <QtCore/QRandomGenerator>
#include <QtCore/QtMath>

#include <types.h>
#include <orchestration.h>
#include <master.h>

namespace gatp = ::gos::arduino::tools::pid;
namespace gatpm = ::gos::arduino::tools::pid::master;
namespace gatpd = ::gos::arduino::tools::pid::datasource;

QT_CHARTS_USE_NAMESPACE

Q_DECLARE_METATYPE(QAbstractSeries*)
Q_DECLARE_METATYPE(QAbstractAxis*)

namespace gos {
namespace arduino {
namespace tools {
namespace pid {

std::unique_ptr<std::ofstream> _file;

typedef std::chrono::steady_clock Clock;
typedef Clock::duration Duration;
typedef Clock::time_point Time;

Time _start;

status _status = status::undefined;

namespace real {
QString format(const float& real, const int& precision) {
  std::stringstream stream;
  stream << std::setprecision(precision) << real;
  return QString::fromStdString(stream.str());
}
float parse(const QString& string) {
  return static_cast<float>(string.toDouble());
}
}

Orchestration::Orchestration(QQuickView* appViewer, QObject* parent) :
  QObject(parent),
  appViewer_(appViewer),
  count_(0),
  isTunningWithT_(false),
  isConnected_(false),
  lastErrorNumber_(0),
  refreshInterval_(1000),
  refreshFrequency_(1.0),
  manual_(0),
  setpoint_(0.0),
  kp_(0.0),
  ki_(0.0),
  kd_(0.0),
  ti_(0.0),
  td_(0.0) {
  qRegisterMetaType<QAbstractSeries*>();
  qRegisterMetaType<QAbstractAxis*>();
}

Orchestration::~Orchestration() {
  if (isConnected_) {
    gatp::master::disconnect();
  }
  gatp::master::shutdown();
}

bool Orchestration::initialize(QQmlContext* context) {
  gatp::master::result result;
  configuration_ = std::make_unique<Configuration>(this);
  if (configuration_) {
    QSettings* settings = configuration_->read();
    if (settings != nullptr) {
      parameters_ = std::make_unique<gatpd::Parameters>(this);
      if (parameters_) {
        QString serialPort = configuration_->serialPort();
        std::string serialport = serialPort.toStdString();
        int serialbaud = configuration_->serialBaud();
        int slaveid = configuration_->slaveId();
        result = gatp::master::initialize(
          serialport.c_str(),
          serialbaud,
          slaveid);
        if (result == gatpm::result::success) {
          setRefreshInterval(configuration_->refreshInterval());
          refreshFrequency_ = 1.0 / (refreshInterval_ / 1000.0);
          _status = status::idle;
          return true;
        }
      } else {
        qCritical() << "Failed to crate parameters";
      }
    } else {
      qCritical() << "Failed to create configuration";
    }
  } else {
    qCritical() << "Failed to create configuration";
  }
  return false;
}

bool Orchestration::connectDisconnect() {
  status saved = _status;
  gatp::master::result result;
  if (isConnected_) {
    _status = status::disconnecting;
    result = gatp::master::disconnect();
    if (result == gatp::master::result::success) {
      std::cout << "Disconnected successfully" << std::endl;
      setIsConnected(false);
      setStatusString("Disconnected");
      _status = status::idle;
      return true;
    } else {
      _status = saved;
      std::cerr << "Disconnecting failed" << std::endl;
      setLastErrorString(QString::fromStdString(gatpm::report::error::last()));
      setLastErrorNumber(gatp::master::report::error::errorno());
      return false;
    }
  } else {
    _status = status::connecting;
    result = gatp::master::connect();
    if (result == gatp::master::result::success) {
      result = gatpm::read::tunecoil(isTunningWithT_);
      if (result == gatpm::result::success) {
        std::cout << "Successfully got tuning with T as "
          << (isTunningWithT_ ? "true" : "false") << std::endl;
        emit isTunningWithTChanged();
      }
      uint16_t m;
      float s, kp, ki, kd, ti, td;
      result = gatpm::read::holding(m, s, kp, ki, kd, ti, td);
      if (result == gatpm::result::success) {
        setManual(m);
        setSetpoint(static_cast<double>(s));
        setKp(static_cast<double>(kp));
        _status = status::connected;
        setStatusString("Connected");
        setIsConnected(true);
        return true;
      }
    } else {
      setLastErrorString(QString::fromStdString(gatpm::report::error::last()));
      setLastErrorNumber(gatp::master::report::error::errorno());
    }
    _status = saved;
    return false;
  }
}

bool Orchestration::startStopLogging() {
  if (_file) {
    if (_file->is_open()) {
      std::cout << "Closing the logging file" << std::endl;
      _file->flush();
      _file->close();
    }
    _file.reset(nullptr);
    emit isLoggingChanged();
    return true;
  } else {
    _file = std::make_unique<std::ofstream>("pid.csv", std::ios::out);
    if (_file->is_open()) {
      _start = Clock::now();
      std::cout << "Logging file opened successful" << std::endl;
      emit isLoggingChanged();
      return true;
    } else {
      std::cerr << "Failed to open a logging file" << std::endl;
      _file.reset(nullptr);
    }
  }
  return false;
}

bool Orchestration::switchTuning() {
  bool saved = isTunningWithT_;
  gatp::master::result result;
  if (isConnected_) {
    if (isTunningWithT_) {
      result = gatp::master::set::tunecoil(false);
      if (result == gatpm::result::success) {
        std::cout << "Successful set tuning to K" << std::endl;
        isTunningWithT_ = false;
      } else {
        std::cout << "Failed to set tuning to K" << std::endl;
        return false;
      }
    } else {
      result = gatp::master::set::tunecoil(true);
      if (result == gatpm::result::success) {
        std::cout << "Successful set tuning to T" << std::endl;
        isTunningWithT_ = true;
      } else {
        std::cout << "Failed to set tuning to T" << std::endl;
        return false;
      }
    }
  }
  if (saved != isTunningWithT_) {
    emit isTunningWithTChanged();
  }
  return true;
}

bool Orchestration::isTunningWithT() { return isTunningWithT_; }
bool Orchestration::isConnected() { return isConnected_; }
bool Orchestration::isLogging() {
  if (_file) {
    return _file->is_open();
  } else {
    return false;
  }
}
QString Orchestration::statusString() { return statusString_; }
QString Orchestration::lastErrorString() { return lastErrorString_; }
errno_t Orchestration::lastErrorNumber() { return lastErrorNumber_; }
int Orchestration::refreshInterval() { return refreshInterval_; }
double Orchestration::refreshFrequency() { return refreshFrequency_; }
int Orchestration::manual() { return manual_; }
double Orchestration::setpoint() { return setpoint_; }
double Orchestration::kp() { return kp_; }
double Orchestration::ki() { return ki_; }
double Orchestration::kd() { return kd_; }
double Orchestration::ti() { return ti_; }
double Orchestration::td() { return td_; }

void Orchestration::setRefreshFrequency(const double& value) {
  if (refreshFrequency_ != value && value >= 0.1 && value <= 10) {
    refreshFrequency_ = value;
    setRefreshInterval(static_cast<int>(1000.0 / refreshFrequency_));
    emit refreshFrequencyChanged();
  }
}

void Orchestration::setManual(const int& manual) {
  if (manual_ != manual && manual >= 0 && manual <= 255) {
    if (_status == status::connected) {
      gatp::master::result result =
        gatp::master::set::manual(manual);
    }
    manual_ = manual;
    emit manualChanged();
  }
}

void Orchestration::setSetpoint(const double& setpoint) {
  if (setpoint_ != setpoint && setpoint >= 0.0 && setpoint <= 300.0) {
    if (_status == status::connected) {
      gatp::master::result result =
        gatp::master::set::setpoint(setpoint);
    }
    setpoint_ = setpoint;
    emit setpointChanged();
  }
}

void Orchestration::setKp(const double& value) {
  if (kp_ != value && value >= 0.0 && value <= 100.0) {
    if (_status == status::connected) {
      gatp::master::result result =
        gatp::master::set::kp(value);
      if (result == gatpm::result::success) {
        qDebug() << "Successfully set Kp to " << kp_;
      } else {
        qWarning() << "Failed to set Kp to " << value << " error is " <<
          QString::fromStdString(gatp::master::report::error::last());
      }
    } else {
      qDebug() << "Not connected when Kp changed from "
        << kp_ << " to " << value;
    }
    kp_ = value;
    emit kpChanged();
  }
}

void Orchestration::setKi(const double& value) {
  if (ki_ != value && value >= 0.0 && value <= 100.0) {
    if (_status == status::connected) {
      gatp::master::result result =
        gatp::master::set::ki(value);
      if (result == gatpm::result::success) {
        qDebug() << "Successfully set Ki to " << ki_;
      } else {
        qWarning() << "Failed to set Ki to " << value << " error is " <<
          QString::fromStdString(gatp::master::report::error::last());
      }
    } else {
      qDebug() << "Not connected when Ki changed from "
        << ki_ << " to " << value;
    }
    ki_ = value;
    emit kpChanged();
  }
}

void Orchestration::setKd(const double& value) {
  if (kd_ != value && value >= 0.0 && value <= 100.0) {
    if (_status == status::connected) {
      gatp::master::result result =
        gatp::master::set::kd(value);
      if (result == gatpm::result::success) {
        qDebug() << "Successfully set Kd to " << kd_;
      } else {
        qWarning() << "Failed to set Kd to " << value << " error is " <<
          QString::fromStdString(gatp::master::report::error::last());
      }
    } else {
      qDebug() << "Not connected when Kd changed from "
        << kd_ << " to " << value;
    }
    kd_ = value;
    emit kpChanged();
  }
}

void Orchestration::setTi(const double& value) {
  if (ti_ != value && value >= 0.0 && value <= 100.0) {
    if (_status == status::connected) {
      gatp::master::result result =
        gatp::master::set::ti(value);
      if (result == gatpm::result::success) {
        qDebug() << "Successfully set Ti to " << ti_;
      } else {
        qWarning() << "Failed to set Ti to " << value << " error is " <<
          QString::fromStdString(gatp::master::report::error::last());
      }
    } else {
      qDebug() << "Not connected when Ti changed from "
        << ti_ << " to " << value;
    }
    ti_ = value;
    emit kpChanged();
  }
}

void Orchestration::setTd(const double& value) {
  if (td_ != value && value >= 0.0 && value <= 100.0) {
    if (_status == status::connected) {
      gatp::master::result result =
        gatp::master::set::td(value);
      if (result == gatpm::result::success) {
        qDebug() << "Successfully set Td to " << td_;
      } else {
        qWarning() << "Failed to set Td to " << value << " error is " <<
          QString::fromStdString(gatp::master::report::error::last());
      }
    } else {
      qDebug() << "Not connected when Td changed from "
        << td_ << " to " << value;
    }
    td_ = value;
    emit kpChanged();
  }
}

int Orchestration::update(
  QAbstractSeries* output,
  QAbstractSeries* temperature,
  QAbstractSeries* setpoints) {
  if (_status == status::connected) {
    uint16_t output;
    float temperature;
    gatp::master::result result =
      gatp::master::read::inputs(output, temperature);
    if (result == gatp::master::result::success) {
      if (_file) {
        if (_file->is_open()) {
          Duration duration = Clock::now() - _start;
          Duration seconds =
            std::chrono::duration_cast<std::chrono::seconds>(duration);
          (*_file)
            << seconds.count() << ","
            << output << ","
            << temperature << ","
            << setpoint_ << std::endl;
          _file->flush();
        }
      }
      std::cout << "Getting " << output << " as output and "
        << temperature << " as temperature" << std::endl;
      std::cout << "Updating with " << count_ << " points" << std::endl;
      if (output >= 0 && output <= 255) {
        if (temperature >= 0.0 && temperature <= 300.0) {
          float x = static_cast<float>(count_);
          outputs_.append(QPointF(x, static_cast<double>(output)));
          temperature_.append(QPointF(x, static_cast<double>(temperature)));
          setpoints_.append(QPointF(x, setpoint_));
          count_++;
        }
      }
    }
  }
  if (count_ > 2) {
    if (output) {
      QXYSeries* xySeries = static_cast<QXYSeries*>(output);
      xySeries->replace(outputs_);
    }
    if (temperature) {
      QXYSeries* xySeries = static_cast<QXYSeries*>(temperature);
      xySeries->replace(temperature_);
    }
    if (setpoints) {
      QXYSeries* xySeries = static_cast<QXYSeries*>(setpoints);
      xySeries->replace(setpoints_);
    }
  }
  return count_;
}

void Orchestration::setIsConnected(const bool& value) {
  if (isConnected_ != value) {
    isConnected_ = value;
    emit isConnectedChanged();
  }
}

void Orchestration::setStatusString(const QString& value) {
  if (statusString_ != value) {
    statusString_ = value;
    emit statusStringChanged();
  }
}

void Orchestration::setLastErrorString(const QString& value) {
  if (lastErrorString_ != value) {
    lastErrorString_ = value;
    emit lastErrorStringChanged();
  }
}

void Orchestration::setLastErrorNumber(const errno_t& value) {
  if (lastErrorNumber_ != value) {
    lastErrorNumber_ = value;
    emit lastErrorNumberChanged();
  }
}

void Orchestration::setRefreshInterval(const int& value) {
  if (refreshInterval_ != value) {
    refreshInterval_ = value;
    emit refreshIntervalChanged();
  }
}

}
}
}
}

