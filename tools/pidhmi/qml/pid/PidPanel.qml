import QtQuick 2.1
import QtQuick.Layouts 1.2
import QtQuick.Controls 2.12

ColumnLayout {
  id: pidParameterPanel

  property string tunePlaceholderText: qsTr("0.0")

  Component.onCompleted: {
    enableInput(false)
  }

  Text {
    text: "PID"
    Layout.fillWidth: false
    font.pointSize: 18
  }

  GridLayout {
    columns: 2
    rows: 9

    ColumnLayout {
      id: manualColumn
      Layout.fillWidth: true
      Layout.column: 0
      Layout.row: 0
      Label {
        text: qsTr("Manual")
      }
      SpinBox {
        id: manualInput
        editable: true
        Layout.fillWidth: true
        onValueChanged: {
          orchestration.manual = value;
        }
      }
      Connections {
        target: orchestration
        onManualChanged: {
          manualInput.value = orchestration.manual;
        }
      }
    }

    ColumnLayout {
      id: setpointColumn
      Layout.fillWidth: true
      Layout.column: 0
      Layout.row: 1
      Label {
        text: qsTr("Setpoint")
      }
      RealSpinBox {
        id: setpointInput
        Layout.fillWidth: true
        decimals: 1
        onValueChanged: {
          orchestration.setpoint = value / 1000;
        }
      }
      Connections {
        target: orchestration
        onSetpointChanged: {
          setpointInput.value = 1000 * orchestration.setpoint;
        }
      }
    }

    ColumnLayout {
      id: kpColumn
      Layout.fillWidth: true
      Layout.column: 0
      Layout.row: 2
      Label {
        text: qsTr("Kp")
      }
      RealSpinBox {
        id: kpInput
        Layout.fillWidth: true
        onValueChanged: {
          orchestration.kp = value / 1000;
        }
      }
      Connections {
        target: orchestration
        onSetpointChanged: {
          kpInput.value = 1000 * orchestration.kp;
        }
      }
    }

    ColumnLayout {
      id: tuneColumn
      Layout.fillWidth: true
      Layout.column: 1
      Layout.row: 2
      Label {
        text: qsTr("Tunning")
      }
      Button {
        id: tuneButton
        text: qsTr("--")
        onClicked: {
          orchestration.switchTuning();
        }
      }
      Connections {
        target: orchestration
        onIsTunningWithTChanged: {
          if(orchestration.isTunningWithT) {
            tuneButton.text = qsTr("With T");
          } else {
            tuneButton.text = qsTr("With K");
          }
        }
      }
    }

    ColumnLayout {
      id: kiColumn
      Layout.fillWidth: true
      Layout.column: 0
      Layout.row: 3
      Label {
        text: qsTr("Ki")
      }
      RealSpinBox {
        id: kiInput
        Layout.fillWidth: true
        onValueChanged: {
          orchestration.ki = value / 1000;
        }
      }
      Connections {
        target: orchestration
        onSetpointChanged: {
          kiInput.value = 1000 * orchestration.ki;
        }
      }
    }

    ColumnLayout {
      id: tiColumn
      Layout.fillWidth: true
      Layout.column: 1
      Layout.row: 3
      Label {
        text: qsTr("Ti")
      }
      RealSpinBox {
        id: tiInput
        Layout.fillWidth: true
        onValueChanged: {
          orchestration.ti = value / 1000;
        }
      }
      Connections {
        target: orchestration
        onSetpointChanged: {
          tiInput.value = 1000 * orchestration.ti;
        }
      }
    }

    ColumnLayout {
      id: kdColumn
      Layout.fillWidth: true
      Layout.column: 0
      Layout.row: 4
      Label {
        text: qsTr("Kd")
      }
      RealSpinBox {
        id: kdInput
        Layout.fillWidth: true
        onValueChanged: {
          orchestration.kd = value / 1000;
        }
      }
      Connections {
        target: orchestration
        onSetpointChanged: {
          kdInput.value = 1000 * orchestration.kd;
        }
      }
    }

    ColumnLayout {
      id: tdColumn
      Layout.fillWidth: true
      Layout.column: 1
      Layout.row: 4
      Label {
        text: qsTr("Td")
      }
      RealSpinBox {
        id: tdInput
        Layout.fillWidth: true
        onValueChanged: {
          orchestration.td = value / 1000;
        }
      }
      Connections {
        target: orchestration
        onSetpointChanged: {
          tdInput.value = 1000 * orchestration.td;
        }
      }
    }

    ColumnLayout {
      id: frequencyColumn
      Layout.fillWidth: true
      Layout.column: 0
      Layout.row: 5
      Layout.columnSpan: 2
      Label {
        text: qsTr("Frequency")
      }
      Slider {
        id: frequencySlider
        stepSize: 0.25
        value: 1
        from: 0.1
        to: 10
        Layout.fillWidth: true
        onValueChanged: {
          orchestration.refreshFrequency = value;
        }
      }
    }

    Button {
      id: connectDisconnectButton
      Layout.fillWidth: true
      Layout.column: 0
      Layout.row: 6
      text: qsTr("Connect")
      onClicked: {
        orchestration.connectDisconnect();
      }
    }

    Button {
      id: startStopLoggingButton
      Layout.fillWidth: true
      Layout.column: 1
      Layout.row: 6
      text: qsTr("Start logging")
      onClicked: {
        orchestration.startStopLogging();
      }
    }

    ColumnLayout {
      Layout.fillWidth: true
      Layout.column: 0
      Layout.row: 7
    }

    ColumnLayout {
      Layout.fillWidth: true
      Layout.column: 1
      Layout.row: 7
      Label {
        text: qsTr("Interval")
      }
      Text {
        id: intervalText
        text: qsTr("0 ms")
        Layout.minimumWidth: 130
      }
      Connections {
        target: orchestration
        onRefreshIntervalChanged: {
          intervalText.text = orchestration.refreshInterval.toString();
        }
      }
    }

    ColumnLayout {
      Layout.fillWidth: true
      Layout.columnSpan: 2
      Layout.column: 0
      Layout.row: 8
      Label {
        text: qsTr("Status")
      }
      Text {
        id: statusText
        text: qsTr("Starting")
        Layout.fillWidth: true
      }
      Connections {
        target: orchestration
        onStatusStringChanged: {
          statusText.text = orchestration.statusString;
        }
      }
    }
  }

  Connections {
    target: orchestration
    onIsLoggingChanged: {
      if(orchestration.isLogging) {
        startStopLoggingButton.text = "Stop logging";
      } else {
        startStopLoggingButton.text = "Start logging";
      }
    }
  }

  Connections {
    target: orchestration
    onIsConnectedChanged: {
      enableInput(orchestration.isConnected);
      if(orchestration.isConnected) {
        intervalText.text = orchestration.refreshInterval.toString();
        connectDisconnectButton.text = "Disconnect";
        frequencySlider.value = orchestration.refreshFrequency;
        manualInput.value = orchestration.manual;
        setpointInput.value = 1000 * orchestration.setpoint;
        kpInput.value = 1000 * orchestration.kp;
        kiInput.value = 1000 * orchestration.ki;
        kdInput.value = 1000 * orchestration.kd;
        tiInput.value = 1000 * orchestration.ti;
        tdInput.value = 1000 * orchestration.td;
      } else {
        connectDisconnectButton.text = "Connect";
      }
    }
  }

  function enableInput(enable) {
    manualInput.enabled = enable
    setpointInput.enabled = enable
    kpInput.enabled = enable
    kiInput.enabled = enable
    kdInput.enabled = enable
    tiInput.enabled = enable
    tdInput.enabled = enable
  }
}
