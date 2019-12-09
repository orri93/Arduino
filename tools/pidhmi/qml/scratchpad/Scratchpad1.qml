import QtQuick 2.1
import QtQuick.Layouts 1.2
import QtQuick.Controls 2.12

ColumnLayout {
  id: pidParameterPanel

  property int inputWidth: 220

  property alias manualInput: manualInput
  property alias setpointInput: setpointInput
  property alias kpInput: kpInput
  property alias kiInput: kiInput
  property alias kdInput: kdInput
  property alias tiInput: tiInput
  property alias tdInput: tdInput

  property string tunePlaceholderText: qsTr("0.0")

  DoubleValidator {
    id: tuneValidator
    top: 99.0
    bottom: 0.0
    decimals: 3
    notation: DoubleValidator.StandardNotation
    locale: qsTr("en_EN")
  }

  Text {
    text: "PID"
    font.pointSize: 18
  }

  GridLayout {
    columns: 2
    rows: 2

    ColumnLayout {
      id: manualColumn
      Layout.column: 0
      Layout.row: 0
      Label {
        text: qsTr("Manual")
      }
      SpinBox {
        id: manualInput
      }
    }
    ColumnLayout {
      id: kiColumn
      Layout.column: 0
      Layout.row: 1
      Label {
        text: qsTr("Ki")
      }
      FloatSpinBox {
        id: kiInput
      }
    }
    ColumnLayout {
      id: tiColumn
      Layout.column: 1
      Layout.row: 1
      Label {
        text: qsTr("Ti")
      }
      FloatSpinBox {
        id: tiInput
      }
    }
  }
}
