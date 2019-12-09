import QtQuick 2.1
import QtQuick.Layouts 1.2
import QtQuick.Controls 2.12

SpinBox {
    id: realSpinBox
    from: 0
    value: 0
    to: 100 * 1000
    stepSize: 10
    editable: true
    /* anchors.centerIn: parent */

    property int decimals: 3

    validator: DoubleValidator {
        bottom: Math.min(realSpinBox.from, realSpinBox.to)
        top:  Math.max(realSpinBox.from, realSpinBox.to)
    }

    textFromValue: function(value, locale) {
        return Number(value / 1000).toLocaleString(
          locale, 'f', realSpinBox.decimals)
    }

    valueFromText: function(text, locale) {
        return 1000 * Number.fromLocaleString(locale, text)
    }
}
