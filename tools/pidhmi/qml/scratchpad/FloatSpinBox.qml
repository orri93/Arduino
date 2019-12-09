import QtQuick 2.1
import QtQuick.Layouts 1.2
import QtQuick.Controls 2.12

SpinBox {
    id: spinbox
    from: 0
    to: 100 * (10 ^ decimals)
    stepSize: (10 ^ decimals)
    anchors.centerIn: parent

    property int decimals: 3
    property real realValue: value / (10 ^ decimals)

    validator: DoubleValidator {
        bottom: Math.min(spinbox.from, spinbox.to)
        top:  Math.max(spinbox.from, spinbox.to)
    }

    textFromValue: function(value, locale) {
        return Number(value / (10 ^ decimals)).toLocaleString(
          locale, 'f', spinbox.decimals)
    }

    valueFromText: function(text, locale) {
        return Number.fromLocaleString(locale, text) * (10 ^ decimals)
    }
}
