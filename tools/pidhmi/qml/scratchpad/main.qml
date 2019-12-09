import QtQuick 2.4
import QtQuick.Controls 1.3
import QtQuick.Layouts 1.1
import QtQml.Models 2.1

ApplicationWindow {
id: root_window
title: "Hello world"
visible: true
color: "white"
width: 480
height: 520

Rectangle {
    id: smon_user_app_header
    height: 50
    color: "blue"
    width: parent.width
}

ListView {
    id: navigation

    orientation: ListView.Horizontal
    interactive: true // disable manual pageChange

    snapMode: ListView.SnapOneItem // while moving to right, finish move
    highlightRangeMode: ListView.StrictlyEnforceRange // mouse gesture make currentIndex change
    highlightMoveDuration: 400 // speed up pages change (swap)

    anchors.top: smon_user_app_header.bottom
    anchors.bottom: root_window.bottom

    width: parent.width
    height: 400

    model: ObjectModel {
        /* First page with login capabilities */
        Rectangle{
            id: one
            anchors.fill: parent
            color: "red"
        }
    }
}
}
