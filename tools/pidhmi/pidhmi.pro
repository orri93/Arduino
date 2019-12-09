QT += charts qml quick

HEADERS += \
    master.h \
    datasource.h \
    configuration.h \
    orchestration.h \
    pidparsource.h

SOURCES += \
    main.cpp \
    master.cpp \
    datasource.cpp \
    configuration.cpp \
    orchestration.cpp \
    pidparsource.cpp

RESOURCES += \
    resources.qrc

DISTFILES += \
    qml/pid/* \
    qml/pid/Middle.qml \
    qml/qmloscilloscope/* \
    qml/scratchpad/* \
    qml/pid/*
