QT += core
QT -= gui
QT += network
CONFIG += C++11

TARGET = EAICar-vision
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    dector.cpp \
    socketclient.cpp


HEADERS += \
    dector.h \
    socketclient.h \
#    dector.h \
#    socketclient.h


INCLUDEPATH += /usr/local/include \
/usr/local/include/opencv \
/usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_*.so \
