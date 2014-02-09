#-------------------------------------------------
#
# Project created by QtCreator 2014-02-06T00:07:19
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport serialport

TARGET = pi-gui
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    qcustomplot.cpp \
    crc.c \
    crc_table.c \
    centralwidget.cpp \
    serial/serialportlistener.cpp \
    serial/serialportdialog.cpp \
    serial/serialframedescriptor.cpp

HEADERS  += mainwindow.h \
    qcustomplot.h \
    experiment.h \
    crc.h \
    centralwidget.h \
    serial/serialportlistener.h \
    serial/serialportdialog.h \
    serial/serialportconfig.h \
    serial/serialframedescriptor.h
