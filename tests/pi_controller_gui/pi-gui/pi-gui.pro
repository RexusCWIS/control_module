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
    serialportlistener.cpp \
    crc.c \
    crc_table.c \
    centralwidget.cpp \
    serialportdialog.cpp

HEADERS  += mainwindow.h \
    qcustomplot.h \
    serialportlistener.h \
    experiment.h \
    crc.h \
    centralwidget.h \
    serialportdialog.h \
    serialportconfig.h
