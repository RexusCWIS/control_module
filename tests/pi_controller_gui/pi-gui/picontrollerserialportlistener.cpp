#include "picontrollerserialportlistener.h"

PIControllerSerialPortListener::PIControllerSerialPortListener(QObject *parent) :
        SerialPortListener(parent) {}

PIControllerSerialPortListener::PIControllerSerialPortListener(QObject *parent,
                                                               const SerialFrameDescriptor &sfd,
                                                               const QString &device,
                                                               QSerialPort::BaudRate baudrate,
                                                               QSerialPort::DataBits dataBits,
                                                               QSerialPort::Parity parity,
                                                               QSerialPort::StopBits stopBits) :
    SerialPortListener(parent, sfd, device, baudrate, dataBits, parity, stopBits) {}

PIControllerSerialPortListener::PIControllerSerialPortListener(QObject *parent,
                                                               const SerialFrameDescriptor &sfd,
                                                               const SerialPortConfig &config) :
    SerialPortListener(parent, sfd, config) {}

void PIControllerSerialPortListener::parseData(const unsigned char *frame) {

    PIControlData data;

    data.time = (((unsigned int) frame[5]) << 24) + (((unsigned int) frame[4]) << 16) +
            (((unsigned int) frame[3]) << 8) + ((unsigned int) frame[2]);

    data.temperature = (((unsigned int) frame[7]) << 8) + ((unsigned int) frame[6]);

    data.dutyCycle = (unsigned int) frame[8];

    emit newData(data);
}
