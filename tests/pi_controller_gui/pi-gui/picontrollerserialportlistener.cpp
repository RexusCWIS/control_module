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

}
