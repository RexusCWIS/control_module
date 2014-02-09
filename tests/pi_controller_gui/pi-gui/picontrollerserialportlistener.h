#ifndef PICONTROLLERSERIALPORTLISTENER_H
#define PICONTROLLERSERIALPORTLISTENER_H

#include "serial/serialportlistener.h"

#include "picontroldata.h"

class PIControllerSerialPortListener : public SerialPortListener {

    Q_OBJECT

public:
    PIControllerSerialPortListener(QObject *parent);

    PIControllerSerialPortListener(QObject *parent,
                                   const SerialFrameDescriptor &sfd,
                                   const QString &device,
                                   QSerialPort::BaudRate baudrate,
                                   QSerialPort::DataBits dataBits = QSerialPort::Data8,
                                   QSerialPort::Parity parity = QSerialPort::NoParity,
                                   QSerialPort::StopBits stopBits = QSerialPort::OneStop);

    PIControllerSerialPortListener(QObject *parent,
                                   const SerialFrameDescriptor &sfd,
                                   const SerialPortConfig &config);

signals:
    void newData(const PIControlData &data);

protected:
    void parseData(const unsigned char *frame);
};

#endif // PICONTROLLERSERIALPORTLISTENER_H
