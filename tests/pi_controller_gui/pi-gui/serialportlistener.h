#ifndef SERIALPORTLISTENER_H
#define SERIALPORTLISTENER_H

#include "experiment.h"
#include "serialportconfig.h"
#include "serialframedescriptor.h"

#include <QVector>
#include <QThread>
#include <QSerialPort>
#include <QSerialPortInfo>

class SerialPortListener: public QThread {

    Q_OBJECT

    public:

        SerialPortListener(QObject *parent);

        SerialPortListener(QObject *parent, const QString &device,
                           QSerialPort::BaudRate baudrate,
                           QSerialPort::DataBits dataBits = QSerialPort::Data8,
                           QSerialPort::Parity parity = QSerialPort::NoParity,
                           QSerialPort::StopBits stopBits = QSerialPort::OneStop);

        SerialPortListener(QObject *parent, const SerialPortConfig &config);

        virtual ~SerialPortListener();

        void start(void);
        void stop(void);

        void setSerialFrameDescriptor(const SerialFrameDescriptor &sfd);

    public slots:
        void setSerialPortConfig(const SerialPortConfig &config);
        void clearRecordedData(void);
        void saveRecordedData(const QString &filename) const;

    signals:
        void newStatus(unsigned char);
        void newSensorData(ExperimentData_s);

    protected:
        void run();

        QVector<unsigned char> *m_recordedData;

        SerialFrameDescriptor m_sfd;

        QString m_serialPort;
        QSerialPort::BaudRate m_baudrate;
        QSerialPort::DataBits m_dataBits;
        QSerialPort::Parity   m_parity;
        QSerialPort::StopBits m_stopBits;

        volatile bool m_stop;
};

#endif // SERIALPORTLISTENER_H
