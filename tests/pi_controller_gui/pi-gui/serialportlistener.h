#ifndef SERIALPORTLISTENER_H
#define SERIALPORTLISTENER_H

#include "experiment.h"

#include <QVector>
#include <QThread>
#include <QSerialPort>
#include <QSerialPortInfo>

class SerialPortListener: public QThread {

    Q_OBJECT

    public:
        SerialPortListener(QObject *parent, const QString &device,
                           QSerialPort::BaudRate baudrate,
                           QSerialPort::DataBits dataBits = QSerialPort::Data8,
                           QSerialPort::Parity parity = QSerialPort::NoParity,
                           QSerialPort::StopBits stopBits = QSerialPort::OneStop);
        virtual ~SerialPortListener();

        void start(void);
        void stop(void);

    public slots:
        void setSerialPort(const QString &device);
        void setBaudrate(QSerialPort::BaudRate baudrate);
        void clearRecordedData(void);
        void saveRecordedData(const QString &filename) const;

    signals:
        void newStatus(unsigned char);
        void newSensorData(ExperimentData_s);

    protected:
        void run();

    private:
        QVector<ExperimentData_s> *m_recordedData;
        QString m_serialPort;
        QSerialPort::BaudRate m_baudrate;
        QSerialPort::DataBits m_dataBits;
        QSerialPort::Parity   m_parity;
        QSerialPort::StopBits m_stopBits;
        volatile bool m_stop;
};

#endif // SERIALPORTLISTENER_H
