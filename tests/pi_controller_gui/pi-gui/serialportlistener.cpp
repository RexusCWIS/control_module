#include "serialportlistener.h"

#include "crc.h"

#include <QDebug>
#include <QFile>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QByteArray>

/* Required for memcpy */
#include <cstring>

SerialPortListener::SerialPortListener(QObject *parent) :
        QThread(parent) {

    m_stop   = false;
    m_recordedData = new QVector<ExperimentData_s>(0);
}

SerialPortListener::~SerialPortListener() {

    stop();
    delete m_recordedData;
}

void SerialPortListener::start() {

    m_stop = false;
    QThread::start();
}

void SerialPortListener::stop() {

    m_stop = true;
    while(isRunning())
        ;
}

void SerialPortListener::setSerialPort(const QString &device) {

    stop();
    m_serialPort = device;
    start();
}

/** @todo Fix this */
void SerialPortListener::setSerialPort(const QSerialPortInfo &port) {

    (void) port;
    stop();
    start();
}

void SerialPortListener::clearRecordedData(void) {

    m_recordedData->clear();
}

void SerialPortListener::saveRecordedData(const QString &filename) const {

    if(m_recordedData->isEmpty()) {
        return;
    }

    QFile file(filename);
    if(file.open(QFile::WriteOnly | QFile::Truncate)) {
        QTextStream out(&file);
        out << "Time [ms]\tTemperature 1\tTemperature2\tTemperature3\tPressure\tStatus Flags\n";

        QVector<ExperimentData_s>::iterator it;
        for(it = m_recordedData->begin(); it != m_recordedData->end(); it++) {
            out << it->time << "\t" << it->temperature[0] << "\t" << it->temperature[1] << "\t" <<
                   it->temperature[2] << "\t" << it->pressure << "\t" << it->status << "\n";
        }

        file.close();
    }
}

void SerialPortListener::run() {

    QSerialPort serial(m_serialPort);
    unsigned char dataFrame[24];

    ExperimentData_s experimentData;

    qRegisterMetaType<ExperimentData_s>("ExperimentData_s");
    qDebug() << "Thread started";

    unsigned int invalidFrames = 0;

    bool outOfSync  = true,
         validFrame = true;
    unsigned char status[2] = {0, 0};

    serial.open(QIODevice::ReadOnly);

    /* Default parameters: 57600 bps, 8N1 */
    serial.setBaudRate(QSerialPort::Baud38400);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::OneStop);

    while(!m_stop) {

        /** @todo Detect loss of synchronisation */
        outOfSync = true;
        /* Detect start of frame ("UU") */
        while(outOfSync) {

            while(serial.bytesAvailable() < 2) {
                serial.waitForReadyRead(100);
            }
            serial.read((char *) dataFrame, 1);

            if(dataFrame[0] == 'U') {
                serial.read((char *) &dataFrame[1], 1);

                if(dataFrame[1] == 'U') {
                    outOfSync = false;
                }
            }
        }

        while(serial.bytesAvailable() < 22) {
            serial.waitForReadyRead(100);
        }
        /* Read the remaining 22 bytes */
        serial.read((char *) &dataFrame[2], 22);

        validFrame = (crc(dataFrame, 24) == 0);

        if(validFrame) {

            /* Separate data and emit related signals */
            /** @todo Implement unit conversion */
            experimentData.time = (((unsigned int) dataFrame[5]) << 24) + (((unsigned int) dataFrame[4]) << 16) +
                   (((unsigned int) dataFrame[3]) << 8) + ((unsigned int) dataFrame[2]);

            for(int index = 6; index < 12; index += 2) {
                experimentData.temperature[(index >> 1) - 3] = (((unsigned int) dataFrame[index + 1]) << 8) +
                                                ((unsigned int) dataFrame[index]);
            }

            experimentData.pressure = (((unsigned int) dataFrame[13]) << 8) +
                                      ((unsigned int) dataFrame[12]);

            status[0] = dataFrame[14];
            status[1] = dataFrame[15];

            m_recordedData->append(experimentData);

            qDebug() << experimentData.time << ": " << experimentData.pressure << "\n";
            emit newStatus((status[0] & 0x7));
            emit newSensorData(experimentData);
        }

        else {
            invalidFrames++;
            qDebug() << "Invalid Frames: " << invalidFrames << "\n";
        }
    }

    serial.close();
}
