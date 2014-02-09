#include "serialportlistener.h"

#include "crc.h"

#include <QDebug>
#include <QFile>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QByteArray>

SerialPortListener::SerialPortListener(QObject *parent) :
            QThread(parent) {

    m_stop = false;
    m_recordedData = new QVector<unsigned char>(0);
}

SerialPortListener::SerialPortListener(QObject *parent,
                                       const SerialFrameDescriptor &sfd,
                                       const QString &device,
                                       QSerialPort::BaudRate baudrate,
                                       QSerialPort::DataBits dataBits,
                                       QSerialPort::Parity parity,
                                       QSerialPort::StopBits stopBits) : QThread(parent) {

    m_serialPort = device;

    m_baudrate = baudrate;
    m_dataBits = dataBits;
    m_parity   = parity;
    m_stopBits = stopBits;

    m_stop = false;
    m_recordedData = new QVector<unsigned char>(0);

    this->setSerialFrameDescriptor(sfd);

    start();
}

SerialPortListener::SerialPortListener(QObject *parent,
                                       const SerialFrameDescriptor &sfd,
                                       const SerialPortConfig &config) : QThread(parent) {

    this->setSerialPortConfig(config);

    this->setSerialFrameDescriptor(sfd);

    m_stop = false;
    m_recordedData = new QVector<unsigned char>(0);

    start();
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

void SerialPortListener::setSerialFrameDescriptor(const SerialFrameDescriptor &sfd) {

    m_sfd = sfd;
}

void SerialPortListener::setSerialPortConfig(const SerialPortConfig &config) {

    stop();
    m_serialPort = config.device;
    m_baudrate   = config.baudrate;
    m_dataBits   = config.dataBits;
    m_parity     = config.parity;
    m_stopBits   = config.stopBits;
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

        for(int f = 0; f < m_recordedData->size(); f += m_sfd.size()) {
            for(int index = 0; index < m_sfd.size(); index++) {
                out << m_recordedData->at(f * m_sfd.size() + index) << "\t";
            }

            out << "\n";
        }

        file.close();
    }
}

void SerialPortListener::run() {

    qDebug() << "SerialPortListener thread started.";
    QSerialPort serial(m_serialPort);

    //qRegisterMetaType<ExperimentData_s>("ExperimentData_s");

    unsigned int invalidFrames = 0;

    bool outOfSync  = true,
         validFrame = true;

    serial.open(QIODevice::ReadOnly);

    serial.setBaudRate(m_baudrate);
    serial.setDataBits(m_dataBits);
    serial.setParity(m_parity);
    serial.setStopBits(m_stopBits);

    unsigned int syncFrameSize = m_sfd.getSynchronisationFrame().size();
    unsigned int frameSize = m_sfd.size();
    const char* syncFrame  = m_sfd.getSynchronisationFrame().toStdString().c_str();

    unsigned char* frame = new unsigned char[frameSize];

    while(!m_stop) {

        /** @todo Detect loss of synchronisation */
        outOfSync = true;
        /* Detect start of frame */
        /** Adapt this for synchronization frames longer than 2 bytes */
        while(outOfSync && !m_stop) {

            while((serial.bytesAvailable() < syncFrameSize) && !m_stop) {
                serial.waitForReadyRead(100);
            }
            serial.read((char *) frame, 1);

            if(frame[0] == (unsigned char) syncFrame[0]) {

                serial.read((char *) &frame[1], 1);

                if(frame[1] == (unsigned char) syncFrame[1]) {
                    outOfSync = false;
                }
            }
        }

        while((serial.bytesAvailable() < (m_sfd.size() - syncFrameSize)) && !m_stop) {
            serial.waitForReadyRead(100);
        }

        /* Exit the thread loop if the m_stop signal was sent */
        if(m_stop) {
            break;
        }

        /* Read the rest of the frame */
        serial.read((char *) &frame[syncFrameSize - 1], frameSize - syncFrameSize);

        if(m_sfd.hasCRC()) {
            validFrame = (crc(frame, frameSize) == 0);
        }

        if(validFrame) {

            parseData(frame);

            for(unsigned int index = 0; index < frameSize; index++) {
                m_recordedData->append(frame[index]);
            }
        }

        else {
            invalidFrames++;
            qDebug() << "Invalid Frames: " << invalidFrames << "\n";
        }
    }

    delete [] frame;
    serial.close();
    qDebug() << "SerialPortListener thread stopped.";
}

void SerialPortListener::parseData(const unsigned char *frame) {

    (void) frame;
    return;
}
