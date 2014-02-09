#ifndef SERIALPORTCONFIG_H
#define SERIALPORTCONFIG_H

#include <QSerialPort>
#include <QString>

/**
 * @brief Defines serial port informations for use with the @ref QSerialPort class.
 */
typedef struct {
    QString device;                     /**< Name of the serial device. */
    QSerialPort::BaudRate baudrate;     /**< Serial line baudrate. */
    QSerialPort::DataBits dataBits;     /**< Number of data bits in the data frame. */
    QSerialPort::Parity   parity;       /**< Selected parity for the transmission. */
    QSerialPort::StopBits stopBits;     /**< Number of stop bits. */
}SerialPortConfig;

#endif // SERIALPORTCONFIG_H
