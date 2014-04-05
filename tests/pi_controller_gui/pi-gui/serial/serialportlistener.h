#ifndef SERIALPORTLISTENER_H
#define SERIALPORTLISTENER_H

#include "serialportconfig.h"
#include "serialframedescriptor.h"

#include <QVector>
#include <QThread>
#include <QSerialPort>
#include <QSerialPortInfo>

/**
 * @brief The SerialPortListener class provides means to read and parse constant-length data from a serial port.
 * @details The SerialPortListener aims at providing a simple interface to receive and parse constant-length
 * data frames from a serial line.
 *
 * You can set the serial port configuration and data frame settings by using the @ref setSerialPortConfig
 * and @ref setSerialFrameDescriptor functions or by specifying those parameters to one of the constructors.
 * The serial port configuration can be modified at runtime.
 *
 * The class is built as a set of functions used to communicate with a thread in charge of the data reception.
 * This thread polls the given serial line, performs frame synchronisation and CRC computation.
 *
 * By creating a child class and reimplementing the @ref parseData method, the user can implement
 * application-specific treatment of the data (such as isolating data fields and sending signals to other objects).
 * The @ref parseData function is automatically called by the thread for every frame acquired.
 *
 * All acquired data is stored inside a @ref QVector and can be saved anytime to persistent memory by using
 * the @ref saveRecordedData function.
 */
class SerialPortListener: public QThread {

Q_OBJECT

public:

    /**
     * @brief Default constructor.
     * @details Initializes the instance as a QThread without
     *          any serial port configuration.
     * @param[in] parent QObject that holds this @ref SerialPortListener instance.
     */
    SerialPortListener(QObject *parent);

    /**
     * @details Builds a @ref SerialPortListener object with the given
     * serial port configuration and serial frame. Serial port parameters
     * are passed one by one.
     * @param[in] parent    QObject that holds this @ref SerialPortListener instance.
     * @param[in] sfd       Description of the application-specific serial frame.
     * @param[in] device    Name of the serial device.
     * @param[in] baudrate  Baudrate used for the reception of data.
     * @param[in] dataBits  Number of data bits.
     * @param[in] parity    Parity.
     * @param[in] stopBits  Number of stop bits.
     */
    SerialPortListener(QObject *parent,
                       const SerialFrameDescriptor &sfd,
                       const QString &device,
                       QSerialPort::BaudRate baudrate,
                       QSerialPort::DataBits dataBits = QSerialPort::Data8,
                       QSerialPort::Parity parity = QSerialPort::NoParity,
                       QSerialPort::StopBits stopBits = QSerialPort::OneStop);

    /**
     * @brief Complete constructor.
     * @details Builds a @ref SerialPortListener instance with the given
     * @ref SerialFrameDescriptor and @ref SerialPortConfig.
     * @param[in] parent QObject that holds this @ref SerialPortListener instance.
     * @param[in] sfd    Description of the application-specific serial frame.
     * @param[in] config Serial port configuration.
     */
    SerialPortListener(QObject *parent,
                       const SerialFrameDescriptor &sfd,
                       const SerialPortConfig &config);

    /**
     * @brief Destroys this @ref SerialPortListener instance.
     */
    virtual ~SerialPortListener();

    /**
     * @brief Starts data reception.
     * @details Launches the data reception thread.
     */
    void start(void);

    /**
     * @brief Stops data reception.
     * @details Stops and kills the data reception thread. Once this function
     * has been called, no more data is read from the serial port.
     */
    void stop(void);

    /**
     * @brief Sets the @ref SerialFrameDescriptor used for the application.
     * @param sfd
     * @note Calling this function once the data reception thread has been
     * started has no effect until the thread is stopped and restarted.
     * The reason is that serial frame parameters are extracted from
     * the @ref SerialFrameDescriptor object contained in the @ref SerialPortListener
     * for ease of use, safety and performance.
     */
    void setSerialFrameDescriptor(const SerialFrameDescriptor &sfd);

public slots:

    /**
     * @brief Sets a new serial port configuration.
     * @details This function sets the new serial port configuration
     * after stopping the data reception thread. After setting the new
     * configuration, it automatically launches the thread again.
     * @note The data reception thread is started by this function
     * whether or not it was running before the function call.
     * @param config
     */
    void setSerialPortConfig(const SerialPortConfig &config);

    /**
     * @brief Clears all data recorded up until now.
     * @details Clears the private @ref QVector used for data storage.
     * All data is lost if it has not been saved to persistent storage
     * using the @ref saveRecordedData method.
     * @see saveRecordedData()
     */
    void clearRecordedData(void);

    /**
     * @brief Saves the recorded data to the given @p file.
     * @details Saves all data acquired by the data reception thread
     * since it was started (or cleared). By default, this function writes one data frame
     * per line and separates all bytes in the frame by a tab. Reimplement this function
     * in a child class to change this behaviour.
     * @param[in] filename Name of the save file.
     * @see clearRecordedData()
     */
    void saveRecordedData(const QString &filename) const;

    /**
     * @brief Returns the number of invalid frames detected.
     * @details A frame is declared invalid if its CRC is not zero.
     * This function returns the number of frames that were containing an
     * error detectable by CRC computation.
     *
     * This number is reset once the thread is restarted or when calling
     * the @ref clearRecordedData function.
     *
     * @returns The number of frames that were containing an error
     * detected by CRC computation.
     */
    int getNumberOfInvalidFrames(void) const;

protected:

    /**
     * @brief Data reception thread loop.
     * @details This is the main part of the @ref SerialPortListener
     * object. It polls the serial line for data, performs synchronisation
     * and CRC computation and saves the received data in a private
     * @ref QVector. The @ref parseData function can be reimplemented
     * by child classes to perform application-specific data parsing
     * and communicate it to other objects.
     * @see parseData(), start(), stop(), saveRecordedData(), clearRecordedData()
     */
    void run();

    /**
     * @brief Parses the received data frame to extract useful data.
     * @details This function parses the received data @p frame (including
     * synchronisation and CRC bytes) to extract application-specific
     * data.
     *
     * In a standard @ref SerialPortListener object, this function
     * does nothing: the user must create a class that inherits
     * @ref SerialPortListener and reimplement this method to perform
     * the data parsing.
     * @param[in] frame Byte array representing the received data frame.
     *            All bytes (including synchronisation and CRC) are passed
     *            to this function.
     */
    virtual void parseData(const unsigned char* frame);

    /**
     * @brief @ref QVector holding all received bytes.
     */
    QVector<unsigned char> *m_recordedData;

    /** @brief Description of the application-specific serial frame. */
    SerialFrameDescriptor m_sfd;

    /** @brief Name of the currently used serial port. */
    QString m_serialPort;
    /** @brief Baudrate used for serial communication. */
    QSerialPort::BaudRate m_baudrate;
    /** @brief Number of data bits. */
    QSerialPort::DataBits m_dataBits;
    /** @brief Type (if any) of parity in a received byte. */
    QSerialPort::Parity   m_parity;
    /** @brief Number of stop bits per received byte. */
    QSerialPort::StopBits m_stopBits;

    int m_validFrames;
    /** @brief Number of frames containing an error detected by CRC computation. */
    int m_invalidFrames;

    /** @brief Boolean value used to stop the data reception thread. */
    volatile bool m_stop;
};

#endif // SERIALPORTLISTENER_H
