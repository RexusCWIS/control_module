#ifndef DEF_SERIALFRAMEDESCRIPTOR_H
#define DEF_SERIALFRAMEDESCRIPTOR_H

#include <QString>

/**
 * @brief The SerialFrameDescriptor class is used to describe the architecture of a data frame received over a serial line.
 * @details This class provides informations about serial data frames. It defines the frame used for synchronisation
 * (i.e. the data bytes at the start of each frame) and the number of data bytes.
 *
 * Optional informations are also given on CRC computation. The @ref SerialFrameDescriptor::CRC enum defines CRC polynomials
 * used for error checking.
 */
class SerialFrameDescriptor
{
public:

    enum CRC {
        NO_CRC = 0,
        CRC16_CCITT = 0x1021
    };

    SerialFrameDescriptor();

    SerialFrameDescriptor(int dataSize,
                          const QString &synchronisationFrame,
                          SerialFrameDescriptor::CRC crcType = SerialFrameDescriptor::NO_CRC);

    int size(void) const;

    int getDataSize(void) const;

    QString getSynchronisationFrame(void) const;

    bool hasCRC(void) const;

protected:
    QString m_syncFrame;

    int m_size;
    int m_dataSize;
    unsigned char *m_data;

    SerialFrameDescriptor::CRC m_crcType;
};

#endif /* DEF_SERIALFRAMEDESCRIPTOR_H */
