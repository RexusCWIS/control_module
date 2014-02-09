#ifndef DEF_SERIALFRAMEDESCRIPTOR_H
#define DEF_SERIALFRAMEDESCRIPTOR_H

#include <QString>

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
