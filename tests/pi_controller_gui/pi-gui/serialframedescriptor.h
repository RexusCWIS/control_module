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

    SerialFrameDescriptor(unsigned int dataSize,
                          const QString &synchronisationFrame,
                          SerialFrameDescriptor::CRC crcType = SerialFrameDescriptor::NO_CRC);

    unsigned int size(void) const;
    unsigned int getDataSize(void) const;
    unsigned char* getData(void) const;

protected:
    QString m_syncFrame;

    unsigned int m_size;
    unsigned int m_dataSize;
    unsigned char *m_data;

    SerialFrameDescriptor::CRC m_crcType;
};

#endif /* DEF_SERIALFRAMEDESCRIPTOR_H */
