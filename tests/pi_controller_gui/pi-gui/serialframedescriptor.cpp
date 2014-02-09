#include "serialframedescriptor.h"

SerialFrameDescriptor::SerialFrameDescriptor(int dataSize,
                                             const QString &synchronisationFrame,
                                             SerialFrameDescriptor::CRC crcType) {

    m_syncFrame = synchronisationFrame;
    m_dataSize  = dataSize;
    m_crcType   = crcType;

    m_size = dataSize + m_syncFrame.size() + 2;
}

SerialFrameDescriptor::SerialFrameDescriptor() {

    m_dataSize = 0;
    m_size     = 0;
    m_crcType  = SerialFrameDescriptor::NO_CRC;
}

QString SerialFrameDescriptor::getSynchronisationFrame() const {

    return m_syncFrame;
}

int SerialFrameDescriptor::size(void) const {

    return m_size;
}

int SerialFrameDescriptor::getDataSize(void) const {

    return m_dataSize;
}

bool SerialFrameDescriptor::hasCRC() const {

    return (m_crcType != SerialFrameDescriptor::NO_CRC);
}
