#include "serialframedescriptor.h"

SerialFrameDescriptor::SerialFrameDescriptor(unsigned int dataSize,
                                             const QString &synchronisationFrame,
                                             SerialFrameDescriptor::CRC crcType) {

    m_syncFrame = synchronisationFrame;
    m_dataSize  = dataSize;
    m_crcType   = crcType;

    m_size = dataSize + m_syncFrame.size() + 2;
}

unsigned int SerialFrameDescriptor::size(void) const {

    return m_size;
}

unsigned int SerialFrameDescriptor::getDataSize(void) const {

    return m_dataSize;
}
