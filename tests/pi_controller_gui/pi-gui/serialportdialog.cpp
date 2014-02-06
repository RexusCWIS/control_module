#include "serialportdialog.h"

#include <QSerialPortInfo>

SerialPortDialog::SerialPortDialog(QWidget *parent) :
    QDialog(parent)
{
    m_deviceSelector   = new QComboBox(this);
    m_baudrateSelector = new QComboBox(this);
    m_dataBitsSelector = new QComboBox(this);
    m_paritySelector   = new QComboBox(this);
    m_stopBitsSelector = new QComboBox(this);

    m_layout = new QVBoxLayout(this);

    m_layout->addWidget(m_deviceSelector);
    m_layout->addWidget(m_baudrateSelector);
    m_layout->addWidget(m_dataBitsSelector);
    m_layout->addWidget(m_paritySelector);
    m_layout->addWidget(m_stopBitsSelector);

    this->setLayout(m_layout);

    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
        this->addItem(info.portName());
    }
}

SerialPortConfig * SerialPortDialog::getSerialPortConfig() const {
    return m_serialPortConfig;
}
