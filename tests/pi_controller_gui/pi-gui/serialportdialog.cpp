#include "serialportdialog.h"

#include <QSerialPortInfo>

SerialPortDialog::SerialPortDialog(QWidget *parent) :
    QDialog(parent)
{

    m_deviceLabel   = new QLabel(tr("Serial device:"));
    m_baudrateLabel = new QLabel(tr("Baudrate:"));
    m_dataBitsLabel = new QLabel(tr("Data bits:"));
    m_parityLabel   = new QLabel(tr("Parity:"));
    m_stopBitsLabel = new QLabel(tr("Stop bits:"));

    m_deviceSelector   = new QComboBox();
    m_baudrateSelector = new QComboBox();
    m_dataBitsSelector = new QComboBox();
    m_paritySelector   = new QComboBox();
    m_stopBitsSelector = new QComboBox();

    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
        m_deviceSelector->addItem(info.portName());
    }

    m_selectorsLayout = new QGridLayout();

    m_selectorsLayout->addWidget(m_deviceLabel, 0, 0);
    m_selectorsLayout->addWidget(m_baudrateLabel, 1, 0);
    m_selectorsLayout->addWidget(m_dataBitsLabel, 2, 0);
    m_selectorsLayout->addWidget(m_parityLabel, 3, 0);
    m_selectorsLayout->addWidget(m_stopBitsLabel, 4, 0);

    m_selectorsLayout->addWidget(m_deviceSelector, 0, 1);
    m_selectorsLayout->addWidget(m_baudrateSelector, 1, 1);
    m_selectorsLayout->addWidget(m_dataBitsSelector, 2, 1);
    m_selectorsLayout->addWidget(m_paritySelector, 3, 1);
    m_selectorsLayout->addWidget(m_stopBitsSelector, 4, 1);

    m_okButton     = new QPushButton(tr("OK"));
    m_cancelButton = new QPushButton(tr("Cancel"));

    QObject::connect(m_cancelButton, SIGNAL(clicked()), this, SLOT(reject()));

    m_buttonsLayout = new QHBoxLayout();

    m_buttonsLayout->addWidget(m_okButton);
    m_buttonsLayout->addWidget(m_cancelButton);

    m_mainLayout = new QVBoxLayout();

    m_mainLayout->addLayout(m_selectorsLayout);
    m_mainLayout->addLayout((m_buttonsLayout));

    this->setLayout(m_mainLayout);
}

SerialPortConfig * SerialPortDialog::getSerialPortConfig() const {
    return m_serialPortConfig;
}
