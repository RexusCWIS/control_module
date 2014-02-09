#include "serialportdialog.h"

#include <QSerialPortInfo>

SerialPortDialog::SerialPortDialog(QWidget *parent) :
    QDialog(parent)
{
    /* Generating combo boxes and associated labels */
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

        if(!info.isBusy()) {
            m_deviceSelector->addItem(info.portName());
        }
    }

    m_baudrateSelector->insertItem(m_baudrateSelector->count(), "1200 bps", QSerialPort::Baud1200);
    m_baudrateSelector->insertItem(m_baudrateSelector->count(), "2400 bps", QSerialPort::Baud2400);
    m_baudrateSelector->insertItem(m_baudrateSelector->count(), "4800 bps", QSerialPort::Baud4800);
    m_baudrateSelector->insertItem(m_baudrateSelector->count(), "9600 bps", QSerialPort::Baud9600);
    m_baudrateSelector->insertItem(m_baudrateSelector->count(), "19200 bps", QSerialPort::Baud19200);
    m_baudrateSelector->insertItem(m_baudrateSelector->count(), "38400 bps", QSerialPort::Baud38400);
    m_baudrateSelector->insertItem(m_baudrateSelector->count(), "57600 bps", QSerialPort::Baud57600);
    m_baudrateSelector->insertItem(m_baudrateSelector->count(), "115200 bps", QSerialPort::Baud115200);

    m_dataBitsSelector->insertItem(m_dataBitsSelector->count(), "5 bits", QSerialPort::Data5);
    m_dataBitsSelector->insertItem(m_dataBitsSelector->count(), "6 bits", QSerialPort::Data6);
    m_dataBitsSelector->insertItem(m_dataBitsSelector->count(), "7 bits", QSerialPort::Data7);
    m_dataBitsSelector->insertItem(m_dataBitsSelector->count(), "8 bits", QSerialPort::Data8);

    m_paritySelector->insertItem(m_paritySelector->count(), tr("No parity"), QSerialPort::NoParity);
    m_paritySelector->insertItem(m_paritySelector->count(), tr("Even parity"), QSerialPort::EvenParity);
    m_paritySelector->insertItem(m_paritySelector->count(), tr("Odd parity"), QSerialPort::OddParity);
    m_paritySelector->insertItem(m_paritySelector->count(), tr("Space parity"), QSerialPort::SpaceParity);
    m_paritySelector->insertItem(m_paritySelector->count(), tr("Mark parity"), QSerialPort::MarkParity);

    m_stopBitsSelector->insertItem(m_stopBitsSelector->count(), "1", QSerialPort::OneStop);
    m_stopBitsSelector->insertItem(m_stopBitsSelector->count(), "1 1/2", QSerialPort::OneAndHalfStop);
    m_stopBitsSelector->insertItem(m_stopBitsSelector->count(), "2", QSerialPort::TwoStop);

    m_baudrateSelector->setCurrentIndex(5);
    m_dataBitsSelector->setCurrentIndex(3);

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
    QObject::connect(m_okButton, SIGNAL(clicked()), this, (SLOT(accept())));

    m_buttonsLayout = new QHBoxLayout();

    m_buttonsLayout->addWidget(m_okButton);
    m_buttonsLayout->addWidget(m_cancelButton);

    m_mainLayout = new QVBoxLayout();

    m_mainLayout->addLayout(m_selectorsLayout);
    m_mainLayout->addLayout((m_buttonsLayout));

    this->setLayout(m_mainLayout);

    this->setWindowTitle(tr("Serial port parameters"));
}

SerialPortConfig SerialPortDialog::getSerialPortConfig() const {

    SerialPortConfig config;

    config.device = m_deviceSelector->currentText();

    config.baudrate = (QSerialPort::BaudRate) m_baudrateSelector->itemData(m_baudrateSelector->currentIndex()).toInt();
    config.dataBits = (QSerialPort::DataBits) m_dataBitsSelector->itemData(m_dataBitsSelector->currentIndex()).toInt();
    config.parity   = (QSerialPort::Parity)   m_paritySelector->itemData(m_paritySelector->currentIndex()).toInt();
    config.stopBits = (QSerialPort::StopBits) m_stopBitsSelector->itemData(m_stopBitsSelector->currentIndex()).toInt();

    return config;
}
