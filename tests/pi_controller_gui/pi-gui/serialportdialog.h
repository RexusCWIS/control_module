#ifndef SERIALPORTDIALOG_H
#define SERIALPORTDIALOG_H

#include <QDialog>
#include <QComboBox>
#include <QVBoxLayout>

#include "serialportconfig.h"

class SerialPortDialog : public QDialog
{
    Q_OBJECT
public:
    explicit SerialPortDialog(QWidget *parent = 0);
    SerialPortConfig *getSerialPortConfig(void) const;

signals:

public slots:

private:
    QComboBox *m_deviceSelector;
    QComboBox *m_baudrateSelector;
    QComboBox *m_dataBitsSelector;
    QComboBox *m_paritySelector;
    QComboBox *m_stopBitsSelector;

    QVBoxLayout *m_layout;

    SerialPortConfig *m_serialPortConfig;
};

#endif // SERIALPORTDIALOG_H
