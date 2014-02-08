#ifndef SERIALPORTDIALOG_H
#define SERIALPORTDIALOG_H

#include <QDialog>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include "serialportconfig.h"

class SerialPortDialog : public QDialog
{
    Q_OBJECT
public:
    explicit SerialPortDialog(QWidget *parent = 0);
    SerialPortConfig getSerialPortConfig(void) const;

signals:

public slots:

private:
    QComboBox *m_deviceSelector;
    QComboBox *m_baudrateSelector;
    QComboBox *m_dataBitsSelector;
    QComboBox *m_paritySelector;
    QComboBox *m_stopBitsSelector;

    QLabel *m_deviceLabel;
    QLabel *m_baudrateLabel;
    QLabel *m_dataBitsLabel;
    QLabel *m_parityLabel;
    QLabel *m_stopBitsLabel;

    QPushButton *m_okButton;
    QPushButton *m_cancelButton;

    QGridLayout *m_selectorsLayout;
    QHBoxLayout *m_buttonsLayout;
    QVBoxLayout *m_mainLayout;
};

#endif // SERIALPORTDIALOG_H
