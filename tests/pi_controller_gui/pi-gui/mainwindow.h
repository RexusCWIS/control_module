#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "centralwidget.h"
#include "serial/serialportdialog.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void showSerialConfigDlg(void);

private:
    void createActions(void);
    void createMenus(void);

    CentralWidget *m_stepPlot;
    QMenu *m_fileMenu;
    QMenu *m_configMenu;

    QAction *m_saveAction;
    QAction *m_serialConfigAction;

    SerialPortDialog *m_serialConfigDlg;
};

#endif // MAINWINDOW_H
