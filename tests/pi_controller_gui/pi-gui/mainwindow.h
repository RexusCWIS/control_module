#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "centralwidget.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    CentralWidget *m_stepPlot;
    QMenu *m_fileMenu;
    QMenu *m_configMenu;
};

#endif // MAINWINDOW_H
