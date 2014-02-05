#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    m_stepPlot = new CentralWidget(this);

    setCentralWidget(m_stepPlot);
}

MainWindow::~MainWindow()
{

}
