#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    m_stepPlot = new CentralWidget(this);
    this->setCentralWidget(m_stepPlot);

    QAction *saveAction = new QAction(tr("&Save"), this);
    saveAction->setShortcut(QKeySequence::Save);
    QAction *serialPortConfigAction = new QAction(tr("&Serial"), this);

    m_fileMenu   = this->menuBar()->addMenu(tr("&File"));
    m_configMenu = this->menuBar()->addMenu(tr("&Config"));

    m_fileMenu->addAction(saveAction);
    m_configMenu->addAction(serialPortConfigAction);
}

MainWindow::~MainWindow()
{

}
