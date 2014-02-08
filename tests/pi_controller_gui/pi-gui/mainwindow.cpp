#include "mainwindow.h"
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    m_stepPlot = new CentralWidget(this);
    this->setCentralWidget(m_stepPlot);

    m_serialConfigDlg = new SerialPortDialog();

    this->createActions();
    this->createMenus();
}

MainWindow::~MainWindow()
{
    delete m_serialConfigDlg;
}

void MainWindow::showSerialConfigDlg(void) {

    int rvalue = m_serialConfigDlg->exec();

    if(rvalue == QDialog::Accepted) {

        qDebug() << "New serial configuration";
    }
}

void MainWindow::createActions(void) {

    m_saveAction = new QAction(tr("&Save"), this);
    m_saveAction->setShortcut(QKeySequence::Save);

    m_serialConfigAction = new QAction(tr("&Serial"), this);
    QObject::connect(m_serialConfigAction, SIGNAL(triggered()), this, SLOT(showSerialConfigDlg()));
}

void MainWindow::createMenus(void) {

    m_fileMenu   = this->menuBar()->addMenu(tr("&File"));
    m_fileMenu->addAction(m_saveAction);

    m_configMenu = this->menuBar()->addMenu(tr("&Config"));
    m_configMenu->addAction(m_serialConfigAction);
}
