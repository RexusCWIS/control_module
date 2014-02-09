#include "mainwindow.h"
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      m_sfd(9, "UU", SerialFrameDescriptor::CRC16_CCITT)
{
    m_stepPlot = new CentralWidget(this);
    this->setCentralWidget(m_stepPlot);

    m_serialConfigDlg = new SerialPortDialog();

    this->createActions();
    this->createMenus();

    m_serialPortListener = new PIControllerSerialPortListener(this);
    m_serialPortListener->setSerialFrameDescriptor(m_sfd);

    QObject::connect(m_serialPortListener, SIGNAL(newData(PIControlData)), m_stepPlot, SLOT(refresh(PIControlData)));
}

MainWindow::~MainWindow()
{
    delete m_serialConfigDlg;
}

void MainWindow::showSerialConfigDlg(void) {

    int rvalue = m_serialConfigDlg->exec();

    if(rvalue == QDialog::Accepted) {

        const SerialPortConfig config = m_serialConfigDlg->getSerialPortConfig();
        qDebug() << "New serial configuration:\nDevice: " << config.device <<
                    "\nBaudrate: " << config.baudrate << "\nData bits: " <<
                    config.dataBits << "\nParity: " << config.parity <<
                    "\nStop bits: " << config.stopBits;

        m_serialPortListener->setSerialPortConfig(config);
    }
}

void MainWindow::createActions(void) {

    m_saveAction = new QAction(tr("Save"), this);
    m_saveAction->setShortcut(QKeySequence::Save);

    m_serialConfigAction = new QAction(tr("Serial"), this);
    QObject::connect(m_serialConfigAction, SIGNAL(triggered()), this, SLOT(showSerialConfigDlg()));
}

void MainWindow::createMenus(void) {

    m_fileMenu   = this->menuBar()->addMenu(tr("&File"));
    m_fileMenu->addAction(m_saveAction);

    m_configMenu = this->menuBar()->addMenu(tr("&Config"));
    m_configMenu->addAction(m_serialConfigAction);
}
