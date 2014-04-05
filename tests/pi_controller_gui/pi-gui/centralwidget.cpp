#include "centralwidget.h"

CentralWidget::CentralWidget(QWidget *parent) :
    QWidget(parent) {

    m_plot = new QCustomPlot(this);

    m_plot->legend->setVisible(true);
    QFont legendFont = font();
    legendFont.setPointSize(9);
    m_plot->legend->setFont(legendFont);

    m_plot->addGraph();
    m_plot->graph(0)->setPen(QPen(Qt::blue));
    m_plot->graph(0)->setName("Duty cycle");

    m_plot->addGraph();
    m_plot->graph(1)->setPen(QPen(Qt::green));
    m_plot->graph(1)->setName("Temperature");

    m_plot->xAxis->setLabel("Time [s]");
    m_plot->xAxis->setRange(0, 100);
    m_plot->yAxis->setLabel("Temperature [Celsius]");
    m_plot->yAxis->setRange(20, 70);
    m_plot->yAxis2->setVisible(true);
    m_plot->yAxis2->setRange(0, 100);
    m_plot->yAxis2->setLabel("Duty Cycle [%]");

    m_plot->plotLayout()->insertRow(0);
    m_plot->plotLayout()->addElement(0, 0, new QCPPlotTitle(m_plot, "Step response"));

    m_layout = new QHBoxLayout(this);
    m_layout->addWidget(m_plot);
    this->setLayout(m_layout);

    m_plot->setMinimumSize(640, 480);
}

void CentralWidget::refresh(const PIControlData &data) {

    float time = data.getTime();

    //m_plot->graph(0)->addData(time, data.getDutyCycle());
    m_plot->graph(0)->addData(time, 61);
    m_plot->graph(1)->addData(time, 42);
    //m_plot->graph(1)->addData(time, data.getTemperature());

    qDebug() << time << "s: " << data.getDutyCycle() << data.getTemperature();

    m_plot->replot();
}
