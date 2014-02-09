#ifndef CENTRALWIDGET_H
#define CENTRALWIDGET_H

#include <QWidget>
#include <QHBoxLayout>

#include "picontroldata.h"
#include "qcustomplot.h"

class CentralWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CentralWidget(QWidget *parent = 0);

signals:

public slots:
    void refresh(const PIControlData &data);

private:
    QCustomPlot *m_plot;
    QHBoxLayout *m_layout;
};

#endif // CENTRALWIDGET_H
