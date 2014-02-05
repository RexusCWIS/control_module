#ifndef CENTRALWIDGET_H
#define CENTRALWIDGET_H

#include <QWidget>
#include <QHBoxLayout>

#include "qcustomplot.h"

class CentralWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CentralWidget(QWidget *parent = 0);

signals:

public slots:

private:
    QCustomPlot *m_plot;
    QHBoxLayout *m_layout;
};

#endif // CENTRALWIDGET_H
