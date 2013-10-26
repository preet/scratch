#ifndef HELPER_H
#define HELPER_H

#include <QQuickView>
#include <QQmlContext>
#include <QObject>
#include <QDebug>
#include <QThread>
#include <QDir>
#include <QElapsedTimer>

#include "duktape.h"


struct NumericalData   {
    double value;
    double min;
    double max;
    QString units;
};

class Helper : public QObject
{
    Q_OBJECT

public:
    explicit Helper(QQuickView * qqview, QObject * parent=0);
    Q_INVOKABLE void runTest();
    
signals:
    void testComplete(double average_ms);

private:
    QQuickView * m_view;
};

#endif // HELPER_H
