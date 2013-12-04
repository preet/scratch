#ifndef HELPER_H
#define HELPER_H

#include <QQuickView>
#include <QQmlContext>
#include <QObject>
#include <QDebug>
#include <QThread>
#include <QDir>
#include <QElapsedTimer>
#include <QJsonObject>

#include "duktape.h"


struct NumericalData   {
    double value;
    double min;
    double max;
    QString units;
};

struct AnotherTest   {
    double arg1;
    double arg2;
    double arg3;
    double arg4;
    double arg5;
    double arg6;
    double arg7;
    double arg8;
};

class AnotherThread : public QThread
{
    Q_OBJECT

public:
    AnotherThread(QObject * parent=0);

protected:
    void run();
};

class Another : public QObject
{
    Q_OBJECT

public:
    Another(QObject * parent=0);

public slots:
    void onRxData(double arg1,double arg2, double arg3, double arg4, double arg5, double arg6, double arg7, double arg8);
    void onRxData(AnotherTest at);
    void onRxData(QJsonObject at);
};

class Helper : public QObject
{
    Q_OBJECT

public:
    explicit Helper(QQuickView * qqview, QObject * parent=0);
    Q_INVOKABLE void runTest();
    
    void runAnotherTest();

signals:
    void txData(double arg1,double arg2, double arg3, double arg4, double arg5, double arg6, double arg7, double arg8);
    void txData(AnotherTest at);
    void txData(QJsonObject at);
    void testComplete(double average_ms);

private:
    QQuickView * m_view;

    QJsonObject m_json_at;
};

#endif // HELPER_H
