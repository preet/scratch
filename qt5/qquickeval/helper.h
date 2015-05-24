#ifndef HELPER_H
#define HELPER_H

#include <QQuickView>
#include <QQmlContext>
#include <QObject>
#include <QDebug>
#include <QThread>
#include <QDir>
#include <QElapsedTimer>

class Helper : public QObject
{
    Q_OBJECT

public:
    explicit Helper(QQuickView * qqview, QObject * parent=0);
    Q_INVOKABLE virtual void runTest();
    
signals:
    void testComplete(QString message);

private:
    QQuickView * m_view;
};

#endif // HELPER_H
