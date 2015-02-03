#ifndef HELPER_H
#define HELPER_H

#include <QQuickView>
#include <QQmlContext>
#include <QObject>
#include <QDebug>
#include <QThread>
#include <QDir>
#include <QElapsedTimer>

class Thing
{
public:
    Thing() : x(0)
    {

    }

    Thing(Thing const &other)
    {
        this->x = other.x;
//        qDebug() << "Copy constructed Thing!";
    }

    Thing(Thing &&other)
    {
        this->x = other.x;
//        qDebug() << "Move constructed Thing!";
    }

    Thing & operator = (Thing const &other)
    {
        this->x = other.x;
//        qDebug() << "Copied Thing!";
        return (*this);
    }

    Thing & operator = (Thing && other)
    {
        this->x = other.x;
//        qDebug() << "Moved Thing!";
        return (*this);
    }

    int x;
};

class Helper : public QObject
{
    Q_OBJECT

public:
    explicit Helper(QQuickView * qqview, QObject * parent=0);
    Q_INVOKABLE virtual void runTest()=0;
    
signals:
    void testComplete(QString message);

private:
    QQuickView * m_view;
};

#endif // HELPER_H
