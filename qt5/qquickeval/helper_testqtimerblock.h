#ifndef HELPER_TEST_QTIMER_BLOCK_H
#define HELPER_TEST_QTIMER_BLOCK_H

#include "helper.h"

#include <QTimer>

class Receiver : public QObject
{
    Q_OBJECT

public:
    Receiver();

public slots:
    void OnTimeout();
};


class HelperTestQTimerBlock : public Helper
{
    Q_OBJECT

public:
    explicit HelperTestQTimerBlock(QQuickView * qqview,
                                   QObject * parent=0);

    void runTest();

public slots:
    void OnTimeout();

private:
    QTimer timer;
};


#endif // HELPER_TEST_QTIMER_BLOCK_H
