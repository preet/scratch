#ifndef HELPER_TEST_QT_SIG_SLOT_SPEED_H
#define HELPER_TEST_QT_SIG_SLOT_SPEED_H

#include "helper.h"

class Receiver : public QObject
{
    Q_OBJECT

public:
    Receiver();
    int m_signal_count;

public slots:
    void OnSignalThing(Thing const &thing);
    void OnSignalStopThread(QThread * thread);
};


class HelperTestQtSigSlotSpeed : public Helper
{
    Q_OBJECT

public:
    explicit HelperTestQtSigSlotSpeed(QQuickView * qqview,
                                      QObject * parent=0);

    void runTest();

signals:
    void signalThing(Thing thing);
    void signalStopThread(QThread * thread);
};


#endif // HELPER_TEST_QT_SIG_SLOT_SPEED_H
