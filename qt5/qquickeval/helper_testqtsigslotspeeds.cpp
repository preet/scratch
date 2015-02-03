#include <thread>
#include <chrono>
#include <string>

#include "helper_testqtsigslotspeeds.h"


Receiver::Receiver() :
    QObject(0),
    m_signal_count(0)
{
    // empty
}

void Receiver::OnSignalThing(Thing const &)
{
    m_signal_count++;
}

void Receiver::OnSignalStopThread(QThread * thread)
{
    thread->quit();
}

HelperTestQtSigSlotSpeed::HelperTestQtSigSlotSpeed(QQuickView * qqview,
                                                   QObject * parent) :
    Helper(qqview,parent)
{
    qRegisterMetaType<Thing>("Thing");
}

void HelperTestQtSigSlotSpeed::runTest()
{
    QThread thread_receiver;
    thread_receiver.start();

    Receiver * r0 = new Receiver();
    r0->moveToThread(&thread_receiver);

    connect(this,SIGNAL(signalThing(Thing)),
            r0,SLOT(OnSignalThing(Thing)));

    connect(this,SIGNAL(signalStopThread(QThread*)),
            r0,SLOT(OnSignalStopThread(QThread*)));

    Thing thing;

    // test 1 signal -> 1 slot
    size_t one_one_count=1000;
    std::chrono::time_point<std::chrono::steady_clock> start,end;
    start = std::chrono::steady_clock::now();

    for(size_t i=0; i < one_one_count; i++) {
        emit signalThing(thing);
    }
    emit signalStopThread(&thread_receiver);
    thread_receiver.wait();

    end = std::chrono::steady_clock::now();
    std::chrono::microseconds elapsed_ms =
            std::chrono::duration_cast<std::chrono::microseconds>(
                end-start);

    QString time_to_string =
            QString::number(elapsed_ms.count());

    std::string message0 = "1:1 signal:slot mapping:\n"
                          "1000 signals took: " +
            time_to_string.toStdString() + " us";

    // ============================================================= //

    thread_receiver.start();

    Receiver * r1 = new Receiver();
    Receiver * r2 = new Receiver();
    Receiver * r3 = new Receiver();

    r1->moveToThread(&thread_receiver);
    r2->moveToThread(&thread_receiver);
    r3->moveToThread(&thread_receiver);

    connect(this,SIGNAL(signalThing(Thing)),
            r1,SLOT(OnSignalThing(Thing)));

    connect(this,SIGNAL(signalThing(Thing)),
            r2,SLOT(OnSignalThing(Thing)));

    connect(this,SIGNAL(signalThing(Thing)),
            r3,SLOT(OnSignalThing(Thing)));

    // test 1 signal -> 4 slots
    size_t one_many_count=1000;
    start = std::chrono::steady_clock::now();

    for(size_t i=0; i < one_many_count; i++) {
        emit signalThing(thing);
    }
    emit signalStopThread(&thread_receiver);
    thread_receiver.wait();

    end = std::chrono::steady_clock::now();
    elapsed_ms = std::chrono::duration_cast<
            std::chrono::microseconds>(end-start);

    disconnect(this,SIGNAL(signalThing(Thing)),
               r0,SLOT(OnSignalThing(Thing)));

    disconnect(this,SIGNAL(signalStopThread(QThread*)),
               r0,SLOT(OnSignalStopThread(QThread*)));

    disconnect(this,SIGNAL(signalThing(Thing)),
               r1,SLOT(OnSignalThing(Thing)));

    disconnect(this,SIGNAL(signalThing(Thing)),
               r2,SLOT(OnSignalThing(Thing)));

    disconnect(this,SIGNAL(signalThing(Thing)),
               r3,SLOT(OnSignalThing(Thing)));

    r0->deleteLater();
    r1->deleteLater();
    r2->deleteLater();
    r3->deleteLater();

    time_to_string = QString::number(elapsed_ms.count());

    std::string message1 = "\n\n1:4 signal:slot mapping:\n"
                           "1000 signals took: " +
            time_to_string.toStdString() + " us";


    emit testComplete(QString::fromStdString(message0+message1));
}
