
#include "helper_testqtimerblock.h"

#include <QGuiApplication>
#include <thread>
#include <chrono>

Receiver::Receiver() :
    QObject(0)
{

}

void Receiver::OnTimeout()
{
    qDebug() << "Receiver::OnTimeout()";
}



HelperTestQTimerBlock::HelperTestQTimerBlock(QQuickView *qqview,
                                             QObject *parent) :
    Helper(qqview,parent)
{

}

void HelperTestQTimerBlock::runTest()
{
    QThread thread;
    thread.start();

    Receiver * receiver = new Receiver();
    receiver->moveToThread(&thread);

    connect(&timer,SIGNAL(timeout()),receiver,SLOT(OnTimeout()));
    timer.setInterval(100);
    timer.start();

    std::this_thread::sleep_for(
                std::chrono::milliseconds(1000));

    timer.stop();
    emit testComplete("ok!");

    receiver->deleteLater();

    thread.quit();
    thread.wait();
}

void HelperTestQTimerBlock::OnTimeout()
{
    qDebug() << "HelperTestQTimerBlock::OnTimeout()";
}
