#include <QGuiApplication>
#include <QQuickView>
#include <QTimer>
#include <QMutex>

#include "helper_testqtsigslotspeeds.h"
#include "helper_testkssigslotspeeds.h"
//#include "helper_testqtimerblock.h"

int main(int argc, char **argv)
{
    QGuiApplication app(argc,argv);
    QQuickView * view = new QQuickView;

    HelperTestKsSigSlotSpeed * helper =
            new HelperTestKsSigSlotSpeed(view);

//    HelperTestQtSigSlotSpeed * helper =
//            new HelperTestQtSigSlotSpeed(view);

//    HelperTestQTimerBlock * helper =
//            new HelperTestQTimerBlock(view);

    int rval = app.exec();

    delete helper;
    delete view;

    return rval;
}
