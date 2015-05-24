#include <QGuiApplication>
#include <QQuickView>
#include <QTimer>
#include <QMutex>

#include "helper_testsptrsortanddiff.h"

//#include "helper_testqtsigslotspeeds.h"
//#include "helper_testkssigslotspeeds.h"
//#include "helper_testqtimerblock.h"

int main(int argc, char **argv)
{
    QGuiApplication app(argc,argv);
    QQuickView * view = new QQuickView;

//    Helper * helper = new Helper(view);

    HelperTestSharedPtrSortAndDiff* helper =
            new HelperTestSharedPtrSortAndDiff(view);

//    HelperTestKsSigSlotSpeed * helper =
//            new HelperTestKsSigSlotSpeed(view);

//    HelperTestQtSigSlotSpeed * helper =
//            new HelperTestQtSigSlotSpeed(view);

//    HelperTestQTimerBlock * helper =
//            new HelperTestQTimerBlock(view);

    int rval = app.exec();

    delete helper;
    delete view;

    return rval;
}
