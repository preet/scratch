#include <QtGui/QApplication>
#include "ringzoom.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    RingZoom w;
    w.show();

    return a.exec();
}
