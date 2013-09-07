#include <QtGui/QApplication>
#include "spindial.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    SpinDial w;
    w.show();

    return a.exec();
}
