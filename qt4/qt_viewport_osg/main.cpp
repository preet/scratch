#include <QApplication>
#include "viewport.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Viewport w;
    w.setFixedSize(800,480);
    w.show();
    return a.exec();
}
