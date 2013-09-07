#include <QtGui/QApplication>
#include "imageviewer.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ImageViewer w;
    w.show();

    return a.exec();
}
