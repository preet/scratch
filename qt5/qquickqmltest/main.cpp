#include <QGuiApplication>
#include <QQuickView>
#include <QDebug>
#include <QImage>

int main(int argc, char **argv)
{
    QGuiApplication app(argc,argv);
    QQuickView * qqview = new QQuickView();
    qqview->setResizeMode(QQuickView::SizeRootObjectToView);
    qqview->setSource(QUrl("/home/preet/Dev/scratch/qt5/qquickqmltest/binpack_shelf.qml"));
    qqview->show();
    
    int rval = app.exec();
    delete qqview;

    return rval;
}
