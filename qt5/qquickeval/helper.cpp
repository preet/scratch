#include "helper.h"

#include <QStandardPaths>

Helper::Helper(QQuickView *qqview, QObject *parent) :
    QObject(parent)
{
    m_view = qqview;
    m_view->rootContext()->setContextProperty("Helper",this);
    m_view->setResizeMode(QQuickView::SizeRootObjectToView);
    m_view->setSource(QUrl("qrc:/main.qml"));
    m_view->show();
}

void Helper::runTest()
{
    // do nothing
    qDebug() << "Hello World!";
}
