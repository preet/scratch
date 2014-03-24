#include <QCoreApplication>
#include <QTimer>

#include "QGVGraph.h"

int main(int argc, char **argv)
{
    QCoreApplication app(argc,argv);

    QGVGraph * qgv = new QGVGraph(&app);
    QObject::connect(qgv,SIGNAL(doneDrawingGraph()),
                     &app,SLOT(quit()));

    // build a graph
    qgv->onAddNode("A");
    qgv->onAddNode("B");
    qgv->onAddNode("C");
    qgv->onAddNode("D");
    qgv->onAddNode("E");
    qgv->onAddNode("F");

    qgv->onAddEdge("A","B");
    qgv->onAddEdge("A","C");
    qgv->onAddEdge("B","D");
    qgv->onAddEdge("B","E");
    qgv->onAddEdge("C","F");

    QTimer::singleShot(0,qgv,SLOT(onDrawGraph()));

    int rval = app.exec();
    return rval;
}
