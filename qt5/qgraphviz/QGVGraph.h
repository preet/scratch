/*
   Copyright 2014 Preet Desai

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#ifndef QGRAPHVIZ_H
#define QGRAPHVIZ_H

// qt
#include <QObject>
#include <QColor>
#include <QHash>
#include <QList>

// graphviz
#include <gvc.h>
#include <cgraph.h>

class QGVGraph : public QObject
{
    Q_OBJECT

public:
    QGVGraph(QObject * parent=0);
    ~QGVGraph();

public slots:
    void onAddNode(QString const &name);
    void onAddEdge(QString const &source, QString const &target);
    void onDrawGraph();

signals:
    void drawNode(qreal center, qreal height, qreal width);
    void drawEdge(qreal p1, qreal p2, qreal thickness);
    void doneDrawingGraph();

private:
    // graphviz

    qreal m_node_size;

    QList<QString> m_list_nodes;
    QList<QPair<QString,QString> > m_list_edges;
};


#endif // QGRAPHVIZ_H
