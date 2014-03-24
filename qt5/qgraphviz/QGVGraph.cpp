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

#include <QGVGraph.h>

QGVGraph::QGVGraph(QObject * parent) :
    QObject(parent)
{

}

QGVGraph::~QGVGraph()
{

}

// ============================================================== //

void QGVGraph::onAddNode(QString const & name)
{
    m_list_nodes.push_back(name);
}

void QGVGraph::onAddEdge(QString const &source,QString const &target)
{
    QPair<QString,QString> edge;
    edge.first = source;
    edge.second = target;
    m_list_edges.push_back(edge);
}

void QGVGraph::onDrawGraph()
{
    // create graphviz context
    GVC_t * gv_context = gvContext();

    // create the graph
    Agraph_t * gv_graph = agopen("qgv_graph",Agstrictdirected,NULL);

    // set some graph attributes
    agsafeset(gv_graph,"splines","true","true");
    agsafeset(gv_graph,"dpi","96.0","96.0");
    agsafeset(gv_graph,"nodesep","0.25","0.25");
    agsafeset(gv_graph,"ranksep","0.5","0.5");

    // set some node attribute defaults
    agattr(gv_graph,AGNODE,"color","blue");
    agattr(gv_graph,AGNODE,"regular","true");
    agattr(gv_graph,AGNODE,"shape","polygon");
    agattr(gv_graph,AGNODE,"sides","4");
    agattr(gv_graph,AGNODE,"height","0.25");
    agattr(gv_graph,AGNODE,"width","0.25");
    agattr(gv_graph,AGNODE,"fixedsize","true");
    agattr(gv_graph,AGNODE,"label","");

    // default edge attributes
    agattr(gv_graph,AGEDGE,"arrowhead","none");
    agattr(gv_graph,AGEDGE,"arrowtail","none");


    // create the nodes and edges
    QHash<QString,Agnode_t*> list_gv_nodes;
    for(int i=0; i < m_list_nodes.size(); i++) {
        char * name = m_list_nodes[i].toLocal8Bit().data();
        Agnode_t * node = agnode(gv_graph,name,1);
        list_gv_nodes.insert(m_list_nodes[i],node);
    }

    QList<Agedge_t*> list_gv_edges;
    for(int i=0; i < m_list_edges.size(); i++) {
        Agnode_t * source = list_gv_nodes.value(m_list_edges[i].first.toLocal8Bit().data());
        Agnode_t * target = list_gv_nodes.value(m_list_edges[i].second.toLocal8Bit().data());
        QString edge_name = m_list_edges[i].first + m_list_edges[i].second;
        char * name = edge_name.toLocal8Bit().data();
        Agedge_t * edge = agedge(gv_graph,source,target,name,1);
        list_gv_edges.push_back(edge);
    }

    // layout the graph
    gvLayout(gv_context,gv_graph,"dot");

    // render
    gvRenderFilename(gv_context,gv_graph,"png","graph.png");

    // cleanup
    gvFreeLayout(gv_context,gv_graph);
    agclose(gv_graph);
    gvFreeContext(gv_context);

    emit doneDrawingGraph();
}

// ============================================================== //

