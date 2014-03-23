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

#include <cmath>

#include <QQuickLine.h>
#include <QSGGeometryNode>
#include <QSGFlatColorMaterial>

QQuickLineItem::QQuickLineItem(QQuickItem *parent) :
    QQuickItem(parent)
{
    // init color to some immediately 'visible' type
    m_color.setRedF(1.0);
    m_color.setGreenF(0.0);
    m_color.setBlueF(1.0);
    m_color.setAlphaF(1.0);

    // init thickness to a sane default
    m_thickness = 10;

    // init points to some sane defaults
    m_p1 = QPointF(0,0);
    m_p2 = QPointF(0,100);

    //
    setFlag(ItemHasContents, true);
}

QQuickLineItem::~QQuickLineItem()
{

}

// ============================================================== //

QSGNode * QQuickLineItem::updatePaintNode(QSGNode *prev_node,
                                          UpdatePaintNodeData *upd_data)
{
    Q_UNUSED(upd_data);

    QSGGeometryNode * node = static_cast<QSGGeometryNode*>(prev_node);
    QSGGeometry * geometry = NULL;
    QSGFlatColorMaterial * material = NULL;

    if(!node) {
        // http://qt-project.org/doc/qt-5/qsggeometrynode.html
        node = new QSGGeometryNode;
        geometry = new QSGGeometry(QSGGeometry::defaultAttributes_Point2D(),4);
        geometry->setDrawingMode(GL_TRIANGLE_STRIP);
        node->setGeometry(geometry);
        node->setFlag(QSGNode::OwnsGeometry);

        material = new QSGFlatColorMaterial;
        material->setColor(m_color);
        node->setMaterial(material);
        node->setFlag(QSGNode::OwnsMaterial);
    }
    else {
        geometry = node->geometry();
        geometry->allocate(4); // we have to call allocate to invalidate
                               // the older vertex buffer
        material = static_cast<QSGFlatColorMaterial*>(node->material());
    }

    // geometry
    std::vector<QPointF> list_vx;
    if(!calcTriStrip(list_vx)) {
        list_vx.clear();
        list_vx.push_back(QPointF(0,0));
        list_vx.push_back(QPointF(0,0));
        list_vx.push_back(QPointF(0,0));
        list_vx.push_back(QPointF(0,0));
    }

    QSGGeometry::Point2D * vertices =
            geometry->vertexDataAsPoint2D();

    for(size_t i=0; i < list_vx.size(); i++) {
        vertices[i].set(list_vx[i].x(),
                        list_vx[i].y());
    }

    node->markDirty(QSGNode::DirtyGeometry);

    // material
    material->setColor(m_color);
    node->markDirty(QSGNode::DirtyMaterial);

    return node;
}

// ============================================================== //

QPointF QQuickLineItem::getP1() const
{
    return m_p1;
}

QPointF QQuickLineItem::getP2() const
{
    return m_p2;
}

qreal QQuickLineItem::getThickness() const
{
    return m_thickness;
}

QColor QQuickLineItem::getColor() const
{
    return m_color;
}

// ============================================================== //

void QQuickLineItem::setP1(QPointF const &p1)
{
    m_p1 = p1;
    this->update();
    emit p1Changed();
}

void QQuickLineItem::setP2(QPointF const &p2)
{
    m_p2 = p2;
    this->update();
    emit p2Changed();
}

void QQuickLineItem::setThickness(qreal thickness)
{
    m_thickness = thickness;
    this->update();
    emit thicknessChanged();
}

void QQuickLineItem::setColor(QColor const &color)
{
    m_color = color;
    this->update();
    emit colorChanged();
}

// ============================================================== //

bool QQuickLineItem::calcTriStrip(std::vector<QPointF> &list_vx)
{
    if(m_p1 == m_p2 || m_thickness < 1.0) {
        return false;
    }

    // calculate the direction vector
    QPointF vec_dirn_p1p2 = m_p2-m_p1;

    // calculate the normal
    double inv_magnitude = 1.0/sqrt((vec_dirn_p1p2.x()*vec_dirn_p1p2.x()) +
                                    (vec_dirn_p1p2.y()*vec_dirn_p1p2.y()));

    QPointF vec_norm_p1p2(vec_dirn_p1p2.y()*-1.0*inv_magnitude*m_thickness,
                          vec_dirn_p1p2.x()*inv_magnitude*m_thickness);

    // offset the start
    QPointF p1_a = m_p1+vec_norm_p1p2;
    QPointF p1_b = m_p1+(vec_norm_p1p2*-1.0);

    QPointF p2_a = m_p2+vec_norm_p1p2;
    QPointF p2_b = m_p2+(vec_norm_p1p2*-1.0);

    list_vx.reserve(4);
    list_vx.push_back(p1_a);
    list_vx.push_back(p1_b);
    list_vx.push_back(p2_a);
    list_vx.push_back(p2_b);
    return true;
}
