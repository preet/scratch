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

// This implementation is largely based on the info provided at:
// http://devmag.org.za/2011/04/05/bzier-curves-a-tutorial/

#include <QQuickCubicBezier.h>
#include <QSGGeometryNode>

QQuickCubicBezierItem::QQuickCubicBezierItem(QQuickItem *parent) :
    QQuickItem(parent)
{
    // init color to some immediately 'visible' type
    m_color.setRedF(1.0);
    m_color.setGreenF(0.0);
    m_color.setBlueF(1.0);
    m_color.setAlphaF(1.0);

    // init segments to some sane default
    m_segments = 10;

    // init points to some sane defaults
    m_p1 = QPointF(0,0);
    m_p4 = QPointF(100,0);
    m_p2 = QPointF(25,25);
    m_p2 = QPointF(50,25);
}

QQuickCubicBezierItem::~QQuickCubicBezierItem()
{

}

// ============================================================== //

QSGNode * QQuickCubicBezierItem::updatePaintNode(QSGNode *prevNode,
                                                 UpdatePaintNodeData *updData)
{
    Q_UNUSED(updData);

    QSGGeometryNode * node = static_cast<QSGGeometryNode*>(prevNode);
    QSGGeometry * geometry;

    if(!node) {

    }
    else {
        geometry = node->geometry();

    }
}

// ============================================================== //

QPointF QQuickCubicBezierItem::getP1() const
{
    return m_p1;
}

QPointF QQuickCubicBezierItem::getP2() const
{
    return m_p2;
}

QPointF QQuickCubicBezierItem::getP3() const
{
    return m_p3;
}

QPointF QQuickCubicBezierItem::getP4() const
{
    return m_p4;
}

quint32 QQuickCubicBezierItem::getSegments() const
{
    return m_segments;
}

QColor QQuickCubicBezierItem::getColor() const
{
    return m_color;
}

// ============================================================== //

void QQuickCubicBezierItem::setP1(QPointF const &p1)
{
    m_p1 = p1;
    this->update();
    emit p1Changed();
}

void QQuickCubicBezierItem::setP2(QPointF const &p2)
{
    m_p2 = p2;
    this->update();
    emit p2Changed();
}

void QQuickCubicBezierItem::setP3(QPointF const &p3)
{
    m_p3 = p3;
    this->update();
    emit p3Changed();
}

void QQuickCubicBezierItem::setP4(QPointF const &p4)
{
    m_p4 = p4;
    this->update();
    emit p4Changed();
}

void QQuickCubicBezierItem::setSegments(quint32 segments)
{
    m_segments = segments;
    this->update();
    emit segmentsChanged();
}

void QQuickCubicBezierItem::setColor(QColor const &color)
{
    m_color = color;
    this->update();
    emit colorChanged();
}
