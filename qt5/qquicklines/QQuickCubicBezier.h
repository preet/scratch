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

#ifndef QQUICK_CUBIC_BEZIER_H
#define QQUICK_CUBIC_BEZIER_H

#include <QQuickItem>
#include <QColor>

class QQuickCubicBezierItem : public QQuickItem
{
    Q_OBJECT

    Q_PROPERTY(QPointF p1 READ getP1 WRITE setP1 NOTIFY p1Changed)
    Q_PROPERTY(QPointF p2 READ getP2 WRITE setP2 NOTIFY p2Changed)
    Q_PROPERTY(QPointF p3 READ getP3 WRITE setP3 NOTIFY p3Changed)
    Q_PROPERTY(QPointF p4 READ getP4 WRITE setP4 NOTIFY p4Changed)
    Q_PROPERTY(quint32 segments READ getSegments WRITE setSegments NOTIFY segmentsChanged)
    Q_PROPERTY(qreal thickness READ getThickness WRITE setThickness NOTIFY thicknessChanged)
    Q_PROPERTY(QColor color READ getColor WRITE setColor NOTIFY colorChanged)

public:
    QQuickCubicBezierItem(QQuickItem * parent=0);
    ~QQuickCubicBezierItem();

signals:
    void p1Changed();
    void p2Changed();
    void p3Changed();
    void p4Changed();
    void segmentsChanged();
    void colorChanged();

protected:
    // updatePaintNode
    // * triggered after a call to QQuickItem::update()
    QSGNode * updatePaintNode(QSGNode *, UpdatePaintNodeData *);

private:
    QPointF getP1() const;
    QPointF getP2() const;
    QPointF getP3() const;
    QPointF getP4() const;
    quint32 getSegments() const;
    QColor getColor() const;

    void setP1(QPointF const &p1);
    void setP2(QPointF const &p2);
    void setP3(QPointF const &p3);
    void setP4(QPointF const &p4);
    void setSegments(quint32 segments);
    void setColor(QColor const &color);

    QPointF m_p1;
    QPointF m_p2;
    QPointF m_p3;
    QPointF m_p4;
    quint32 m_segments;
    QColor m_color;
};


#endif // QQUICK_CUBIC_BEZIER_H
