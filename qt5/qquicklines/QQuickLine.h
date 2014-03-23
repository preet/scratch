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

#ifndef QQUICK_LINE_H
#define QQUICK_LINE_H

#include <QQuickItem>
#include <QColor>

class QQuickLineItem : public QQuickItem
{
    Q_OBJECT

    Q_PROPERTY(QPointF p1 READ getP1 WRITE setP1 NOTIFY p1Changed)
    Q_PROPERTY(QPointF p2 READ getP2 WRITE setP2 NOTIFY p2Changed)
    Q_PROPERTY(qreal thickness READ getThickness WRITE setThickness NOTIFY thicknessChanged)
    Q_PROPERTY(QColor color READ getColor WRITE setColor NOTIFY colorChanged)

public:
    QQuickLineItem(QQuickItem * parent=0);
    ~QQuickLineItem();

signals:
    void p1Changed();
    void p2Changed();
    void thicknessChanged();
    void colorChanged();

protected:
    // updatePaintNode
    // * triggered after a call to QQuickItem::update()
    QSGNode * updatePaintNode(QSGNode *, UpdatePaintNodeData *);

private:
    QPointF getP1() const;
    QPointF getP2() const;
    qreal getThickness() const;
    QColor getColor() const;

    void setP1(QPointF const &p1);
    void setP2(QPointF const &p2);
    void setThickness(qreal thickness);
    void setColor(QColor const &color);

    bool calcTriStrip(std::vector<QPointF> &list_vx);

    QPointF m_p1;
    QPointF m_p2;
    qreal m_thickness;
    QColor m_color;
};


#endif // QQUICK_LINE_H
