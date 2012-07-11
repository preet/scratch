#ifndef QDECTOUCHAREA_H
#define QDECTOUCHAREA_H

#include <math.h>
#include <QString>
#include <QTouchEvent>
#include <QDeclarativeItem>
#include <QGraphicsSceneMouseEvent>

class QDecTouchPoint : public QObject
{
    Q_OBJECT

    Q_PROPERTY(qreal x READ x NOTIFY xChanged)
    Q_PROPERTY(qreal y READ y NOTIFY yChanged)
    Q_PROPERTY(bool isActive READ isActive NOTIFY isActiveChanged)

public:
    QDecTouchPoint(QObject *parent=0)
        : m_x(0.0),
          m_y(0.0),
          m_isActive(false)
    {}

    qreal x()                       { return m_x; }
    qreal y()                       { return m_y; }
    bool isActive()                 { return m_isActive; }

    void setX(qreal nx)             { m_x = nx; emit xChanged(); }
    void setY(qreal ny)             { m_y = ny; emit yChanged(); }
    void setActive(bool isActive)   { m_isActive = isActive; emit isActiveChanged(); }

signals:
    void xChanged();
    void yChanged();
    void isActiveChanged();

private:
    qreal m_x;
    qreal m_y;
    bool m_isActive;

};

QML_DECLARE_TYPE(QDecTouchPoint)

class QDecTouchArea : public QDeclarativeItem
{
    Q_OBJECT

    // this is stupid but i dont like QVariantList or QDeclarativeListProperty
    // this is also intentionally xenophobic (MAX TEN FINGERS ALLOWED ALIEN SCUM)
    Q_PROPERTY(QDecTouchPoint* touchPoint0 READ touchPoint0)
    Q_PROPERTY(QDecTouchPoint* touchPoint1 READ touchPoint1)
    Q_PROPERTY(QDecTouchPoint* touchPoint2 READ touchPoint2)
    Q_PROPERTY(QDecTouchPoint* touchPoint3 READ touchPoint3)
    Q_PROPERTY(QDecTouchPoint* touchPoint4 READ touchPoint4)
    Q_PROPERTY(QDecTouchPoint* touchPoint5 READ touchPoint5)
    Q_PROPERTY(QDecTouchPoint* touchPoint6 READ touchPoint6)
    Q_PROPERTY(QDecTouchPoint* touchPoint7 READ touchPoint7)
    Q_PROPERTY(QDecTouchPoint* touchPoint8 READ touchPoint8)
    Q_PROPERTY(QDecTouchPoint* touchPoint9 READ touchPoint9)

    Q_PROPERTY(qreal diffScaleFactor READ diffScaleFactor NOTIFY scaleFactorChanged)
    Q_PROPERTY(qreal diffRotationAngle READ diffRotationAngle NOTIFY rotationAngleChanged)

    Q_PROPERTY(qreal totalScaleFactor READ totalScaleFactor NOTIFY scaleFactorChanged)
    Q_PROPERTY(qreal totalRotationAngle READ totalRotationAngle NOTIFY rotationAngleChanged)

public:
    QDecTouchArea(QDeclarativeItem *parent = 0);
    ~QDecTouchArea();

    QDecTouchPoint* touchPoint0()       { return m_listTouchPoints[0]; }
    QDecTouchPoint* touchPoint1()       { return m_listTouchPoints[1]; }
    QDecTouchPoint* touchPoint2()       { return m_listTouchPoints[2]; }
    QDecTouchPoint* touchPoint3()       { return m_listTouchPoints[3]; }
    QDecTouchPoint* touchPoint4()       { return m_listTouchPoints[4]; }
    QDecTouchPoint* touchPoint5()       { return m_listTouchPoints[5]; }
    QDecTouchPoint* touchPoint6()       { return m_listTouchPoints[6]; }
    QDecTouchPoint* touchPoint7()       { return m_listTouchPoints[7]; }
    QDecTouchPoint* touchPoint8()       { return m_listTouchPoints[8]; }
    QDecTouchPoint* touchPoint9()       { return m_listTouchPoints[9]; }

    qreal diffScaleFactor()          { return m_diffScaleFactor;    }
    qreal diffRotationAngle()        { return m_diffRotationAngle;  }
    qreal totalScaleFactor()         { return m_totalScaleFactor;   }
    qreal totalRotationAngle()       { return m_totalRotationAngle; }

signals:
    void touchPointChanged(int touchPtId);
    void touchPointPressed(int touchPtId);
    void touchPointMoved(int touchPtId);
    void touchPointReleased(int touchPtId);

    void scaleFactorChanged();
    void rotationAngleChanged();
    void touchEnded();
    void touchMoved();
    void touchStarted();

private:
    bool sceneEvent(QEvent *);
    void updateTouchData(QEvent *);

    QList<QDecTouchPoint*> m_listTouchPoints;
    qreal m_diffScaleFactor;
    qreal m_totalScaleFactor;
    qreal m_diffRotationAngle;
    qreal m_totalRotationAngle;

    qreal m_lk_2t_dist;
    qreal m_lk_2t_angle;
};

#endif
